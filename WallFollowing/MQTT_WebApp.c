// Standard includes
#include <stdlib.h>
#include <string.h>

#include "driverlib.h"
#include "simplelink.h"
#include "sl_common.h"
#include "MQTTClient.h"
#include "Motor.h"
#include "Clock.h"
#include "CortexM.h"

#include "TimerA1.h"
#include "TA3InputCapture.h"
#include "Bump.h"
#include "I2CB1.h"
#include "opt3101.h"
#include "LaunchPad.h"

/*
 * Values for below macros shall be modified per the access-point's (AP) properties
 * SimpleLink device will connect to following AP when the application is executed
 */
//#define SSID_NAME       "ECE DESIGN LAB 2.4"       /* Access point name to connect to. */
#define SSID_NAME        "khalid"
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
#define PASSKEY         "coolwifi"
//#define PASSKEY         "ecedesignlab12345"   /* Password in case of secure AP */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)  /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
#define MQTT_BROKER_SERVER  "broker.hivemq.com"
#define SUBSCRIBE_TOPIC "ECE1188Lab6_2"
#define PUBLISH_TOPIC "ECE1188Ironman"
#define PUBLISH_TOPIC_LEFT "ECE1188IronmanL"
#define PUBLISH_TOPIC_RIGHT "ECE1188IronmanR"
#define PUBLISH_TOPIC_TIME "ECE1188IronmanT"
#define PUBLISH_TOPIC_CRASHES "ECE1188IronmanC"
#define PUBLISH_TOPIC_DISTANCES0 "ECE1188IronmanD0"
#define PUBLISH_TOPIC_DISTANCES1 "ECE1188IronmanD1"
#define PUBLISH_TOPIC_DISTANCES2 "ECE1188IronmanD2"

// MQTT message buffer size
#define BUFF_SIZE 32


#define APPLICATION_VERSION "1.0.0"

#define MCLK_FREQUENCY 48000000
#define PWM_PERIOD 255

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define min(X,Y) ((X) < (Y) ? (X) : (Y))


/*
 * GLOBAL VARIABLES -- Start
 */
/* Button debounce state variables */
volatile unsigned int S1buttonDebounce = 0;
volatile unsigned int S2buttonDebounce = 0;
volatile int publishID = 0;

unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address

Network n;
Client hMQTTClient;     // MQTT Client

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;

/* Port mapper configuration register */
const uint8_t port_mapping[] =
{
    //Port P2:
    PM_TA0CCR1A, PM_TA0CCR2A, PM_TA0CCR3A, PM_NONE, PM_TA1CCR1A, PM_NONE, PM_NONE, PM_NONE
};

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_8,          // SMCLK/8 = 6MHz
        90000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/*
 * GLOBAL VARIABLES -- End
 */


/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void displayBanner();
static void messageArrived(MessageData*);
static void generateUniqueID();


/*
 * STATIC FUNCTION DEFINITIONS -- End
 */


/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
            switch( pSock->EventData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */


/*
 * Application's entry point
 */
int sendData(uint16_t leftMaxRPM, uint16_t rightMaxRPM, uint32_t trackTime, uint32_t numCrashes, char *DistancesLogInts[100][3])
{
    Clock_Init48MHz();
    _i32 retVal = -1;

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    /* Configure command line interface */
    CLI_Configure();

    displayBanner();

    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) ||
        (ROLE_STA != retVal) )
    {
        CLI_Write(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Device started as STATION \n\r");

    /* Connecting to WLAN AP */
    retVal = establishConnectionWithAP();
    if(retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

    // Obtain MAC Address
    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);

    // Print MAC Addres to be formatted string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);

    // Generate 32bit unique ID from TLV Random Number and MAC Address
    generateUniqueID();

    int rc = 0;
    unsigned char buf[100];
    unsigned char readbuf[100];

    NewNetwork(&n);
    rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, 1883);

    if (rc != 0) {
        CLI_Write(" Failed to connect to MQTT broker \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Connected to MQTT broker \n\r");

    MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
    MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
    cdata.MQTTVersion = 3;
    cdata.clientID.cstring = uniqueID;
    rc = MQTTConnect(&hMQTTClient, &cdata);

    if (rc != 0) {
        CLI_Write(" Failed to start MQTT client \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Started MQTT client successfully \n\r");

    rc = MQTTSubscribe(&hMQTTClient, SUBSCRIBE_TOPIC, QOS0, messageArrived);

    if (rc != 0) {
        CLI_Write(" Failed to subscribe to /msp/cc3100/demo topic \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Subscribed to /msp/cc3100/demo topic \n\r");

    rc = MQTTSubscribe(&hMQTTClient, uniqueID, QOS0, messageArrived);

    if (rc != 0) {
        CLI_Write(" Failed to subscribe to uniqueID topic \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Subscribed to uniqueID topic \n\r");

    // Message variables
    char payloadL[20] = "", payloadR[20] = "", payloadT[20] = "", payloadC[20] = "";

    rc = MQTTYield(&hMQTTClient, 10);
    if (rc != 0) {
        CLI_Write(" MQTT failed to yield \n\r");
        LOOP_FOREVER();
    }

    // create message
    MQTTMessage data;
    data.dup = 0;
    data.id = 0;
    data.payloadlen = 20;
    data.qos = QOS0;
    data.retained = 0;

    snprintf(payloadL, sizeof(payloadL), "%d", leftMaxRPM);
    data.payload = payloadL;
    rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_LEFT, &data);

    snprintf(payloadR, sizeof(payloadR), "%d", rightMaxRPM);
    data.payload = payloadR;
    rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_RIGHT, &data);

    snprintf(payloadT, sizeof(payloadT), "%d", trackTime);
    data.payload = payloadT;
    rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_TIME, &data);

    snprintf(payloadC, sizeof(payloadC), "%d", numCrashes);
    data.payload = payloadC;
    rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_CRASHES, &data);

    data.payloadlen = 50;

    for (int i = 0; i < 1000; i++){
        if (DistancesLogInts[i][0] != NULL){
            char payloadD0[50] = "", payloadD1[50] = "", payloadD2[50] = "";

            snprintf(payloadD0, sizeof(payloadD0), "%d", DistancesLogInts[i][0]);
            data.payload = payloadD0;
            rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_DISTANCES0, &data);

            snprintf(payloadD1, sizeof(payloadD1), "%d", DistancesLogInts[i][1]);
            data.payload = payloadD1;
            rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_DISTANCES1, &data);

            snprintf(payloadD2, sizeof(payloadD2), "%d", DistancesLogInts[i][2]);
            data.payload = payloadD2;
            rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_DISTANCES2, &data);
        }
    }
}

static void generateUniqueID() {
    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData(TLV->RANDOM_NUM_2);
    CRC32_set32BitData(TLV->RANDOM_NUM_3);
    CRC32_set32BitData(TLV->RANDOM_NUM_4);
    int i;
    for (i = 0; i < 6; i++)
    CRC32_set8BitData(macAddressVal[i], CRC32_MODE);

    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
    sprintf(uniqueID, "%06X", crcResult);
}

//****************************************************************************
//
//!    \brief MQTT message received callback - Called when a subscribed topic
//!                                            receives a message.
//! \param[in]                  data is the data passed to the callback
//!
//! \return                        None
//
//****************************************************************************
static void messageArrived(MessageData* data) {
    char buf[BUFF_SIZE];

    char *tok;
//    long color;

    // Check for buffer overflow
    if (data->topicName->lenstring.len >= BUFF_SIZE) {
//      UART_PRINT("Topic name too long!\n\r");
        return;
    }
    if (data->message->payloadlen >= BUFF_SIZE) {
//      UART_PRINT("Payload too long!\n\r");
        return;
    }

    strncpy(buf, data->topicName->lenstring.data,
        min(BUFF_SIZE, data->topicName->lenstring.len));
    buf[data->topicName->lenstring.len] = 0;



    strncpy(buf, data->message->payload,
        min(BUFF_SIZE, data->message->payloadlen));
    buf[data->message->payloadlen] = 0;

    tok = strtok(buf, " ");


    if (strcmp(tok, "go") == 0){
        Motor_Forward(3000, 1000);
    }
    else if (strcmp(tok, "stop") == 0){
        Motor_Stop();
    }

    return;
}

/*
 * Port 1 interrupt handler. This handler is called whenever the switch attached
 * to P1.1 is pressed.
 */
void PORT1_IRQHandler(void)
{
    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1)
    {
        if (S1buttonDebounce == 0)
        {
            S1buttonDebounce = 1;

            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

            // Publish the unique ID
            publishID = 1;

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
    if (status & GPIO_PIN4)
    {
        if (S2buttonDebounce == 0)
        {
            S2buttonDebounce = 1;

            CLI_Write(" MAC Address: \n\r ");
            CLI_Write(macStr);
            CLI_Write("\n\r");

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
}

//void TA1_0_IRQHandler(void)
//{
//    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
//    if (P1IN & GPIO_PIN1)
//    {
//        S1buttonDebounce = 0;
//    }
//    if (P1IN & GPIO_PIN4)
//    {
//        S2buttonDebounce = 0;
//    }
//
//    if ((P1IN & GPIO_PIN1) && (P1IN & GPIO_PIN4))
//    {
//        Timer_A_stopTimer(TIMER_A1_BASE);
//    }
//    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
//                TIMER_A_CAPTURECOMPARE_REGISTER_0);
//}


/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}

/*!
    \brief This function displays the application's banner

    \param      None

    \return     None
*/
static void displayBanner()
{
    CLI_Write("\n\r\n\r");
    CLI_Write(" MQTT Twitter Controlled RGB LED - Version ");
    CLI_Write(APPLICATION_VERSION);
    CLI_Write("\n\r*******************************************************************************\n\r");
}
