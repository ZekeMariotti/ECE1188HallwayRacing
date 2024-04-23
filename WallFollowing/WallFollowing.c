#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "I2CB1.h"
#include "CortexM.h"
#include "LPF.h"
#include "opt3101.h"
#include "LaunchPad.h"
#include "Bump.h"
#include "Motor.h"
#include "UART0.h"
#include "SSD1306.h"
#include "FFT.h"
#include "Reflectance.h"
#include "SysTickInts.h"
//#include "MQTT_WebApp.c"
#include "Tachometer.h"

// Functions and defines from Lab21_OPT3101_TestMain.c
#define USEUART

#ifdef USEUART
// this batch configures for UART link to PC
//#include "UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY)
{
    if(newX == 6)
    {
        UART0_OutString("\n\rTxChannel= ");
        UART0_OutUDec(newY-1);
        UART0_OutString(" Distance= ");
    }
    else
    {
        UART0_OutString("\n\r");
    }
}
void UartClear(void){ UART0_OutString("\n\r"); };
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutSDec UART0_OutSDec
#endif

uint32_t DistancesLogInts[1000][3];
uint32_t Distances[3];
char DistanceString[30];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t Noises[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec

bool pollDistanceSensor(void)
{
    if(OPT3101_CheckDistanceSensor())
    {
        TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
        return true;
    }
    return false;
}

// calibrated for 500mm track
// right is raw sensor data from right sensor
// return calibrated distance from center of Robot to right wall
int32_t Right(int32_t right) { return (right*(59*right + 7305) + 2348974)/32768; }

// left is raw sensor data from left sensor
// return calibrated distance from center of Robot to left wall
int32_t Left(int32_t left) { return (1247*left)/2048 + 22; }

// assumes track is 500mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
int32_t Ki=1;  // integral controller gain
int32_t Kp=4;  // proportional controller gain //was 4
int32_t UR, UL;  // PWM duty 0 to 14,998

#define TOOCLOSE 200 //was 200
#define DESIRED 250 //was 250
int32_t SetPoint = 250; // mm //was 250
int32_t LeftDistance,CenterDistance,RightDistance; // mm
#define TOOFAR 400 // was 400

#define PWMNOMINAL 5000 // was 2500
#define SWING 2000 //was 1000
#define PWMMIN (PWMNOMINAL-SWING)
#define PWMMAX (PWMNOMINAL+SWING)
void Controller(void)
{ // runs at 100 Hz
    if(Mode)
    {
        if((LeftDistance>DESIRED)&&(RightDistance>DESIRED))
        {
            SetPoint = (LeftDistance+RightDistance)/2;
        }
        else
        {
            SetPoint = DESIRED;
        }
        if(LeftDistance < RightDistance )
        {
            Error = LeftDistance-SetPoint;
        }
        else
        {
            Error = SetPoint-RightDistance;
        }
//        UR = UR + Ki*Error;      // adjust right motor
        UR = PWMNOMINAL+Kp*Error; // proportional control
        UL = PWMNOMINAL-Kp*Error; // proportional control
        if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
        if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
        if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
        if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;
        Motor_Forward(UL,UR);
    }
}

void Controller_Right(void)
{ // runs at 100 Hz
    if(Mode)
    {
        if((RightDistance>DESIRED))
        {
            SetPoint = (RightDistance)/2;
        }
        else
        {
            SetPoint = DESIRED;
        }
        /*if(LeftDistance < RightDistance ){
          Error = LeftDistance-SetPoint;
        }else {
          Error = SetPoint-RightDistance;
        }*/

        Error = SetPoint-RightDistance;
        //UR = UR + Ki*Error;      // adjust right motor
        UR = PWMNOMINAL+Kp*Error; // proportional control
        UR = UR + Ki*Error;      // adjust right motor
        UL = PWMNOMINAL-Kp*Error; // proportional control
        if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
        if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
        if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
        if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;

        //turns left if the center measurement and right measurement is small enough that we will hit the wall if we don't turn
        if((RightDistance<250) && (CenterDistance <250))
        {
            UL = 0;
            UR = PWMNOMINAL;
        }
        Motor_Forward(UL,UR);
    }
}

void Pause(void)
{
    int i;
    while(Bump_Read())
    { // wait for release
        Clock_Delay1ms(200); LaunchPad_Output(0); // off
        Clock_Delay1ms(200); LaunchPad_Output(1); // red
    }
    while(Bump_Read()==0)
    {// wait for touch
        Clock_Delay1ms(100); LaunchPad_Output(0); // off
        Clock_Delay1ms(100); LaunchPad_Output(3); // red/green
    }
    while(Bump_Read())
    { // wait for release
        Clock_Delay1ms(100); LaunchPad_Output(0); // off
        Clock_Delay1ms(100); LaunchPad_Output(4); // blue
    }
    for(i=1000;i>100;i=i-200)
    {
        Clock_Delay1ms(i); LaunchPad_Output(0); // off
        Clock_Delay1ms(i); LaunchPad_Output(2); // green
    }
    // restart Jacki
    UR = UL = PWMNOMINAL;    // reset parameters
    Mode = 1;
}

volatile uint32_t Time = 0, MainCount, TimeSeconds = 0;
uint8_t lightSensorResult = 0;
uint8_t past_start = 0;
uint8_t hit_white_paper = 0;
uint8_t finish_line_orientations[9] = {0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0x7F, 0x3F, 0x1F, 0x0F};

uint32_t num_crashes = 0;
/* tachometer */
uint16_t avg(uint16_t *array, int length)
{
  int i;
  uint32_t sum = 0;
  for(i=0; i<length; i=i+1){
    sum = sum + array[i];
  }
  return (sum/length);
}

uint16_t ActualL;
uint16_t ActualR;
int iBuf = 0;

#define TACHBUFF 10
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
int32_t RightSteps;

/* end tachometer */

int main(void)
{
    uint16_t leftMaxRPM = 0, rightMaxRPM = 0;
    uint32_t channel = 1;
    Time = MainCount = 0;
    DisableInterrupts();
    Clock_Init48MHz();
    SysTick_Init(48000, 2);
    Bump_Init();
    LaunchPad_Init(); // built-in switches and LEDs
    Tachometer_Init();
    Motor_Init();
    Motor_Stop(); // initialize and stop
    Mode = 0;
    I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
    UART0_Init();
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();
    OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
    TxChannel = 3;
    OPT3101_StartMeasurementChannel(channel);
    LPF_Init(100,8);
    LPF_Init2(100,8);
    LPF_Init3(100,8);
    UR = UL = PWMNOMINAL; //initial power
    Pause();
    EnableInterrupts();
    Motor_Forward(7000, 7000);
    past_start = 1;

    Motor_Stop();
    Mode = 0;

    // reset time
    TimeSeconds = 0;

    // initialize distance log values and set iDist to 0
    for (int i = 0; i < 1000; i++){
        DistancesLogInts[i][0] = NULL;
        DistancesLogInts[i][1] = NULL;
        DistancesLogInts[i][2] = NULL;
    }

    int iDist = 0;

    Motor_Forward(7000, 7000);
    Mode = 1;
    LaunchPad_LED(0);

    char command;
    while(1)
    {
        // set time
        if (MainCount % 1000 == 0){
            TimeSeconds++;
        }

        // blue tooth
        command = UART0_InChar();
        if (command == 'g') {
            Motor_Forward(7000, 7000);
            Mode = 1;
            LaunchPad_LED(0);
        }
        else if (command == 's') {
            Motor_Stop();
            Mode = 0;
            LaunchPad_LED(1);
        }
        Reflectance_Start();

        Clock_Delay1ms(1);

        lightSensorResult = Reflectance_End();
        if (lightSensorResult == 0x00)
        {
            hit_white_paper = 1;
        }
        if (hit_white_paper && past_start)
        {
            int i;
            for (i = 0; i < 9; i++)
            {
                if (lightSensorResult == finish_line_orientations[i])
                {
                    Mode = 0;
                    Motor_Stop();
                    sendData(leftMaxRPM, rightMaxRPM, TimeSeconds, num_crashes, DistancesLogInts);
                    break;
                }
            }
        }

        /* tachometer */
        if (MainCount%100 == 0){
            Tachometer_Get(&LeftTach[iBuf], &LeftDir, &LeftSteps, &RightTach[iBuf], &RightDir, &RightSteps);
            iBuf = iBuf + 1;

            if(iBuf >= TACHBUFF)
            {
                //Reset the buffer index
                iBuf = 0;

                //Compute the Average Revolutions Per Minute over the most recent  TACHBUFF # of Samples
                // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
                ActualL = 2000000/avg(LeftTach, TACHBUFF);
                ActualR = 2000000/avg(RightTach, TACHBUFF);

                if (ActualL > leftMaxRPM){
                    leftMaxRPM = ActualL;
                }

                if (ActualR > rightMaxRPM){
                    rightMaxRPM = ActualR;
                }
            }
        }
        /* end tachometer */


        if(Bump_Read())
        { // collision
            Mode = 0;
            Motor_Stop();
            Clock_Delay1ms(500);
//            Mode = 1;
            num_crashes++;
        }

        if(TxChannel <= 2)
        { // 0,1,2 means new data
            if(TxChannel==0)
            {
                if(Amplitudes[0] > 1000)
                {
                    LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
                }
                else
                {
                    LeftDistance = FilteredDistances[0] = 500;
                }
            }
            else if(TxChannel==1)
            {
                if(Amplitudes[1] > 1000)
                {
                    CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
                }
                else
                {
                    CenterDistance = FilteredDistances[1] = 500;
                }
            }
            else
            {
                if(Amplitudes[2] > 1000){
                    RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
                }
                else
                {
                    RightDistance = FilteredDistances[2] = 500;
                }
            }
            SetCursor(2, TxChannel+1);
            OutUDec(FilteredDistances[TxChannel]); OutChar(','); OutUDec(Amplitudes[TxChannel]);
            TxChannel = 3; // 3 means no data
            channel = (channel+1)%3;
            OPT3101_StartMeasurementChannel(channel);
        }

        if (MainCount % 100 == 0){
            DistancesLogInts[iDist][0] = Distances[0];
            DistancesLogInts[iDist][1] = Distances[1];
            DistancesLogInts[iDist][2] = Distances[2];
            iDist++;
        }

        Controller();
        WaitForInterrupt();
        MainCount++;
    }
}
