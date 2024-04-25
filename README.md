# ECE1188HallwayRacing
Aiden Seo
Christopher Brubaker
Ezekiel Mariotti
Khalid Brelvi

├── _WallFollowing                           - Main Project Folder  
│   ├── .launches                            - CCS files  
│   ├── .settings                            - CCS files  
│   ├── board                                - Lab21_OPT3101 Included Files  
│   ├── cli_uart                             - Lab21_OPT3101 Included Files  
│   ├── driverlib/MSP432P4xx                 - MSP432P4 specific files (Defaulted from Project)  
│   ├── mqtt                                 - MQTT files used by MQTT_WebApp.c  
│   ├── simplelink                           - Lab21_OPT3101 Included Files  
│   ├── spi_cc3100                           - Lab21_OPT3101 Included Files  
│   ├── targetConfigs                        - CCS files  
│   ├── uart_cc3100                          - Lab21_OPT3101 Included Files  
│   ├── .ccsproject, .cproject, .project     - CCS files  
│   ├── Bump.c, Bump.h                       - Header and Source files for the bump sensors (Not Interrupt Triggered)  
│   ├── Clock.c, Clock.h                     - Header and Source files for the Clock module (TI Given)  
│   ├── CortexM.c, CortexM.h                 - Header and Source files for the CortexM module (TI Given)  
│   ├── FFT.c, FFT.h                         - Header and Source files for FFT (Lab21_OPT3101 Given)  
│   ├── I2CB1.c, I2CB1.h                     - Header and Source files for the I2C module (TI Given)  
│   ├── LPF.c, LPF.h                         - Header and Source files for the LPF module (Lab21_OPT3101 Given)  
│   ├── Lab21_OPT3101_TestMain.c             - Original main file that our project was based on (Lab21_OPT3101 Given)  
│   ├── LaunchPad.c, LaunchPad.h             - Header and Source files for the LaunchPad board (TI Given)  
│   ├── MQTT_WebApp.c                        - File for running all the MQTT program and sending data to the web app (Function called in WallFollowing.c)  
│   ├── Motor.c, Motor.h                     - Header and Source files to control the motors  
│   ├── Nokia5110.c, Nokia5110.h             - Not Used  
│   ├── PWM.c, PWM.h                         - Header and Source files to use PWM  
│   ├── Reflectance.c, Reflectance.h         - Header and Source files to read and sense data from the IR reflectance sensor on the bottom of the board  
│   ├── SSD1306.c, SSD1306.h                 - Not Used  
│   ├── SysTickInts.c, SysTickInts.h         - Header and Source files for the SysTick Module (Interrupt Triggered)  
│   ├── TA3InputCapture.c, TA3InputCapture.h - Header and Source files used with the Tachometer (Lab 5 Given)  
│   ├── Tachometer.c, Tachometer.h           - Header and Source files used with the Tachometer (Lab 5 Given)  
│   ├── TimerA1.c, TimerA1.h                 - Header and Source files for the A1 Timer module (Lab 5 Given)  
│   ├── UART0.c, UART0.h                     - Header and Source files for controlling the UART 0 Module (TI Given)  
│   ├── WallFollowing.c                      - Main file that the code executes from. Includes our controller and the main function  
│   ├── msp432p401r.cmd                      - CCS file  
│   ├── opt3101.c, opt3101.h                 - Header and Source files for the distance sensor (Lab 5 Given)  
│   ├── sl_common.h                          - Included in MQTT_WebApp.c 
│   ├── startup_msp432p401r_ccs.c            - CCS file  
│   └── system_msp432p401r.c                 - CCS file  
└── README.md                                - Current README document  
