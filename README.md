# chk_fmu
## Flight Microcontroller Unit for LARR

This package is for '**Flight Microcontroller Unit (FMU) by chk**.
It is built from the scratch using a STM32F407VET6 MCU.

CHK_FMU can provide belows:

* 12 PWM outputs with a configurable frequency (1 Hz ~ 3 kHZ tested, 3.3 Volts HIGH)

* 4 analog reads (0 ~ 3.3 Volts)

* 1 kHz IMU datastream (3-D acc. 3-D gyro, 3-D Magnetometers from Invensense MPU-9250)

* Serial communication with PC by micro-USB cable (Attached to USART1, baudrate up to 921600, using CH340G TTL-USB converter)

* USART port (Attached to USART2, baudrate up to 921600) for telemetry

* I2C port (Attached to I2C1, clock frequency up to 400 kHz)

* two 3.3 V (LM1117-3.3V regulator) pinouts and two 5.0 V (USB) pinouts. (only for peripheral MEMS sensors with low power consumption)

1.Requirements
------
* CHK_FMU board
* Mbed Studio (software for uploading the code to the FMU)
* ST-LINK/V2 (hardware for uploading the code to the FMU)
* four female-to-female jumper cables (to connect ST-LINK/V2 and the FMU)
* micro-USB cable (5V power from PC to FMU, and USB communication with PC)

- `aa`: dfdf
<p align = "left">
<img src= "https://github.com/JunhaAgu/AutoL2LCalib/blob/main/imgs/pcd_structure.png" alt="pcd file inside" width="250" height="270">
</p> 
2.Upload FMU program
------
  1) Install the Mbed Studio (https://os.mbed.com/studio/)

  2) Run Mbed Studio

  3) 'File'-'New Program...'

  4) In the pop-up menu (titled by 'New program'), select 'empty Mbed OS program' from dropbar menu, and put a program name you want. For example, 'CHK_FMU_Program'.
  
  5) (Linux) Open 'Files'. At the 'Home' folder, you can find 'Mbed Programs' folder, and 'CHK_FMU_Program' folder in it.
  
  6) Copy all contents of the 'MBED_FILES' folder of this repository to the 'Mbed Programs/CHK_FMU_Program/'.
  7) Back to the Mbed Studio program.
  8) Now, you can see the file navigator on the left that 'TARGET_CHKBOARD' folder, 'custom_targets.json' file, and many cpp and h files are added.
  9) 
<p align = "center">
<img src= "https://github.com/JunhaAgu/AutoL2LCalib/blob/main/imgs/aligned_four_lidars.png" alt="aligned four lidars via the AutoL2LCalib" width="450" height="470">
</p> 

  9) Connect ST-LINK/V2 and the FMU. 
  10) 
  11) At the left navigator bar, drop down the 'Target' dropdown menu, and click 'Manage custom targets' (chip-shaped icon).
  12)  
