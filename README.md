# PRIMARY-ATTITUDE-DETERMINATION-TEST
Description: Provides IMU measurement functionality and radio communication with a ground station. This will be used as the test software for the Primary Attitude Validation Test for the Guidance, Navigation and Control subsystem.

+# Prerequisites
 +## Software
 +* Code Composer Studio v7.0 (TI ARM Compiler v16.9.LTS)
 +* TivaWare for C Series v2.1 (full with driverlib and sensorlib)
 
 +## Hardware
 +* Texas Instruments EK-TM4C1294XL LaunchPad
 +* Texas Instruments BOOSTXL-SENSORS
 +* 3D Robotics Radio V2
 
 +# Installation
 +1. Add variable SW_ROOT in CCS Workspace. (For example via import vars.ini file with SW_ROOT = C:\Ti\TivaWare_C_Series-2.1.4.178 )
 +2. Import project into Workspace (Project -> Import CCS Project)
 +3. Build.
