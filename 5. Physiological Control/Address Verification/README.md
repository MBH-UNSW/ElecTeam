# MCL Sensor Address Verification 

## Purpose 
This test verifies I2C communication between the ESP32 and the sensors used. 
The program checks that each device responds at its expected I2C address before data acquisition or calibration is performed. 

## Hardware 
- ESP32S3 
- TCA9548A I2C Mux 
- 2x Honeywell MPR pressure sensors 
- 2x Honeywell ABP2 preload pressure sensors 
- SSD1309 OLED display 

## Running the test
1. Connect the MCL PCB to the computer. 
2. When compiling in VsCode ensure PlatformIO extension is installed. 
3. Build the project using PlatformIO and upload program. 
4. Open the Serial Monitor at '115200' baud. 
5. Check OLED and Serial Monitor for verification results. 

A correct functioning report should indicate all "PASS". 

## Required libraries 
This program uses the Arduino framework and requires the following libraries: 
- **U8g2** - SSD1309 OLED display 

If using the Arduino IDE instead of VS Code / PlatformIO: 
1. Copy the code from src/main.cpp 
2. Select **ESP32-S3 Dev Module** and COM port. 
3. Ensure Serial Monitor baud rate is set to 115200. 


