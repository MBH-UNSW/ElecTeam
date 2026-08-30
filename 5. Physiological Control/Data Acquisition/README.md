# MCL Data Acquisition System 
## Purpose 
Firmware for MCL PCB. 

The program reads: 
- 2x Flow sensors 
- 2x Honeywell ABP2 preload pressure sensors 
- 2x Adafruit/Honeywell MPRLS pressure sensors

and displays readings onto an SSD1309 OLED display 

## Additional Notes 
- I2C is configured to operate at 400 kHz Fast Mode. 
- Sensor readings and the OLED display are updated once per second. 
- MPR sensors are ZEROED at startup. Ensure they are under the intended zero pressure condition before powering/resetting the system. 

## Required libraries 
This program uses the Arduino framework and requires the following libraries: 
- **U8g2** - SSD1309 OLED display 
- **Adafruit MPRLS** 

If using the Arduino IDE instead of VS Code / PlatformIO: 
1. Copy the code from src/main.cpp 
2. Select **ESP32-S3 Dev Module** and COM port. 
3. Ensure Serial Monitor baud rate is set to 115200. 

