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

## Using Arduino IDE instead of VS Code / PlatformIO 
Before compiling, install 
1. **ESP32 board package** by Espressif Systems through Boards Manager.
2. **U8g2** through Library Manager. 
3. **Adafruit MPRLS Library** through Library Manager and any required dependencies. 

Copy the code from `src/main.cpp` into Arduino and select the appropriate ESP32-S3 board and COM port. 
Set the Serial Monitor baud rate to **115200**. 


