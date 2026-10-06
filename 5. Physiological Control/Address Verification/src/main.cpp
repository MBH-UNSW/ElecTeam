#include <Arduino.h> 
#include <Wire.h> 
#include <U8g2lib.h>

#define I2C_SDA 48 
#define I2C_SCL 47 

#define I2C_FREQ 400000 

#define MUX_ADDR 0x70
#define OLED_ADDR 0x3C
#define MPR_ADDR 0x18 
#define PRELOAD_ADDR 0x28

U8G2_SSD1309_128X64_NONAME0_F_HW_I2C oled(
    U8G2_R0,
    U8X8_PIN_NONE
);

void selectMux(uint8_t channel) {
    Wire.beginTransmission(MUX_ADDR); 
    Wire.write(1 << channel); 
    Wire.endTransmission(); 
}

bool deviceFound(uint8_t address) {
    Wire.beginTransmission(address); 
    return Wire.endTransmission() == 0; 
}


void setup() {
    Serial.begin(115200); 
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ); 
    bool OLEDFOUND = deviceFound(OLED_ADDR); 
    bool MUXFOUND = deviceFound(MUX_ADDR); 
    selectMux(0); 
    bool MPR1FOUND = deviceFound(MPR_ADDR); 
    selectMux(1); 
    bool MPR2FOUND = deviceFound(MPR_ADDR); 
    selectMux(4); 
    bool PRELOAD1FOUND = deviceFound(PRELOAD_ADDR); 
    selectMux(5); 
    bool PRELOAD2FOUND = deviceFound(PRELOAD_ADDR); 
    
    Serial.println("=== MCL ADDRESS TEST ===");

    Serial.print("OLED:     ");
    Serial.println(OLEDFOUND ? "PASS" : "FAIL");

    Serial.print("MUX:      ");
    Serial.println(MUXFOUND ? "PASS" : "FAIL");

    Serial.print("MPR1:     ");
    Serial.println(MPR1FOUND ? "PASS" : "FAIL");

    Serial.print("MPR2:     ");
    Serial.println(MPR2FOUND ? "PASS" : "FAIL");

    Serial.print("PRELOAD1: ");
    Serial.println(PRELOAD1FOUND ? "PASS" : "FAIL");

    Serial.print("PRELOAD2: ");
    Serial.println(PRELOAD2FOUND ? "PASS" : "FAIL");

    if (OLEDFOUND) {
        oled.setI2CAddress(OLED_ADDR << 1);
        oled.begin();

        oled.clearBuffer();

        oled.setFont(u8g2_font_6x10_tf);

        oled.drawStr(0, 9, "MCL ADDRESS TEST");

        oled.setCursor(0, 19);
        oled.print("MUX       ");
        oled.print(MUXFOUND? "PASS" : "FAIL");

        oled.setCursor(0, 29);
        oled.print("MPR1      ");
        oled.print(MPR1FOUND ? "PASS" : "FAIL");

        oled.setCursor(0, 39);
        oled.print("MPR2      ");
        oled.print(MPR2FOUND ? "PASS" : "FAIL");

        oled.setCursor(0, 49);
        oled.print("PRELOAD1  ");
        oled.print(PRELOAD1FOUND ? "PASS" : "FAIL");

        oled.setCursor(0, 59);
        oled.print("PRELOAD2  ");
        oled.print(PRELOAD2FOUND ? "PASS" : "FAIL");

        oled.sendBuffer();
    }
}

void loop() {

}
