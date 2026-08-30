#include <Arduino.h> 
#include <Wire.h> 
#include <U8g2lib.h> 
#include <Adafruit_MPRLS.h> 

#define FLOW1_PIN 4 
#define FLOW2_PIN 5 

#define I2C_SDA 48 
#define I2C_SCL 47 

#define I2C_FREQ 400000 

#define MUX_ADDR 0x70
#define MPR1_CH 0
#define MPR2_CH 1
#define PRELOAD_LEFT_CH 4
#define PRELOAD_RIGHT_CH 5

const float PULSES_PER_LITRE = 1875.0; // From datasheet 

Adafruit_MPRLS mpr = Adafruit_MPRLS(); 
float mpr1Zero = 0; 
float mpr2Zero = 0; 

// interrupt counters 
volatile uint32_t flow1PulseCount = 0; 
volatile uint32_t flow2PulseCount = 0; 

unsigned long previousTime = 0; 

// sensor reading variables 
float flowRate1 = 0; 
float flowRate2 = 0;
float preloadLeft = 0; 
float preloadRight = 0; 
float MPR1 = 0; 
float MPR2 = 0; 

U8G2_SSD1309_128X64_NONAME0_F_HW_I2C oled(
    U8G2_R0,
    U8X8_PIN_NONE
);

void selectMux(uint8_t channel) {
    Wire.beginTransmission(MUX_ADDR); 
    Wire.write(1 << channel); 
    Wire.endTransmission(); 
}

float readPreload(uint8_t muxChannel) {
    const uint8_t PRELOAD_ADDR = 0x28; 

    const float PMIN = -1.0; 
    const float PMAX = 1.0; 

    const uint32_t OUTPUT_MIN = 1677722;
    const uint32_t OUTPUT_MAX = 15099494;

    uint8_t data[7]; 
    
    selectMux(muxChannel); 

    delay(2); 

    Wire.beginTransmission(PRELOAD_ADDR); 
    Wire.write(0xAA); 
    Wire.write(0x00); 
    Wire.write(0x00); 
    Wire.endTransmission(); 

    delay(10); 

    Wire.requestFrom(PRELOAD_ADDR, 7); 

    if (Wire.available() < 7) {return NAN;}

    // stores 7 bytes of pressure reading in data 
    for (int i = 0; i < 7; i++) {
        data[i] = Wire.read(); 
    }

    // combines three bytes into one pressure output (24 bits) 
    uint32_t pressureCounts = ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];

    // Convert counts -> psi
    float pressurePSI = ((float)pressureCounts - (float)OUTPUT_MIN) * (PMAX - PMIN) / ((float)OUTPUT_MAX - (float)OUTPUT_MIN) + PMIN;
    
    // Convert psi -> mmHg 
    float pressureMMHG = pressurePSI * 51.715; 

    return pressureMMHG; 
}

float readMPR(uint8_t muxChannel) {
    selectMux(muxChannel); 

    delay(2); 

    float pressureHPA = mpr.readPressure(); 

    if (isnan(pressureHPA)) {return NAN;}

    float pressureMMHG = pressureHPA * 0.750062; 

    return pressureMMHG;  
}

void IRAM_ATTR flow1ISR() {
    flow1PulseCount++; 
}

void IRAM_ATTR flow2ISR() {
    flow2PulseCount++; 
}

void printSensorData() {
    Serial.print("Flow 1: ");
    Serial.print(flowRate1, 2);
    Serial.print(" L/min");

    Serial.print(" | Flow 2: ");
    Serial.print(flowRate2, 2);
    Serial.print(" L/min");

    Serial.print(" | Left Preload: ");
    Serial.print(preloadLeft, 2);
    Serial.print(" mmHg");

    Serial.print(" | Right Preload: ");
    Serial.print(preloadRight, 2);
    Serial.print(" mmHg");

    Serial.print(" | MPR 1: ");
    Serial.print(MPR1, 2);
    Serial.print(" mmHg");

    Serial.print(" | MPR 2: ");
    Serial.print(MPR2, 2);
    Serial.println(" mmHg"); 
}

void updateOLED() {
    oled.clearBuffer(); 
    oled.setFont(u8g2_font_5x7_tr);

    char line1[30];
    char line2[30];
    char line3[30];
    char line4[30];
    char line5[30];
    char line6[30];

    // Flow sensors 
    snprintf(line1, sizeof(line1),
             "Flow 1: %.2f L/min", flowRate1);

    snprintf(line2, sizeof(line2),
             "Flow 2: %.2f L/min", flowRate2);

    // Preload sensors
    snprintf(line3, sizeof(line3),
             "Pre L : %.2f mmHg", preloadLeft);

    snprintf(line4, sizeof(line4),
             "Pre R : %.2f mmHg", preloadRight);

    // MPR sensors 
    snprintf(line5, sizeof(line5),
             "Pres 1: %.2f mmHg", MPR1);

    snprintf(line6, sizeof(line6),
             "Pres 2: %.2f mmHg", MPR2);

    // Display
    oled.drawStr(0, 8,  line1);
    oled.drawStr(0, 18, line2);
    oled.drawStr(0, 28, line3);
    oled.drawStr(0, 38, line4);
    oled.drawStr(0, 48, line5);
    oled.drawStr(0, 58, line6);

    oled.sendBuffer();
}

void setup() {
    Serial.begin(115200); 
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ); 

    oled.begin();  
    oled.clearBuffer(); 
    oled.setFont(u8g2_font_ncenB08_tr); 
    oled.drawStr(0, 15, "MCL Data Acquisition"); 
    oled.drawStr(0, 35, "Starting..."); 
    oled.sendBuffer(); 

    pinMode(FLOW1_PIN, INPUT); 
    pinMode(FLOW2_PIN, INPUT); 

    attachInterrupt(digitalPinToInterrupt(FLOW1_PIN), flow1ISR, RISING); 
    attachInterrupt(digitalPinToInterrupt(FLOW2_PIN), flow2ISR, RISING); 

    selectMux(MPR1_CH); 
    mpr.begin(); 
    selectMux(MPR2_CH); 
    mpr.begin(); 

    mpr1Zero = readMPR(MPR1_CH); 
    mpr2Zero = readMPR(MPR2_CH); 

    previousTime = millis(); 
}

void loop() {
    unsigned long currentTime = millis(); 

    if (currentTime - previousTime >= 1000) {
        float deltaTime = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime; 

        // copies pulse counts safely 
        noInterrupts(); 
        uint32_t pulses1 = flow1PulseCount; 
        uint32_t pulses2 = flow2PulseCount; 
        flow1PulseCount = 0; 
        flow2PulseCount = 0; 
        interrupts(); 

        flowRate1 = ((pulses1 / PULSES_PER_LITRE) / deltaTime) * 60.0; 
        flowRate2 = ((pulses2 / PULSES_PER_LITRE) / deltaTime) * 60.0; 
        
        preloadLeft = readPreload(PRELOAD_LEFT_CH); 
        preloadRight = readPreload(PRELOAD_RIGHT_CH); 

        MPR1 = readMPR(MPR1_CH) - mpr1Zero; 
        MPR2 = readMPR(MPR2_CH) - mpr2Zero; 

        printSensorData(); 

        updateOLED(); 
    }
}
