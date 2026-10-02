#define NANO_RX 33

HardwareSerial NanoSerial(1);

void setup()
{
    // USB Serial Monitor
    Serial.begin(115200);

    // UART coming from Arduino Nano
    NanoSerial.begin(115200, SERIAL_8N1, NANO_RX, -1);

    Serial.println("MCL receiver test started");
}

void loop()
{
    if (NanoSerial.available())
    {
        String received = NanoSerial.readStringUntil('\n');

        received.trim();

        Serial.print("Received from Nano: ");
        Serial.println(received);
    }
}