/*
 * UBH Control PCB -> Arduino Nano -> MCL PCB
 *
 * Purpose:
 *   1. Read one analog Hall sensor from the Control PCB
 *   2. Detect Hall waveform cycles
 *   3. Calculate mechanical RPM
 *   4. Send RPM as plain numeric UART data to the MCL PCB
 *
 * Wiring:
 *
 * CONTROL PCB                    ARDUINO NANO
 * ------------------------------------------------
 * Hall_Sensor1A (J100 pin 4) --> A0
 * GND           (J100 pin 5) --> GND
 *
 *
 * ARDUINO NANO                   MCL PCB
 * ------------------------------------------------
 * D1 / TX --> voltage divider --> ESP32 RX GPIO
 * GND -------------------------> GND
 *
 * IMPORTANT:
 * Nano TX is 5 V logic.
 * MCL ESP32 is 3.3 V logic.
 * Use a voltage divider / level shifter.
 */


// ============================================================
// USER SETTINGS
// ============================================================

const int HALL_PIN = A0;

// ADC thresholds.
//
// Nano ADC range = 0-1023.
//
// These are INITIAL values.
// They should eventually be adjusted after observing the
// actual minimum and maximum ADC readings from Hall_Sensor1A.
//
const int HALL_THRESHOLD_HIGH = 250;
const int HALL_THRESHOLD_LOW  = 175;


// Number of detected rising Hall cycles per mechanical revolution.
//
// THIS MUST BE SET ACCORDING TO THE MOTOR.
//
// For initial testing leave this at 1.
// Once the motor pole-pair count / Hall relationship is known,
// change this value.
//
const float EVENTS_PER_REVOLUTION = 2.0;


// UART baud rate to MCL PCB
const unsigned long BAUD_RATE = 115200;


// How frequently RPM is transmitted to MCL
const unsigned long TRANSMIT_INTERVAL_MS = 100;


// If no Hall edge occurs for this long, assume motor stopped
const unsigned long MOTOR_TIMEOUT_MS = 1000;


// Reject transitions that occur unrealistically quickly.
// Helps prevent electrical noise being counted as rotation.
//
// 500 us corresponds to a maximum event frequency of 2000 Hz.
//
const unsigned long MIN_EDGE_INTERVAL_US = 500;


// ============================================================
// VARIABLES
// ============================================================

bool hallHigh = false;

unsigned long previousEdgeTime = 0;
unsigned long lastValidEdgeTime = 0;

unsigned long lastTransmitTime = 0;

float rpm = 0.0;


// Optional smoothing
const float FILTER_ALPHA = 0.25;

float filteredRPM = 0.0;


// ============================================================
// SETUP
// ============================================================

void setup()
{
    pinMode(HALL_PIN, INPUT);

    // UART:
    //
    // Nano D1 / TX sends the RPM to the MCL PCB.
    //
    // Nano D0 / RX is unused in this implementation.
    Serial.begin(BAUD_RATE);

    // Give everything time to initialise
    delay(500);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // 1. Read Hall sensor
    // --------------------------------------------------------

    int hallADC = analogRead(HALL_PIN);

    // --------------------------------------------------------
    // 2. Detect rising threshold crossing
    // --------------------------------------------------------

    if (!hallHigh && hallADC >= HALL_THRESHOLD_HIGH)
    {
        hallHigh = true;

        unsigned long currentTime = micros();


        // Ignore the first edge because there is no previous
        // edge from which to calculate a period.
        if (previousEdgeTime != 0)
        {
            unsigned long period_us =
                currentTime - previousEdgeTime;


            // Reject unrealistically short periods caused
            // by noise.
            if (period_us >= MIN_EDGE_INTERVAL_US)
            {
                calculateRPM(period_us);

                previousEdgeTime = currentTime;
                lastValidEdgeTime = millis();
            }
        }
        else
        {
            // First detected edge
            previousEdgeTime = currentTime;
            lastValidEdgeTime = millis();
        }
    }

    // --------------------------------------------------------
    // 3. Re-arm Hall edge detector
    // --------------------------------------------------------
    //
    // Using separate HIGH and LOW thresholds gives hysteresis.
    //
    // For example:
    //
    // ADC
    //
    // 700       HIGH
    // 600 ------ rising trigger
    //
    //           dead-band
    //
    // 400 ------ reset trigger
    // 300       LOW
    //
    // This prevents noise around a single threshold from
    // producing multiple fake edges.
    //

    if (hallHigh && hallADC <= HALL_THRESHOLD_LOW)
    {
        hallHigh = false;
    }


    // --------------------------------------------------------
    // 4. Detect stopped motor
    // --------------------------------------------------------

    if (lastValidEdgeTime != 0)
    {
        if (millis() - lastValidEdgeTime > MOTOR_TIMEOUT_MS)
        {
            rpm = 0.0;
            filteredRPM = 0.0;

            // Reset measurement so startup begins cleanly
            previousEdgeTime = 0;
        }
    }


    // --------------------------------------------------------
    // 5. Send RPM to MCL PCB
    // --------------------------------------------------------

    if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL_MS)
    {
        lastTransmitTime = millis();

        sendRPM();
    }
}


// ============================================================
// RPM CALCULATION
// ============================================================

void calculateRPM(unsigned long period_us)
{
    if (period_us == 0)
        return;


    // Frequency of detected Hall events:
    //
    // f = 1 / T
    //
    // micros() gives us the period in microseconds, therefore:
    //
    // f = 1,000,000 / period_us

    float hallFrequency =
        1000000.0 / (float)period_us;


    // Convert event frequency to mechanical RPM:
    //
    // RPM = (frequency * 60) / events per revolution

    rpm =
        (hallFrequency * 60.0)
        / EVENTS_PER_REVOLUTION;


    // --------------------------------------------------------
    // Simple low-pass filter
    // --------------------------------------------------------
    //
    // Prevents the RPM value jumping around excessively.
    //

    if (filteredRPM == 0.0)
    {
        filteredRPM = rpm;
    }
    else
    {
        filteredRPM =
            FILTER_ALPHA * rpm
            + (1.0 - FILTER_ALPHA) * filteredRPM;
    }
}


// ============================================================
// SEND RPM
// ============================================================

void sendRPM()
{
    // Send ONLY the numerical RPM.
    //
    // Example:
    //
    // 1523.4
    // 1524.1
    // 1522.8
    //
    // This makes parsing on the MCL ESP32 simple.

    Serial.println(filteredRPM, 1);
}