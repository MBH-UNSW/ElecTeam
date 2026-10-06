import requests
import csv
import time
from datetime import datetime
from pathlib import Path


# ============================================================
# CONFIGURATION
# ============================================================

# ESP32 address when connected to the "UBH-MCL" Wi-Fi network
ESP32_URL = "http://192.168.4.1/api/data"

SAMPLE_INTERVAL = 1.0  # seconds

DATA_FOLDER = Path("recorded_data")
DATA_FOLDER.mkdir(exist_ok=True)

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
CSV_FILE = DATA_FOLDER / f"MCL_data_{timestamp}.csv"

def get_value(sensor):
    """
    Return the sensor value if it is connected.
    Otherwise return an empty CSV cell.
    """

    if sensor.get("connected", False):
        return sensor.get("value")

    return ""

print("========================================")
print("UNSW Bionic Heart - MCL Data Log")
print("========================================")
print()
print(f"ESP32: {ESP32_URL}")
print(f"Saving to: {CSV_FILE}")
print()
print("Press Ctrl+C to stop recording.")
print()

try:

    with open(CSV_FILE, "w", newline="") as file:

        writer = csv.writer(file)

        # ----------------------------------------------------
        # CSV COLUMN HEADINGS
        # ----------------------------------------------------

        writer.writerow([
            "Computer Time",
            "Packet",
            "ESP32 Uptime (s)",
            "Left Flow (L/min)",
            "Right Flow (L/min)",
            "Left Preload (mmHg)",
            "Right Preload (mmHg)",
            "Pressure 1 (mmHg)",
            "Pressure 2 (mmHg)"
        ])

        # ----------------------------------------------------
        # DATA ACQUISITION LOOP
        # ----------------------------------------------------

        while True:

            try:

                # Request latest sensor data from ESP32
                response = requests.get(
                    ESP32_URL,
                    timeout=2
                )

                response.raise_for_status()

                # Convert JSON into Python dictionary
                data = response.json()


                # ------------------------------------------------
                # TIME INFORMATION
                # ------------------------------------------------

                computer_time = datetime.now().strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )[:-3]

                uptime_seconds = data["uptime"] / 1000.0


                # ------------------------------------------------
                # SENSOR VALUES
                # ------------------------------------------------

                flow1 = get_value(data["flow1"])
                flow2 = get_value(data["flow2"])

                preload_left = get_value(data["preloadLeft"])
                preload_right = get_value(data["preloadRight"])

                pressure1 = get_value(data["mpr1"])
                pressure2 = get_value(data["mpr2"])


                # ------------------------------------------------
                # WRITE TO CSV
                # ------------------------------------------------

                writer.writerow([
                    computer_time,
                    data["packet"],
                    uptime_seconds,
                    flow1,
                    flow2,
                    preload_left,
                    preload_right,
                    pressure1,
                    pressure2
                ])

                # Immediately save data to disk
                file.flush()


                # ------------------------------------------------
                # TERMINAL DISPLAY
                # ------------------------------------------------

                print(
                    f"Packet {data['packet']} | "
                    f"Time {uptime_seconds:.1f}s | "
                    f"Flow L: {flow1} | "
                    f"Flow R: {flow2} | "
                    f"Preload L: {preload_left} | "
                    f"Preload R: {preload_right}"
                )


            except requests.RequestException as error:

                print(f"ESP32 connection error: {error}")


            except (KeyError, ValueError) as error:

                print(f"Invalid data received: {error}")


            time.sleep(SAMPLE_INTERVAL)


# ============================================================
# STOP RECORDING
# ============================================================

except KeyboardInterrupt:

    print()
    print("========================================")
    print("Recording stopped.")
    print(f"Data saved to: {CSV_FILE}")
    print("========================================")