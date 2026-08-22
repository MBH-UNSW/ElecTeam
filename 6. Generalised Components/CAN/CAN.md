# CAN Receive Process

1. CAN messages are transmitted across the CAN bus using the **CANH** and **CANL** differential lines.

2. The CAN transceiver converts the differential CAN bus signal into a logic-level signal on its **RXD** output.

3. The transceiver **RXD** line is connected to the assigned **CAN_RX** pin on the STM32.

4. The STM32 **CAN/FDCAN peripheral** receives the [CAN base frame](###Complete-CAN-Base-Frame-Format) and checks the message identifier, DLC, and frame validity.

    - **Identifier:** The base CAN frame uses an 11-bit identifier, allowing (2^11 = 2048) possible CAN IDs. The identifier is used for message identification, arbitration, and hardware filtering.

    - **DLC (Data Length Code):** Specifies the number of data bytes contained in the frame. In Classical CAN, this ranges from 0-8 bytes and is represented using 4 bits.

        For example:

        - `0101` = 5 data bytes = (5 x 8 = 40) bits of payload data.

        - `1000` = 8 data bytes = (8 x 8 = 64) bits of payload data. **(MAXIMUM POSSIBLE CONFIGURATION)**

        - `1101` = 13 data bytes = (13 x 8 = 104) bits of payload data. **THIS IS INVALID (8 is the maximum number of bytes)**

    - **CRC (Cyclic Redundancy Check):** The STM32 CAN peripheral calculates the CRC of the received frame and compares it against the CRC transmitted within the frame. A mismatch indicates that the frame was corrupted during transmission.

        - Using CRC-15 (generator polynomial of x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1) or 0xC599.

        - A higher-degrees CRC polynomial provides stronger error detection at the cost of additional transmitted CRC bits.

        - A generic CRC remainder generator and checker can be found at [CRC Remainder Generator and Checker](/6.%20Generalised%20Components/CAN/Cyclic%20Redundancy%20Check/crc.c)

    - **Frame Format:** The CAN peripheral checks that fixed-format fields, such as the CRC delimiter, ACK delimiter, and EOF bits, contain the expected dominant or recessive values.

    - **ACK:** Receiving nodes acknowledge a correctly received frame by asserting a dominant bit in the ACK slot. A transmitter can detect an ACK error if no receiver acknowledges the frame.

    - **Error detection:** If a CRC, form, bit, stuffing, or ACK error is detected, the CAN controller handles the error according to the CAN protocol rather than passing the corrupted frame to the application.

5. Accepted CAN frames are placed into the STM32's **CAN receive FIFO**.

6. The CAN driver reads the frame from the FIFO and passes the **CAN ID + payload** to the internal CAN message handler.

7. The message handler identifies the type of data based on the **CAN ID** and decodes the payload into meaningful values, such as:
   - `motorSpeed`
   - `pressure`
   - `temperature`
   - `flowRate`

8. These decoded values are stored in a **central internal data structure** that other parts of the STM32 software can access.

9. Control, monitoring, telemetry, and safety functions then read the required values from this internal data structure.


## Complete CAN Base Frame Format
![Complete CAN Base Frame Format](https://upload.wikimedia.org/wikipedia/commons/5/54/CAN-bus-frame-with-stuff-bit-and-correct-CRC.png?utm_source=en.wikipedia.org&utm_campaign=imageinfo&utm_content=thumbnail_unscaled)

![Complete CAN Base Frame Format](/6.%20Generalised%20Components/imagesCAN/BaseFrameFormat.png)