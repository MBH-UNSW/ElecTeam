# CAN Receive Process

1. CAN messages are transmitted across the CAN bus using the **CANH** and **CANL** differential lines.
2. The CAN transceiver converts the differential CAN bus signal into a logic-level signal on its **RXD** output.
3. The transceiver **RXD** line is connected to the assigned **CAN_RX** pin on the STM32.
4. The STM32 **CAN/FDCAN peripheral** receives the CAN frame (*CAN base frame* which supports 11 bits for the identifier, hence, 2^11 = 2048 unique IDs) and checks the message identifier, DLC, and frame validity.
5. Accepted CAN frames are placed into the STM32's **CAN receive FIFO**.
6. The CAN driver reads the frame from the FIFO and passes the **CAN ID + payload** to the internal CAN message handler.
7. The message handler identifies the type of data based on the **CAN ID** and decodes the payload into meaningful values, such as:
   - `motorSpeed`
   - `pressure`
   - `temperature`
   - `flowRate`
8. These decoded values are stored in a **central internal data structure** that other parts of the STM32 software can access.
9. Control, monitoring, telemetry, and safety functions then read the required values from this internal data structure.