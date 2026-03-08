# UART Custom Binary Communication Protocol

This document details the custom binary communication protocol used over USART3 for the FUE micromotor controller. It is specifically designed to facilitate robust, real-time command streaming and data telemetry between the device and a Graphical User Interface (GUI) or Host PC.

## 1. Packet Structure

All commands sent to the device must adhere exactly to the following byte structure to be accepted by the parsing engine. The protocol uses a rigid header format followed by a variable-length payload and a 16-bit CRC checksum.

| Byte Offset | Field Name       | Size (Bytes) | Description |
| :---        | :---             | :---         | :---        |
| 0           | Header 1         | 1            | Fixed start byte: `0xAA` |
| 1           | Header 2         | 1            | Fixed start byte: `0x55` |
| 2           | Payload Length   | 1            | Number of bytes in the **Payload** section only (0 to 250). |
| 3           | Command Byte     | 1            | The specific action to perform (e.g., `0x10` for Set RPM). |
| 4 to N-2    | Payload          | Variable     | The raw data bytes required by the Command Byte. If `Payload Length` is 0, this section is omitted. **Data is Little-Endian.** |
| N-2         | CRC16 LSB        | 1            | Least Significant Byte of the calculated Modbus CRC16. |
| N-1         | CRC16 MSB        | 1            | Most Significant Byte of the calculated Modbus CRC16. |

*Note: The CRC16 calculation includes the `Payload Length`, `Command Byte`, and the `Payload` data. It uses the Modbus polynomial `0xA001` with a `0xFFFF` initial value. The `Header 1` and `Header 2` are excluded from the CRC.*

## 2. Asynchronous State Machine Logic (Byte-by-Byte Reception)

To guarantee hard real-time performance and prevent the STM32's `HAL_BUSY` hardware lockup during heavy communication loads, the UART reception uses a strictly non-blocking, interrupt-driven state machine (`HAL_UART_RxCpltCallback`).

Instead of instructing the DMA or Interrupts to wait for a full packet array (which causes blocking if bytes are dropped), the system receives data **one byte at a time**.

The state machine operates as follows:
- **State 0 (Wait for Header 1):** The ISR waits for the byte `0xAA`. If received, transitions to State 1.
- **State 1 (Wait for Header 2):** The ISR waits for the byte `0x55`. If received, transitions to State 2. If any other byte is received, drops back to State 0.
- **State 2 (Read Payload Length):** Captures the third byte as the payload length. It immediately calculates the total expected packet size (`length + 6`). If the size exceeds the buffer, it drops the packet. Otherwise, transitions to State 3.
- **State 3 (Read Payload & CRC):** Continues capturing incoming bytes into the buffer until the counter matches the expected total length. Once reached, it triggers the `rx_data_ready` flag for the Main Loop to process the packet and immediately resets to State 0.

*Self-Healing Mechanism:* The ISR includes a `rx_needs_restart` flag. If the UART hardware gets temporarily locked while transmitting telemetry data simultaneously (`HAL_BUSY`), the interrupt flags it. The Main Loop periodically checks this flag and forcibly restarts the single-byte listening process, preventing a "deaf" device scenario.

## 3. Command Reference

The device processes the following commands. All multi-byte payload types (like `float`) must be formatted in **Little-Endian**.

| Command Name       | Hex Code | Payload Length | Payload Data Types                | Expected Device ACK Response String | Description |
| :---               | :---:    | :---:          | :---                              | :---                                | :---        |
| **Ping**           | `0x01`   | 0              | *None*                            | `<DBG: HB_OK>\n`                    | Heartbeat to reset the Watchdog timer. Does not affect motor state. |
| **Set RPM**        | `0x10`   | 4              | 1x `float` (Target RPM)           | `<DBG: CMD_RPM_OK>\n`               | Sets the target speed for continuous velocity control mode. |
| **Stop**           | `0x20`   | 0              | *None*                            | `<DBG: CMD_STOP_OK>\n`              | Immediately stops the motor and resets PID and Trajectory integrators. |
| **Set PID**        | `0x30`   | 8              | 2x `float` (Kp, Ki)               | `<DBG: CMD_PID_OK>\n`               | Dynamically updates the Velocity PI controller gains. |
| **Osc. Angle**     | `0x40`   | 12             | 3x `float` (Angle, MaxRPM, Accel) | `<DBG: CMD_OSC_OK>\n`               | Triggers a position-based oscillation movement. |
| **Osc. Time**      | `0x50`   | 12             | 3x `float` (TimeMs, MaxRPM, Accel)| `<DBG: CMD_OSC_TIME_OK>\n`          | Triggers a time-based (foolproof) oscillation movement. |
| **Get Params**     | `0x60`   | 0              | *None*                            | `<PRM,Kp,Ki,Time,Angle,MaxRPM,Accel>\n`| Requests the current active parameters. Response is formatted as a comma-separated text string. |
| **Save Params**    | `0x70`   | 0              | *None*                            | `<DBG: PARAMS_SAVED_FLASH>\n` <br>*(or `<DBG: ERR_CANT_SAVE_WHILE_RUNNING>\n`)* | Commits the current parameters to non-volatile Flash memory. Will fail if the motor is currently running. |