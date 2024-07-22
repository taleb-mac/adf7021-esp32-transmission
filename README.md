# ADF7021 Transceiver Setup with ESP32

This repository provides a short script for setting up the ADF7021 transceiver to transmit data using the ESP32. The ESP32 is programmed using Arduino C, simplifying development and making it easy to configure and control the transceiver.

## eval adf7021dbx Pinout

| Pin Number | Pin Name   | Description                                      |
|------------|------------|--------------------------------------------------|
| 2          | GND        | Ground connection                                |
| 3          | VDD        | Power supply input (3.3V)                        |
| 5          | CLK OUT    | Clock output used to send data to transmit       |
| 6          | TX/RX CLK  | Used to send data through, not a clock in this configuration |
| 9          | SCLK       | Input clock generated by ESP32                   |
| 10         | SREAD      | Serial data read for reading registers           |
| 11         | SDATA      | Serial data write for writing to registers       |
| 12         | SLE        | Load data into registers from latch              |
| 14         | CE         | Chip enable                                      |

Note: These pins are on the back of the eval adf7021dbx, arranged in two columns. The left column corresponds to even indices, and the right column corresponds to odd indices (when viewed from the back).

## Configuration Settings

The ADF7021 transceiver is configured by modifying 15 specific registers via the SPI protocol. Here are the settings used:

- Center Frequency: 437 MHz
- Frequency Deviation: 2.4 kHz
- Modulation: Gaussian 2FSK
