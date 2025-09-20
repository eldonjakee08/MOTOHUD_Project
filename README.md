# MotoHUD Project
A proof of concept project that aims to integrate a heads-up display (HUD) into a motorcycle helmet, projecting real-time ECU sensor data such as speed, RPM, and engine temperature directly into the rider’s field of view during rides.

This project aims to enhance riders situational awareness and safety by delivering vehicle metrics without requiring the rider to look away from the road.

# System Architecture & Overview
<img width="1148" height="430" alt="image" src="https://github.com/user-attachments/assets/d7842168-d4aa-4c92-aa51-b5765b9115cc" />

<br>**TRANSMITTER**
1. STM32WB5MMG MCU wakes up every 100ms to poll the motorcycle CAN bus for ECU sensor data.(e.g. Speed, RPM, Engine Load, etc.) 
2. Transmitter sends the ECU sensor data via BLE.
3. MCU goes into deep sleep state to conserve power. 

<br>**RECEIVER**
1. STM32WB5MMG MCU wakes up from deep sleep upon reception of sensor data from transmitter
2. MCU Parses the received data and displays into OLED screen.
3. Beamsplitter glass reflects the projection from OLED screen into the riders field of view
4. MCU returns into deep sleep state to conserve power.

# Project Milestones & Progress
1. Designed and fabricated a custom development board for STM32WB5MMG MCU.
2. Designed and fabricated custom module for LM3671 to power the receiver.
3. Designed the initial hardware for both Receiver and Transmitter.
4. Developed custom driver for SSD1315 OLED driver.
5. On-going development of custom driver for MCP2515 CAN controller

# Pending Tasks
1. Develop application firmware for both receiver and transmitter.
2. Fabricate the hardware for both receiver and transmitter.
3. Design and fabricate a receiver enclosure for optimal mounting into the helmet.
4. Design and fabricate an enclosure for the transmitter.

 # Future Improvements & Optimizations
 1. Optimize I2C and SPI data transmission using Direct Memory Access (DMA) to achieve higher throughput and lower power consumption.
 2. Design a load sharing protection circuit for the receiver’s battery to support simultaneous operation and charging.
 3. Music streaming feature via Bluetooth Low Energy (BLE).
 4. END VISION: Seamlessly stream live ECU sensor data into the helmets visor, delivering a real-time Augmented Reality (AR) experience.
