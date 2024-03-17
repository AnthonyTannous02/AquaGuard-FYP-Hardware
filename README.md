# Hardware Code AquaGuard
## Idea of the Project
- A Drowning Detection and Prevention System comprised of:
   - A Wristband (this repository)
   - Machine Learning Models
   - Backend/Database
   - Frontend/Client UI

This project is our Computer Engineering Final Year Project. I, Anthony Tannous, worked on the full hardware,
from C++ coding to Wiring and Soldering.

## Components Used
1. DOIT ESP32 Devkit V1
2. MPU6050 Accelerometer/Gyroscope
3. MAX30102 Pulse Oximeter
4. GT-U7 GPS (u-blox 6)

## How it works
The esp32 is reponsible of reading the values of all sensors (including the GPS), then it will send them in a
controlled manner through the internet (to REST APIs). The system employs Multi-Core and Multi-Threading 
functionalities provided by the powerful native FreeRTOS Operating System designed specifically for microcontrollers.
