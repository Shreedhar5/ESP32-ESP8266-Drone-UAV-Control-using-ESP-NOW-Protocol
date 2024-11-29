
ESP-NOW Drone Project
Overview:
This project demonstrates a custom-built drone utilizing two ESP32 modules communicating via the ESP-NOW protocol. One ESP32 acts as the Controller for transmitting joystick commands, and the other serves as the Flight Controller (FC) on the drone, receiving these commands and controlling the motors in real-time with minimal latency.

Key Features:

ESP-NOW Protocol for lightweight, low-latency communication.
Two ESP32 Modules:
Controller ESP32 for sending joystick input.
Flight Controller ESP32 for motor control.
Wireless Control with fast response for seamless drone operation.
Hardware Requirements:

ESP32 (x2) – One for the controller and one for the flight controller.
Drone Frame – Pre-built or custom.
Brushless Motors & ESCs (Electronic Speed Controllers).
Joysticks for directional control.
Power Distribution Board (PDB).
LiPo Battery for power supply.
Software Requirements:

Arduino IDE – For uploading code to the ESP32 modules.
ESP32 Board Package – Install via the Arduino IDE.
ESP-NOW Library – Built into the ESP32 core package.
Motor Control Library – Optional, based on your ESC setup.
Setup Instructions:

Install ESP32 Support in Arduino IDE:

Go to File > Preferences.
Add the following URL under Additional Board Manager URLs:
https://dl.espressif.com/dl/package_esp32_index.json
Navigate to Tools > Board > Board Manager, search for ESP32, and install it.
Install Required Libraries:

Go to Sketch > Include Library > Manage Libraries and search for any necessary libraries.

Uploading the Code:

Controller Code: Upload to the ESP32 used for the controller.

Flight Controller Code: Upload to the ESP32 on the drone.

Wiring Overview:

Controller ESP32: Connected to the joysticks for input commands.
Flight Controller ESP32: Connected to ESCs and motors to control the drone's movement.


 #Required Libraries for ESP32/ESP8266 Project

- ESP32/ESP8266 Core: Install the ESP32/ESP8266 board manager in Arduino IDE (via `Tools > Board > Board Manager`).
  
- ESP-NOW: This is part of the ESP32/ESP8266 core libraries and should be available by default.
  
- Wire (for I2C communication): This is part of the default Arduino libraries.
  
- Servo: For controlling motors or servos via PWM, install the Servo library in Arduino IDE (via `Sketch > Include Library > Manage Libraries`).

Install the required libraries:
1. Arduino IDE: Go to `Sketch > Include Library > Manage Libraries`.
2. Search for these libraries and install:
   - Servo
   - ESP32 or ESP8266 (depending on which you're using)

Operation:

Power on both ESP32 modules (Controller and Flight Controller).
Establish a connection between the two ESP32 modules via ESP-NOW.
Use the joystick to control the drone's movements.
The Flight Controller ESP32 interprets the commands and adjusts the motors accordingly.

Future Enhancements:

Add real-time telemetry feedback (battery, altitude, and flight status).
Implement autonomous flight modes.
Integrate obstacle detection sensors for enhanced safety.