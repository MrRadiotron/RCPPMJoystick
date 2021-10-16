# RCPPMJoystick
An Arduino Leonardo/Pro Micro version of a RC PPM Trainer port to USB Joystick Converter - using Arduino Joystick Library.

Stick limits are auto detected, cycle all sticks / sliders full range once to calibrate. 
Tested with Hitec Eclipse 7, Mac, PC, various simulators and games.

## Credits
- This project is based on [Leonardo-USB-RC-Adapter](https://github.com/voroshkov/Leonardo-USB-RC-Adapter), but removes the requirement to change core arduino libraries.
- This Fork is based on https://github.com/timonorawski/RCPPMJoystick

## Requirements:
- Single Joystick Library from https://github.com/MHeironimus/ArduinoJoystickLibrary
- an Arduino Leonardo or Arduino Pro Micro
- mono audio cable or mono audio jack and two strands of hookup wire.
- RC TX with PPM Trainer Port (Tested with Turnigy 9x)

## Circuit Diagram

Pretty simple ... 
![Circuit Diagram](https://raw.githubusercontent.com/timonorawski/RCPPMJoystick/master/CircuitDiagram.png)
