# Nunchuck Boat Remote
This repository contains the software of the remote and receiver of a RC boat. As well as some solidworks files of the motor mounts. On the boat there are two brushless DC motors. Both the remote and receiver run on an Atmega328P and communicate with HC-12 433MHz modules
## Remote
The remote part takes input from a nunchuck and sends motor power commands over 433mhz. The incoming data from the nunchuck is translated to differential thrust of the motors. A start bit and checksum is added. The remote is also able to send arm and disarm command sto enable or disable the motors.
## Receiver
The receiver part processes the incoming data, validates the checksum and when everyting is in orders sends a PWM signal to the motors. When the receiver has not gotten a valid packet for 1 second, the motors are disarmed as a safety feature.
