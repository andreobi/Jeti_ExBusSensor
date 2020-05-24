# Jeti_ExBusSensor
This is a Jeti ExBus Sensor package which runs on a 328PB. 
To compile this code in an arduino environment, you need to install the 328PB support. 
Search the internet for "arduino install 328pb support" and follow the guidelines.

This sensor supports:
3x voltage, 1x current and 2x rpm measurement. 
GPS (location, longitude, latitude, ground speed ...) tested with ublox6 and 8 with 9600 baud
BME280 (pressure: altitude, vario, temperature)
BNO055 (orientation, roll, nick, magnatic compass))
Kontronik ESC: current, capacity, voltage, ...

Outputs:
3x flash (position) lights 32 bit pattern length and a period of 256ms upto 8s
2x independent blinking 1Hz / or constant lights
1x buzzer

Input:
1x Switch to ground, will send a message to the TX display

Further function details are in Beschreibung.txt
Hardware and GPS configuration details are described in JetiSensorArduino.pdf

It is not necessary to connect all sensor parts. 
In the JetiBox menu is a configuration for the connected hardware and a selection options for which signals should be send.

The Kontronik ESC is only supported via 16Mhz version, because of the 115200k.

I recommend to use an ISP programmer to download the program to get also information about reset reason (brownout or watchdaog) of the sensor.
