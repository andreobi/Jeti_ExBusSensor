# Jeti_ExBusSensor
This is a Jeti ExBus Sensor package which runs on a 328PB. 

This sensor supports:
3x voltage, 1x current and 2x rpm measurement. 
GPS (location, ground speed ...)
BME280 (pressure: altitude, vario, temperature)
BNO055 (orientation, roll, nick, magnatic compass))
Kontronik ESC: current, capacity, voltage, ...

Outputs for:
3x flash (position) lights 32 bit pattern length and a period of 256ms upto 8s
2x independent blinking 1Hz / or constant lights
1x buzzer

Input:
Switch to ground, will send a message to the TX display

It is not necessary to connect all sensor parts. 
In the JetiBox menu is a configuration for the connected hardware and a selection options for which signals should be send.

The Kontronik ESC is only supported via 16Mhz version, because of the 115200kB.
