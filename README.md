# moonlite-focuser-clone
Electric USB or Bluetooth Focuser that is compatible to the Protocol of the popular Moonlite Focuser

# Moonlite-compatible stepper controller
The original code was written by George Carlson in June 2014, so he deserves full credit for this
This is a more feature rich and compatible version, yet some aspects are still missing
 
The version written by Dr. Lutz Bellmann uses the Arduino Uno Rev.3 Board

# Compatibility
The code has been troubleshot against the original and it should be almost compatible to the Moonlite single focuser
Exeptions:
Half Step mode not included
Range is from 0 to 65535 ticks
no backlight supported
this is ment to function remotely and uses serial communication either via bluetooth or USB; if you want to use a DIY handcontroller, you have to extend the code
tested with kstars-bleeding and indilib 

The center concept at 30000 has been skipped, so the full range from 0 to 65535 can be used, yet the default startposition from empty EEPROM is still 30000
Additional features:
Current position, stepping delay(speed), and temperature coefficient are saved to EEPROM and read back after power out
This enables the focuser to remember the last position in the subsequent imaging night.
Temperature acquisition is done by 1-wire T sensor Dallas DS18B20
Temperature can be calibrated by command (not saved to EEPROM, yet)
2-point calibration can be done with Temperature coefficient in (ticks/degreeC) and Compensation can be switched on/off
no delay in movement on temperature readout; conversion commands are executed only in idle condition

# Hardware
The Code has been tested on a Vellemann Arduino Uno Rev3 Board, yet other Atmega based Arduino-like Boards like the Nano or others should work as well
It makes sense that one builds a "shield" for contacting
- the driver board(2PH64011R) for the stepper motor (In1(blue) - Pin4, In2(pink) - Pin5, In3(yellow) - Pin6, In4(orange) - Pin7; +5V to VCC (5V), -Pin to GND
- stepper motor 28BYJ-48
- the Dallas 18B20 digital temperature sensor; use the one with cable and metal casing to stick onto primary mirror or tube; connect to Pin3 as well as VCC (3.3V or 5V) and GND of Arduino
- optional: bluetooth dongle (f.e. HC-06) to be connected to RX and TX as well as VCC(3.3V) and GND of the Arduino; you can't use USB data conenction with Bluetooth dongle connected as well; I made myself a pair of jumpers to be able to disconnect the HC-06
- Case: I will put a couple of STL files and FreeCAD files in the directory to be used as an "application" example; feel free to use or modify as needed

# Installation
The most comfortable way is, to plug your Arduino Board to your PC with USB and use Arduino IDE to "burn" the code to the Board.
Make sure that you have the Libraries
- OneWire
- DallasTemperature
- EEPROM
installed, since they are going to be used by the code.

On connection with USB the Board can be found as a serial device. Any Moonlite compatible software can connect to the board. The Baud Rate is 9600.

