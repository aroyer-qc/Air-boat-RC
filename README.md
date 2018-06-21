# Air Boat RC
Arduino Air boat RC controller [![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.me/aroyerqc)

## Features
* PPM RC input        
* All standard navigation light : red, green and white
* Arming switch 
* RED rotating beacon (for fun) with controllable speed
	* This circuit has 8 LEDs, the LEDs are arranged in a circle (Opposite side ON at the same time)
	* The LEDs are connected to PWM outputs and their brightness is continuously adjusted on a sine wave pattern
	* This gives the impression of LED brightness rotating around the circle
	* Rotation speed is controlled by a PPM channel
* Motor reversal with protection
	* Relay for thrust reversal on 1 PPM channel
	* Motor is allowed to stop before switching the motor direction
	* 100% speed forward
	* 20% speed reverse
* Green and Red LED
* Flashing White for stern
* Steady white for bow

## Latest Versions
* 1.0.0
  * Initial release
 
## Arduino library
* PPM Reader 
