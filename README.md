Allows you to connect a standard RC receiver to your Arduino, which will then 
present the data as a USB joystick.  This is handy for RC car and plane 
simulators so that you can use your own transmitter for the sim.

This is a combination of two different projects: The RCArduino blog posts and 
UnoJoy.  You can find them at:

http://rcarduino.blogspot.co.uk/
http://code.google.com/p/unojoy/

Getting the USB part of this to work is a little more complicated than your 
standard Arduino setup. You should follow the UnoJoy installation guide here:

http://code.google.com/p/unojoy/wiki/GettingStarted

For the first step, simply compile and upload rc_to_usb instead of the UnoJoy
sample program.
