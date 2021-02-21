# I2CScanner
Simple software to scan an I2C bus and report devices found.
Run this code on the Arduino IDE with a microcontroller connected to any I2C devices. 
The software checks every I2C address from 8 to 119 (0x08-0x77) and reports any devices found. 
The I2C addresses used by a variety of devices are recognised. Where the same address is used by several different devices, the possible devices are listed. 
If it finds a Bosch sensor it reports which device was seen by reading it's identification code.

Kindly note: farmerkeith's original scanned from address 1 to 127, and the scanner on arduino.cc adds address 0, the General Call address.  Any device may respond to address 0; 0-7 plus 120-127 are all reserved for special functions, so this code doesn't attempt to read them.

If you're using non-standard I2C pins, change the Wire.begin(); to match your setup.