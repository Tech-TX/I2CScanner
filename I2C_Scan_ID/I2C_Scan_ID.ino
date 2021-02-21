// --------------------------------------
// Scan and Identify I2C devices by farmerkeith
// modified 21 February 2021
// Based on i2c_scanner Version 6
//
// This sketch tests all standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
// Recognized devices are listed as their device type
// greatly enhanced using https://learn.adafruit.com/i2c-addresses/the-list
// although most of the devices from Adafruit's list are untested / unverified

#include <Wire.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor

  uint8_t rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  } else { // bus clear
    // now we can start Arduino Wire as a master
  }
  Serial.println();  // clear the boot message
  Serial.print(F("\nI2C Scan_ID"));
  Serial.println(F("\nScans for I2C devices and names any that it knows about"));
  Wire.begin();
  Serial.println(F("setup finished\n"));

  //#ifdef ESP8266
  //Wire.setClockStretchLimit(1500);  // only needed if using pre-2.6.3 libraries
  //#endif
} // end of setup()


void loop() {
  byte error, address;
  uint8_t nDevices;
  byte numBytes = 1;  //gets rid of the 'ambiguous' warning

  Serial.println(F("Scanning..."));

  nDevices = 0;
  for (address = 8; address < 120; address++ )  //don't scan reserved addresses <8 or >119
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delayMicroseconds(10);
    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.print(F(" !"));
      nDevices++; // increment device count
    } else if (error == 4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.println(address, HEX);
    }
    if (error == 0) {
      byte data1 = 0;
      switch (address) {
        case 0x0E:
          Serial.println(F(" MAG3110 3-Axis Magnetometer"));
          break;
        case 0x10:
          Serial.println(F(" VEML6075 UV sensor or VEML7700 Ambient Light sensor"));
          break;
        case 0x11:
          Serial.println(F(" Si4713 FM Transmitter with RDS"));
          break;
        case 0x13:
          Serial.println(F(" VCNL40x0 proximity sensor"));
          break;
        case 0x18:
          Serial.println(F(" MCP9808 temp sensor"));
          Serial.println(F(" or LIS3DH 3-axis accelerometer"));
          break;
        case 0x19:
          Serial.println(F(" MCP9808 temp sensor"));
          Serial.println(F(" or LIS3DH 3-axis accelerometer"));
          Serial.println(F(" or LSM303 Accelerometer"));
          break;
        case 0x1A: case 0x1B:
          Serial.println(F(" MCP9808 temp sensor"));
          break;
        case 0x1C:
          Serial.println(F(" LIS3MDL Magetometer"));
          Serial.println(F(" or MCP9808 temp sensor"));
          Serial.println(F(" or MMA845x 3-axis Accelerometer"));
          Serial.println(F(" or FXOS8700 Accelerometer/Magnetometer"));
          Serial.println(F(" or MMA7455L Accelerometer"));
          break;
        case 0x1D:
          Serial.println(F(" ADXL345 Accelerometer"));
          Serial.println(F(" or MCP9808 temp sensor"));
          Serial.println(F(" or MMA845x 3-axis Accelerometer"));
          Serial.println(F(" or FXOS8700 Accelerometer/Magnetometer"));
          Serial.println(F(" or LSM9DS0 9-axis IMU"));
          Serial.println(F(" or MMA7455L Accelerometer"));
          break;
        case 0x1E:
          Serial.println(F(" HMC5883L 3-axis digital compass"));
          Serial.println(F(" or FXOS8700 Accelerometer/Magnetometer"));
          Serial.println(F(" or LIS2MDL Magnetometer"));
          Serial.println(F(" or LIS3MDL Magnetometer"));
          Serial.println(F(" or LSM303 Magnetometer"));
          Serial.println(F(" or LSM9DS0 9-axis IMU"));
          Serial.println(F(" or MCP9808 temp sensor"));
          break;
        case 0x1F:
          Serial.println(F(" FXOS8700 Accelerometer/Magnetometer"));
          Serial.println(F(" or MCP9808 temp sensor"));
          break;
        case 0x20:
          Serial.println(F(" PCF8574, MCP23008, MCP23017, PCAL6408A or PCAL6416A I/O expander"));
          Serial.println(F(" or FXAS21002 gyro"));
          Serial.println(F(" or Chirp! Water sensor"));
          break;
        case 0x21:
          Serial.println(F(" PCF8574, MCP23008, MCP23017, PCAL6408A or PCAL6416A I/O expander"));
          Serial.println(F(" or FXAS21002 gyro"));
          break;
        case 0x22: case 0x23: case 0x24: case 0x25:
          Serial.println(F(" PCF8574, MCP23008 or MCP23017 I/O expander"));
          break;
        case 0x26:
          Serial.println(F(" PCF8574, MCP23008 or MCP23017 I/O expander"));
          Serial.println(F(" or MSA301 Triple Axis Accelerometer"));
          break;
        case 0x27:
          Serial.println(F(" PCF8574, MCP23008 or MCP23017 I/O expander"));
          Serial.println(F(" or LCD with I2C backpack"));
          break;
        case 0x28:
          Serial.println(F(" BNO055 9-DOF IMU"));
          Serial.println(F(" or CAP1188 8-channel Capacitive Touch"));
          Serial.println(F(" or DS1841 I2C Digital Logarithmic Potentiometer"));
          Serial.println(F(" or DS3502 I2C Digital 10K Potentiometer"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          Serial.println(F(" or TSL2591 light sensor"));
          break;
        case 0x29:
          Serial.println(F(" TSC3472 color sensor"));
          Serial.println(F(" or BNO055 9-DOF Absolute Orientation sensor"));
          Serial.println(F(" or DS1841 I2C Digital Logarithmic Potentiometer"));
          Serial.println(F(" or DS3502 I2C Digital 10K Potentiometer"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          Serial.println(F(" or TSL2561/TLS2591 light sensor"));
          Serial.println(F(" or VL53L0x/VL6180X ToF distance"));
          Serial.println(F(" or CAP1188 8-channel Capacitive Touch"));
          break;
        case 0x2A: case 0x2B:
          Serial.println(F(" CAP1188 8-channel Capacitive Touch"));
          Serial.println(F(" or DS1841 I2C Digital Logarithmic Potentiometer"));
          Serial.println(F(" or DS3502 I2C Digital 10K Potentiometer"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          break;
        case 0x2C: case 0x2D:
          Serial.println(F(" CAP1188 8-channel Capacitive Touch"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          break;
        case 0x2E:
          Serial.println(F(" PCT2075 Temperature Sensor"));
          break;
        case 0x33:
          Serial.println(F(" MLX90640 IR Thermal Camera"));
          break;
        case 0x38:
          Serial.println(F(" PCF8574A I/O expander"));
          Serial.println(F(" or VEML6070 UV Index"));
          Serial.println(F(" or FT6x06 Capacitive Touch Driver"));
          break;
        case 0x39:
          Serial.println(F(" PCF8574A I/O expander"));
          Serial.println(F(" or TSC3472 color sensor"));
          Serial.println(F(" or TSL2561 light sensor"));
          Serial.println(F(" or VEML6070 UV Index"));
          Serial.println(F(" or APDS-9960 IR/Color/Proximity Sensor"));
          break;
        case 0x3A: case 0x3B:
          Serial.println(F(" PCF8574A I/O expander"));
          break;
        case 0x3C: case 0x3D:
          Serial.println(F(" PCF8574A I/O expander"));
          Serial.println(F(" or OLED with SSD1305/SSD1306 controller"));
          break;
        case 0x3E:
          Serial.println(F(" PCF8574A I/O expander"));
          break;
        case 0x3F:
          Serial.println(F(" PCF8574A I/O expander"));
          Serial.println(F(" or LCD with I2C backpack"));
          break;
        case 0x40:
          Serial.println(F(" HTU21D Humidity/Temp Sensor"));
          Serial.println(F(" or Si7021 Humidity/Temp sensor"));
          Serial.println(F(" or HDC1008 Humidity/Temp sensor"));
          Serial.println(F(" or TMP006/TMP007 IR Temperature sensor"));
          Serial.println(F(" or PCA9685 16-channel PWM"));
          Serial.println(F(" or INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          break;
        case 0x41:
          Serial.println(F(" INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          Serial.println(F(" or HDC1008 Humidity/Temp sensor"));
          Serial.println(F(" or TMP006/TMP007 IR Temperature sensor"));
          Serial.println(F(" or STMPE610/STMPE811 Resistive Touch controller"));
          break;
        case 0x42: case 0x43:
          Serial.println(F(" INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          Serial.println(F(" or HDC1008 Humidity/Temp sensor"));
          Serial.println(F(" or TMP006/TMP007 IR Temperature sensor"));
          break;
        case 0x44:
          Serial.println(F(" INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          Serial.println(F(" or SHT3x-DIS temperature & humidity sensor"));
          Serial.println(F(" or TMP006/TMP007 IR Temperature sensor"));
          Serial.println(F(" or ISL29125 Color Sensor"));
          Serial.println(F(" or STMPE610/STMPE811 Resistive Touch controller"));
          break;
        case 0x45:
          Serial.println(F(" INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          Serial.println(F(" or SHT3x-DIS temperature & humidity sensor"));
          Serial.println(F(" or TMP006/TMP007 IR Temperature sensor"));
          break;
        case 0x46: case 0x47:
          Serial.println(F(" INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          Serial.println(F(" or TMP006/TMP007 IR Temperature sensor"));
          break;
        case 0x48:
          Serial.println(F(" ADS1x13, ADS1x14, ADS1x15 A/D converter"));
          Serial.println(F(" or INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          Serial.println(F(" or PN532 NFC/RFID controller"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          Serial.println(F(" or TMP102 Temperature sensor"));
          break;
        case 0x49: case 0x4A: case 0x4B:
          Serial.println(F(" ADS1x13, ADS1x14, ADS1x15 A/D converter"));
          Serial.println(F(" or INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          Serial.println(F(" or TSL2561 light sensor"));
          Serial.println(F(" or TMP102 Temperature sensor"));
          break;
        case 0x4C: case 0x4D: case 0x4E: case 0x4F:
          Serial.println(F(" INA219 High-Side DC Current/Voltage sensor"));
          Serial.println(F(" or INA260 Precision DC Current/Power Sensor"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          break;
        case 0x50: case 0x51: case 0x54: case 0x55: case 0x56:
          Serial.println(F(" AT24C32/64 EEPROM family"));
          Serial.println(F(" or MB85RC FRAM"));
          break;
        case 0x52:
          Serial.println(F(" AT24C32/64 EEPROM family"));
          Serial.println(F(" or MB85RC FRAM"));
          Serial.println(F(" or Nintendo Nunchuck controller"));
          break;
        case 0x53:
          Serial.println(F(" AT24C32/64 EEPROM family"));
          Serial.println(F(" or MB85RC FRAM"));
          Serial.println(F(" or ADXL345 digital accelerometer"));
          break;
        case 0x57:
          Serial.println(F(" AT24C32/64 EEPROM family"));
          Serial.println(F(" or MB85RC FRAM"));
          Serial.println(F(" or MAX3010x Pulse & Oximetry sensor"));
          break;
        case 0x58:
          Serial.println(F(" TPA2016 I2C-controlled Amplifier"));
          Serial.println(F(" or SGP30 Gas Sensor"));
          break;
        case 0x5A: case 0x5B:
          Wire.beginTransmission(address);
          // Select register
          Wire.write(0x20); // 0x20 hex address of ID register
          // Stop I2C Transmission
          Wire.endTransmission();
          delayMicroseconds(10);
          // Request 1 bytes of data
          Wire.requestFrom(address, numBytes);
          // Read 1 byte of data
          if (Wire.available() == 1)  {
            data1 = Wire.read();
          }
          if (data1 == 0x81) {
            Serial.print(F("Device ID="));
            Serial.print(data1, HEX);
            Serial.println(F(" AMS CCS811 eCO2 TVOC sensor"));
          }
          else
          {
            Serial.println(F(" MPR121 12-point capacitive touch sensor"));
            if (address == 0x5a) Serial.println(F(" or MLX9061x IR temperature sensor"));
          }
          break;
        case 0x5C:
          Serial.println(F(" AM2315 temperature & humidity sensor"));
          Serial.println(F(" or AM2320 Humidity/Temp sensor"));
          Serial.println(F(" or LPS25 Pressure Sensor"));
          Serial.println(F(" or LPS33HW Ported Pressure Sensor"));
          Serial.println(F(" or LPS35HW Pressure Sensor"));
          Serial.println(F(" or MPR121 12-point capacitive touch sensor"));
          break;
        case 0x5D:
          Serial.println(F(" LPS25 Pressure Sensor"));
          Serial.println(F(" or LPS33HW Ported Pressure Sensor"));
          Serial.println(F(" or LPS35HW Pressure Sensor"));
          Serial.println(F(" or MPR121 12-point capacitive touch sensor"));
          break;
        case 0x5E:
          Serial.println(F(" TLV493D triple-axis Magnetometer"));
          break;
        case 0x60:
          Serial.println(F(" MCP4728 Quad DAC"));
          Serial.println(F(" or MPL115A2 Barometric Pressure"));
          Serial.println(F(" or MPL3115A2 Barometric Pressure"));
          Serial.println(F(" or Si5351A Clock Generator"));
          Serial.println(F(" or Si1145 Light/IR Sensor"));
          Serial.println(F(" or MCP4725A0 12-bit DAC"));
          Serial.println(F(" or TEA5767 Radio receiver"));
          Serial.println(F(" or VCNL4040 Proximity and Ambient Light sensor"));
          break;
        case 0x61:
          Serial.println(F(" Si5351A Clock Generator"));
          Serial.println(F(" or MCP4725A0 12-bit DAC"));
          break;
        case 0x62:
          Serial.println(F(" MCP4725A1 12-bit DAC"));
          break;
        case 0x63:
          Serial.println(F(" Si4713 FM Transmitter with RDS"));
          Serial.println(F(" or MCP4725A1 12-bit DAC"));
          break;
        case 0x64: case 0x65:
          Serial.println(F(" MCP4725A2 12-bit DAC"));
          break;
        case 0x66: case 0x67:
          Serial.println(F(" MCP4725A3 12-bit DAC"));
          break;
        case 0x68:
          Serial.println(F(" DS1307 or DS3231 Real Time Clock"));
          Serial.println(F(" or PCF8523 RTC"));
          Serial.println(F(" or MPU9250 9-DoF IMU"));
          Serial.println(F(" or MPU-60X0 Accel+Gyro"));
          Serial.println(F(" or L3G4200D gyroscope"));
          Serial.println(F(" or ITG3200 gyroscope"));
          Serial.println(F(" or ICM-20649 Accel+Gyro"));
          Serial.println(F(" or AMG8833 IR Thermal Camera"));
          break;
        case 0x69:
          Serial.println(F(" MPU9250 9-DoF IMU"));
          Serial.println(F(" or MPU-60X0 Accel+Gyro"));
          Serial.println(F(" or L3G4200D gyroscope"));
          Serial.println(F(" or ITG3200 gyroscope"));
          Serial.println(F(" or ICM-20649 Accel+Gyro"));
          Serial.println(F(" or AMG8833 IR Thermal Camera"));
          break;
        case 0x6A: case 0x6B:
          Serial.println(F(" ICM330DHC 6-axis IMU"));
          Serial.println(F(" or L3GD20H gyroscope"));
          Serial.println(F(" or LSM6DS33 6-axis IMU"));
          Serial.println(F(" or LSM6DSOX 6-axis IMU"));
          Serial.println(F(" or LSM9DS0 9-axis IMU"));
          break;
        case 0x70: case 0x71: case 0x72: case 0x73:
          Serial.println(F(" HT16K33 LED Matrix Driver"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          Serial.println(F(" or TCA9548 1-to-8 I2C Multiplexer"));
          break;
        case 0x74: case 0x75:
          Serial.println(F(" HT16K33 LED Matrix Driver"));
          Serial.println(F(" or PCT2075 Temperature Sensor"));
          Serial.println(F(" or TCA9548 1-to-8 I2C Multiplexer"));
          Serial.println(F(" or IS31FL3731 144-LED CharliePlex driver"));
          break;
        case 0x76: case 0x77:
          Serial.println(F(" BMP180, BMP280, BME280 or BME680 or MS5607,MS5611,MS5637"));
          // note: address 0x77 may be BMP085,BMA180 and may not be MS5607 or MS5637 CHECK
          Wire.beginTransmission(address);
          // Select register
          Wire.write(0xD0); // 0xD0 hex address of Bosch chip_ID
          // Stop I2C Transmission
          Wire.endTransmission();
          delayMicroseconds(10);
          // Request 1 bytes of data
          Wire.requestFrom(address, numBytes);
          // Read 1 byte of data
          if (Wire.available() == 1)  {
            data1 = Wire.read();
          } // end of if Wire.available()
          Serial.print(F("Device ID="));
          Serial.print(data1, HEX);
          if (data1 == 0x58) Serial.println(F(" = BMP280"));
          else if (data1 == 0x60) Serial.println(F(" = BME280"));
          else if (data1 == 0x55) Serial.println(F(" = BMP180"));
          else if (data1 == 0x61) Serial.println(F(" = BME680"));
          else
          {
            Serial.println(F(" MS5607, MS5611 or MS5637 barometric pressure sensor"));
            Serial.println(F(" or HT16K33 LED Matrix Driver"));
            Serial.println(F(" or IS31FL3731 144-LED CharliePlex driver"));
            Serial.println(F(" or PCT2075 Temperature Sensor"));
            Serial.println(F(" or TCA9548 1-to-8 I2C Multiplexer"));
          }
          break;
        default:
          Serial.println(F("device not in list"));
          break;
      }
    }
  } // end of for (address = bottom; address < top; address++ )

  if (nDevices == 0) {
    Serial.println(F("No I2C devices found\n"));
  } else {
    Serial.println(F("scan complete\n"));
  }
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);  // toggle the LED so we know the scanner isn't hung
  delay(2000);           // wait 2 seconds for next scan
} // end of loop()

/**
   I2C_ClearBus
   (http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html)
   (c)2014 Forward Computing and Control Pty. Ltd.
   NSW Australia, www.forward.com.au
   This code may be freely used for both private and commerical use

  This routine turns off the I2C bus and clears it
  on return SCA and SCL pins are tri - state inputs.
              You need to call Wire.begin() after this to re - enable I2C
              This routine does NOT use the Wire library at all.

              returns 0 if bus cleared
                      1 if SCL held low.
                      2 if SDA held low by slave clock stretch for > 2sec
                      3 if SDA held low after 20 clocks.
*/
uint8_t I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);
  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 & similar slow parts to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to finish uploading the program before existing sketch
  // confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master.
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  uint8_t clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    uint8_t counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}
