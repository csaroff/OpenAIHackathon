#include <Arduino.h>
#include <SPI.h>

#include "AeroBridge.h"

// Base Node
const byte baseMode = 1;                  // normal mode = 1
const byte baseWriteMask = B00000111;     // Bxxxxx111 to enable the LEDs, Bxxx11xxx to enable writes to the Base Node encoders

// Core Node
const byte coreMode = 1;                  // normal mode = 1
const byte coreWriteMask = B00001111;     // Bxxxx1111 to enable the motors, Bxx11xxxx to enable writes to the Core Node encoders


void QUBEAero::reset() {
  // set the slaveSelectPin as an output
  pinMode (slaveSelectPin, OUTPUT);

  // initialize SPI
  SPI.begin();

  // set motors and encoders to zero
  this->motor0MSB = 0;
  this->motor0LSB = 0;
  this->motor1MSB = 0;
  this->motor1LSB = 0;
  this->encoder0ByteSet[2] = 0;
  this->encoder0ByteSet[1] = 0;
  this->encoder0ByteSet[0] = 0;
  this->encoder1ByteSet[2] = 0;
  this->encoder1ByteSet[1] = 0;
  this->encoder1ByteSet[0] = 0;

  // send to qube
  this->spi_transfer();

  this->prevUpdateMicros = micros();
}


void QUBEAero::update(float motor0Voltage, float motor1Voltage, int LEDRed, int LEDGreen, int LEDBlue, bool print) {
  // limit calling too often
  unsigned long currMicros = micros();
  if (currMicros - this->prevUpdateMicros < sampleTime) {
    return;
  }
  this->prevUpdateMicros = currMicros;

  // convert the LED intensities to MSB and LSB
  this->LEDRedMSB = (byte)(LEDRed >> 8);
  this->LEDRedLSB = (byte)(LEDRed & 0x00FF);
  this->LEDGreenMSB = (byte)(LEDGreen >> 8);
  this->LEDGreenLSB = (byte)(LEDGreen & 0x00FF);
  this->LEDBlueMSB = (byte)(LEDBlue >> 8);
  this->LEDBlueLSB = (byte)(LEDBlue & 0x00FF);


  // set motor voltages within limits 
  if (motor0Voltage > maxMotorVoltage) {
    motor0Voltage = maxMotorVoltage;
  }
  else if (motor0Voltage < -maxMotorVoltage) {
    motor0Voltage = -maxMotorVoltage;
  }
  if (motor1Voltage > maxMotorVoltage) {
    motor1Voltage = maxMotorVoltage;
  }
  else if (motor1Voltage < -maxMotorVoltage) {
    motor1Voltage = -maxMotorVoltage;
  }

  // convert the analog value to the PWM duty cycle that will produce the same average voltage
  float motor0PWM = motor0Voltage * (625.0 / 15.0);
  float motor1PWM = motor1Voltage * (625.0 / 15.0);

  int motor0 = (int)motor0PWM;  // convert float to int (2 bytes)
  motor0 = motor0 | 0x8000;  // motor0 command MSB must be B1xxxxxxx to enable amplifier0
  this->motor0MSB = (byte)(motor0 >> 8);
  this->motor0LSB = (byte)(motor0 & 0x00FF);
  
  int motor1 = (int)motor1PWM;  // convert float to int (2 bytes)
  motor1 = motor1 | 0x8000;  // motor1 command MSB must be B1xxxxxxx to enable amplifier1
  this->motor1MSB = (byte)(motor1 >> 8);     
  this->motor1LSB = (byte)(motor1 & 0x00FF);

  // exchange class values with micro
  this->spi_transfer();

  if (print) {
    this->debug();
  }
}


void QUBEAero::spi_transfer() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(slaveSelectPin, LOW);

  // send and receive the Base Node data via SPI
  this->baseModuleIDMSB = SPI.transfer(this->baseMode);            // read the module ID MSB, send the mode
  this->baseModuleIDLSB = SPI.transfer(0);                         // read the module ID LSB
  this->encoder2Byte[2] = SPI.transfer(this->baseWriteMask);       // read encoder2 byte 2, send the write mask
  this->encoder2Byte[1] = SPI.transfer(this->LEDRedMSB);           // read encoder2 byte 1, send the red LED MSB
  this->encoder2Byte[0] = SPI.transfer(this->LEDRedLSB);           // read encoder2 byte 0, send the red LED LSB
  this->encoder3Byte[2] = SPI.transfer(this->LEDGreenMSB);         // read encoder3 byte 2, send the green LED MSB
  this->encoder3Byte[1] = SPI.transfer(this->LEDGreenLSB);         // read encoder3 byte 1, send the green LED LSB
  this->encoder3Byte[0] = SPI.transfer(this->LEDBlueMSB);          // read encoder3 byte 0, send the blue LED MSB
  this->tach2Byte[2] = SPI.transfer(this->LEDBlueLSB);             // read tachometer2 byte 2, send the blue LED LSB
  this->tach2Byte[1] = SPI.transfer(this->encoder2ByteSet[2]);     // read tachometer2 byte 1, send encoder2 byte 2
  this->tach2Byte[0] = SPI.transfer(this->encoder2ByteSet[1]);     // read tachometer2 byte 0, send encoder2 byte 1
  this->tach3Byte[2] = SPI.transfer(this->encoder2ByteSet[0]);     // read tachometer3 byte 2, send encoder2 byte 0
  this->tach3Byte[1] = SPI.transfer(this->encoder3ByteSet[2]);     // read tachometer3 byte 1, send encoder3 byte 2
  this->tach3Byte[0] = SPI.transfer(this->encoder3ByteSet[1]);     // read tachometer3 byte 0, send encoder3 byte 1
  SPI.transfer(this->encoder3ByteSet[0]);                          // send encoder3 byte 0


  // send and receive the Core Node data via SPI
  this->coreModuleIDMSB = SPI.transfer(this->coreMode);            // read the module ID MSB, send the mode
  this->coreModuleIDLSB = SPI.transfer(0);                   // read the module ID LSB
  this->currentSense0MSB = SPI.transfer(this->coreWriteMask);      // read motor0 current sense MSB, send the write mask
  this->currentSense0LSB = SPI.transfer(this->motor0MSB);          // read motor0 current sense LSB, send motor0 command MSB
  this->currentSense1MSB = SPI.transfer(this->motor0LSB);          // read motor1 current sense MSB, send motor0 command LSB
  this->currentSense1LSB = SPI.transfer(this->motor1MSB);          // read motor1 current sense LSB, send motor1 command MSB
  this->tach0Byte[2] = SPI.transfer(this->motor1LSB);              // read tachometer0 byte 2, send motor1 command LSB
  // ==> commented out from here for code reduction
  this->tach0Byte[1] = SPI.transfer(this->encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2       
  this->tach0Byte[0] = SPI.transfer(this->encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
  this->tach1Byte[2] = SPI.transfer(this->encoder0ByteSet[0]);     // read tachometer1 byte 2, send encoder0 byte 0
  this->tach1Byte[1] = SPI.transfer(this->encoder1ByteSet[2]);     // read tachometer1 byte 1, send encoder1 byte 2
  this->tach1Byte[0] = SPI.transfer(this->encoder1ByteSet[1]);     // read tachometer1 byte 0, send encoder1 byte 1
  this->moduleStatus = SPI.transfer(this->encoder1ByteSet[0]);     // read the status, send encoder1 byte 0
  this->encoder0Byte[2] = SPI.transfer(0);                   // read encoder0 byte 2
  this->encoder0Byte[1] = SPI.transfer(0);                   // read encoder0 byte 1
  this->encoder0Byte[0] = SPI.transfer(0);                   // read encoder0 byte 0
  this->encoder1Byte[2] = SPI.transfer(0);                   // read encoder1 byte 2
  this->encoder1Byte[1] = SPI.transfer(0);                   // read encoder1 byte 1
  this->encoder1Byte[0] = SPI.transfer(0);                   // read encoder1 byte 0
  this->xAccelLSB = SPI.transfer(0);                         // read X-axis accelerometer LSB
  this->xAccelMSB = SPI.transfer(0);                         // read X-axis accelerometer MSB
  this->yAccelLSB = SPI.transfer(0);                         // read Y-axis accelerometer LSB
  this->yAccelMSB = SPI.transfer(0);                         // read Y-axis accelerometer MSB
  this->zAccelLSB = SPI.transfer(0);                         // read Z-axis accelerometer LSB
  this->zAccelMSB = SPI.transfer(0);                         // read Z-axis accelerometer MSB
  this->xGyroLSB = SPI.transfer(0);                          // read X-axis gyroscope LSB
  this->xGyroMSB = SPI.transfer(0);                          // read X-axis gyroscope MSB
  this->yGyroLSB = SPI.transfer(0);                          // read Y-axis gyroscope LSB
  this->yGyroMSB = SPI.transfer(0);                          // read Y-axis gyroscope MSB
  this->zGyroLSB = SPI.transfer(0);                          // read Z-axis gyroscope LSB
  this->zGyroMSB = SPI.transfer(0);                          // read Z-axis gyroscope MSB


  // calculate common values
  this->LEDRed   = (this->LEDRedMSB << 8)   | this->LEDRedLSB; 
  this->LEDGreen = (this->LEDGreenMSB << 8) | this->LEDGreenLSB; 
  this->LEDBlue  = (this->LEDBlueMSB << 8)  | this->LEDBlueLSB; 

  this->baseModuleID = (baseModuleIDMSB << 8) | baseModuleIDLSB;
  this->coreModuleID = (coreModuleIDMSB << 8) | coreModuleIDLSB;


  long encoder2 = ((long)encoder2Byte[2] << 16) | ((long)encoder2Byte[1] << 8) | encoder2Byte[0];  // pitch
  if (encoder2 & 0x00800000) {
    encoder2 = encoder2 | 0xFF000000;
  }
  long encoder3 = ((long)encoder3Byte[2] << 16) | ((long)encoder3Byte[1] << 8) | encoder3Byte[0];  // yaw
  if (encoder3 & 0x00800000) {
    encoder3 = encoder3 | 0xFF000000;
  }
  this->encoder2Deg = (float)encoder2 * (360.0 / 2048.0);  // pitch
  this->encoder3Deg = (float)encoder3 * (360.0 / 4096.0);  // yaw (encoder3 is higher resolution than the other encoders)

  this->currentSense0 = (currentSense0MSB << 8) | currentSense0LSB;
  this->currentSense1 = (currentSense1MSB << 8) | currentSense1LSB;

  this->motor0 = ((this->motor0MSB << 8) | this->motor0LSB) & 0x7FFF;
  this->motor1 = ((this->motor1MSB << 8) | this->motor1LSB) & 0x7FFF; 

  this->accelX = (this->xAccelMSB<<8) | this->xAccelLSB; 
  this->accelY = (this->yAccelMSB<<8) | this->yAccelLSB; 
  this->accelZ = (this->zAccelMSB<<8) | this->zAccelLSB;

  this->gyroX = (this->xGyroMSB<<8) | this->xGyroLSB;
  this->gyroY = (this->yGyroMSB<<8) | this->yGyroLSB;
  this->gyroZ = (this->zGyroMSB<<8) | this->zGyroLSB;

  // take the slave select pin high to de-select the device
  digitalWrite(slaveSelectPin, HIGH);
  SPI.endTransaction();
}

void QUBEAero::debug() {
  // reconstruct LED values
  Serial.write(" R:");
  Serial.print(this->LEDRed);
  Serial.write(" G:");
  Serial.print(this->LEDGreen);
  Serial.write(" B:");
  Serial.print(this->LEDBlue);

  // Module IDs
  Serial.write(" BaseID:");
  Serial.print(this->baseModuleID);
  Serial.write(" CoreID:");
  Serial.print(this->coreModuleID);

  // Encoder Values  
  Serial.write(" Pitch:");
  Serial.print(this->encoder2Deg);
  Serial.write(" Yaw:");
  Serial.print(this->encoder3Deg);

  //Current Sense Values
  Serial.write(" CS0:");
  Serial.print(this->currentSense0);
  Serial.write(" CS1:");
  Serial.print(this->currentSense1);

  // Motor Settings
  Serial.write(" M0:");
  Serial.print(this->motor0);
  Serial.write(" M1:");
  Serial.print(this->motor1);

  // Accel X Y Z
  Serial.write(" AX:");
  Serial.print(this->accelX); 
  Serial.write(" AY:");
  Serial.print(this->accelY); 
  Serial.write(" AZ:");
  Serial.print(this->accelZ); 

  // Gyro X Y Z 
  Serial.write(" GX:");
  Serial.print(this->gyroX); 
  Serial.write(" GY:");
  Serial.print(this->gyroY); 
  Serial.write(" GZ:");
  Serial.print(this->gyroZ); 
}
