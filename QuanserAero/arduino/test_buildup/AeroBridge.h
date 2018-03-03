#ifndef AeroBridge_h
#define AeroBridge_h

#include <Arduino.h>
#include <SPI.h> 

// set pin 10 as the slave select for the quanser qube
// (note that if a different pin is used for the slave select, pin 10 should be set as
// an output to prevent accidentally putting the arduino uno into slave mode.)
const int slaveSelectPin = 10;


// set the sample time (the interval between SPI transactions) to 1000us = 1ms
const long sampleTime = 10000;

const float maxMotorVoltage = 24.0; // Between +/- 24V


class QUBEAero {
	public:
		void update(float motor0Voltage, float motor1Voltage, int LEDRed, int LEDGreen, int LEDBlue, bool print = false);
		void reset();
		void debug(); // prints statements

		float get_pitch() const { return encoder2Deg; }
		float get_yaw() const { return encoder3Deg; }

	private:
		void spi_transfer(); // moves class variables to the Quanser Microprocessor
		unsigned long prevUpdateMicros = 0; // last update time, TODO: Check for rollover 

    	// initialize the SPI data to be written to the Base Node
		byte baseMode = 1;                  // normal mode = 1
		byte baseWriteMask = B00000111;     // Bxxxxx111 to enable the LEDs, Bxxx11xxx to enable writes to the Base Node encoders
		byte LEDRedMSB = 0;                 // red LED command MSB
		byte LEDRedLSB = 0;                 // red LED command LSB
		byte LEDGreenMSB = 0;               // green LED command MSB
		byte LEDGreenLSB = 0;               // green LED command LSB
		byte LEDBlueMSB = 0;                // blue LED command MSB
		byte LEDBlueLSB = 0;                // blue LED command LSB
		byte encoder2ByteSet[3] = {0,0,0};  // encoder2 is set to this value only when writes are enabled with baseWriteMask
		byte encoder3ByteSet[3] = {0,0,0};  // encoder3 is set to this value only when writes are enabled with baseWriteMask

		// initialize the SPI data to be read from the Base Node
		byte baseModuleIDMSB = 0;           // module ID MSB (module ID for the Base Node is 772 decimal)
		byte baseModuleIDLSB = 0;           // module ID LSB
		byte encoder2Byte[3] = {0,0,0};     // encoder2 counts (pitch)
		byte encoder3Byte[3] = {0,0,0};     // encoder3 counts (yaw)
		byte tach2Byte[3] = {0,0,0};        // tachometer2 (pitch)
		byte tach3Byte[3] = {0,0,0};        // tachometer3 (yaw)

		// initialize the SPI data to be written to the Core Node
		byte coreMode = 1;                  // normal mode = 1
		byte coreWriteMask = B00001111;     // Bxxxx1111 to enable the motors, Bxx11xxxx to enable writes to the Core Node encoders
		byte motor0MSB = 0x80;              // motor0 command MSB must be B1xxxxxxx to enable amplifier0
		byte motor0LSB = 0;                 // motor0 command LSB
		byte motor1MSB = 0x80;              // motor1 command MSB must be B1xxxxxxx to enable amplifier1
		byte motor1LSB = 0;                 // motor1 command LSB
		byte encoder0ByteSet[3] = {0,0,0};  // encoder0 is set to this value only when writes are enabled with coreWriteMask
		byte encoder1ByteSet[3] = {0,0,0};  // encoder1 is set to this value only when writes are enabled with coreWriteMask

		// initialize the SPI data to be read from the Core Node
		byte coreModuleIDMSB = 0;           // module ID MSB (module ID for the Core Node is 775 decimal)
		byte coreModuleIDLSB = 0;           // module ID LSB
		byte currentSense0MSB = 0;          // motor0 current sense MSB 
		byte currentSense0LSB = 0;          // motor0 current sense LSB
		byte currentSense1MSB = 0;          // motor1 current sense MSB 
		byte currentSense1LSB = 0;          // motor1 current sense LSB
		byte tach0Byte[3] = {0,0,0};        // tachometer0
		byte tach1Byte[3] = {0,0,0};        // tachometer1
		byte moduleStatus = 0;              // module status (the Quanser Aero sends status = 0 when there are no errors)
		byte encoder0Byte[3] = {0,0,0};     // encoder0 counts
		byte encoder1Byte[3] = {0,0,0};     // encoder1 counts
		byte xAccelLSB = 0;                 // X-axis accelerometer LSB
		byte xAccelMSB = 0;                 // X-axis accelerometer MSB
		byte yAccelLSB = 0;                 // Y-axis accelerometer LSB
		byte yAccelMSB = 0;                 // Y-axis accelerometer MSB
		byte zAccelLSB = 0;                 // Z-axis accelerometer LSB
		byte zAccelMSB = 0;                 // Z-axis accelerometer MSB
		byte xGyroLSB = 0;                  // X-axis gyroscope LSB
		byte xGyroMSB = 0;                  // X-axis gyroscope MSB
		byte yGyroLSB = 0;                  // Y-axis gyroscope LSB
		byte yGyroMSB = 0;                  // Y-axis gyroscope MSB
		byte zGyroLSB = 0;                  // Z-axis gyroscope LSB
		byte zGyroMSB = 0;                  // Z-axis gyroscope MSB

		// derived values
		int LEDRed;
		int LEDGreen;
		int LEDBlue;
		int baseModuleID;
		int coreModuleID;
		float encoder2Deg;  // pitch
		float encoder3Deg;  // yaw 
		float currentSense0;
		float currentSense1;
		float motor0; // set value, not in volts
		float motor1; // set value, not in volts
		float accelX; 
		float accelY;
		float accelZ;
		float gyroX;
		float gyroY;
		float gyroZ;
};

#endif
