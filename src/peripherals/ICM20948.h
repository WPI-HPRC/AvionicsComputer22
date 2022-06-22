/*
 * ICM20948.h
 *
 *  Created on: Feb 3, 2022
 *  Author: Charlie Tribble and Samay Govani
 */

#include <Wire.h>
#include "Arduino.h"

#define ICM20948_ADDRESS			 0x68

// ICM20948 register addresses
#define ICM20948_PWR_MGMT_1          0x06
#define ICM20948_ODR_ALIGN_EN        0x09

#define GYRO_CONFIG_1				 0x01	// Register ID in bank 2
#define ACCEL_CONFIG_1				 0x14	// Register ID in bank 2

#define ICM20948_ACCEL_OUT           0x2D // accel data registers begin
#define ICM20948_GYRO_OUT            0x33 // gyro data registers begin

#define ICM20948_REG_BANK_SEL        0x7F

#define ICM20948_RESET               0x80


// ICM20948 register data bits
#define ICM20948_SLEEP               0x40

#define PLUS_MINUS_250DPS			 0		// 131 	 = 131  LSB / (dps)
#define PLUS_MINUS_500DPS			 1		// 131/2 = 65.5 LSB / (dps)
#define PLUS_MINUS_1000DPS			 2		// 131/4 = 32.8 LSB / (dps)
#define PLUS_MINUS_2000DPS			 3		// 131/8 = 16.4 LSB / (dps)

#define PLUS_MINUS_2G				 0		// 2^14 = 16384 LSB/g
#define PLUS_MINUS_4G				 1		// 2^13 = 8192  LSB/g
#define PLUS_MINUS_8G				 2		// 2^12 = 4096  LSB/g
#define PLUS_MINUS_16G				 3		// 2^11 = 2048  LSB/g

#define DT_LOOPER					 0.01

struct Vector
{
    int16_t x;
    int16_t y;
    int16_t z;
};

class ICM20948
{

	uint8_t I2C_addr;

    TwoWire *_wire;

public:

    ICM20948(uint8_t address);

    bool init();
    void setGyroScale(uint8_t value);
    void setAccScale(uint8_t value);

    void readSensorData();
    void calibrateGyro();
    void complementaryFilter();

    void printVector(Vector print);
    void printBuffer();

    Vector getAccRawValues();  //TODO FOR TESTING ONLY
    Vector getGyroRawValues(); // TODO FOR TESTING ONLY
    int16_t getTempRawValues();

    int16_t getAccX();
    int16_t getAccY();
    int16_t getAccZ();
    int16_t getGyroX();
    int16_t getGyroY();
    int16_t getGyroZ();
    float getRoll();
    float getPitch();

    uint8_t getPlusMinus250DPS();
    uint8_t getPlusMinus500DPS();
    uint8_t getPlusMinus1000DPS();
    uint8_t getPlusMinus2000DPS();

    uint8_t getPlusMinus2Gs();
    uint8_t getPlusMinus4Gs();
    uint8_t getPlusMinus8Gs();
    uint8_t getPlusMinus16Gs();

private:

    uint8_t currBank;
    uint8_t regVal;
    uint8_t SensorRegister[14];			// last polled sensor register values stored here

	const uint16_t calibrationSamples = 2000;	//This is the total number of calibration samples to be taken per calibration routine.
    int32_t gyro_x_cal = 0;
	int32_t gyro_y_cal = 0;
	int32_t gyro_z_cal = 0;

	int16_t acc_x_raw = 0;
	int16_t acc_y_raw = 0;
	int16_t acc_z_raw = 0;
	int16_t gyro_x_raw = 0;
	int16_t gyro_y_raw = 0;
	int16_t gyro_z_raw = 0;

	const float dt = DT_LOOPER;
	const float radToDeg = 180 / 3.14159;
	const float alpha = 0.98;
	const float maxGravity = 2;


	float roll = 0;
	float pitch = 0;
	float yaw = 0;

	float totalAccelVector = 0;
	float anglePitchAccel = 0 ;
	float angleRollAccel = 0;

	float filteredPitch = 0;
	float filteredRoll = 0;

	int16_t temperature = 0;

    uint8_t resetSensor();
    void sleep(bool sleep);

    void switchBank(uint8_t newBank);
    int16_t processHighLowBytes(uint8_t Hbyte, uint8_t Lbyte);

    uint8_t writeRegister8(uint8_t bank, uint8_t reg, uint8_t val);
    uint8_t writeRegister16(uint8_t bank, uint8_t reg, int16_t val);
    uint8_t readRegister8(uint8_t bank, uint8_t reg);
    int16_t readRegister16(uint8_t bank, uint8_t reg);
    uint8_t read8I2C(uint8_t reg);
    uint8_t write8I2C(uint8_t reg, uint8_t val);

};


