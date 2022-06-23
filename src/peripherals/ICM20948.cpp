/*
 * ICM20948.cpp
 *
 *  Created on: Feb 3, 2022
 *  Author: Charlie Tribble and Samay Govani
 */

#include "ICM20948.h"

/*
 * TODO comment this boy
 * @param address
 */
ICM20948::ICM20948(uint8_t address)
{
	_wire = &Wire;
	I2C_addr = address;
}

/*
 * TODO comment this boy
 * @return TODO
 */
bool ICM20948::init()
{
	currBank = 0;
	if (!resetSensor())
	{
		return false;
	}
	sleep(false);
	writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1);
	return true; //TODO should this return true?
}

/*
 * Set the GYRO_FS_SEL register to configure the sensitivity range of measurements
 * @param value is 0-3 to set the sensitivity
 */
void ICM20948::setGyroScale(uint8_t value) {

    if(value > 3) value = 3; 			// GYRO_FS_SEL cannot be larger than B011
    value <<= 1;           				// align it for the GYRO_CONFIG_1 register

	switchBank(2);						// register ID is for bank 2

	uint8_t setting = 0;
	setting = read8I2C(GYRO_CONFIG_1);

	setting &= B11111001;				// clear out old GYRO_FS_SEL bits
	setting |= value;					// mask in new GYRO_FS_SEL bits

	write8I2C(GYRO_CONFIG_1, setting);

}


/*
 * Set the ACCEL_FS_SEL register to configure the sensitivity range of measurements
 * @param value is 0-3 to set the sensitivity
 * 0: +-2g
 * 1: +-4g
 * 2: +-8g
 * 3: +-16g
 */
void ICM20948::setAccScale(uint8_t value) {

    if(value > 3) value = 3; 			// ACCEL_FS_SEL cannot be larger than B011
    value <<= 1;           				// align it for the ACCEL_CONFIG_1 register

	switchBank(2);						// register ID is for bank 2

	uint8_t setting = 0;
	setting = read8I2C(ACCEL_CONFIG_1);

	setting &= B11011001;				// clear out old ACCEL_FS_SEL bits
	setting |= value;					// mask in new ACCEL_FS_SEL bits

	write8I2C(ACCEL_CONFIG_1, setting);

}


/*
 * TODO comment this boy
 * @return TODO
 */
uint8_t ICM20948::resetSensor()
{
	uint8_t ack = writeRegister8(0, ICM20948_PWR_MGMT_1, ICM20948_RESET);
	delay(10); // wait for registers to reset
	return (ack == 0);
}

/*
 * TODO comment this boy
 * @param sleep TODO
 */
void ICM20948::sleep(bool sleep)
{
	regVal = this->readRegister8(0, ICM20948_PWR_MGMT_1);
	if (sleep)
	{
		regVal |= ICM20948_SLEEP; // TODO what this doing?
	}
	else
	{
		regVal &= ~ICM20948_SLEEP; // TODO what this doing?
	}
	writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}

/*
 * Read all data from the 14 IMU data registers (ACCELXYZ, GYROXYZ, TEMP)
 * and save it to a buffer in this class for parsing/reading
 */
void ICM20948::readSensorData(){

	switchBank(0); 				// Should be done every time

	_wire->beginTransmission(I2C_addr);
	_wire->write(ICM20948_ACCEL_OUT);
	_wire->endTransmission(false);

	_wire->requestFrom(I2C_addr,14);

	for(uint8_t i = 0; i < 14; i++){
		this->SensorRegister[i] = _wire->read();
	}

	this->acc_x_raw = processHighLowBytes(SensorRegister[0],SensorRegister[1]);
	this->acc_y_raw = processHighLowBytes(SensorRegister[2],SensorRegister[3]);
	this->acc_z_raw = processHighLowBytes(SensorRegister[4],SensorRegister[5]);

	this->gyro_x_raw = processHighLowBytes(SensorRegister[6],SensorRegister[7]) - gyro_x_cal;
	this->gyro_y_raw = processHighLowBytes(SensorRegister[8],SensorRegister[9]) - gyro_y_cal;
	this->gyro_z_raw = processHighLowBytes(SensorRegister[10],SensorRegister[11]) - gyro_z_cal;


}

/*
 * based off 2020-2021 code
 */
void ICM20948::calibrateGyro() {

	Serial.println(F("Gyro calibrating, don't move it!!!"));

	for(int cal_int = 0; cal_int < calibrationSamples; cal_int++){
	  if(cal_int % 125 == 0)
		  Serial.print(".");
	  readSensorData();
	  gyro_x_cal += gyro_x_raw;
	  gyro_y_cal += gyro_y_raw;
	  gyro_z_cal += gyro_z_raw;
	  delay(3);
	}

	gyro_x_cal /= calibrationSamples;
	gyro_y_cal /= calibrationSamples;
	gyro_z_cal /= calibrationSamples;

	Serial.println(gyro_x_cal); 				//Check to see if calibration was successfull. Value should be very close to 0.
	Serial.println(gyro_y_cal); 				//Check to see if calibration was successfull. Value should be very close to 0.
	Serial.println(gyro_z_cal); 				//Check to see if calibration was successfull. Value should be very close to 0.

	delay(2000); 	//DELETE WHEN DONE!!! Delay to allow for previous prints to be read. DELETE WHEN DONE!!!

	Serial.println(F("Done Calibrating!"));	//Alerts that the Gyro has completed it's calibration routine.
}

/*
 * An iterative function that filters the roll and pitch for every time the data updates
 * Based on code from 2020-2021 year
 * when launched, roll and pitch are useful
 * when landed, pitch and yaw are useful
 * yaw -> roll
 * pitch = pitch
 * roll -> yaw
 */
void ICM20948::complementaryFilter(bool landed) {
	totalAccelVector = sqrt((acc_x_raw*acc_x_raw)+(acc_y_raw*acc_y_raw)+(acc_z_raw*acc_z_raw));  	//Calculates the total accelerometer vector.

	// if payload is vertical (during flight), pitch should use the x axis
	filteredRoll += gyro_y_raw / 131 * dt;	//Integrates the angular rate of X axis over dt to return absolute position of X axis.
	filteredPitch += gyro_x_raw / 131 * dt;	//Integrates the angular rate of Y axis over dt to return absolute position of Y axis.

	anglePitchAccel = atan2f(acc_y_raw, (sqrt((acc_x_raw * acc_x_raw) + (acc_z_raw * acc_z_raw)))) * radToDeg;
	angleRollAccel = atan2f(-acc_x_raw, acc_z_raw) * radToDeg;

	if (totalAccelVector < maxGravity) {
		filteredPitch = filteredPitch * alpha + anglePitchAccel * (1-alpha);
		filteredRoll = filteredRoll * alpha + angleRollAccel * (1-alpha);
	}


	// TODO prints for debugging, remove when done !
//	Serial.println(filteredPitch);

}

// helper functions

/*
 * TODO comment this boy
 * @param bank TODO
 */
void ICM20948::switchBank(uint8_t bank)
{
	if (bank != currBank)
	{
		currBank = bank;
		_wire->beginTransmission(I2C_addr);
		_wire->write(ICM20948_REG_BANK_SEL);
		_wire->write(currBank << 4);
		_wire->endTransmission(false);
	}
}

/*
 * TODO comment this boy
 * @param Hbyte TODO
 * @param Lbyte TODO
 * @return TODO
 */
int16_t ICM20948::processHighLowBytes(uint8_t Hbyte, uint8_t Lbyte){
	return (int16_t) ((Hbyte << 8)|Lbyte); // TODO what is this casting?
}

/*
 * TODO comment this boy
 * @return A Vector containing the raw accel data
 */
Vector ICM20948::getAccRawValues(){
	Vector accRaw;
	accRaw.x = this->acc_x_raw; // TODO what is this casting?
	accRaw.y = this->acc_y_raw; // TODO what is this casting?
	accRaw.z = this->acc_z_raw; // TODO what is this casting?
	return accRaw;
}

/*
 * TODO comment this boy
 * @return A Vector containing the raw gyro data
 */
Vector ICM20948::getGyroRawValues(){
	Vector gyroRaw;
	gyroRaw.x = gyro_x_raw; // TODO what is this casting?
	gyroRaw.y = gyro_y_raw; // TODO what is this casting?
	gyroRaw.z = gyro_z_raw; // TODO what is this casting?
	return gyroRaw;
}


/*
 * TODO comment this boy
 * @return Temperature in ??? TODO
 */
int16_t ICM20948::getTempRawValues(){
	return processHighLowBytes(SensorRegister[12],SensorRegister[13]);
}

/*
 * TODO comment this boy
 * @param Vector to print
 */
void ICM20948::printVector(Vector print){
	Serial.print(print.x);
	Serial.print(",");
	Serial.print(print.y);
	Serial.print(",");
	Serial.print(print.z);
	Serial.print("\n");
}

void ICM20948::printBuffer(){
//	for(int i = 0; i<14;i++){
//		Serial.print(SensorRegister[i]);
//		Serial.print("\t");
//	}

		// Accelerometer x-axis
		Serial.write(65); // A
		Serial.write(88); // X
		Serial.write(SensorRegister[0]);
		Serial.write(SensorRegister[1]);

		// Accelerometer y-axis
		Serial.write(65); // A
		Serial.write(89); // Y
		Serial.write(SensorRegister[2]);
		Serial.write(SensorRegister[3]);

		// Accelerometer z-axis
		Serial.write(65); // A
		Serial.write(90); // Z
		Serial.write(SensorRegister[4]);
		Serial.write(SensorRegister[5]);

		// Gyro x-axis
		Serial.write(71); // G
		Serial.write(88); // X
		Serial.write(SensorRegister[6]);
		Serial.write(SensorRegister[7]);

		// Gyro y-axis
		Serial.write(71); // G
		Serial.write(89); // Y
		Serial.write(SensorRegister[8]);
		Serial.write(SensorRegister[9]);

		// Gyro z-axis
		Serial.write(71); // G
		Serial.write(90); // Z
		Serial.write(SensorRegister[10]);
		Serial.write(SensorRegister[11]);
}

/*
 * writes 8 bits from val to the specified register
 * returns the endTransmission code
 */
uint8_t ICM20948::writeRegister8(uint8_t bank, uint8_t reg, uint8_t val){
	switchBank(bank); // TODO do we need to set this every time?
	//Serial.println(I2C_addr);
	_wire->beginTransmission(I2C_addr);
	_wire->write(reg);
	_wire->write(val);

	return _wire->endTransmission();
}

/*
 * reads 8 bits from the specified register
 * returns the value read
 */
uint8_t ICM20948::readRegister8(uint8_t bank, uint8_t reg){
    switchBank(bank);
    uint8_t regValue = 0;
    _wire->beginTransmission(I2C_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(I2C_addr,1);
    if(_wire->available()){
        regValue = _wire->read();
    }
    return regValue;
}

/*
 * reads 16 bits from the specified register
 * returns the value read
 */
int16_t ICM20948::readRegister16(uint8_t bank, uint8_t reg){
    switchBank(bank);
    uint8_t MSByte = 0, LSByte = 0;
    int16_t reg16Val = 0;
    _wire->beginTransmission(I2C_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(I2C_addr,2);
    if(_wire->available()){
        MSByte = _wire->read();
        LSByte = _wire->read();
    }
    reg16Val = (MSByte<<8) + LSByte;
    return reg16Val;
}

/*
 * writes 16 bits from val to the specified I2C address
 * returns the endTransmission code
 */
uint8_t ICM20948::writeRegister16(uint8_t bank, uint8_t reg, int16_t val){
    switchBank(bank);
    int8_t MSByte = (int8_t)((val>>8) & 0xFF);
    uint8_t LSByte = val & 0xFF;
    _wire->beginTransmission(I2C_addr);
    _wire->write(reg);
    _wire->write(MSByte);
    _wire->write(LSByte);

    return _wire->endTransmission();
}


/*
 * reads 8 bits from the specified I2C address
 * returns the value read
 * (redundant function)
 */
uint8_t ICM20948::read8I2C(uint8_t reg) {

    _wire->beginTransmission(ICM20948_ADDRESS);
    _wire->write(reg);        			// Address to read data from
    _wire->endTransmission(false);   		// Send data to I2C with option for a repeated start
    _wire->requestFrom(ICM20948_ADDRESS, 1);   // Request the data
    return Wire.read();

}

/*
 * writes 8 bits from val to the specified I2C address
 * returns the endTransmission code
 * (redundant function)
 */
uint8_t ICM20948::write8I2C(uint8_t reg, uint8_t val) {

	_wire->beginTransmission(ICM20948_ADDRESS);
	_wire->write(reg);
	_wire->write(val);

	return _wire->endTransmission(true);
}

// getter functions

int16_t ICM20948::getAccX() { return acc_x_raw; }
int16_t ICM20948::getAccY() { return acc_y_raw; }
int16_t ICM20948::getAccZ() { return acc_z_raw; }
int16_t ICM20948::getGyroX() { return gyro_x_raw; }
int16_t ICM20948::getGyroY() { return gyro_y_raw; }
int16_t ICM20948::getGyroZ() { return gyro_z_raw; }
float ICM20948::getRoll() { return filteredRoll; }
float ICM20948::getPitch() { return filteredPitch; }

uint8_t ICM20948::getPlusMinus250DPS() { return PLUS_MINUS_250DPS; }
uint8_t ICM20948::getPlusMinus500DPS() { return PLUS_MINUS_500DPS; }
uint8_t ICM20948::getPlusMinus1000DPS() { return PLUS_MINUS_1000DPS; }
uint8_t ICM20948::getPlusMinus2000DPS() { return PLUS_MINUS_2000DPS; }
uint8_t ICM20948::getPlusMinus2Gs() { return PLUS_MINUS_2G; }
uint8_t ICM20948::getPlusMinus4Gs() { return PLUS_MINUS_4G; }
uint8_t ICM20948::getPlusMinus8Gs() { return PLUS_MINUS_8G; }
uint8_t ICM20948::getPlusMinus16Gs() { return PLUS_MINUS_16G; }
