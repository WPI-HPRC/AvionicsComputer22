/*
 * Airbrakes.cpp
 *
 *  Created on: Jun 22, 2022
 *      Author: abbyg
 */

#include "Airbrakes.h"

Airbrakes::Airbrakes() {
	// TODO Auto-generated constructor stub

}

bool Airbrakes::systemInit(){

	// Set up barometer
	baro->init();
	baro->setModeAltimeter();
	baro->setOverSample6ms();

	// Set up IMU
	imu->init();
	imu->setPlusMinus2000DPS();			// maximum angular rate measuring
	imu->setPlusMinus16Gs();			// maximum acceleration measuring


	//	delay(10);							// let sensors start up

	Serial.print("START UP STATE: IDLE");


	return true;

}

/*
 * Function to register all loops to the system looper. The looper must have the
 * total number of system loops predefined, TOTAL_LOOPS must equal the number of
 * registerLoop() calls in this function, see Constants.h
 * @param runningLooper is the looper instance of the system manager to call
 * for adding loops
 */
void Airbrakes::registerAllLoops(Looper * runningLooper){

	runningLooper->registerLoop(airbrakesLoop);

}

// Call this in the IDLE state
void Airbrakes::zeroAllSensors(){

	baro->calibrateMPL3115A2();

}

/*
 * Pulls raw values from IMU and barometer
 */
void Airbrakes::pullSensorValues() {

	baro->readSensorData();
	imu->readSensorData();

	pressure_raw = baro->getPressure();
	temp_raw = baro->getTemperature();

	gyro_raw = imu->getGyroRawValues();
	accel_raw = imu->getAccRawValues();
}

/*
 * Tunes the coeff of drag by running three simulations with a binary tree
 */
void Airbrakes::runSimulations() {

	// first round, run on cd from kalman filter

	// second round, choose range hi/lo and run on either side

	// third round, run on either side again

	// tune final range of values and set CD again


}

/*
 * Takes CD and maps it to an extension of the airbrakes, then commands the airbrakes to extend to that length
 */
void Airbrakes::mapToAirbrakes() {

	// determine airbrakes length based on cd

	// command servo to certain pos based on length

}

/*
 * Configuring airbrakes subsystems for start of mission states sequence
 */
void Airbrakes::beginStateMachine(){

	zeroAllSensors();

}

//void Airbrakes::EWMAFilter() {
//	prevAltitude = altitude;
//	rawAltitude = baro->getAltitude() - altitudeAGLOffset;
//
//	// Filter
//	altitude = 	rawAltitude + ALPHA * (prevAltitude - rawAltitude);
//}

void Airbrakes::updateStateMachine(uint32_t timestamp){

	// airbrakes state machine
	switch (airbrakesState) {

		case DEV:

			break;

		case DATA_COLLECTION:
			// input raw data values into Kalman filter, get altitude/latVelocity/vertVelocity/cd
			pullSensorValues();
			// cd = kalmanFilter(pressure_raw, temp_raw, gyro_raw, accel_raw, cd);

			airbrakesState = UPDATE_AIRBRAKES;
			break;

		case UPDATE_AIRBRAKES:
			// run simulations
			runSimulations();

			// map to extension and activate airbrakes
			mapToAirbrakes();

			airbrakesState = DATA_COLLECTION;
			break;

		default:

			Serial.print("Code == broke");
			break;

	}

}


void Airbrakes::endStateMachine(){

}

