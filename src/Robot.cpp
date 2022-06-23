/*
 * Robot.cpp
 * Created on: Sept 12, 2021
 * Author: Peter Dentch
 */

#include "Robot.h"


/*
* Constructor for the robot system object, should only be one instance, one main system per processor
*/
Robot::Robot(){};


/*
 * Init function for the system, should be run after instantiation
 * Should take SPI/I2C/Serial objects as input parameters?
 */
bool Robot::systemInit(){


	// Set up barometer
	baro->init();

	baro->setModeAltimeter();
	baro->setOverSample6ms();

	// Set up IMU
	imu->init();

	imu->setGyroScale(imu->getPlusMinus2000DPS());
	imu->setAccScale(imu->getPlusMinus8Gs());
	imu->calibrateGyro();


	// Datalogger
	dataLogger->subsystemInit();



	return true;

}

/*
 * Function to register all loops to the system looper. The looper must have the
 * total number of system loops predefined, TOTAL_LOOPS must equal the number of
 * registerLoop() calls in this function, see Constants.h
 * @param runningLooper is the looper instance of the system manager to call
 * for adding loops
 */
void Robot::registerAllLoops(Looper * runningLooper){

	runningLooper->registerLoop(robotLoop);

	dataLogger->registerLoops(runningLooper);

}


void Robot::zeroAllSensors(){

	//robotStateEstimator->reset(millis());

	//selfRighting->zeroSensors();

}


/*
 * Configuring robot subsystems for start of mission states sequence
 */
void Robot::beginStateMachine(){

	//zeroAllSensors();

}

void Robot::updateStateMachine(uint32_t timestamp){

	// Read sensors
	baro->readSensorData();
	imu->readSensorData();

	// Build a rocket telemetry data packet using sensor data
	packet.setTimestamp(timestamp);
	packet.setState(0);						// state zero hardcode for now
	//packet.setAltitude(9999.99);
	//packet.setTemperature(-4.5);
	packet.setAltAndTempCombined(baro->getPressureAndTempCombined());
	packet.setAccelX(imu->getAccX());
	packet.setAccelY(imu->getAccY());
	packet.setAccelZ(imu->getAccZ());
	packet.setGyroX(imu->getGyroX());
	packet.setGyroY(imu->getGyroY());
	packet.setGyroZ(imu->getGyroZ());

	packet.updateToTelemPacket();			// for transmitter to use

	switch(robotState) {

		case ROBOT_STARTUP:
			break;
		case ROBOT_IDLE:
			float currAccelZ = packet.getAccelZ() * (1/2048);
			if(currAccelZ > 3) {
				launchDetectionTicks++;
				if(launchDetectionTicks > 10) {
					robotState = ROBOT_LAUNCH;
				}
			}

			break;
		case ROBOT_LAUNCH:

			if()
			break;
		case ROBOT_COAST:
			break;
	}


	//Serial.println(packet.getAltitude());
	//Serial.println(packet.getTemperature());

	// Update the packet for the dataLogger to transmit
	dataLogger->setCurrentDataPacket(packet.getTelemRocketPacketPtr(), 20);


	// Testing copying data from one packet object to another
	testPacket.setRocketTelemPacket(packet.getTelemRocketPacketPtr());
	testPacket.updateFromTelemPacket();		// for receiver to use

	//Serial.println(testPacket.getTemperature());

}


void Robot::endStateMachine(){

}
