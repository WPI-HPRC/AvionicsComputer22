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

	abServo->attach(23,600,2400);
	Serial.println("Sensors & Servo Init Complete");
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

void Robot::controlAirbrakes(float ctrlCMD) {
	if(ctrlCMD < 0) {
		ctrlCMD = 0;
	} else if(ctrlCMD > 1) {
		ctrlCMD = 1;
	}

	float cmdEffort = 180 - (ctrlCMD*34);

	abServo->write(cmdEffort);
}
int16_t lastAccVal = 0;
long int LaunchTime = 0;
uint8_t impulseCheckBurnout = 0;
uint8_t impulseCheckLaunch = 0;
long int CoastTime = 0;
float LastPressure = 0;
float pressureCheck = 0;
void Robot::updateStateMachine(uint32_t timestamp){

	// Read sensors
	baro->readSensorData();
	imu->readSensorData();
//	Serial.print(imu->getAccX());
//	Serial.print(",");
//	Serial.print(imu->getAccY());
//	Serial.print(",");
//	Serial.print(imu->getAccZ());
//	Serial.println();
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

	//Serial.println(packet.getAltitude());
	//Serial.println(packet.getTemperature());

	// Update the packet for the dataLogger to transmit
	dataLogger->setCurrentDataPacket(packet.getTelemRocketPacketPtr(), 20);


	// Testing copying data from one packet object to another
	testPacket.setRocketTelemPacket(packet.getTelemRocketPacketPtr());
	testPacket.updateFromTelemPacket();		// for receiver to use

	//Serial.println(testPacket.getTemperature());
	switch(robotState){
		case ROBOT_IDLE:
			int16_t currAcc = imu->getAccZ();
			if(abs(currAcc)>6144){ //3g
				impulseCheckLaunch++;
			}
			else{
				impulseCheckLaunch = 0;
			}
			if(impulseCheckLaunch>50){ //.5 seconds
				LaunchTime = timestamp;
				robotState = ROBOT_LAUNCH;
			}
			break;
		case ROBOT_LAUNCH:
			if((imu->getAccZ()<200)){
				impulseCheckBurnout++;
			}
			else{
				impulseCheckBurnout = 0;
			}
			if(((timestamp-LaunchTime)>5000) && (imu->getAccZ()<0) && impulseCheckBurnout>33){ //third of a second
				robotState = ROBOT_COAST;
				CoastTime = timestamp;
			}
			else if((timestamp-LaunchTime)>8000){
				robotState = ROBOT_COAST;
				CoastTime = timestamp;

			}
			break;
		case ROBOT_COAST:
			long int TimeSinceCoast = timestamp-CoastTime;
			Serial.println((TimeSinceCoast/1000)%5 * 0.25);
			controlAirbrakes((TimeSinceCoast/1000)%5 * 0.25);
			float currPressure = baro->getPressure();
			if(currPressure<LastPressure && (timestamp - CoastTime)>10000){
				pressureCheck++;
			}
			else if(currPressure>LastPressure){
				pressureCheck = 0;
			}
			if(pressureCheck>33){
				robotState = ROBOT_APOGEE_REACHED;
				controlAirbrakes(0);
			}
			if((timestamp - CoastTime)>20000){
				controlAirbrakes(0);
				robotState = ROBOT_APOGEE_REACHED;
			}
			LastPressure = currPressure;
			break;
		case ROBOT_APOGEE_REACHED:

			break;
	}
}


void Robot::endStateMachine(){

}
