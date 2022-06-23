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
	imu->getPlusMinus2000DPS();			// maximum angular rate measuring
	imu->getPlusMinus16Gs();			// maximum acceleration measuring


	//	delay(10);							// let sensors start up

	airbrakesMotor->enable();

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

void Airbrakes::lowPassFilter() {

	if (!init) {
		accelZ = (float) accel_raw.z;
		prevAccelZ = accelZ;
		init = true;
	} else {
		prevAccelZ = accelZ;
		accelZ = ((float) accel_raw.z) + ALPHA * (accelZ - ((float)accel_raw.z));
	}

}

/*
 * predicts apogee
 * inputs: xCurr, cd
 * xCurr is the current set of states (aka characteristics to describe the rocket) so px, py, vx, vy
 * returns: apogee prediction in ft
 */
double Airbrakes::predictApogee (std::array<double, NSTATES> xCurr, double cd) {

    int i;
    double dt = 0.5;
    double tNew;
    std::array<double, NSTATES> xNew;

    // run 1000 simulations
    for (i = 0; i < 1000; i++) {
        xNew = rk2 (dt, xCurr, 0.445);
        tNew = dt;

        if (xNew[3] <= 0) {

        	int ii = 1;

        	// run a binary search to get a more accurate cd
        	while (abs(xNew[3]) > 0.1) {

        		int high = dt;
        		int low = 0;

        		dt = low + (high-low)/2;
        		xNew = rk2 (dt, xCurr, 0.445);

        		// pick the next half of the time section to test
        		if (xNew[3] < 0) {
					high = dt;
        		} else {
					low = dt;
        		}

				ii = ii + 1;

				if (ii > 5) {
				    break;
				}
        	}
        }

        xCurr = xNew;
    }

    return xNew[1];
}

/*
 * Takes CD and maps it to an extension of the airbrakes, then commands the airbrakes to extend to that length
 */
void Airbrakes::mapToAirbrakes() {

	// determine airbrakes length based on cd

	// command servo to certain pos based on length
	// 1:1 ratio, aka 100% rotation = 100% extension

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
			lowPassFilter();


			airbrakesState = UPDATE_AIRBRAKES;
			break;

		case UPDATE_AIRBRAKES:
			// run simulations
			cd = predictApogee(*xCurr, cd);

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

/*
 * HELPER FUNCTIONS
 */

/*
 * performs an rk2 calculation
 * inputs: arr, dt, xCurr, cd
 * returns: x
 */
std::array<double, NSTATES> Airbrakes::rk2(double dt, std::array<double, NSTATES> xCurr, double cd) {

    int i;
    double t;
    std::array<double, NSTATES> x, k1, k2, xMid;

    // Perform first integration step
    k1 = dxdt (0, xCurr, cd);

    // Compute state input for 2nd inegration step
    for ( i = 0; i < NSTATES; i++ ) {
        xMid[i] = xCurr[i] + dt * k1[i];
    }

    // Perform second integration step
    k2 = dxdt (0 + dt, xMid, cd);

    // Compute states at next timestep
    t = 0 + dt;
    for (i = 0; i < NSTATES; i++) {
        x[i] = xCurr[i] + 0.5 * dt * (k1[i] + k2[i]);
    }

    return x;
}

/*
 * does a thing
 * inputs: t, x, cd
 * returns ??
 */
std::array<double, NSTATES> Airbrakes::dxdt (double t, std::array<double, 4> x, double cd) {
	std::array<double, NSTATES> f;

    double rho, flight_angle, vel_sq, drag_total, drag_X, drag_Y;
    const double m = 25.9025;
//    const double A = 0.0193;

    rho = -9.553e-05*x[1] + 1.197;

    flight_angle = atan2(x[3],x[2]);
    vel_sq = pow(x[2],2) + pow(x[3],2);

    drag_total = 0.5 * rho * cd * vel_sq * A;

    drag_X = drag_total * cos(flight_angle);
    drag_Y = drag_total * sin(flight_angle);

    f[0] = x[2];
    f[1] = x[3];
    f[2] = -drag_X / m;
    f[3] = -drag_Y / m - 9.80655;

    return f;

}

