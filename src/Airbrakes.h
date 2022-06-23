/*
 * Airbrakes.h
 *
 *  Created on: Jun 22, 2022
 *      Author: abbyg
 */

#ifndef SRC_AIRBRAKES_H_
#define SRC_AIRBRAKES_H_

#include <cmath>
#include <array>
#include "Arduino.h"

#include "../Constants.h"
#include "SystemInterface.h"
#include "loops/Looper.h"
#include "loops/loop.h"

#include "peripherals/ICM20948.h"
#include "peripherals/MPL3115A2.h"
#include "peripherals/ServoMotor.h"

#define ICM_ADDRESS 0x68
#define SERVO_PIN 0 					// TODO: determine which pin this is

#define VEHICLE_MASS 57.115 / 2.205
#define A 0.0192896388
#define ATMO_PRESSURE 0 				// TODO: FILL THIS IN ON LAUNCH DAY
#define NSTATES 4
#define DT 0.01

enum AirbrakesState {

	DEV,
	DATA_COLLECTION,
	UPDATE_AIRBRAKES
};

class Airbrakes : public SystemInterface {

private:

//	AirbrakesState airbrakesState = DEV;

	ICM20948 * imu = new ICM20948(ICM_ADDRESS);
	MPL3115A2 * baro = new MPL3115A2();

	ServoMotor * airbrakesServo = new ServoMotor(SERVO_PIN);

	// latest raw sensor values to send into Kalman filter
	float pressure_raw;
	float temp_raw;

	Vector gyro_raw;
	Vector accel_raw;

	bool init = false;
	float accelZ;
	float prevAccelZ;
	const float ALPHA = 0.6;
//
//	float altitude = 0;
//
//	float totalVel = 0;

	// Kalman filtered values
//	std::array<double, NSTATES> * xCurr = new std::array<double, NSTATES>(); // in the order of px py vx vy
//	double cd = 0;
//	double targetCD = 0;
//	double rho = 0;
//	double dragTarget = 0;
//	double airbrakeDragTarget = 0;
//	double qPsi;

	// helper functions
	std::array<double, NSTATES> rk2(double dt, std::array<double, NSTATES> xCurr, double cd);
	std::array<double, NSTATES> dxdt (double t, std::array<double, 4> x, double cd);
	double Airbrakes::predictApogee (std::array<double, NSTATES> xCurr, double cd);


public:

	Airbrakes();

	bool systemInit();
	void extend(uint8_t ext); // only two functions we're actually using
//	void registerAllLoops(Looper * runningLooper);

	void zeroAllSensors();
	void pullSensorValues();
	void lowPassFilter();
	void mapToAirbrakes();
	void updateAirbrakes();

//	void beginStateMachine();
//	void updateStateMachine(uint32_t timestamp);
//	void endStateMachine();

};

#endif /* SRC_AIRBRAKES_H_ */
