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
#include "utilities/KalmanFilter.h"

#include "peripherals/ICM20948.h"
#include "peripherals/MPL3115A2.h"
#include "peripherals/ServoMotor.h"

#define ICM_ADDRESS 0x68
#define SERVO_PIN 0 					// TODO: determine which pin this is

#define VEHICLE_MASS 57.115 / 2.205
#define A 0.0192896388
#define ATMO_PRESSURE 0 				// TODO: FILL THIS IN ON LAUNCH DAY
#define NSTATES 4

enum AirbrakesState {

	DEV,
	DATA_COLLECTION,
	UPDATE_AIRBRAKES
};

class Airbrakes : public SystemInterface {

private:

	AirbrakesState airbrakesState = DEV;

	ICM20948 * imu = new ICM20948(ICM_ADDRESS);
	MPL3115A2 * baro = new MPL3115A2();

	ServoMotor * airbrakesMotor = new ServoMotor(SERVO_PIN);

	// latest raw sensor values to send into Kalman filter
	float pressure_raw;
	float temp_raw;

	Vector gyro_raw;
	Vector accel_raw;

	// Kalman filtered values
	std::array<double, NSTATES> * xCurr = new std::array<double, NSTATES>(); // in the order of px py vx vy
	double cd = 0;

	void pullSensorValues();
	void mapToAirbrakes();

	// helper functions
	std::array<double, NSTATES> rk2(double dt, std::array<double, NSTATES> xCurr, double cd);
	std::array<double, NSTATES> dxdt (double t, std::array<double, 4> x, double cd);
	double Airbrakes::predictApogee (std::array<double, NSTATES> xCurr, double cd);


public:

	Airbrakes();

	class AirbrakesLoop : public Loop {
		Airbrakes * airbrakes_;

	public:
		AirbrakesLoop(Airbrakes * instance){
			airbrakes_ = instance;
		};

		void onStart(uint32_t timestamp){
			airbrakes_->beginStateMachine();
		}
		void onLoop(uint32_t timestamp){
			airbrakes_->updateStateMachine(timestamp);
		}
		void onStop(uint32_t timestamp){
			airbrakes_->endStateMachine();
		}
	} * airbrakesLoop = new AirbrakesLoop(this);		// instantiate the main system loop and pass it the system instance

	bool systemInit();
	void registerAllLoops(Looper * runningLooper);

	void zeroAllSensors();

	void beginStateMachine();
	void updateStateMachine(uint32_t timestamp);
	void endStateMachine();

};

#endif /* SRC_AIRBRAKES_H_ */
