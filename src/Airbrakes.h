/*
 * Airbrakes.h
 *
 *  Created on: Jun 22, 2022
 *      Author: abbyg
 */

#ifndef SRC_AIRBRAKES_H_
#define SRC_AIRBRAKES_H_

#include "Arduino.h"

#include "../Constants.h"
#include "SystemInterface.h"
#include "loops/Looper.h"
#include "loops/loop.h"

#include "peripherals/ICM20948.h"
#include "peripherals/MPL3115A2.h"

#define ICM_ADDRESS 0x68
#define VEHICLE_MASS 57.115 / 2.205
#define A 0.0192896388
#define ATMO_PRESSURE 0 				// TODO: Fill this in day of launch

/*
 * Airbrakes have TODO primary states of autonomous operation throughout its mission which begins when the system is powered on
 *
 * STATE DETAILS
 */
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

	// will need a good ol servo object here

	// latest raw sensor values to send into Kalman filter
	float pressure_raw;
	float temp_raw;

	Vector gyro_raw;
	Vector accel_raw;

	// Kalman filtered values
	float altitude = 0;
	float latVelocity = 0;
	float vertVelocity = 0;
	float cd = 0; 				// sent into the next iteration of Kalman filtering

	void pullSensorValues();
	void runSimulations();
	void mapToAirbrakes();


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
