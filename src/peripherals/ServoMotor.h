/*
 * ICM20948.h
 *
 *  Created on: April 26, 2022
 *  Author: Abby Hyde
 */

#include <Wire.h>
#include <PWMServo.h>

#ifndef SRC_PERIPHERAL_SERVOMOTOR_H_
#define SRC_PERIPHERAL_SERVOMOTOR_H_

class ServoMotor
{

	PWMServo servo;

    TwoWire *_wire;

public:

    ServoMotor(uint8_t pwmPin);
    void enable();
    void disable();
    void setPosition(uint8_t position);

private:
	uint8_t motorPWMpin;

};

#endif /* SRC_PERIPHERAL_SERVOMOTOR_H_ */
