/*
 * ServoMotor.cpp
 *
 *  Created on: April 26, 2022
 *      Author: Abby Hyde
 */

#include "ServoMotor.h"

/*
 * Constructor for ServoMotor using a potentiometer as a rotary encoder for feedback
 * @param pwmPin is the pin delivering a PWM signal for the motor controller
 * @param directionPin is the direction toggling for the motor controller
 * @param currentSensorPin is the analog input pin for the motor controller current feedback
 * @param encoderPin is the analog input pin for the potentiometer feedback
 */
ServoMotor::ServoMotor(uint8_t pwmPin) {

	motorPWMpin = pwmPin;
}

void ServoMotor::enable() {

	servo.attach(motorPWMpin);

}

void ServoMotor::disable() {

}

//void ServoMotor::update() {
//
//}

/*
 * sets the servo motor to a position (aka the angle)
 * ranges between 0 and 180
 */
void ServoMotor::setPosition(uint8_t position) {

	//analogWrite(motorPWMpin, position);

	servo.write(position);

}
