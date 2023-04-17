#ifndef SERVO_FUNC_H
#define SERVO_FUNC_H



/*
 * Define GPIO --> WiringPi
 */


#define GPIO1 17	// PIN 11/GPIO0
#define GPIO2 18	// PIN 12/GPIO1
#define GPIO3 23	// PIN 16/GPIO4
#define GPIO4 24	// PIN 18/GPIO5
#define GPIO5 4		// PIN 7/GPIO7

#include <wiringPi.h>
#include <softPwm.h>


void pwmDefine();
std::vector<double> servoAngleMap(std::vector<double> jointAngles);
void servoWrite(std::vector<double> jointPos, double dt, double tf);

#endif 





