#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <cmath>
#include <vector>


#include "robot_arm.h"



using namespace std;


// GPIO Used for PWM
#define GPIO1 17	// PIN 11/GPIO0
#define GPIO2 18	// PIN 12/GPIO1
#define GPIO3 23	// PIN 16/GPIO4
#define GPIO4 24	// PIN 18/GPIO5
#define GPIO5 4		// PIN 7/GPIO7

constexpr auto tf = 2;		                  // The motion should complete in 1second;
constexpr auto dt = 0.01;                    // Time steps;
#define M_PI 3.14159265358979323846         // PI



int main() {

    Robot_Arm robotArm(0.128, 0.25, 0.180);
    std::vector<double> jointAngles = robotArm.getJointAngles();
    robotArm.setPosition(0.25,0.25,0.25);
    jointAngles = robotArm.getJointAngles();



    // Angle mapping
    std::vector<double> servoAngle(5, 0.0);
    servoAngle[0] = -0.1111 * jointAngles[0] * 180 / M_PI + 25;
    servoAngle[1] = -0.1111 * jointAngles[1] * 180 / M_PI + 15;
    servoAngle[2] =  0.1111 * jointAngles[2] * 180 / M_PI + 5;
    servoAngle[3] = -0.1111 * jointAngles[3] * 180 / M_PI + 25;
    servoAngle[4] = -0.1111 * jointAngles[4] * 180 / M_PI + 15;

    vector<double> jointPos;
    std::vector<double> curAngle(5, 0.0);
    jointPos = robotArm.genTrajectory(curAngle, servoAngle, dt, tf);


    // Print the joint angles
    std::cout << "Joint angles: ";
    for (double angle : jointAngles) {
        std::cout << angle * 180 / M_PI << " ";
    }
    cout << "\n";
    cout << "Servo Values: ";


    for (int i = 0; i < 5; i++)
        cout << servoAngle[i] << " ";
    cout << "\n\n";
  



}