#include <wiringPi.h>
#include <softPwm.h>
#include <cmath>
#include<vector>
#include <iostream>
#include <chrono>
#include <thread>


#include "servo_func.h"


#define M_PI 3.14159265358979323846 




void pwmDefine()
{


    pinMode(GPIO1, OUTPUT);
    digitalWrite(GPIO1, LOW);

    pinMode(GPIO2, OUTPUT);
    digitalWrite(GPIO2, LOW);

    pinMode(GPIO3, OUTPUT);
    digitalWrite(GPIO3, LOW);

    pinMode(GPIO4, OUTPUT);
    digitalWrite(GPIO4, LOW);

    pinMode(GPIO5, OUTPUT);
    digitalWrite(GPIO5, LOW);

    softPwmCreate(GPIO1, 0, 200);
    softPwmCreate(GPIO2, 0, 200);
    softPwmCreate(GPIO3, 0, 200);
    softPwmCreate(GPIO4, 0, 200);
    softPwmCreate(GPIO5, 0, 200);

}


std::vector<double> servoAngleMap(std::vector<double> jointAngles)
{
    // Angle mapping
    std::vector<double> servoAngle(5, 0.0);
    servoAngle[0] = -0.1111 * jointAngles[0] * 180 / M_PI + 25;
    servoAngle[1] = -0.1111 * jointAngles[1] * 180 / M_PI + 15;
    servoAngle[2] =  0.1111 * jointAngles[2] * 180 / M_PI + 5;
    servoAngle[3] = -0.1111 * jointAngles[3] * 180 / M_PI + 25;
    servoAngle[4] = -0.1111 * jointAngles[4] * 180 / M_PI + 15;

    return servoAngle;
}



void servoWrite(std::vector<double> jointPos, double dt, double tf)
{

    for (int i = 0; i < jointPos.size()/5; i++)
    {
        softPwmWrite(GPIO1, jointPos[i]);
        softPwmWrite(GPIO2, jointPos[i +   jointPos.size() / 5]);
        softPwmWrite(GPIO3, jointPos[i + 2*jointPos.size() / 5]);
        softPwmWrite(GPIO4, jointPos[i + 3*jointPos.size() / 5]);
        softPwmWrite(GPIO5, jointPos[i + 4*jointPos.size() / 5]);

        std::cout << jointPos[i] << " " << jointPos[i + jointPos.size() / 5] << " " << jointPos[i + 2*jointPos.size() / 5] << " " << jointPos[i + 3*jointPos.size() / 5] << " " << jointPos[i + 4*jointPos.size() / 5] << "\n";
        
        std::this_thread::sleep_for(std::chrono::milliseconds(long(dt*1000))); // duration in milliseconds

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // duration in milliseconds

}
