#include <iostream>
#include <thread>
#include <atomic>
#include <wiringPi.h>
#include <softPwm.h>
#include <cmath>
#include <vector>

/*
 * User defined header files
 */

#include "trajectory_predictor.h"
#include "robot_arm.h"
#include "servo_func.h"


constexpr auto tf = 2.0;		                  // The motion should complete in 1second;
constexpr auto dt = 0.01;                         // Time steps;

#define M_PI 3.14159265358979323846 


std::vector<double> jointAngle;
std::vector<double> servoAngle;
std::vector<double> curAngle(5, 0.0);





void run(Robot_Arm& robotArm) {


    jointAngle = robotArm.getJointAngles();
    servoAngle = servoAngleMap(jointAngle);
    std::vector<double> jointPos;
    jointPos = robotArm.genTrajectory(curAngle, servoAngle, dt, tf);
    servoWrite(jointPos, dt, tf);

}



int main()
{

    void pwmDefine();

    std::vector<double> positionZ;
    std::vector<double> positionY;




    //Create instances of TrajectoryPredictor

    std::unique_ptr<TrajectoryPredictor> cap1 = std::make_unique<TrajectoryPredictor>(2); // SideCamera
    std::unique_ptr<TrajectoryPredictor> cap2 = std::make_unique<TrajectoryPredictor>(0); // BackCamera


    Robot_Arm robotArm(0.128, 0.25, 0.180);


    while (true)
    {

        // Initiate the trajectory prediction
        std::thread t1([&]() {
            positionZ = cap1->getPredictedTrajectory(0);
            });


        std::thread t2([&]() {
            positionY = cap2->getPredictedTrajectory(1);
            });


        t1.join();

        cap2->stopLoop();

        t2.join();


        std::cout << positionZ[0] << "\nPrediction:\n ";
        std::cout << positionZ[0] << " " << positionZ[1] << "\n";
        std::cout << positionY[0] << " " << positionY[1] << "\n";

        /*
         * Mapping from pixels to meters
         * Resolutuon used:
         */


         robotArm.setPosition(0.25, 0.25, 0.25);

         std::thread t3(run, std::ref(robotArm));

         t3.join();
         
    }
}
