
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
#include "mapping.h"

constexpr auto tf = 2.0;		                  // The motion should complete in 1second;
constexpr auto dt = 0.01;                         // Time steps;

#define M_PI 3.14159265358979323846 


std::vector<double> jointAngle;
std::vector<double> servoAngle;
std::vector<double> curAngle = {25,15,5,15,25};





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

    std::vector<double> pos(3,0.25);
   


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


        //std::cout << "\nPrediction:\n ";
        //std::cout << positionZ[0] << " " << positionZ[1] << " " << positionZ[2]<< " " << positionZ[3] << "\n";
        //std::cout << positionY[0] << " " << positionY[1] << " " << positionY[2]<< " " << positionY[3]<< "\n";

        /*
         * Mapping from pixels to meters
         * Resolutuon used:
         */
        pos = getMapping(positionZ, positionY);

        std::cout << pos[0] << " " << pos[1] << " " << pos[2]<<"\n";

        robotArm.setPosition(pos[0], pos[1], pos[2]);

        std::thread t3(run, std::ref(robotArm));

        t3.join();


        int key = cv::waitKey(30);
        if (key == 27) {
        // Terminate the main function if the "Esc" key is pressed
        return 0;
    }
    }
}
