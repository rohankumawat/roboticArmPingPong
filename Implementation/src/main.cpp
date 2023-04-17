#include <iostream>
#include <thread>
#include <atomic>


/*
#include <sstream>
#include <opencv2/opencv.hpp>
*/


#include "trajectory_predictor.h"
#include "robot_arm.h"


/*
 * Define GPIO --> WiringPi 
 */

#define GPIO1 17	// PIN 11/GPIO0
#define GPIO2 18	// PIN 12/GPIO1
#define GPIO3 23	// PIN 16/GPIO4
#define GPIO4 24	// PIN 18/GPIO5
#define GPIO5 4		// PIN 7/GPIO7


constexpr auto tf = 2;		                  // The motion should complete in 1second;
constexpr auto dt = 0.01;                    // Time steps;
#define M_PI 3.14159265358979323846 





std::vector<double> servoControl(std::vector<double> jointAngles)
{
    // Angle mapping
    std::vector<double> servoAngle(5, 0.0);
    servoAngle[0] = -0.1111 * jointAngles[0] * 180 / M_PI + 25;
    servoAngle[1] = -0.1111 * jointAngles[1] * 180 / M_PI + 15;
    servoAngle[2] = 0.1111 * jointAngles[2] * 180 / M_PI + 5;
    servoAngle[3] = -0.1111 * jointAngles[3] * 180 / M_PI + 25;
    servoAngle[4] = -0.1111 * jointAngles[4] * 180 / M_PI + 15;

    return servoAngle;
}

/*

void servoControl(std::vector<double> jointPos) {

    for (int i = 0; i < 200; i++)
    {

        softPwmWrite(GPIO1, jointPos[i]); // Set the pulse high for 1.5ms/ 1500us (15*100us)
        softPwmWrite(GPIO2, jointPos[i + 200]);
        softPwmWrite(GPIO3, jointPos[i + 400]);
        softPwmWrite(GPIO4, jointPos[i + 600]);
        softPwmWrite(GPIO5, jointPos[i + 800]);

        delay(10);

    }
    delay(2000);
}

*/

int main()
{


    std::vector<double> positionZ;
    std::vector<double> positionY;
    std::vector<double> jointAngle;
    std::vector<double> servoAngle;
    std::vector<double> curAngle(5, 0.0);

    //Create instances of TrajectoryPredictor
    std::unique_ptr<TrajectoryPredictor> cap1 = std::make_unique<TrajectoryPredictor>(0); // SideCamera
    std::unique_ptr<TrajectoryPredictor> cap2 = std::make_unique<TrajectoryPredictor>(1); // BackCamera
    Robot_Arm robotArm(0.128, 0.25, 0.180);

    //while (true)
    //{
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
        std::cout << positionZ[0] << " " << positionZ[1]<< "\n";
        std::cout << positionY[0] << " " << positionY[1] << "\n";
        

        /*
         * Mapping from pixels to meters
         * Resolutuon used:
         */

        robotArm.setPosition(0.25, 0.25, 0.25);
        jointAngle = robotArm.getJointAngles();

        servoAngle = servoControl(jointAngle);
        //Trajectory Generation
        std::vector<double> jointPos;
        jointPos = robotArm.genTrajectory(curAngle, servoAngle, dt, tf);
        for (int i = 0; i < jointAngle.size(); i++)
            std::cout << servoAngle[i] << " ";


    //}
}