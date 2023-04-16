#include <vector>
#include <cmath>

#include "robot_arm.h"

#define M_PI 3.14159265358979323846


/* Mechanical design
 * The robot arm has one degree of freedom less than the conventional 6-DOF robot
 * Spherical wrist minus one degree of freedom
 * The inverse kinematics is calculated under the following assumption:
 * First three joint angles are responsible for positioning the end effector
 * Last two joint angles are responsible for the orientation of the end effector
 *
 */


Robot_Arm::Robot_Arm(double _x, double _y, double _z)
{
	//jointAngles(5, 0.0);
	x = _x;
	y = _y;
	z = _z;

    //a[5] = { 0.0, 0.2, 0.0, 0.0, 0.095 };
    //d[5] = { 0.04, 0.0, 0.0, 0.258, 0.0 };

    flag = false;
}



/*
 * Updates new position for the end effector
 */
void Robot_Arm::setPosition(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
    flag = false;
}



/*
 *Calculates the joint angles required for positioning the end-effector
 */
std::vector<double> Robot_Arm::getPosition()
{

  /*
   * D-H table for forward kinematics
   * Joint angles are to be found out using inverse-kinematics
   * double a[5] = { 0.0, 0.2, 0.0, 0.0, 0.095 };
   * double d[5] = { 0.04, 0.0, 0.0, 0.258, 0.0 };
   * double theta'[5] = { 0, 0, 0, 0, 0 };
   * double alpha[5] = { M_PI / 2, 0, M_PI / 2, -M_PI / 2, 0 };
   */

    //double a[5] = { 0.0, 0.2, 0.0, 0.0, 0.095 };
    //double d[5] = { 0.04, 0.0, 0.0, 0.258, 0.0 };


    flag = true;
    double l1 = sqrt(x * x + y * y);
    double l2 = z - d[0];
    double l3 = sqrt(l1 * l1 + l2 * l2);
    double t[3];


    //std::vector <double> theta(3);


    t[0] = atan2(y, x);
    t[1] = M_PI / 2 - atan2(l2, l1) - acos((a[1] * a[1] + l3 * l3 - d[3] * d[3]) / (2 * a[1] * l3));
    t[2] = acos((l3 * l3 - a[1] * a[1] - d[3] * d[3]) / (2 * a[1] * d[3]));
    jointAngles[0] = t[0];
    jointAngles[1] = t[1];
    jointAngles[2] = t[2];

    return jointAngles;


}



/*
 * Calculates the joint angles required for the desired end - effector orientation
 */
std::vector<double> Robot_Arm::getOrientation() {

    if (flag == false)
        jointAngles = Robot_Arm::getPosition();

    int i, j;

    float determinant = 0;
    double R0_5[3][3] = { {0, 0, -1}, {0, 1, 0 }, {1, 0, 0} };
    double t[3] = { jointAngles[0],jointAngles[1], jointAngles[2] };

    double R0_3[3][3] = { {-cos(t[0]) * cos(t[1]) * sin(t[2]) - cos(t[0]) * sin(t[1]) * cos(t[2]), sin(t[0]), cos(t[0]) * cos(t[1]) * cos(t[2]) - cos(t[0]) * sin(t[1]) * sin(t[2])},
    {-sin(t[0]) * cos(t[1]) * sin(t[2]) - sin(t[0]) * sin(t[1]) * cos(t[2]), -cos(t[0]), sin(t[0]) * cos(t[1]) * cos(t[2]) - sin(t[0]) * sin(t[1]) * sin(t[2])} ,
    {-sin(t[1]) * sin(t[2]) + cos(t[1]) * cos(t[2]), 0, sin(t[1]) * cos(t[2]) + cos(t[1]) * sin(t[2])} };

    double R3_5[3][3] = { {0, 0, 0}, {0, 0, 0 }, {0, 0, 0} };

    /*
     * Final rotation matrix for a 5-DOF robot manipulator --> R0_5 = R0_1 * R1_2 * R2_3 * R3_4 * R4_5
     * The joint angles responsible for orientation can be obtained by finding out R3_5
     * R3_5 = inv(R0_3) * R0_5
     */

    std::vector<std::vector<double>> R0_3Inv(3, std::vector<double>(3, 0));

    for (i = 0; i < 3; i++)
        determinant = determinant + (R0_3[0][i] * (R0_3[1][(i + 1) % 3] * R0_3[2][(i + 2) % 3] - R0_3[1][(i + 2) % 3] * R0_3[2][(i + 1) % 3]));
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++)
            R0_3Inv[i][j] = ((R0_3[(j + 1) % 3][(i + 1) % 3] * R0_3[(j + 2) % 3][(i + 2) % 3]) - (R0_3[(j + 1) % 3][(i + 2) % 3] * R0_3[(j + 2) % 3][(i + 1) % 3])) / determinant;
    }

    // Calculate R3_5

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
            {
                R3_5[i][j] += R0_3Inv[i][k] * R0_5[k][j];
            }

    jointAngles[3] = acos(R3_5[1][2]);
    jointAngles[4] = acos(R3_5[2][0]);


    /*

      for (i = 0; i < 3; i++) {
          cout << "\n";
          for (j = 0; j < 3; j++)
              cout << R3_5[i][j] << "\t";
      }

    */
    return jointAngles;
}

std::vector<double> Robot_Arm::getJointAngles() {

    jointAngles = Robot_Arm::getOrientation();
    return jointAngles;
}


// TRAJECTORY
std::vector<double> Robot_Arm::genTrajectory(std::vector<double> curAngle, std::vector<double> newAngle, double dt, double tf) {

    std::vector<double> timeSteps;
    for (double i = 0.0; i <= tf; i += dt)
    {
        timeSteps.push_back(i);
    }

    std::vector<double> jointPos1, jointPos2, jointPos3, jointPos4, jointPos5, jointPos;

    for (int i = 0; i < 5; i++)
    {
        double a0 = curAngle[i];
        double a1 = 0.0;
        double a2 = 3.0 * (newAngle[i] - curAngle[i]) / pow(tf, 2);
        double a3 = -2.0 * (newAngle[i] - curAngle[i]) / pow(tf, 3);

        for (double j = 0.0; j <= tf; j += dt) {

            double pos = a3 * pow(j, 3) + a2 * pow(j, 2) + a1 * j + a0;
            jointPos.push_back(pos);

        }
    }
    return jointPos;

}