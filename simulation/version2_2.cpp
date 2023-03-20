#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <iostream>
#include <cmath>
#include <vector>

#define TIME_STEP 3200
#define M_PI 3.14159265358979323846

using namespace std;
using namespace webots;

vector<double> getAngle(double x, double y, double z) {
    //double ja[5] = { 0, 0, 0, 0, 0 };
    double a[5] = { 0.0, 0.18, 0.0, 0.0, 0.1 };
    double d[5] = { 0.05, 0.0, 0.0, 0.28, 0.0 };
    //double alpha[5] = { M_PI / 2, 0, M_PI / 2, -M_PI / 2, 0 };
    double l1 = sqrt(x * x + y * y);
    double l2 = z - d[0];
    double l3 = sqrt(l1 * l1 + l2 * l2);
    //double l[3] = { l1, l2, l3 };
    double t[3];
    vector <double> angle(3);
    t[0] = atan2(y, x);
    t[1] = atan2(l2, l1) - acos((a[1] * a[1] + l3 * l3 - d[3] * d[3]) / (2 * a[1] * l3));
    t[2] = acos((l3 * l3 - a[1] * a[1] - d[3] * d[3]) / (2 * a[1] * d[3]));
    angle[0] = t[0] * 180 / M_PI;
    angle[1] = t[1] * 180 / M_PI;
    angle[2] = t[2] * 180 / M_PI;
    return angle;
}

int main() {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  Motor *joint1 = robot->getMotor("m1");
  Motor *joint2 = robot->getMotor("m2");
  Motor *joint3 = robot->getMotor("m3");
  joint1->setPosition(0);
  joint2->setPosition(0);
  joint3->setPosition(0);
  
  vector<double> angle_1 = getAngle(0.4, 0.0, 0.145);
  vector<double> angle_2 = getAngle(0.30, 0.128, 0.180);
  
  double tf = 2.0;
  double dt = 0.1;
  vector<double> t;
  for (double i = 0.0; i <= tf; i += dt) {
    t.push_back(i);
  }
  
  // initialize joint variable
  vector<double> jointPos1, jointPos2, jointPos3, jointPos;
  
  for (int i = 0; i < static_cast<int>(angle_1.size()); i++) {
    double a0 = angle_1[i];
    double a1 = 0.0;
    double a2 = 3.0 * (angle_2[i] - angle_1[i]) / pow(tf, 2);
    double a3 = -2.0 * (angle_2[i] - angle_1[i]) / pow(tf, 3);
  
    for (double j = 0.0; j <= tf; j += dt) {
    //jointVel.push_back(3.0 * a3 * pow(j, 2) + 2.0 * a2 * j + a1);
    //jointAcc.push_back(6.0 * a3 * j + 2.0 * a2);
      double pos = a3 * pow(j, 3) + a2 * pow(j, 2) + a1 * j + a0;
      // jointPos.push_back(pos);
      jointPos.push_back(pos); // joint 1 position
    }
   }
   
  for (int i = 0; i < 20; i++) {
     jointPos1.push_back(jointPos[i]);
  }
  for (int i = 21; i < 40; i++) {
      jointPos2.push_back(jointPos[i]);
  }
  for (int i = 41; i < 60; i++) {
      jointPos3.push_back(jointPos[i]);
  }
   
   // cout << "Vector v: ";
   // for (int i = 0; i < static_cast<int>(jointPos1.size()); i++) {
            // cout << jointPos1[i] << " ";
        // }
        // cout << endl;
  
  while (robot->step(TIME_STEP) != -1) {
    for(int i = 0; i < static_cast<int>(jointPos1.size()); i++) {
      joint1->setPosition(jointPos1[i]);
      joint2->setPosition(jointPos2[i]);
      joint3->setPosition(jointPos3[i]);
      cout << jointPos2[i] << endl;
    }
  }
  
  delete robot;
  return 0;
}
