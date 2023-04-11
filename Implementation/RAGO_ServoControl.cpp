
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;


// GPIO Used for PWM
#define GPIO1 17	// PIN 11/GPIO0
#define GPIO2 18	// PIN 12/GPIO1
#define GPIO3 23	// PIN 16/GPIO4
#define GPIO4 24	// PIN 18/GPIO5
#define GPIO5 4		// PIN 7/GPIO7

constexpr auto tf = 1;		                  // The motion should complete in 1second;
constexpr auto dt = 0.01;                    // Time steps;
#define M_PI 3.14159265358979323846         // PI




// POSITION
vector<double> getPosition(double x, double y, double z) {


    //double ja[5] = { 0, 0, 0, 0, 0 };
    double a[5] = { 0.0, 0.17275, 0.0, 0.0, 0.095 };
    double d[5] = { 0.04, 0.0, 0.0, 0.258, 0.0 };
    //double alpha[5] = { M_PI / 2, 0, M_PI / 2, -M_PI / 2, 0 };
    double l1 = sqrt(x * x + y * y);
    double l2 = z - d[0];
    double l3 = sqrt(l1 * l1 + l2 * l2);
    //double l[3] = { l1, l2, l3 };
    double t[3];
    vector <double> theta(3);
    t[0] = atan2(y, x);
    t[1] = M_PI / 2 - atan2(l2, l1) - acos((a[1] * a[1] + l3 * l3 - d[3] * d[3]) / (2 * a[1] * l3));
    t[2] = acos((l3 * l3 - a[1] * a[1] - d[3] * d[3]) / (2 * a[1] * d[3]));
    theta[0] = t[0];
    theta[1] = t[1];
    theta[2] = t[2];


    return theta;

}



// ORIENTATION
vector<double> getOrientation(vector<double> t) {
    int i, j;

    float determinant = 0;

    double R0_5[3][3] = { {0, 0, -1}, {0, 1, 0 }, {1, 0, 0} };

    double R0_3[3][3] = {{-cos(t[0]) * cos(t[1]) * sin(t[2]) - cos(t[0]) * sin(t[1]) * cos(t[2]), sin(t[0]), cos(t[0]) * cos(t[1]) * cos(t[2]) - cos(t[0]) * sin(t[1]) * sin(t[2])}, 
    {-sin(t[0]) * cos(t[1]) * sin(t[2]) - sin(t[0]) * sin(t[1]) * cos(t[2]), -cos(t[0]), sin(t[0]) * cos(t[1]) * cos(t[2]) - sin(t[0]) * sin(t[1]) * sin(t[2])} ,
    {-sin(t[1]) * sin(t[2]) + cos(t[1]) * cos(t[2]), 0, sin(t[1]) * cos(t[2]) + cos(t[1]) * sin(t[2])} };
    
    double R3_5[3][3] = { {0, 0, 0}, {0, 0, 0 }, {0, 0, 0} };

    //Inverse of R0_3 
    vector<vector<double>> R0_3Inv(3, std::vector<double>(3, 0));
    for (i = 0; i < 3; i++)
        determinant = determinant + (R0_3[0][i] * (R0_3[1][(i + 1) % 3] * R0_3[2][(i + 2) % 3] - R0_3[1][(i + 2) % 3] * R0_3[2][(i + 1) % 3]));
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++)
            R0_3Inv[i][j] = ((R0_3[(j + 1) % 3][(i + 1) % 3] * R0_3[(j + 2) % 3][(i + 2) % 3]) - (R0_3[(j + 1) % 3][(i + 2) % 3] * R0_3[(j + 2) % 3][(i + 1) % 3])) / determinant ;
    }


 
    // R0_3Inv * R0_5 = R3_5
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
            {
                R3_5[i][j] += R0_3Inv[i][k] * R0_5[k][j];
            }

    t[3] = acos(R3_5[1][2]);
    t[4] = acos(R3_5[2][0]);



 //  for (i = 0; i < 3; i++) {
 //      cout << "\n";
 //      for (j = 0; j < 3; j++)
 //          cout << R3_5[i][j] << "\t";
 //  }
// 
    return t;
}


// TRAJECTORY
vector<double> genTrajectory(vector<double> curAngle, vector<double> newAngle) {

    vector<double> timeSteps;
    for (double i = 0.0; i <= tf; i += dt)
    {
        timeSteps.push_back(i);
    }

    vector<double> jointPos1, jointPos2, jointPos3, jointPos4, jointPos5, jointPos;
    //vector<double> jointVel1, jointVel2, jointVel3,jointVel4,jointVel5, jointVel;

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


// SERVO CONTROL
void servoControl(vector<double> jointPos) {

    for (int i = 0; i < 100; i++)
    {

        softPwmWrite(GPIO1, jointPos[i]); // Set the pulse high for 1.5ms/ 1500us (15*100us)
        softPwmWrite(GPIO2, jointPos[i+100]);
        softPwmWrite(GPIO3, jointPos[i+200]);
        softPwmWrite(GPIO4, jointPos[i+300]);
        softPwmWrite(GPIO5, jointPos[i+400]);

        delay(10);

    }
    delay(2000);
}
int main() {


    if (wiringPiSetupGpio() < 0) return 1;
    


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


    std::vector<double> curAngle(5, 0.0);
    vector<double> jointPos;
    vector<double> temp;
    double pos[3] = { 0.30, 0.128, 0.180 };
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];

    // Get position
    double theta[5] = { 0,0,0,0,0 };
    temp = getPosition(x, y, z);
    theta[0] = temp[0];
    theta[1] = temp[1];
    theta[2] = temp[2];

    // Get orientation
    std::vector<double> t(theta, theta + 5);
    temp = getOrientation(t);
    theta[3] = temp[3];
    theta[4] = temp[4];

    // Gen trajectory
    jointPos = genTrajectory(curAngle, temp);

    // Actuate servos
    servoControl(jointPos);

    
    for (int i = 0; i < 5; i++)
        cout << theta[i] << "\n";
    cout << "NEXT" << "\n\n\n";

    for(int i =0; i<jointPos.size(); i++)
        cout << jointPos[i] << "\n";

}
