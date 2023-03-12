#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

int main() {
    // Define the parameters
    double M_PI  = 3.14;
    double tf = 2.0;
    vector<double> t;
    for (double i = 0; i <= tf; i += 0.1) {
        t.push_back(i);
    }

    // Define the initial and final angles for each pair
    vector<double> theta1 = { 0, M_PI / 4, M_PI / 3, M_PI / 2, M_PI / 1.5 };
    vector<double> theta2 = { 0, M_PI / 4, M_PI / 3, M_PI / 2, M_PI / 2 };

    // Iterate over each pair of angles
    for (int i = 0; i < theta1.size(); i++) {
        // Calculate the coefficients for the cubic polynomial
        double a0 = theta1[i];
        double a1 = 0;
        double a2 = 3 * (theta2[i] - theta1[i]) / pow(tf, 2);
        double a3 = -2 * (theta2[i] - theta1[i]) / pow(tf, 3);

        // Calculate the joint velocity and acceleration
        vector<double> jointVel;
        vector<double> jointAcc;
        for (int j = 0; j < t.size(); j++) {
            double vel = 3 * a3 * pow(t[j], 2) + 2 * a2 * t[j] + a1;
            jointVel.push_back(vel);

            double acc = 6 * a3 * t[j] + 2 * a2;
            jointAcc.push_back(acc);
        }

        // Plot the joint velocity as a function of time
        cout << "Joint velocity for angles " << theta1[i] << " to " << theta2[i] << ":" << endl;
        for (int j = 0; j < t.size(); j++) {
            cout << "t = " << t[j] << ", vel = " << jointVel[j] << endl;
        }
    }

    return 0;
}
