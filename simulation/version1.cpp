#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

int main() {
    double M_PI = 3.14;
    double ja[5] = { 0, 0, 0, 0, 0 };
    double a[5] = { 0.0, 0.18, 0.0, 0.0, 0.1 };
    double d[5] = { 0.05, 0.0, 0.0, 0.28, 0.0 };
    double alpha[5] = { M_PI / 2, 0, M_PI / 2, -M_PI / 2, 0 };
    double pos[2][3] = { {0.4, 0.0, 0.145}, {0.30, 0.128, 0.180} };
    double tf = 2.0;
    vector<double> t;
    for (double i = 0; i <= tf; i += 0.1) {
        t.push_back(i);
    }
    //double angles[2][3];
    vector<vector<double>> angles_vec;
    for (int i = 0; i < 2; i++) {
        double x = pos[i][0];
        double y = pos[i][1];
        double z = pos[i][2];
        double l1 = sqrt(x * x + y * y);
        double l2 = z - d[0];
        double l3 = sqrt(l1 * l1 + l2 * l2);
        double l[3] = { l1, l2, l3 };
        double t[3];
        double angle[3];
        t[0] = atan2(y, x);
        t[1] = atan2(l2, l1) - acos((a[1] * a[1] + l3 * l3 - d[3] * d[3]) / (2 * a[1] * l3));
        t[2] = acos((l3 * l3 - a[1] * a[1] - d[3] * d[3]) / (2 * a[1] * d[3]));
        angle[0] = t[0] * 180 / M_PI;
        angle[1] = t[1] * 180 / M_PI;
        angle[2] = t[2] * 180 / M_PI;
        vector<double> angle_vec;
        angle_vec.push_back(angle[0]);
        angle_vec.push_back(angle[1]);
        angle_vec.push_back(angle[2]);
        angles_vec.push_back(angle_vec);
    }
    // assuming `angles_vec` is your vector<vector<double>> variable
    for (const auto& inner_vec : angles_vec) {
        for (const auto& angle : inner_vec) {
            std::cout << angle << " ";
        }
        std::cout << std::endl;
    }
    // Iterate over each pair of angles
    for (int i = 0; i < 2; i++) {
        //cout << "size = " << angles_vec[1].size() << endl;
        // Calculate the coefficients for the cubic polynomial
        double a0 = angles_vec[i][0];
        double a1 = 0;
        double a2 = 3 * (angles_vec[1][i] - angles_vec[0][i]) / pow(tf, 2);
        double a3 = -2 * (angles_vec[1][i] - angles_vec[0][i]) / pow(tf, 3);
        cout << "1 = " << angles_vec[1][i] << ", 2 = " << angles_vec[0][i] << endl;

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
        cout << "Joint velocity for angles " << angles_vec[0][i] << " to " << angles_vec[1][i] << ":" << endl;
        for (int j = 0; j < t.size(); j++) {
            cout << "t = " << t[j] << ", vel = " << jointVel[j] << endl;
        }
    }

    return 0;
}
