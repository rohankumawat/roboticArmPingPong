#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

vector<double> getAngle(double x, double y, double z) {
    double M_PI = 3.14;
    double ja[5] = { 0, 0, 0, 0, 0 };
    double a[5] = { 0.0, 0.18, 0.0, 0.0, 0.1 };
    double d[5] = { 0.05, 0.0, 0.0, 0.28, 0.0 };
    double alpha[5] = { M_PI / 2, 0, M_PI / 2, -M_PI / 2, 0 };
    double l1 = sqrt(x * x + y * y);
    double l2 = z - d[0];
    double l3 = sqrt(l1 * l1 + l2 * l2);
    double l[3] = { l1, l2, l3 };
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
        double tf = 2.0;
        double dt = 0.1;
        vector<double> t;
        for (double i = 0.0; i <= tf; i += dt) {
            t.push_back(i);
        }

        vector<double> angle_1 = getAngle(0.4, 0.0, 0.145);
        vector<double> angle_2 = getAngle(0.30, 0.128, 0.180);

        for (int i = 0; i < angle_1.size(); i++) {
            double a0 = angle_1[i];
            double a1 = 0.0;
            double a2 = 3.0 * (angle_2[i] - angle_1[i]) / pow(tf, 2);
            double a3 = -2.0 * (angle_2[i] - angle_1[i]) / pow(tf, 3);

            vector<double> jointVel;
            vector<double> jointAcc;
            for (double j = 0.0; j <= tf; j += dt) {
                jointVel.push_back(3.0 * a3 * pow(j, 2) + 2.0 * a2 * j + a1);
                jointAcc.push_back(6.0 * a3 * j + 2.0 * a2);
            }

            cout << "Joint velocity for angles " << angle_1[i] << " to " << angle_2[i] << endl;
            for (int k = 0; k < jointVel.size(); k++) {
                cout << t[k] << "\t" << jointVel[k] << endl;
            }
        }

        return 0;
    }
