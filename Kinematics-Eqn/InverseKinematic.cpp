#include <iostream>
#include <cmath>

using namespace std;

int main() {
    double M_PI = 3.14;
    double ja[5] = { 0, 0, 0, 0, 0 };
    double a[5] = { 0.0, 0.18, 0.0, 0.0, 0.1 };
    double d[5] = { 0.05, 0.0, 0.0, 0.28, 0.0 };
    double alpha[5] = { M_PI / 2, 0, M_PI / 2, -M_PI / 2, 0 };
    double pos[2][3] = { {0.4, 0.0, 0.145}, {0.30, 0.128, 0.180} };

    for (int i = 0; i < 2; i++) {
        double x = pos[i][0];
        double y = pos[i][1];
        double z = pos[i][2];
        double l1 = sqrt(x * x + y * y);
        double l2 = z - d[0];
        double l3 = sqrt(l1 * l1 + l2 * l2);
        double l[3] = { l1, l2, l3 };
        double t[6];
        t[0] = atan2(y, x);
        t[1] = atan2(l2, l1);
        t[2] = acos((a[1] * a[1] + l3 * l3 - d[3] * d[3])/(2 * a[1] * l3));
        t[3] = acos((l3 * l3 - a[1] * a[1] - d[3] * d[3]) / (2 * a[1] * d[3]));
        std::cout << "T value: " << t[0] << std::endl;
        std::cout << "T value: " << t[1] << std::endl;
        std::cout << "T value: " << t[2]<< std::endl;
        double angle[3];
        angle[0] = t[0] * 180 / M_PI;
        angle[1] = t[1] * 180 / M_PI;
        angle[2] = t[2] * 180 / M_PI;
        cout << "Joint angles in degrees for position " << i + 1 << ": " << angle[0] << " " << angle[1] << " " << angle[2] << endl;
    }
    return 0;
}
