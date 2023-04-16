#include <iostream>
#include <Eigen/Dense>

int main()
{
    // Define input data X and output data Y
    Eigen::MatrixXd X(6, 4);
    X << 115.65, 337.501, 323.317, 386.322,
         473.65, 407.733, 537.474, 375.537,
         221.334, 442.08, 279.97, 404.011,
         405.279, 333.191, 466.745, 305.526,
         592.762, 279.095, 658.105, 272.21,
         274.975, 319.252, 643.215, 159.922;

    Eigen::MatrixXd Y(6, 2);
    Y << 404.253, 427.058,
         596.236, 352.481,
         341.338, 367.339,
         526.971, 289.043,
         718.474, 272.891,
         817.109, 130.184;

    // Create polynomial regression model of degree 2
    int degree = 2;
    Eigen::MatrixXd X_poly(X.rows(), X.cols() * (degree + 1));
    for (int i = 0; i < X.rows(); i++) {
        for (int j = 0; j <= degree; j++) {
            for (int k = 0; k < X.cols(); k++) {
                X_poly(i, j * X.cols() + k) = pow(X(i, k), j);
            }
        }
    }
    Eigen::MatrixXd XtX = X_poly.transpose() * X_poly;
    Eigen::MatrixXd XtY = X_poly.transpose() * Y;
    Eigen::MatrixXd w = XtX.ldlt().solve(XtY);

    // Predict new outputs for new input data
    Eigen::MatrixXd new_X(2, 4);
    new_X << 444.5, 267.8, 386.7, 281.2,
             187.3, 395.1, 238.9, 424.6;
    Eigen::MatrixXd new_X_poly(new_X.rows(), new_X.cols() * (degree + 1));
    for (int i = 0; i < new_X.rows(); i++) {
        for (int j = 0; j <= degree; j++) {
            for (int k = 0; k < new_X.cols(); k++) {
                new_X_poly(i, j * new_X.cols() + k) = pow(new_X(i, k), j);
            }
        }
    }
    Eigen::MatrixXd new_Y = new_X_poly * w;

    std::cout << new_Y << std::endl;

    return 0;
}
