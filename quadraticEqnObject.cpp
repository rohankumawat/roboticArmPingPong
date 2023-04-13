#include <opencv2/opencv.hpp>
#include <iostream>
#include <armadillo>

using namespace cv;
using namespace std;
using namespace arma;

int main()
{
    // Capture video from the default camera
    VideoCapture cap(0);

    if (!cap.isOpened())
    {
        cout << "Error: Could not open video capture device" << endl;
        return -1;
    }

    // Define the color range for object detection
    Scalar lower_color_range = Scalar(110, 70, 50);
    Scalar upper_color_range = Scalar(130, 255, 255);

    while (true)
    {
        Mat frame;
        bool success = cap.read(frame);

        if (!success)
        {
            cout << "Error: Could not read frame from video capture device" << endl;
            break;
        }

        // Convert the frame from BGR to HSV color space
        Mat hsv_frame;
        cvtColor(frame, hsv_frame, COLOR_BGR2HSV);

        // Create a mask based on the color range
        Mat mask;
        inRange(hsv_frame, lower_color_range, upper_color_range, mask);

        // Apply the mask to the original frame to extract the object
        Mat object;
        bitwise_and(frame, frame, object, mask);

        // Find contours in the mask to get the boundary of the object
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Draw the boundary of the object on the original frame
        if (!contours.empty())
        {
            drawContours(frame, contours, 0, Scalar(0, 255, 0), 2);
            // Get the center of the object
            Moments mu = moments(contours[0]);
            Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
            circle(frame, center, 5, Scalar(0, 0, 255), -1);

            double x = center.x;
            double y = center.y;

            // Store the x and y coordinates in vectors
            static vec x_data, y_data;
            x_data.insert_rows(x_data.n_elem, 1);
            x_data(x_data.n_elem - 1) = x;
            y_data.insert_rows(y_data.n_elem, 1);
            y_data(y_data.n_elem - 1) = y;

            // Fit a quadratic polynomial to the x and y data
            int n = x_data.n_elem;
            mat X(n, 3);
            X.col(0) = ones(n);
            X.col(1) = x_data;
            X.col(2) = square(x_data);
            vec coeffs = solve(X.t() * X, X.t() * y_data);

            // Evaluate the quadratic polynomial at x
            double a = coeffs(0);
            double b = coeffs(1);
            double c = coeffs(2);
            double y_fit = a + b * x + c * x * x;

            cout << "x: " << x << ", y: " << y << ", y_fit: " << y_fit << endl;
        }

        // Display the extracted object
        imshow("Extracted Object", frame);

        // Press ESC to exit the program
        if (waitKey(1) == 27)
        {
            break;
        }
    }

    return 0;
}
