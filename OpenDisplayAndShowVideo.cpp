#include "opencv2/opencv.hpp"

int main() {

    // Create video capturing object
    // 0 opens default camera, otherwise filename as argument
    cv::VideoCapture video(0);

    // Check that video is opened
    if (!video.isOpened()) return -1;

    // For saving the frame
    cv::Mat frame;

    // Get video resolution
    int frameWidth = video.get(cv::CAP_PROP_FRAME_WIDTH);
    int frameHeigth = video.get(cv::CAP_PROP_FRAME_HEIGHT);

    // Create video writer object
    cv::VideoWriter output("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(frameWidth, frameHeigth));

    // Loop through available frames
    while (video.read(frame)) {

        // Convert the frame to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur to remove noise
        cv::GaussianBlur(gray, gray, cv::Size(7, 7), 0);

        // Detect circles using HoughCircles function
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 200, 50, 0, 0);

        // Draw circles on the frame
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Vec3i c = circles[i];
            cv::circle(frame, cv::Point(c[0], c[1]), c[2], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }

        // Display the frame
        cv::imshow("Video feed", frame);

        // Write video frame to output
        output.write(frame);

        // For breaking the loop
        if (cv::waitKey(25) >= 0) break;

    } // end while (video.read(frame))

    // Release video capture and writer
    output.release();
    video.release();

    // Destroy all windows
    cv::destroyAllWindows();

    return 0;

}
