#ifndef TRAJECTORY_PREDICTOR_H
#define TRAJECTORY_PREDICTOR_H

#include <opencv2/opencv.hpp>

class TrajectoryPredictor {
public:


    TrajectoryPredictor(int device_id); // Constructor
    ~TrajectoryPredictor(); // Destructor

    
    void getMovingObjects();
private:

    cv::Mat createColorMask(cv::Mat RGB);
    

    cv::VideoCapture capture;
    cv::Mat frame_;
};

#endif // TRAJECTORY_PREDICTOR_H

