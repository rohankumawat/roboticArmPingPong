#ifndef TRAJECTORY_PREDICTOR_H
#define TRAJECTORY_PREDICTOR_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <condition_variable>

class TrajectoryPredictor {
public:


    TrajectoryPredictor(int device_id); // Constructor
    ~TrajectoryPredictor(); // Destructor

    
    std::vector<double>  getPredictedTrajectory(int flag);
    void stopLoop();

private:

    cv::Mat createColorMask(cv::Mat RGB);
    Eigen::MatrixXd side_projectile(Eigen::MatrixXd predict);
    Eigen::MatrixXd front_projectile(Eigen::MatrixXd predict);

    cv::VideoCapture capture;
    cv::Mat frame_;

    std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_stop;
};

#endif // TRAJECTORY_PREDICTOR_H

