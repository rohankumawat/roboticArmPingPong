#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/types.hpp>
#include <Eigen/Dense>
#include <condition_variable>


#include "trajectory_predictor.h"



TrajectoryPredictor::TrajectoryPredictor(int device_id) :capture(device_id)
{

    capture.open(device_id);

    if (!capture.isOpened()) {
        std::cerr << "Unable to open device " << device_id << std::endl;
        throw std::runtime_error("Failed to open camera");
    }

     m_mutex;
     m_cv;
     m_stop = false;

}

TrajectoryPredictor::~TrajectoryPredictor() {

 // destructor
}






void TrajectoryPredictor::stopLoop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_stop = true;
    m_cv.notify_one();
}




/*
 * Function to create a color mask to filter the required color from the frame
 * Set to retreive orange color from the frame
 */

cv::Mat TrajectoryPredictor::createColorMask(cv::Mat _image_RGB)
{
    // Convert RGB image to HSV image
    cv::Mat hsv;
    cv::cvtColor(_image_RGB, hsv, cv::COLOR_BGR2HSV);


    // HSV range for Orange color
    //cv::Scalar lowerRange(8, 96, 115);
    //cv::Scalar upperRange(14, 255, 237);


    //HSV range for Pink color
    //cv::Scalar lowerRange(157, 73, 168);
    //cv::Scalar upperRange(171, 138, 255);
    cv::Scalar lowerRange(144, 111, 234);
    cv::Scalar upperRange(165, 255, 255);

    // Create mask based on chosen histogram thresholds
    cv::Mat colorFilteredImage;
    cv::inRange(hsv, lowerRange, upperRange, colorFilteredImage);

    return colorFilteredImage;
}



/*
 * Process the frames from video capture to get the moving objects filtered by color
 * The program looks for a moving pink object
 */

std::vector<double> TrajectoryPredictor::getPredictedTrajectory(int flag)
{

    std::vector<double> prediction(2, 0.0);

    cv::Point2f temp(0, 0);
    cv::Point2f avgCenter(0, 0);

    cv::Mat orgFrame, frame, fgMask, object, filtered_image, dilated_image;
    cv::Scalar objectColor;


    // Kernel definition for noise filter (dialation operation).
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));


    /*
     * Background subtractor to eliminate the stationary background from the frames
     * Generates the moving object
     */
    cv::Ptr<cv::BackgroundSubtractor> pBackSub = cv::createBackgroundSubtractorMOG2();
    cv::Size output_size(640, 360);

    while (!m_stop) {

        capture >> orgFrame;
        if (orgFrame.empty())
            break;


        //cv::pyrDown(orgFrame, frame);
        cv::resize(orgFrame, frame, output_size);
	//std::cout<< frame.size()<< " \n";
        // Extraction of moving objects 
        pBackSub->apply(frame, fgMask);
        frame.copyTo(object, fgMask);
        objectColor = cv::mean(object, fgMask);

        // Filtering the color followed by noise removal
        cv::Mat mskBall = createColorMask(object);
        cv::dilate(mskBall, filtered_image, kernel);

        // Contour detection
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(filtered_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Finding the mean centroid of the object
        if (contours.size() > 1) {
            temp = avgCenter;
            avgCenter.x = 0;
	    avgCenter.y = 0;
            std::cout << contours.size() << " \n ";
            std::vector<cv::Moments> contourMoments(contours.size());
            for (int i = 0; i < contours.size(); i++) 
                contourMoments[i] = moments(contours[i]);
            
  	
            // Calculate the centers of all contours
            std::vector<cv::Point2f> contourCenters(contours.size());
            for (int i = 0; i < contours.size(); i++) {
                contourCenters[i] = cv::Point2f(contourMoments[i].m10 / contourMoments[i].m00, contourMoments[i].m01 / contourMoments[i].m00);
            }

            
            for (int i = 0; i < contourCenters.size(); i++) {
                if (!(isnan(contourCenters[i].x) || isnan(contourCenters[i].y)))
                    avgCenter += contourCenters[i];
            }
		
            int num = contourCenters.size();
            
            avgCenter = avgCenter / num;

            std::cout << avgCenter << "\n";

            if (avgCenter.x > 400 && flag == 0)
                break;

        }
        object.setTo(0);
        //std::cout << "Test";
        //imshow("Foreground mask", fgMask);
        imshow("Moving object", filtered_image);
        int key = cv::waitKey(30);
        if (key == 27)
            break;

        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait_for(lock, std::chrono::milliseconds(10), [this]() { return m_stop; });


    }


    Eigen::MatrixXd predict(1, 4);
    predict << temp.x, temp.y, avgCenter.x, avgCenter.y;

    if (flag == 0)
    {
        Eigen::MatrixXd weight = side_projectile(predict);

        int degree = 3;
        Eigen::MatrixXd new_X_poly(predict.rows(), predict.cols() * (degree + 1));

        for (int i = 0; i < predict.rows(); i++) {
            for (int j = 0; j <= degree; j++) {
                for (int k = 0; k < predict.cols(); k++) {
                    new_X_poly(i, j * predict.cols() + k) = pow(predict(i, k), j);
                }
            }
        }
        Eigen::MatrixXd new_Y = new_X_poly * weight;
        prediction = {avgCenter.x, avgCenter.y, new_Y (0,0), new_Y(0,1)};

       // std::cout << "\nPrediction \n";
        //std::cout << new_Y << std::endl;
    }
    else if (flag == 1)
    {
        Eigen::MatrixXd weight = front_projectile(predict);

        int degree = 1;
        Eigen::MatrixXd new_X_poly(predict.rows(), predict.cols() * (degree + 1));

        for (int i = 0; i < predict.rows(); i++) {
            for (int j = 0; j <= degree; j++) {
                for (int k = 0; k < predict.cols(); k++) {
                    new_X_poly(i, j * predict.cols() + k) = pow(predict(i, k), j);
                }
            }
        }
        Eigen::MatrixXd new_Y = new_X_poly * weight;
        prediction = {avgCenter.x, avgCenter.y, new_Y(0,0), new_Y(0,1) };

        //std::cout << "\nPrediction \n";
        //std::cout << new_Y << std::endl;
    }

    return prediction;
}




Eigen::MatrixXd TrajectoryPredictor::side_projectile(Eigen::MatrixXd predict) {
    //Training Model
    Eigen::MatrixXd X(14, 4);
    X << 564.852, 202.101, 513.882, 244.132,
        366.706, 314.1, 182.149, 206.435,
        182, 206.435, 152.92, 175.879,
        624.496, 177.91, 490.422, 332.452,
        490.422, 332.452, 441.557, 345.255,
        441.557, 345.255, 173.012, 179.152,
        173.012, 179.152, 111.974, 190.435,
        111.974, 190.435, 97.2786, 198.512,
        564.356, 222.481, 570.741, 269.415,
        570.741, 269.415, 209.995, 112.817,
        209.995, 112.817, 81.5716, 80.9702,
        561.644, 215.874, 425.941, 352.155,
        487.5, 250, 111.594, 247.672,
        392.171, 337.589, 179.059, 159.997;




    Eigen::MatrixXd Y(14, 2);
    Y << 366.706, 314.1,
        152.92, 175.879,
        92.6817, 155.952,
        441.557, 345.255,
        173.012, 179.152,
        111.974, 190.435,
        97.2786, 198.512,
        12.9643, 312.481,
        209.995, 112.817,
        81.5716, 80.9702,
        57.0452, 76.0762,
        179.828, 179.824,
        22.2208, 169.888,
        39.9424, 117.165;

    // Create polynomial regression model of degree 2
    int degree = 3;
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

    return w;

}


Eigen::MatrixXd TrajectoryPredictor::front_projectile(Eigen::MatrixXd predict) {
    Eigen::MatrixXd X_big_balls_front(10, 4);
    X_big_balls_front <<
        315.534, 133.75, 388.146, 335.254,
        388.146, 335.254, 400.986, 331.841,
        400.986, 331.841, 419.083, 260.823,
        298.856, 134.659, 309.519, 135.879,
        309.519, 135.879, 335.141, 194.729,
        320.895, 128.21, 215.107, 326.84,
        247.3, 127.643, 247.232, 129.786,
        246.311, 139.012, 271.561, 199.165,
        271.561, 199.165, 311.331, 336.255,
        289.672, 319.989, 250.973, 261.383;

    Eigen::MatrixXd Y_big_balls_front(10, 2);
    Y_big_balls_front <<
        400.986, 331.841,
        419.083, 260.823,
        439.459, 204.612,
        335.141, 194.729,
        175.612, 292.873,
        178.711, 199.436,
        246.883, 136.138,
        311.331, 336.255,
        330.804, 271.217,
        254.564, 209.719;

    // Create polynomial regression model of degree 2
    int degree = 1;
    Eigen::MatrixXd X_poly(X_big_balls_front.rows(), X_big_balls_front.cols() * (degree + 1));
    for (int i = 0; i < X_big_balls_front.rows(); i++) {
        for (int j = 0; j <= degree; j++) {
            for (int k = 0; k < X_big_balls_front.cols(); k++) {
                X_poly(i, j * X_big_balls_front.cols() + k) = pow(X_big_balls_front(i, k), j);
            }
        }
    }


    Eigen::MatrixXd XtX = X_poly.transpose() * X_poly;
    Eigen::MatrixXd XtY = X_poly.transpose() * Y_big_balls_front;
    Eigen::MatrixXd w = XtX.ldlt().solve(XtY);

    return w;

}
