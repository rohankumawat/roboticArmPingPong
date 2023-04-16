#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/types.hpp>


#include "trajectory_predictor.h"



TrajectoryPredictor::TrajectoryPredictor(int device_id) :capture(device_id)
{

    capture.open(device_id, cv::CAP_DSHOW);

    if (!capture.isOpened()) {
        std::cerr << "Unable to open device " << device_id << std::endl;
        throw std::runtime_error("Failed to open camera");
    }
}

TrajectoryPredictor::~TrajectoryPredictor() {

 // destructor
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
    cv::Scalar lowerRange(157, 73, 168);
    cv::Scalar upperRange(171, 138, 255);

    // Create mask based on chosen histogram thresholds
    cv::Mat colorFilteredImage;
    cv::inRange(hsv, lowerRange, upperRange, colorFilteredImage);

    return colorFilteredImage;
}



/*
 * Process the frames from video capture to get the moving objects filtered by color
 * The program looks for a moving pink object
 */

void TrajectoryPredictor::getMovingObjects()
{

    cv::Mat orgFrame, frame, fgMask, object, filtered_image, dilated_image;
    cv::Scalar objectColor;


    // Kernel definition for noise filter (dialation operation).
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));


    /*
     * Background subtractor to eliminate the stationary background from the frames
     * Generates the moving object
     */
    cv::Ptr<cv::BackgroundSubtractor> pBackSub = cv::createBackgroundSubtractorMOG2();
    cv::Size output_size(640, 480);

    while (true) {

        capture >> orgFrame;
        if (orgFrame.empty())
            break;


        //cv::pyrDown(orgFrame, frame);
        cv::resize(orgFrame, frame, output_size);

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
            std::cout << contours.size() << "  ";
            std::vector<cv::Moments> contourMoments(contours.size());
            for (int i = 0; i < contours.size(); i++) 
                contourMoments[i] = moments(contours[i]);
            

            // Calculate the centers of all contours
            std::vector<cv::Point2f> contourCenters(contours.size());
            for (int i = 0; i < contours.size(); i++) {
                contourCenters[i] = cv::Point2f(contourMoments[i].m10 / contourMoments[i].m00, contourMoments[i].m01 / contourMoments[i].m00);
            }

            cv::Point2f avgCenter(0, 0);
            for (int i = 0; i < contourCenters.size(); i++) {
                if (!(isnan(contourCenters[i].x) || isnan(contourCenters[i].y)))
                    avgCenter += contourCenters[i];
            }
            int num = contourCenters.size();
            avgCenter = avgCenter / num;
            std::cout << avgCenter << "\n";
            if (avgCenter.x > 500)
                break;
        }
        object.setTo(0);

        //imshow("Foreground mask", fgMask);
        //imshow("Moving object", filtered_image);

    }
}
