#include <iostream>
#include <sstream>
//#include <opencv2/bgsegm.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace cv;
using namespace std;

const char* input_params = "{ input | space_traffic.mp4 | Define the full input video path }";




cv::Mat createMask(cv::Mat RGB)
{
    // Convert RGB image to chosen color space
    cv::Mat hsv;
    cv::cvtColor(RGB, hsv, cv::COLOR_BGR2HSV);

    // Define thresholds for channel 1 based on histogram settings
    double channel1Min = 160;
    double channel1Max = 179;

    // Define thresholds for channel 2 based on histogram settings
    double channel2Min = 100;
    double channel2Max = 255;

    // Define thresholds for channel 3 based on histogram settings
    double channel3Min = 50;
    double channel3Max = 255;

    Scalar lowerRange(0, 50, 50);
    Scalar upperRange(20, 255, 255);
    // Create mask based on chosen histogram thresholds
    cv::Mat sliderBW;
    cv::inRange(hsv, lowerRange, upperRange, sliderBW);

    return sliderBW;
}




void get_opencv_result(String video_to_process) {
    // create VideoCapture object for further video processing
    VideoCapture capture("test.mp4");
    //VideoCapture capture(0);

    if (!capture.isOpened()) {
        //error in opening the video input
        cerr << "Unable to open: " << video_to_process << endl;
        return;
    }

    // instantiate background subtraction model
   // Ptr<BackgroundSubtractorMOG2> background_subtr_method = createBackgroundSubtractorMOG2();


    Mat frame, fgMask, object;
    Scalar objectColor;

    // initialize the BackgroundSubtractor object
    Ptr<BackgroundSubtractor> pBackSub = createBackgroundSubtractorMOG2();


   // Mat frame, fgMask, background, fgMaskNew, foreground;
    while (true) {
       

        // capture frames from video stream
        while (true) {
            capture >> frame;

            // check whether the frames have been grabbed
            if (frame.empty())
                break;

            // update the background model
            pBackSub->apply(frame, fgMask);

            // extract the moving object using the foreground mask
            frame.copyTo(object, fgMask);

            // obtain the mean color of the moving object
            objectColor = mean(object, fgMask);

            cv::Mat mskBall = createMask(object);

            // display the mean color of the moving object
            cout << "Object color: " << objectColor << endl;

            // display the frames
            //imshow("Frame", frame);
            //imshow("Foreground mask", fgMask);
            //imshow("Moving object", object);
            imshow("Mask", mskBall);

            // check for keyboard input
            int key = waitKey(30);
            if (key == 27)
                break;
        }

        /*
        capture >> frame;

        // check whether the frames have been grabbed
        if (frame.empty())
            break;

        // resize video frames
        resize(frame, frame, Size(640, 360));

        // pass the frame to the background subtractor
        background_subtr_method->apply(frame, fgMask);
        //obtain the background without foreground mask
        background_subtr_method->getBackgroundImage(background);






        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(fgMask, fgMaskNew, MORPH_OPEN, kernel);

        vector<vector<Point>> contours;
        findContours(fgMaskNew, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<Point> ball_contour;
        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area > 100 && area < 500 && contour.size() > 5) {
                // contour is likely to represent the ball
                if (ball_contour.empty() || area > contourArea(ball_contour)) {
                    ball_contour = contour;
                    cout << contour.size() << "\n";
                }
            }
        }

        if (!ball_contour.empty()) {
            Point2f center;
            float radius;
            minEnclosingCircle(ball_contour, center, radius);
            circle(frame, center, static_cast<int>(radius), Scalar(0, 255, 0), 2);
            //cout << "Ball detected";
        }


        // show the current frame, foreground mask, subtracted result
        //imshow("Initial Frames", frame);
        //imshow("Foreground Masks", foreground);
        //imshow("Foreground Masks New", fgMaskNew);

        
        imshow("Subtraction Result", background);

        int keyboard = waitKey(10);
        if (keyboard == 27)
            break;
            */
    }

    
}






int main(int argc, char* argv[])
{
    CommandLineParser parser(argc, argv, input_params);
    // start BS-pipeline
    get_opencv_result(parser.get<String>("input"));

    return 0;
}