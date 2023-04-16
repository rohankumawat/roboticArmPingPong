#include <iostream>
#include <sstream>
//#include <opencv2/bgsegm.hpp>
#include <opencv2/opencv.hpp>
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


    //Scalar lowerRange(0, 50, 50);
    //Scalar upperRange(20, 255, 255);
    Scalar lowerRange(8, 96, 115);
    Scalar upperRange(14, 255, 237);

    //Scalar lowerRange(60, 50, 50);
    //Scalar upperRange(90, 255, 255);
    // Create mask based on chosen histogram thresholds
    cv::Mat sliderBW;
    cv::inRange(hsv, lowerRange, upperRange, sliderBW);

    return sliderBW;
}







void get_opencv_result(String video_to_process) {
    VideoCapture capture("D:/MSc/RealTimeEmbedded/CPP/Training/19.mp4");
    //VideoCapture capture(0);

    if (!capture.isOpened()) {
        //error in opening the video input
        cerr << "Unable to open: " << video_to_process << endl;
        return;
    }


    Mat frame, fgMask, object, orgframe;
    Mat noisy_image, denoised_image;
    Scalar objectColor;

    // initialize the BackgroundSubtractor object
    Ptr<BackgroundSubtractor> pBackSub = createBackgroundSubtractorMOG2();

    Mat downsampled_img;
   // Mat frame, fgMask, background, fgMaskNew, foreground;

    cv::Mat filtered_image;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    cv::Mat kernelD = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat dilated_image;
    Mat eroded_img;
    /////////////////////////////////
    while (true) {
       

        // capture frames from video stream
        while (true) {
            capture >> orgframe;

           
            if (orgframe.empty())
                break;
            pyrDown(orgframe, frame);
            // update the background model
            pBackSub->apply(frame, fgMask);

            // extract the moving object using the foreground mask
            frame.copyTo(object, fgMask);

            // obtain the mean color of the moving object
            objectColor = mean(object, fgMask);

            cv::Mat mskBall = createMask(object);

            //////////////////////////////////////////////////// Blob
            
            //cv::medianBlur(mskBall, filtered_image, 3);

            

            // Apply the erosion operation
            
            //erode(mskBall, eroded_img, kernel);

            cv::dilate(mskBall, filtered_image, kernel);
            /////////////////////////////////////// Contours
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours(filtered_image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            vector<Rect> boundRect(contours.size());
            
                


                ////////////////////////////////////////////////////////////// AVERAGE
             if (contours.size() > 1) {
                 cout << contours.size() << "  ";
                // Assuming you have already detected and stored the contours in the vector "contours"

                // Calculate the moments of all contours
                vector<Moments> contourMoments(contours.size());
                for (int i = 0; i < contours.size(); i++) {
                    contourMoments[i] = moments(contours[i]);
                    
                }

                // Calculate the centers of all contours
                vector<Point2f> contourCenters(contours.size());
                for (int i = 0; i < contours.size(); i++) {
                    contourCenters[i] = Point2f(contourMoments[i].m10 / contourMoments[i].m00, contourMoments[i].m01 / contourMoments[i].m00);
                    //cout << isnan(contourCenters[i].x) << "   " << contourCenters[i] << "\n";
                }

                // Calculate the average center of all contours
                Point2f avgCenter(0, 0);
                for (int i = 0; i < contourCenters.size(); i++) {
                    if(!(isnan(contourCenters[i].x)|| isnan(contourCenters[i].y)))
                    avgCenter += contourCenters[i];
                }
                int num = contourCenters.size();
                
                avgCenter = avgCenter / num;
                cout << avgCenter << "\n";
                
            }
            ///////////////////////////////////////////////////////////////////////////////

            //imshow("Foreground mask", fgMask);
            imshow("Moving object", filtered_image);
            object.setTo(0);


            // check for keyboard input
            int key = waitKey(30);
            if (key == 27)
                break;
        }

    }

    
}






int main(int argc, char* argv[])
{
    CommandLineParser parser(argc, argv, input_params);
    // start BS-pipeline
    get_opencv_result(parser.get<String>("input"));

    return 0;
}
