#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <thread>
/*
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
*/

#include "trajectory_predictor.h"

using namespace cv;
using namespace std;

const char* input_params = "{ input | space_traffic.mp4 | Define the full input video path }";







int main()
{
    //CommandLineParser parser(argc, argv, input_params);

    std::thread t1([&]() {
        TrajectoryPredictor cap1 = TrajectoryPredictor(0);
        cap1.getMovingObjects();
        });

    std::thread t2([&]() {
        TrajectoryPredictor cap2 = TrajectoryPredictor(1);
        cap2.getMovingObjects();
        });
    
    //cap1.getMovingObjects();
    cout << "Cam1 \n";
    t1.join();
    //cap2.getMovingObjects();
    cout << "Cam2 \n";
    t2.join();

    cout << "Done";
    return 0;
}
