#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>


#include "trajectory_predictor.h"

using namespace cv;
using namespace std;

const char* input_params = "{ input | space_traffic.mp4 | Define the full input video path }";







int main(int argc, char* argv[])
{
    CommandLineParser parser(argc, argv, input_params);

    TrajectoryPredictor cap1 = TrajectoryPredictor(0);
    cap1.getMovingObjects();

    cout << "Done";
    return 0;
}