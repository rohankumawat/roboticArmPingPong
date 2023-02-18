// File:          my_controller1.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#define TIME_STEP 64
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Keyboard kb;
  
  Motor *rm1,*rm2;
  rm1=robot->getMotor("RM1");
  rm2=robot->getMotor("RM2");

  kb.enable(TIME_STEP);
  double rotate=0.0;
  while (robot->step(TIME_STEP) != -1) {
    int key=kb.getKey();
    
    if (key==65 && rotate<1.57){
    rotate += 0.05;
    } else if (key==68 && rotate>-1.57){
    rotate += -0.05;
    }else {
    rotate+=0;
    }
    rm1->setPosition(rotate);
    if (key==55 && rotate<1.57){
    rotate += 0.05;
    } else if (key==68 && rotate>-1.57){
    rotate += -0.05;
    }else {
    rotate+=0;
    }
    rm2->setPosition(rotate);
}
  delete robot;
  return 0;
}
