#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>


#define TIME_STEP 64
using namespace webots;


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Keyboard kb;
  
  //Initialize motors
  Motor *A1;
  Motor *A2;
  Motor *A3;
  Motor *A4;
  Motor *A5;
     
  A1 = robot->getMotor("m1");
  A2 = robot->getMotor("m2");
  A3 = robot->getMotor("m3");
  A4 = robot->getMotor("m4");
  A5 = robot->getMotor("m5");

  //A1->setPosition(INFINITY);
  //A1->setVelocity(0.0);
  
  kb.enable(TIME_STEP);

  double a1_rotate = 0.0;
  double a2_rotate = 0.0;
  double a3_rotate = 0.0;
  double a4_rotate = 0.0;
  double a5_rotate = 0.0;

  while (robot->step(TIME_STEP) != -1) {
    int key=kb.getKey();
    //std::cout << key <<std::endl;
    if(key == 49 )
      a1_rotate += 0.05;
    else if(key == 50)
      a1_rotate -= 0.05;
      
    if(key == 51)
      a2_rotate += 0.05;      
    else if(key == 52)
      a2_rotate -= 0.05;  
      
    if(key == 53)
      a3_rotate += 0.05;      
    else if(key == 54)
      a3_rotate -= 0.05;  
      
    if(key == 55)
      a4_rotate += 0.05;      
    else if(key == 56)
      a4_rotate -= 0.05;  

    if(key == 57)
      a5_rotate += 0.05;      
    else if(key == 48)
      a5_rotate -= 0.05;
                          
   A1->setPosition(a1_rotate);
   A2->setPosition(a2_rotate);   
   A3->setPosition(a3_rotate);
   A4->setPosition(a4_rotate);
   A5->setPosition(a5_rotate);

  }
  
  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
