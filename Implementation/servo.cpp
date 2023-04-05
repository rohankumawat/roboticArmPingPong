// CODE updated to include the trajectory generation for two motors. 
// The motors 1 and 2 are rotated 90 degrees counter-clockwise and clockwise respectively(controlled velocity)
// Needs to smoothen out the trajectory to avoid jitter
// The last pwmWrite resets the position back to zero degrees (default velocity)

#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

#define GPIO1 4		// PIN 7/GPIO7
#define GPIO2 23	// PIN 16/GPIO4

int main(int argc, char *argv[])
{


   if (wiringPiSetupGpio() < 0) return 1;
   pinMode(GPIO2, OUTPUT);
   digitalWrite(GPIO2, LOW);

   pinMode(GPIO1, OUTPUT);
   digitalWrite(GPIO1, LOW);

   softPwmCreate(GPIO1, 0, 200); // Create a pulse of frequency 50Hz (20ms => each step is 100us. 200 steps therefore generates a pulse of period 20ms)
   softPwmCreate(GPIO2, 0, 200); // Create a pulse of frequency 50Hz (20ms => each step is 100us. 200 steps therefore generates a pulse of period 20ms)



   double curAngle[2] = {15,15};
   double newAngle[2] = {5,25};


   double tf = 1;		 // The motion should complete in 1second
   double dt = 0.1; 	 //Timestep

   vector<double> timeSteps;
   for (double i = 0.0; i <= tf; i += dt) 
   {
    	timeSteps.push_back(i);
   }

   vector<double> jointPos1, jointPos2, jointPos;
   for(int i = 0; i < 2; i++)
   {
	double a0 = curAngle[i];
	double a1 = 0.0;
	double a2 = 3.0 * (newAngle[i] - curAngle[i]) / pow(tf, 2);
	double a3 = -2.0 * (newAngle[i] - curAngle[i]) / pow(tf, 3);

	for (double j = 0.0; j <= tf; j += dt){

		double pos = a3 * pow(j, 3) + a2 * pow(j, 2) + a1 * j + a0;
		jointPos.push_back(pos);
	}
   }


   for (int i = 0; i <= 10; i++) {
     jointPos1.push_back(jointPos[i]);
     cout << jointPos1[i] << endl;

  }
  for (int i = 11; i <= 21; i++) {
      jointPos2.push_back(jointPos[i]);
      cout << jointPos2[i-11] << endl;

  }

for(int i=0; i<11; i++)
{
   // The PWM range for FT5330M => 500us-2500us (0.5ms -2.5ms)
   // Corresponds to 5-25 steps.

   softPwmWrite(GPIO1, jointPos1[i]); // Set the pulse high for 1.5ms/ 1500us (15*100us)
   softPwmWrite(GPIO2, jointPos2[i]);

   delay(100);

}

   delay(1000);
   softPwmWrite(GPIO1, 15); // Set the pulse high for 1.5ms/ 1500us (15*100us)
   softPwmWrite(GPIO2, 15);
   delay(1000);

}
