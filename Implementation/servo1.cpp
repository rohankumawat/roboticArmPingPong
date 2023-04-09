#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

#define GPIO1 17	// PIN 11/GPIO0
#define GPIO2 18	// PIN 12/GPIO1
#define GPIO3 23	// PIN 16/GPIO4
#define GPIO4 24	// PIN 18/GPIO5
#define GPIO5 4		// PIN 7/GPIO7

int main(int argc, char *argv[])
{


   if (wiringPiSetupGpio() < 0) return 1;
   pinMode(GPIO2, OUTPUT);
   digitalWrite(GPIO2, LOW);

   pinMode(GPIO1, OUTPUT);
   digitalWrite(GPIO1, LOW);

   pinMode(GPIO3, OUTPUT);
   digitalWrite(GPIO3, LOW);

   softPwmCreate(GPIO1, 0, 200); // Create a pulse of frequency 50Hz (20ms => each step is 100us. 200 steps
   softPwmCreate(GPIO2, 0, 200); 
   softPwmCreate(GPIO3, 0, 200); 


   double curAngle[5] = {15,15,15,15,15};
   double newAngle[5] = {23,8,10,8,8};
   double v = 1;   
  
  
   vector<double> jointPos1, jointPos2, jointPos3,jointPos4,jointPos5;

   for(int i = 0; i < 5; i++)
   {    
	double tf = abs((newAngle[i] - curAngle[i])/v);
 	double dt = 0.1; //Timestep

        vector<double> timeSteps;
        for(double i = 0.0; i <= tf; i += dt){
    		timeSteps.push_back(i);       
        }


        double a0 = curAngle[i];
	double a1 = 0.0;
	double a2 = 3.0 * (newAngle[i] - curAngle[i]) / pow(tf, 2);
	double a3 = -2.0 * (newAngle[i] - curAngle[i]) / pow(tf, 3);

	for(double j = 0.0; j <= tf; j += dt){
		double pos = a3 * pow(j, 3) + a2 * pow(j, 2) + a1 * j + a0;
		if(i == 0){
                	jointPos1.push_back(pos);
		}
		else if(i == 1){
                	jointPos2.push_back(pos);
		}
		else if(i == 2){
                	jointPos3.push_back(pos);
		}
		else if(i == 3){
                	jointPos4.push_back(pos);
		}
		else{
                	jointPos5.push_back(pos);
		}


	}
        
   }

   for(int i=0; i<jointPos1.size(); i++){
	cout << jointPos1[i] << endl;
   	softPwmWrite(GPIO1, jointPos1[i]); 
   }
   for(int i=0; i<jointPos2.size(); i++){
   	softPwmWrite(GPIO2, jointPos2[i]); 
   }
   for(int i=0; i<jointPos3.size(); i++){
   	softPwmWrite(GPIO3, jointPos3[i]); 
   }
   for(int i=0; i<jointPos4.size(); i++){
   	softPwmWrite(GPIO4, jointPos4[i]); 
   }
   for(int i=0; i<jointPos5.size(); i++){
   	softPwmWrite(GPIO5, jointPos5[i]); 
   }      
   
}
