#include <wiringPi.h>
#include <softPwm.h>

#define GPIO1 4		// PIN 7/GPIO7
#define GPIO2 23	// PIN 16/GPIO4
#define GPIO3 24	// PIN 18/GPIO5

int main(int argc, char *argv[])
{


   if (wiringPiSetupGpio() < 0) return 1;
   pinMode(GPIO2, OUTPUT);
   digitalWrite(GPIO2, LOW);

   pinMode(GPIO1, OUTPUT);
   digitalWrite(GPIO1, LOW);

   softPwmCreate(GPIO1, 0, 200); // Create a pulse of frequency 50Hz (20ms => each step is 100us. 200 steps therefore generates a pulse of period 20ms)
   softPwmCreate(GPIO2, 0, 200); // Create a pulse of frequency 50Hz (20ms => each step is 100us. 200 steps therefore generates a pulse of period 20ms)
   softPwmCreate(GPIO3, 0, 200); // Create a pulse of frequency 50Hz (20ms => each step is 100us. 200 steps therefore generates a pulse of period 20ms)


for(int i=0; i<1; i++)
{
   // The PWM range for FT5330M => 500us-2500us (0.5ms -2.5ms)
   // Corresponds to 5-25 steps.

   softPwmWrite(GPIO1, 15); // Set the pulse high for 1.5ms/ 1500us (15*100us)
   softPwmWrite(GPIO2, 15);
   softPwmWrite(GPIO3, 15);

   delay(1000);


   softPwmWrite(GPIO1, 5.5);
   softPwmWrite(GPIO2, 5.5);
   softPwmWrite(GPIO3, 5.5);

   delay(1000);


   softPwmWrite(GPIO1, 15);
   softPwmWrite(GPIO2, 15);
   softPwmWrite(GPIO3, 15);
   delay(1000);

   softPwmWrite(GPIO1, 24.5);
   softPwmWrite(GPIO2, 24.5);
   softPwmWrite(GPIO3, 24.5);
   delay(1000);

   softPwmWrite(GPIO1, 15);
   softPwmWrite(GPIO2, 15);
   softPwmWrite(GPIO3, 15);
   delay(1000);


}
}
