#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <vector>

class Robot_Arm {

public:

	Robot_Arm(double _x, double _y, double _z);

	// Setters
	void setPosition(double _x, double _y, double _z);

	// Getters

	std::vector<double> getJointAngles();
	std::vector<double> genTrajectory(std::vector<double> curAngle, std::vector<double> newAngle, double dt, double tf);



private:

	// Private methods
	std::vector<double> getPosition();
	std::vector<double> getOrientation();

	// Private variables
	double x;
	double y;
	double z;
	bool flag;

	/*
	 * Define the initial joint angles and joint dimentsions according to D-H table
	 */
	std::vector<double> jointAngles{ 0.0, 0.0, 0.0, 0.0, 0.0 };
	double a[5] = { 0.0, 0.2, 0.0, 0.0, 0.095 };
	double d[5] = { 0.04, 0.0, 0.0, 0.258, 0.0 };;
	
};



#endif