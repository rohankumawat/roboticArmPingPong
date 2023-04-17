#include <iostream>
#include <vector>

#include "mapping.h"

constexpr double y_side  = 640; // 640 pixels corresponds to ~270cm
constexpr double z_side  = 360; // 360 pixels corresponds to ~110cm

constexpr double x_front = 640; // 640 pixels corresponds to ~180cm
constexpr double z_front = 360; // 360 pixels corresponds to ~140cm

 
/* 
 * y_ = 270cm - 30cm. 
 * 270 --> total length of the table
 * 35  --> offset of robot from the edge of the table
 */
constexpr double y_ = 2.40;

std::vector<double> getMapping(std::vector<double>predZ, std::vector<double>predX)
{
	/*
	 * Prrection of the projectile motion
	 * Camera on the side
	 * Approximating the polynomial to a straight line over short distance
	 */
	
	double x, y, z;

	predZ[0] = 2.7 * predZ[0] / y_side;
	predZ[2] = 2.7 * predZ[2] / y_side;

	predZ[1] = 1.1 * predZ[1] / z_side;
	predZ[3] = 1.1 * predZ[3] / z_side;

	predX[0] = 1.8 * predX[0] / x_front;
	predX[2] = 1.8 * predX[2] / x_front;

	predX[1] = 1.4 * predX[1] / z_front;
	predX[3] = 1.4 * predX[3] / z_front;



       if(!(predZ[2] - predZ[0]))
 	  z = predZ[1];
       else
	z = (predZ[3] - predZ[1]) / (predZ[2] - predZ[0]) * (y_ - predZ[0]) + predZ[1];
       if(!(predX[2] - predX[0]))
	  x = predX[1];
       else
	x = (predX[3] - predX[1]) / (predX[2] - predX[0]) * (z - predX[0]) + predX[1];

	y = y_;

	if (abs(z) > 3)
		z = 3;
	if (abs(x) > 3)
		x = 3;


	std::vector<double> pos = { x/10,y/10,z/10 };
return pos;
}
