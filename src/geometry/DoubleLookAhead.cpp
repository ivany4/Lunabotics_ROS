#include "DoubleLookAhead.h"

using namespace lunabotics;

/*
 * All the equations taken from the paper 
 * "DriveR-Compatible Steering System for Wide Speed-Range Path Following"
 */

double DoubleLookAhead::radiusFromPoints(PointArr points)
{	
	//Equation 13
	double radius = 0;
	if (!points.empty()) {
		for (PointArr::iterator it = points.begin(); it < points.end(); it++) {
			radius += distance(*it, this->_center);
		}
		radius /= points.size();
	}
	return radius;
}

double DoubleLookAhead::steeringAngleFromCurvature(double k)
{
	//Equation 15
	double A = -(this->_m1*this->_l1*this->_C_alpha_f-this->_l2*this->_C_alpha_r)/(2*pow((this->_l1+this->_l2), 2)*this->_C_alpha_f*this->_C_alpha_r);
	
	//Equation 14
	return k*(1+A*pow(this->_dxu, 2))/(1/this->_l1+this->_l2);
}

void DoubleLookAhead::guessCurveCenter(PointArr points)
{
	Point p1 = points.at(0);
	Point p2 = points.at(1);
	Point p3 = points.at(2);
	
	//Equation 12
	double l12 = (p2.y-p1.y)/(p2.x-p1.x);
	double l23 = (p3.y-p2.y)/(p3.x-p2.x);
	
	//Equation 10
	this->_center.x = -(p1.y+p2.y)/2+(p2.y+p3.y)/2
					  -(p1.x+p2.x)/(2*l12)+(p2.x+p3.x)/(2*l23);
					  
	//Equation 11
	this->_center.y = -(p2.y+p3.y)/2-this->_center.x/l23*this->_center.x+(p2.x+p3.x)/(2*l23);
}
