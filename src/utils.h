#ifndef _UTILS_H_
#define _UTILS_H_

#include "types.h"

//Bring the angle to the range between (-pi/2, pi/2)
double normalize_angle(double angle);

//2D Distance between two points
double point_distance(point_t p1, point_t p2);

#endif //_UTILS_H_
