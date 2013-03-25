#ifndef _UTILS_H_
#define _UTILS_H_

#include "geometry_msgs/Point.h"

//Bring the angle to the range between (-pi/2, pi/2)
double normalize_angle(double angle);

//2D Distance between two points
double point_distance(geometry_msgs::Point p1, geometry_msgs::Point p2);

//Distance between point and line
double distance_to_line(double angle, double end_dist);

//Angle between further waypoint, closest waypoint, test point
double angle_between_line_and_curr_pos(double line_length, double end1_dist, double end2_dist);

//Closest trajectory point
geometry_msgs::Point closest_trajectory_point(double line_length, double end_dist, double end_angle, geometry_msgs::Point end_point1, geometry_msgs::Point end_point2);

#endif //_UTILS_H_
