#ifndef _UTILS_H_
#define _UTILS_H_

#include "geometry_msgs/Point.h"

double normalize_angle(double angle);
double point_distance(geometry_msgs::Point p1, geometry_msgs::Point p2);
double distance_to_line(double angle, double end_dist);
double angle_between_line_and_curr_pos(double line_length, double end1_dist, double end2_dist);
geometry_msgs::Point closest_trajectory_point(double line_length, double end_dist, double end_angle, geometry_msgs::Point end_point1, geometry_msgs::Point end_point2);

#endif //_UTILS_H_
