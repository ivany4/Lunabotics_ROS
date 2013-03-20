#include "utils.h"

double normalize_angle(double angle)
{
	double res = fmod(angle, M_PI*2);
	if (res < -M_PI) {
		res = M_PI*2-res;
	}
	else if (res > M_PI) {
		res = -(M_PI*2-res);
	}
	return res;
}


double point_distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	return sqrt(pow(p2.x-p1.x, 2)+pow(p2.y-p1.y, 2));
}


double distance_to_line(double angle, double end_dist)
{
	if (fabs(angle) > M_PI_2) {
		return end_dist;
	}
	return end_dist*sin(angle);
}

double angle_between_line_and_curr_pos(double line_length, double end1_dist, double end2_dist)
{
	return acos((pow(end1_dist,2)+pow(line_length,2)-pow(end2_dist,2))/(2*end1_dist*line_length));
}

geometry_msgs::Point closest_trajectory_point(double line_length, double end_dist, double end_angle, geometry_msgs::Point end_point1, geometry_msgs::Point end_point2)
{
	if (fabs(end_angle) > M_PI_2) {
		return end_point1;
	}
	double line_frac = end_dist*cos(end_angle);
	geometry_msgs::Point point;
	point.x = (end_point2.x-end_point1.x)*line_frac/line_length + end_point1.x;
	point.y = (end_point2.y-end_point1.y)*line_frac/line_length + end_point1.y;
	return point;
}
