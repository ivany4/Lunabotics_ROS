#include "geometry_msgs/Point.h"

void GetCurveControlPoints(std::vector<geometry_msgs::Point> knots, std::vector<geometry_msgs::Point> &firstControlPoints, std::vector<geometry_msgs::Point> &secondControlPoints);
std::vector<double> GetFirstControlPoints(std::vector<double> rhs);
geometry_msgs::Point bezier_point(int u, std::vector<geometry_msgs::Point> ctrl_points);
