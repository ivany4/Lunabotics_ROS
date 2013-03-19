#include "bezier_smooth.h"
#include "ros/ros.h"
using namespace std;

	void planning::GetCurveControlPoints(point_arr knots, point_arr &firstControlPoints, point_arr &secondControlPoints)
	{
		unsigned int n = knots.size();
		if (n < 2) {
			ROS_WARN("At least two knot points required");
		}
		else if (n == 2) {
			ROS_INFO("Straight line");
			 // Special case: Bezier curve should be a straight line.
			// 3P1 = 2P0 + P3
			geometry_msgs::Point P0 = knots.at(0);
			geometry_msgs::Point P3 = knots.at(1);
			geometry_msgs::Point P1, P2;
			P1.x = (2*P0.x + P3.x) / 3.0;
			P1.y = (2*P0.y + P3.y) / 3.0;
			firstControlPoints.push_back(P1);

			// P2 = 2P1 – P0
			P2.x = 2*P1.x - P0.x;
			P2.y = 2*P1.y - P0.y;
			secondControlPoints.push_back(P2);

		}
		else {
			ROS_INFO("Polyline");

			// Calculate first Bezier control points
			// Right hand side vector
			vector<double> rhs;
	
			// Set right hand side X values
			rhs.push_back(knots.at(0).x + 2*knots.at(1).x);
			for (unsigned int i = 1; i < n-1; i++) {
				rhs.push_back(4*knots.at(i).x + 2*knots.at(i+1).x);
			}
			rhs.push_back((8*knots.at(n-2).x + knots.at(n-1).x)/2.0);
			
			
			// Get first control points X-values
			vector<double> x = planning::GetFirstControlPoints(rhs);
			
			rhs.clear();
			
			// Set right hand side Y values
			rhs.push_back(knots.at(0).y + 2*knots.at(1).y);
			for (unsigned int i = 1; i < n-1; i++) {
				rhs.push_back(4*knots.at(i).y + 2*knots.at(i+1).y);
			}
			rhs.push_back((8*knots.at(n-2).y + knots.at(n-1).y)/2.0);
			
			// Get first control points Y-values
			vector<double> y = planning::GetFirstControlPoints(rhs);
	
			// Fill output arrays.
			firstControlPoints.clear();
			secondControlPoints.clear();
			for (unsigned int i = 0; i < n; i++) {
				// First control point
				geometry_msgs::Point point;
				point.x = x.at(i);
				point.y = y.at(i);
				firstControlPoints.push_back(point);
				// Second control point
				if (i < n-1) {
					point.x = 2*knots.at(i+1).x - x.at(i+1);
					point.y = 2*knots.at(i+1).y - y.at(i+1);
					secondControlPoints.push_back(point);
				}
				else {
					point.x = (knots.at(n-1).x + x.at(n-1)) / 2;
					point.y = (knots.at(n-1).y + y.at(n-1)) / 2;
					secondControlPoints.push_back(point);
				}
			}
		}
	}

	vector<double> planning::GetFirstControlPoints(vector<double> rhs)
	{
		unsigned int n = rhs.size();
		vector<double> x;// Solution vector.
		vector<double> tmp; // Temp workspace.

		double b = 2.0;
		x.push_back(rhs.at(0) / b);
		tmp.push_back(0);
		for (unsigned int i = 1; i < n; i++) {// Decomposition and forward substitution.
			tmp.push_back(1 / b);
			b = (i < n - 1 ? 4.0 : 3.5) - tmp.at(i);
			x.push_back((rhs.at(i) - x.at(i-1)) / b);
		}
		for (unsigned int i = 1; i < n; i++) {
			x.at(n-i-1) -= tmp.at(n-i) * x.at(n-i); // Backsubstitution.
		}
		return x;
	}


geometry_msgs::Point planning::bezier_point(float u, point_arr ctrl_points) 
{
	geometry_msgs::Point point;
	point.x = pow(u,3)*(ctrl_points.at(3).x+3*(ctrl_points.at(1).x-ctrl_points.at(2).x)-ctrl_points.at(0).x)+3*pow(u,2)*(ctrl_points.at(0).x-2*ctrl_points.at(1).x+ctrl_points.at(2).x)+3*u*(ctrl_points.at(1).x-ctrl_points.at(0).x)+ctrl_points.at(0).x;
	point.y = pow(u,3)*(ctrl_points.at(3).y+3*(ctrl_points.at(1).y-ctrl_points.at(2).y)-ctrl_points.at(0).y)+3*pow(u,2)*(ctrl_points.at(0).y-2*ctrl_points.at(1).y+ctrl_points.at(2).y)+3*u*(ctrl_points.at(1).y-ctrl_points.at(0).y)+ctrl_points.at(0).y;
    return point;
}
