#include "PathFollowingGeometry.h"
#include "basic.h"
#include "tf/tf.h"
#include <algorithm>

using namespace lunabotics;

//---------------- Contructors / Destructor ---------------------//

PathFollowingGeometry::PathFollowingGeometry(AllWheelGeometryPtr geometry): current_pose(), feedback_point(),
has_feedback_point(false), path(), has_feedback_path_point(false),
feedback_path_point(), feedback_path_point_in_local_frame(), feedback_point_in_local_frame(),
feedback_error(0), has_local_frame(false), velocity(0), feedback_point_offset_min(0.05),
feedback_point_offset_multiplier(0.25), robotGeometry(geometry)
{
}

PathFollowingGeometry::PathFollowingGeometry(AllWheelGeometryPtr geometry, float velocityOffset, float velocityMultiplier):
current_pose(), feedback_point(), has_feedback_point(false), path(), has_feedback_path_point(true),
feedback_path_point(), feedback_path_point_in_local_frame(), feedback_point_in_local_frame(),
feedback_error(0), has_local_frame(false), velocity(0), feedback_point_offset_min(velocityOffset),
feedback_point_offset_multiplier(velocityMultiplier), robotGeometry(geometry)
{
}

PathFollowingGeometry::~PathFollowingGeometry()
{
}


//---------------- Getters / Setters ----------------------------//


Point PathFollowingGeometry::getLookAheadPointAtDistance(double distance)
{
	return CreatePoint(
		this->current_pose.position.x + distance*cos(this->current_pose.orientation),
		this->current_pose.position.y + distance*sin(this->current_pose.orientation));
}

float PathFollowingGeometry::getFeedbackLookAheadDistance()
{
	if (!this->has_feedback_look_ahead_distance) {
		this->feedback_look_ahead_distance = this->feedback_point_offset_min + this->velocity * this->feedback_point_offset_multiplier;
		this->has_feedback_look_ahead_distance = true;
	}
	return this->feedback_look_ahead_distance;
}

float PathFollowingGeometry::getFeedforwardLookAheadDistance() 
{	
	if (!this->has_feedforward_look_ahead_distance) {
		this->feedforward_look_ahead_distance = this->feedforward_curvature_detection_start +
	 this->feedforward_point_offset_fraction * std::max(0.0, (double)(this->getFeedbackLookAheadDistance()-this->feedforward_curvature_detection_start));
		this->has_feedforward_look_ahead_distance = true;
	}
	return this->feedforward_look_ahead_distance;
}

Point PathFollowingGeometry::getFeedbackPoint()
{
	if (!this->has_feedback_point) {
		this->feedback_point = this->getLookAheadPointAtDistance(this->getFeedbackLookAheadDistance());
		this->has_feedback_point = true;
	}
	return this->feedback_point;
}

Point PathFollowingGeometry::getFeedbackPointInLocalFrame()
{
	if (!this->has_local_frame) {
		this->updateLocalFrame();
	}
	return this->feedback_point_in_local_frame;
}

Point PathFollowingGeometry::getFeedforwardPoint()
{
	if (!this->has_feedforward_point) {
		this->feedforward_point = this->getLookAheadPointAtDistance(this->getFeedforwardLookAheadDistance());
		this->has_feedforward_point = true;
	}
	return this->feedforward_point;
}

Point PathFollowingGeometry::getFeedforwardPointInLocalFrame()
{
	if (!this->has_local_frame) {
		this->updateLocalFrame();
	}
	return this->feedforward_point_in_local_frame;
}

Point PathFollowingGeometry::getFeedbackPathPointInLocalFrame()
{
	if (!this->has_local_frame) {
		this->updateLocalFrame();
	}
	return this->feedback_path_point_in_local_frame;
}

double PathFollowingGeometry::getFeedbackError()
{
	if (!this->has_feedback_error) {
		if (!this->has_local_frame) {
			this->updateLocalFrame();
		}
		int multiplier = this->feedback_path_point_in_local_frame.y >= 0 ? -1 : 1;
		this->feedback_error = distance(this->feedback_path_point_in_local_frame, this->feedback_point_in_local_frame) * multiplier;
		this->has_feedback_error = true;
	}		
	return this->feedback_error;
}

bool PathFollowingGeometry::getClosestPathPoint(Point referencePoint, Point &resultPoint)
{
	bool result = false;
	if (this->path.size() >= 2) {
		PointArrIt closest_waypoint_it = this->path.begin();
		PointArrIt second_closest_waypoint_it = this->path.begin()+1;
		double closest_distance = distance(referencePoint, *closest_waypoint_it);
		double second_closest_distance = distance(referencePoint, *second_closest_waypoint_it);
		if (second_closest_distance < closest_distance) {
			//Swap values to keep closest_waypoint always the closest one
			std::swap(closest_distance, second_closest_distance);
			std::swap(closest_waypoint_it, second_closest_waypoint_it);
		}
		for (PointArrIt it = path.begin()+2; it < path.end(); it++) {
			double dist = distance(referencePoint, *it);
			if (dist < closest_distance) {
				second_closest_distance = closest_distance;
				second_closest_waypoint_it = closest_waypoint_it;
				closest_distance = dist;
				closest_waypoint_it = it;
			}
			else if (dist < second_closest_distance) {
				second_closest_distance = dist;
				second_closest_waypoint_it = it;
			}
		}
		Point closest_point = *closest_waypoint_it;
		Point second_closest_point = *second_closest_waypoint_it;
		
		double length_between_waypoints = distance(closest_point, second_closest_point);
		double angle = this->getReferenceAngle(length_between_waypoints, closest_distance, second_closest_distance);
		
		//Reference point lies not between waypoints 1 and 2 but beyond
		#pragma message("ALWAYS FALSE FOR TESTING");
		if (false && fabs(angle) > M_PI_2) {
			resultPoint = second_closest_point;
		}
		else {
			double closest_waypoint_to_feedback_path_point = closest_distance*cos(angle);
			double fraction = closest_waypoint_to_feedback_path_point/length_between_waypoints;
			resultPoint.x = (second_closest_point.x-closest_point.x)*fraction + closest_point.x;
			resultPoint.y = (second_closest_point.y-closest_point.y)*fraction + closest_point.y;
		}
		result = true;
	}
	return result;
}

Point PathFollowingGeometry::getFeedbackPathPoint()
{
	if (!this->has_feedback_path_point) {
		this->has_feedback_path_point = this->getClosestPathPoint(this->getFeedbackPoint(), 
															this->feedback_path_point);
	}
	return this->feedback_path_point;
}

void PathFollowingGeometry::setCurrentPose(Pose currentPose)
{
	this->invalidateCache();
	this->current_pose = currentPose;
}

void PathFollowingGeometry::setVelocity(double velocity)
{
	this->invalidateCache();
	this->velocity = velocity;
}

void PathFollowingGeometry::setPath(PointArr path)
{
	this->invalidateCache();
	this->path = path;
}

void PathFollowingGeometry::setFeedbackPointOffsetMultiplier(float velocityMultiplier)
{
	this->invalidateCache();
	this->feedback_point_offset_multiplier = velocityMultiplier;
}

void PathFollowingGeometry::setFeedbackPointOffsetMin(float velocityOffset)
{
	this->invalidateCache();
	this->feedback_point_offset_min = velocityOffset;
}

void PathFollowingGeometry::setFeedforwardPointOffsetFraction(float fraction)
{
	this->invalidateCache();
	this->feedforward_point_offset_fraction = fraction;
}

void PathFollowingGeometry::setFeedforwardCurvatureDetectionStart(float velocityOffset)
{
	this->invalidateCache();
	this->feedforward_curvature_detection_start = velocityOffset;
}

//---------------- Private methods ------------------------------//

//Angle at point1 
double PathFollowingGeometry::getReferenceAngle(Point point1, Point point2, Point point3)
{
	double edge1 = distance(point1, point3);
	double edge2 = distance(point1, point2);
	double edge3 = distance(point2, point3);
	return this->getReferenceAngle(edge1, edge2, edge3);
}

//Angle between edge1 and edge2
double PathFollowingGeometry::getReferenceAngle(double edge1, double edge2, double edge3)
{
	//Cosine rule
	return acos((pow(edge2,2)+pow(edge1,2)-pow(edge3,2))/(2*edge2*edge1));
}

void PathFollowingGeometry::invalidateCache()
{
	this->has_feedback_point = false;
	this->has_feedback_path_point = false;
	this->has_local_frame = false;
	this->has_curvature_detection_points = false;
	this->has_feedforward_point = false;
	this->has_feedback_look_ahead_distance = false;
	this->has_feedforward_look_ahead_distance = false;
	this->has_curve_radius = false;
	this->has_feedforward_prediction = false;
	this->has_feedback_error = false;
	this->has_feedforward_center = false;
}

void PathFollowingGeometry::updateLocalFrame()
{
	//Transform points to robot-centered coordinate frame
	//Depending on where (left or right) feedback path point is
	//make reference distance positive or negative
	
	//Transformation
	this->feedback_path_point_in_local_frame = this->localFramePoint(this->getFeedbackPathPoint());
	this->feedback_point_in_local_frame = this->localFramePoint(this->getFeedbackPoint());
	this->feedforward_point_in_local_frame = this->localFramePoint(this->getFeedforwardPoint());
	
	PointArr points = this->getCurvatureDetectionPoints();
	this->curvature_detection_points_in_local_frame.clear();
	for (PointArrIt it = points.begin(); it < points.end(); it++) {
		this->curvature_detection_points_in_local_frame.push_back(this->localFramePoint(*it));
	}
	this->has_local_frame = true;
}

PointArr PathFollowingGeometry::getCurvatureDetectionPoints()
{
	if (!this->has_curvature_detection_points) {
		Point p1, p2, p3, p4, p5;
		double d1 = this->feedforward_curvature_detection_start;
		double quarter = (this->getFeedforwardLookAheadDistance()-d1)/2;
		double d2 = this->feedforward_curvature_detection_start+quarter;
		double d4 = this->getFeedforwardLookAheadDistance()+quarter;
		double d5 = d4+quarter;
		
		if (this->getClosestPathPoint(this->getFeedforwardPoint(), p3) &&
		this->getClosestPathPoint(this->getLookAheadPointAtDistance(d1), p1) &&
		this->getClosestPathPoint(this->getLookAheadPointAtDistance(d2), p2) &&
		this->getClosestPathPoint(this->getLookAheadPointAtDistance(d4), p4) &&
		this->getClosestPathPoint(this->getLookAheadPointAtDistance(d5), p5)) {
			this->curvature_detection_points.clear();
			this->curvature_detection_points.push_back(p1);
			this->curvature_detection_points.push_back(p2);
			this->curvature_detection_points.push_back(p3);
			this->curvature_detection_points.push_back(p4);
			this->curvature_detection_points.push_back(p5);
			this->has_curvature_detection_points = true;
		}
	}
	return this->curvature_detection_points;
}

PointArr PathFollowingGeometry::getCurvatureDetectionPointsInLocalFrame()
{
	if (!this->has_local_frame) {
		this->updateLocalFrame();
	}
	return this->curvature_detection_points_in_local_frame;
}

Point PathFollowingGeometry::getFeedforwardCurveCenterPoint()
{
	if (!this->has_feedforward_center) {
		PointArr points = this->getCurvatureDetectionPoints();
		if (points.size() >= 5) {
			Point p1 = points.at(0);
			Point p2 = points.at(2);
			Point p3 = points.at(4);
			
			//Equation 12
			double l12 = (p2.y-p1.y)/(p2.x-p1.x);
			double l23 = (p3.y-p2.y)/(p3.x-p2.x);
			
			//Equation 10
			Point center;
			this->feedforward_center.x = -(p1.y+p2.y)/2+(p2.y+p3.y)/2
							  -(p1.x+p2.x)/(2*l12)+(p2.x+p3.x)/(2*l23);
							  
			//Equation 11
			this->feedforward_center.y = -(p2.y+p3.y)/2-this->feedforward_center.x/l23*this->feedforward_center.x+(p2.x+p3.x)/(2*l23);
			
			this->has_feedforward_center = true;
		}
	}
	return this->feedforward_center;			
}

double PathFollowingGeometry::getCurveRadius()
{
	if (!this->has_curve_radius) {
		PointArr points = this->getCurvatureDetectionPoints();	
		if (!points.empty()) {
			Point center = this->getFeedforwardCurveCenterPoint();
			
			//Equation 13
			double radius = 0;
			for (PointArrIt it = points.begin(); it < points.end(); it++) {
				radius += distance(*it, center);
			}
			radius /= points.size();
			this->curve_radius = radius;
			
			
			this->has_curve_radius = true;
		}
	}
	return this->curve_radius;
}

double PathFollowingGeometry::getFeedforwardPrediction()
{
	if (!this->has_feedforward_prediction) {
		
		float l1 = this->robotGeometry->left_front().x;
		float l2 = fabs(this->robotGeometry->left_rear().x);
		
		//Equation 15
		double A = -(MASS*l1*C_ALPHA_F-l2*C_ALPHA_R)/(2*pow((l1+l2), 2)*C_ALPHA_F*C_ALPHA_R);
		
		//Curvature
		double k = 1.0/this->getCurveRadius();
		
		//Equation 14
		this->feedforward_prediction = k*(1+A*pow(this->velocity, 2))/(1/l1+l2);
		this->has_feedforward_prediction = true;
	}
	return this->feedforward_prediction;	
}


Point PathFollowingGeometry::localFramePoint(Point globalFramePoint)
{
	Point result = globalFramePoint;
	result.x -= this->current_pose.position.x;
	result.y -= this->current_pose.position.y;
	result = rotatePoint(result, this->current_pose.orientation, CW);
	return result;
}
