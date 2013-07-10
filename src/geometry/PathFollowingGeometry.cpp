#include "PathFollowingGeometry.h"
#include "basic.h"
#include "tf/tf.h"
#include <algorithm>

#define INTERPOLATION_PTS_PER_M 100

using namespace lunabotics;

//---------------- Contructors / Destructor ---------------------//

PathFollowingGeometry::PathFollowingGeometry(AllWheelGeometryPtr geometry):
has_lateral_deviation(false),
lateral_deviation(0),
deviation_path_point(),
deviation_path_point_in_local_frame(),
has_deviation_path_point(false),
feedback_point(),
feedback_point_in_local_frame(),
has_feedback_point(false),
feedback_point_offset_min(0.05),
feedback_point_offset_multiplier(0.25),
feedback_look_ahead_distance(0),
has_feedback_look_ahead_distance(false),
feedback_error(0),
has_feedback_error(false),
feedback_path_point(),
feedback_path_point_in_local_frame(),
has_feedback_path_point(false),
heading_error(0),
has_heading_error(false),
feedforward_point(),
feedforward_point_in_local_frame(),
has_feedforward_point(false),
feedforward_point_offset_fraction(0),
feedforward_curvature_detection_start(0),
feedforward_look_ahead_distance(0),
has_feedforward_look_ahead_distance(false),
feedforward_prediction(0),
has_feedforward_prediction(false),
curvature_detection_points(),
curvature_detection_points_in_local_frame(),
has_curvature_detection_points(false),
feedforward_center(),
has_feedforward_center(false),
curve_radius(0),
has_curve_radius(false),
current_pose(),
path(),
has_local_frame(false),
robotGeometry(geometry),
velocity(0)
{
}

PathFollowingGeometry::PathFollowingGeometry(AllWheelGeometryPtr geometry, float feedback_offset,
float feedback_multiplier, float feedforward_offset, float feedforward_fraction):
has_lateral_deviation(false),
lateral_deviation(0),
deviation_path_point(),
deviation_path_point_in_local_frame(),
has_deviation_path_point(false),
feedback_point(),
feedback_point_in_local_frame(),
has_feedback_point(false),
feedback_point_offset_min(feedback_offset),
feedback_point_offset_multiplier(feedback_multiplier),
feedback_look_ahead_distance(0),
has_feedback_look_ahead_distance(false),
feedback_error(0),
has_feedback_error(false),
feedback_path_point(),
feedback_path_point_in_local_frame(),
has_feedback_path_point(false),
heading_error(0),
has_heading_error(false),
feedforward_point(),
feedforward_point_in_local_frame(),
has_feedforward_point(false),
feedforward_point_offset_fraction(feedforward_fraction),
feedforward_curvature_detection_start(feedforward_offset),
feedforward_look_ahead_distance(0),
has_feedforward_look_ahead_distance(false),
feedforward_prediction(0),
has_feedforward_prediction(false),
curvature_detection_points(),
curvature_detection_points_in_local_frame(),
has_curvature_detection_points(false),
feedforward_center(),
has_feedforward_center(false),
curve_radius(0),
has_curve_radius(false),
current_pose(),
path(),
has_local_frame(false),
robotGeometry(geometry),
velocity(0)
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

Point PathFollowingGeometry::getDeviationPathPointInLocalFrame()
{
	if (!this->has_local_frame) {
		this->updateLocalFrame();
	}
	return this->deviation_path_point_in_local_frame;
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
		if (this->has_feedback_path_point) {
			int multiplier = this->feedback_path_point_in_local_frame.y >= 0 ? -1 : 1;
			this->feedback_error = distance(this->feedback_path_point_in_local_frame, this->feedback_point_in_local_frame) * multiplier;
			this->has_feedback_error = true;
		}
		else {
			this->feedback_error = 0;
		}
	}		
	return this->feedback_error;
}

double PathFollowingGeometry::getLateralDeviation()
{
	if (!this->has_lateral_deviation) {
		if (!this->has_local_frame) {
			this->updateLocalFrame();
		}
		if (this->has_deviation_path_point) {
			int multiplier = this->deviation_path_point_in_local_frame.y >= 0 ? -1 : 1;
			this->lateral_deviation = distance(this->deviation_path_point_in_local_frame, CreateZeroPoint()) * multiplier;
			this->has_lateral_deviation = true;
			//ROS_INFO("Getting lateral deviation %f", this->lateral_deviation);
		}
		else {
			this->lateral_deviation = 0;
			//ROS_WARN("Can't get lateral deviation");
		}
	}		
	return this->lateral_deviation;
}

//Allow same: whether allow closest obstacle point to be same as reference point
//ClosestIsNext: whether closest path point is in the direction of motion
bool PathFollowingGeometry::getTwoPathPointsClosestToPoint(Point referencePoint, Point &closestPoint, Point &secondClosestPoint, bool allowSame, bool &closestIsNext)
{
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
			bool isSame = fabs(dist) < 0.001;
			if (!isSame || (isSame && allowSame)) {
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
		}
		if (allowSame || fabs(closest_distance) >= 0.001) {
			closestPoint = *closest_waypoint_it;
			secondClosestPoint = *second_closest_waypoint_it;
			
			closestIsNext = closest_waypoint_it > second_closest_waypoint_it;
			
			return true;
		}
	}
	return false;
}

PointArr PathFollowingGeometry::interpolate(Point p1, Point p2)
{
	PointArr result;
	double length_between_waypoints = distance(p1, p2);
	int temp = round(length_between_waypoints*INTERPOLATION_PTS_PER_M);
	unsigned int num_points = std::max(1, temp);
	double step = (double)(length_between_waypoints/num_points);
	
	//ROS_INFO("Interpolating %f with %d pts", length_between_waypoints, num_points);
	
	result.push_back(p1);
	for (unsigned int i = 1; i < num_points-1; i++) {
		double fraction = (double)(i/(double)num_points);
		//ROS_INFO("Fraction %f", fraction);
		double x = p1.x+(p2.x-p1.x)*fraction;
		double y = p1.y+(p2.y-p1.y)*fraction;
		result.push_back(CreatePoint(x, y));
	}
	result.push_back(p2);
	return result;
}


Point PathFollowingGeometry::getClosestPointFromSet(Point referencePoint, PointArr pointSet)
{
	Point result;
	if (pointSet.size() > 0) {
		result = pointSet.at(0);
		double closest_distance = distance(referencePoint, result);
		for (PointArrIt it = pointSet.begin()+1; it < pointSet.end(); it++) {
			double new_distance = distance(referencePoint, *it);
			
			//ROS_INFO("%.2f,%.2f -> %.2f,%.2f = %.4f", referencePoint.x, referencePoint.y, (*it).x, (*it).y, new_distance);
			if (new_distance < closest_distance) {
				closest_distance = new_distance;
				result = *it;
			}
		}
		
	//	ROS_WARN("Shortest %.2f,%.2f -> %.2f,%.2f = %.4f", referencePoint.x, referencePoint.y, result.x, result.y, closest_distance);
	}
	else {
		result = referencePoint;
	}
	return result;
}

bool PathFollowingGeometry::getClosestPathPoint(Point referencePoint, Point &resultPoint)
{
	bool result = false;
	if (this->path.size() >= 2) {
		
		Point closest_point, second_closest_point;
		bool closest_is_next;
		if (this->getTwoPathPointsClosestToPoint(referencePoint, closest_point, second_closest_point, true, closest_is_next)) {
			PointArr candidates = this->interpolate(closest_point, second_closest_point);
			resultPoint = this->getClosestPointFromSet(referencePoint, candidates);
			result = true;			
		}
		
		/*
		Point closest_point, second_closest_point;
		bool dummy;
		
		if (this->getTwoPathPointsClosestToPoint(referencePoint, closest_point, second_closest_point, true, dummy)) {
			double angle = angleFromTriangle(closest_point, referencePoint, second_closest_point);
			
			//Reference point lies not between waypoints 1 and 2 but beyond
			#pragma message("ALWAYS FALSE FOR TESTING");
			if (false && fabs(angle) > M_PI_2) {
			//if (fabs(angle) > M_PI_2) {
				resultPoint = second_closest_point;
				result = true;
			}
			else {
				double length_between_waypoints = distance(closest_point, second_closest_point);
				double closest_distance = distance(closest_point, referencePoint);
				double closest_waypoint_to_path_point = closest_distance*cos(angle);
				double fraction = closest_waypoint_to_path_point/length_between_waypoints;
				resultPoint.x = (second_closest_point.x-closest_point.x)*fraction + closest_point.x;
				resultPoint.y = (second_closest_point.y-closest_point.y)*fraction + closest_point.y;
				result = true;
			}
		}	*/
			
	}
	else if (this->path.size() == 1) {
		resultPoint = this->path.at(0);
		ROS_WARN("Path size is 1. Assigning it as the closest point");
	}	
	else {
		ROS_WARN("PAth size is less than 1. Not able to find closest path point");
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

Point PathFollowingGeometry::getDeviationPathPoint()
{
	if (!this->has_deviation_path_point) {
		this->has_deviation_path_point = this->getClosestPathPoint(this->current_pose.position, 
															this->deviation_path_point);
		if (!this->has_deviation_path_point) {
			ROS_ERROR("Failed to get deviation path point");
		}
	}
	return this->deviation_path_point;
}

double PathFollowingGeometry::getHeadingError()
{
	if (!this->has_heading_error) {
		double requiredHeading;
		if (this->has_heading_error = this->getTangentAtPoint(this->getFeedbackPathPoint(), requiredHeading)) {
			this->heading_error = normalizedAngle(requiredHeading-this->current_pose.orientation);
		}		
	}
	return this->heading_error;
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
	this->has_heading_error = false;
	this->has_lateral_deviation = false;
	this->has_deviation_path_point = false;
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
	this->deviation_path_point_in_local_frame = this->localFramePoint(this->getDeviationPathPoint());
	
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
			/*ROS_INFO("Detection p1 %.2f,%.2f", p1.x, p1.y);
			ROS_INFO("Detection p2 %.2f,%.2f", p2.x, p2.y);
			ROS_INFO("Detection p3 %.2f,%.2f", p3.x, p3.y);
			ROS_INFO("Detection p4 %.2f,%.2f", p4.x, p4.y);
			ROS_INFO("Detection p5 %.2f,%.2f", p5.x, p5.y);
			*/
			
			this->curvature_detection_points.clear();
			this->curvature_detection_points.push_back(p1);
			this->curvature_detection_points.push_back(p2);
			this->curvature_detection_points.push_back(p3);
			this->curvature_detection_points.push_back(p4);
			this->curvature_detection_points.push_back(p5);
			this->has_curvature_detection_points = true;
		}
		else {
		//	ROS_WARN("Can't get curvature detection points");
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
			
			//ROS_INFO("p1 %.2f,%2.f; p2 %.2f,%2.f; p3 %.2f,%2.f", p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
			
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
		else {
			ROS_WARN("Can't get curve center. Expected 5 detection points.");
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
			
			//std::stringstream sstr;
			//sstr << center;
						
			//ROS_INFO("Getting curve radius %f, pts.size %d. %.2f, %.2f", radius, (unsigned int)points.size(), center.x, center.y);
			
			this->has_curve_radius = true;
		}
		else {
			ROS_WARN("Failed to calculate radius. Detection points are empty");
		}
	}
	return this->curve_radius;
}

double PathFollowingGeometry::getFeedforwardPrediction()
{
	if (!this->has_feedforward_prediction) {
		
		this->getCurveRadius();
		if (this->has_curve_radius) {
		
			float l1 = this->robotGeometry->left_front().x;
			float l2 = fabs(this->robotGeometry->left_rear().x);
			
			//Equation 15
			double A = -(MASS*l1*C_ALPHA_F-l2*C_ALPHA_R)/(2*pow((l1+l2), 2)*C_ALPHA_F*C_ALPHA_R);
			
			//Curvature
			double k = 1.0/this->curve_radius;
			
			//Equation 14
			this->feedforward_prediction = k*(1+A*pow(this->velocity, 2))/(1/l1+l2);
			this->has_feedforward_prediction = true;
		}
		else {
			this->feedforward_prediction = 0;
		}
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

bool PathFollowingGeometry::getTangentAtPoint(Point point, double &heading)
{
	Point closest_point, second_closest_point;
	bool closestIsNext;
	
	if (this->getTwoPathPointsClosestToPoint(point, closest_point, second_closest_point, true, closestIsNext)) {
		if (!closestIsNext) {
			std::swap(closest_point, second_closest_point);
		}
		double dx = closest_point.x - point.x;
		double dy = closest_point.y - point.y;
		double lineHeading = atan2(dy, dx);
		double angle = angleFromTriangle(closest_point, point, second_closest_point);
		
		heading = lineHeading + angle;			
		return true;
	}
	return false;
		
}
