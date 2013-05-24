#include "PathFollowingGeometry.h"
#include "basic.h"
#include "tf/tf.h"

using namespace lunabotics;

//---------------- Contructors / Destructor ---------------------//

PathFollowingGeometry::PathFollowingGeometry(): current_pose(), feedback_point(),
has_feedback_point(false), path(), has_closest_path_point(false),
closest_path_point(), closest_path_point_in_local_frame(), feedback_point_in_local_frame(),
feedback_error(0), has_local_frame(false), velocity(0), feedback_point_offset_min(0.05),
feedback_point_offset_multiplier(0.25)
{
}

PathFollowingGeometry::PathFollowingGeometry(float velocityOffset, float velocityMultiplier):
current_pose(), feedback_point(), has_feedback_point(false), path(), has_closest_path_point(true),
closest_path_point(), closest_path_point_in_local_frame(), feedback_point_in_local_frame(),
feedback_error(0), has_local_frame(false), velocity(0), feedback_point_offset_min(velocityOffset),
feedback_point_offset_multiplier(velocityMultiplier)
{
}

PathFollowingGeometry::~PathFollowingGeometry()
{
}


//---------------- Getters / Setters ----------------------------//

Point PathFollowingGeometry::getFeedbackLookAheadPoint()
{
	if (!this->has_feedback_point) {
		double theta = this->current_pose.orientation;
		double velocityVector = this->feedback_point_offset_min + this->velocity * this->feedback_point_offset_multiplier;
		this->feedback_point.x = this->current_pose.position.x + velocityVector*cos(theta);
		this->feedback_point.y = this->current_pose.position.y + velocityVector*sin(theta);
	}
	return this->feedback_point;
}

Point PathFollowingGeometry::getFeedbackLookAheadPointLocalFrame()
{
	if (!this->has_local_frame) {
		this->updateLocalFrame();
	}
	return this->feedback_point_in_local_frame;
}

Point PathFollowingGeometry::getClosestPathPointInLocalFrame()
{
	if (!this->has_local_frame) {
		this->updateLocalFrame();
	}
	return this->closest_path_point_in_local_frame;
}

double PathFollowingGeometry::getFeedbackError()
{
	if (!this->has_local_frame) {
		this->updateLocalFrame();
	}
	return this->feedback_error;
}

Point PathFollowingGeometry::getClosestPathPoint()
{
	if (!this->has_closest_path_point) {
		this->closest_path_point = this->getFeedbackLookAheadPoint();
		if (path.size() >= 2) {
			PointArr::iterator closestWaypointIterator = path.begin();
			PointArr::iterator secondClosestWaypointIterator = path.begin()+1;
			double closestDistance = distance(this->getFeedbackLookAheadPoint(), *closestWaypointIterator);
			double secondClosestDistance = distance(this->getFeedbackLookAheadPoint(), *secondClosestWaypointIterator);
			if (secondClosestDistance < closestDistance) {
				//Swap values to keep closestWaypoint always the closest one
				double tmp_dist = closestDistance;
				closestDistance = secondClosestDistance;
				secondClosestDistance = tmp_dist;
				PointArr::iterator tmp_waypoint = closestWaypointIterator;
				closestWaypointIterator = secondClosestWaypointIterator;
				secondClosestWaypointIterator = tmp_waypoint;
			}
			for (PointArr::iterator it = path.begin()+2; it < path.end(); it++) {
				double dist = distance(this->getFeedbackLookAheadPoint(), *it);
				if (dist < closestDistance) {
					secondClosestDistance = closestDistance, secondClosestWaypointIterator = closestWaypointIterator;
					closestDistance = dist, closestWaypointIterator = it;
				}
				else if (dist < secondClosestDistance) {
					secondClosestDistance = dist, secondClosestWaypointIterator = it;
				}
			}
			Point closestPoint = *closestWaypointIterator;
			Point secondClosestPoint = *secondClosestWaypointIterator;
			
			double lengthBetweenWaypoints = distance(closestPoint, secondClosestPoint);
			double angle = this->getReferenceAngle(lengthBetweenWaypoints, closestDistance, secondClosestDistance);
			
			//Reference point lies not between waypoints 1 and 2 but beyond
			#pragma message("ALWAYS FALSE FOR TESTING");
			if (false && fabs(angle) > M_PI_2) {
				this->closest_path_point = closestPoint;
			}
			else {
				double distanceFromClosestWaypointToClosestTrajectoryPoint = closestDistance*cos(angle);
				double fraction = distanceFromClosestWaypointToClosestTrajectoryPoint/lengthBetweenWaypoints;
				this->closest_path_point.x = (secondClosestPoint.x-closestPoint.x)*fraction + closestPoint.x;
				this->closest_path_point.y = (secondClosestPoint.y-closestPoint.y)*fraction + closestPoint.y;
			}
			this->has_closest_path_point = true;
		}
	}
	return this->closest_path_point;
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
	this->has_closest_path_point = false;
	this->has_local_frame = false;
}

void PathFollowingGeometry::updateLocalFrame()
{
	//Transform points to robot-centered coordinate frame
	//Depending on where (left or right) closest trajectory point is
	//make reference distance positive or negative
	
	//Transformation
	this->closest_path_point_in_local_frame = this->getClosestPathPoint();
	this->closest_path_point_in_local_frame.x -= this->current_pose.position.x;
	this->closest_path_point_in_local_frame.y -= this->current_pose.position.y;
	this->closest_path_point_in_local_frame = rotatePoint(this->closest_path_point_in_local_frame, this->current_pose.orientation, CW);
	
	this->feedback_point_in_local_frame = this->getFeedbackLookAheadPoint();
	this->feedback_point_in_local_frame.x -= this->current_pose.position.x;
	this->feedback_point_in_local_frame.y -= this->current_pose.position.y;
	this->feedback_point_in_local_frame = rotatePoint(this->feedback_point_in_local_frame, this->current_pose.orientation, CW);
	
	int multiplier = this->closest_path_point_in_local_frame.y >= 0 ? -1 : 1;
	
	this->feedback_error = distance(this->closest_path_point_in_local_frame, this->feedback_point_in_local_frame) * multiplier;
	
	this->has_local_frame = true;
}
