#include "PID.h"
#include "basic.h"
#include "tf/tf.h"

//---------------- Contructors / Destructor ---------------------//

geometry::PID::PID(): _currentPose(), _referencePoint(), _referencePointIsValid(false), _trajectory(), _closestTrajectoryPointIsValid(false), _closestTrajectoryPoint(), _closestTrajectoryPointInLocalFrame(), _referencePointInLocalFrame(), _referenceDistance(0), _localFrameIsValid(false), _linearVelocity(0), _velocityOffset(0.05), _velocityMultiplier(0.25)
{
}

geometry::PID::PID(float velocityOffset, float velocityMultiplier): _currentPose(), _referencePoint(), _referencePointIsValid(false), _trajectory(), _closestTrajectoryPointIsValid(true), _closestTrajectoryPoint(), _closestTrajectoryPointInLocalFrame(), _referencePointInLocalFrame(), _referenceDistance(0), _localFrameIsValid(false), _linearVelocity(0), _velocityOffset(velocityOffset), _velocityMultiplier(velocityMultiplier)
{
}

geometry::PID::~PID()
{
}


//---------------- Getters / Setters ----------------------------//

point_t geometry::PID::getReferencePoint()
{
	if (!_referencePointIsValid) {
		double theta = tf::getYaw(_currentPose.orientation);
		double velocityVector = _velocityOffset + _linearVelocity * _velocityMultiplier;
		_referencePoint.x = _currentPose.position.x + velocityVector*cos(theta);
		_referencePoint.y = _currentPose.position.y + velocityVector*sin(theta);
	}
	return _referencePoint;
}

point_t geometry::PID::getReferencePointInLocalFrame()
{
	if (!_localFrameIsValid) {
		this->updateLocalFrame();
	}
	return _referencePointInLocalFrame;
}

point_t geometry::PID::getClosestTrajectoryPointInLocalFrame()
{
	if (!_localFrameIsValid) {
		this->updateLocalFrame();
	}
	return _closestTrajectoryPointInLocalFrame;
}

double geometry::PID::getReferenceDistance()
{
	if (!_localFrameIsValid) {
		this->updateLocalFrame();
	}
	return _referenceDistance;
}

point_t geometry::PID::getClosestTrajectoryPoint()
{
	if (!_closestTrajectoryPointIsValid) {
		_closestTrajectoryPoint = this->getReferencePoint();
		if (_trajectory.size() >= 2) {
			point_arr::iterator closestWaypointIterator = _trajectory.begin();
			point_arr::iterator secondClosestWaypointIterator = _trajectory.begin()+1;
			double closestDistance = geometry::distanceBetweenPoints(this->getReferencePoint(), *closestWaypointIterator);
			double secondClosestDistance = geometry::distanceBetweenPoints(this->getReferencePoint(), *secondClosestWaypointIterator);
			if (secondClosestDistance < closestDistance) {
				//Swap values to keep closestWaypoint always the closest one
				double tmp_dist = closestDistance;
				closestDistance = secondClosestDistance;
				secondClosestDistance = tmp_dist;
				point_arr::iterator tmp_waypoint = closestWaypointIterator;
				closestWaypointIterator = secondClosestWaypointIterator;
				secondClosestWaypointIterator = tmp_waypoint;
			}
			for (point_arr::iterator it = _trajectory.begin()+2; it < _trajectory.end(); it++) {
				double dist = geometry::distanceBetweenPoints(this->getReferencePoint(), *it);
				if (dist < closestDistance) {
					secondClosestDistance = closestDistance, secondClosestWaypointIterator = closestWaypointIterator;
					closestDistance = dist, closestWaypointIterator = it;
				}
				else if (dist < secondClosestDistance) {
					secondClosestDistance = dist, secondClosestWaypointIterator = it;
				}
			}
			point_t closestPoint = *closestWaypointIterator;
			point_t secondClosestPoint = *secondClosestWaypointIterator;
			
			double lengthBetweenWaypoints = geometry::distanceBetweenPoints(closestPoint, secondClosestPoint);
			double angle = this->getReferenceAngle(lengthBetweenWaypoints, closestDistance, secondClosestDistance);
			
			//Reference point lies not between waypoints 1 and 2 but beyond
			if (fabs(angle) > M_PI_2) {
				_closestTrajectoryPoint = closestPoint;
			}
			else {
				double distanceFromClosestWaypointToClosestTrajectoryPoint = closestDistance*cos(angle);
				double fraction = distanceFromClosestWaypointToClosestTrajectoryPoint/lengthBetweenWaypoints;
				_closestTrajectoryPoint.x = (secondClosestPoint.x-closestPoint.x)*fraction + closestPoint.x;
				_closestTrajectoryPoint.y = (secondClosestPoint.y-closestPoint.y)*fraction + closestPoint.y;
			}
			_closestTrajectoryPointIsValid = true;
		}
	}
	return _closestTrajectoryPoint;
}

void geometry::PID::setCurrentPose(pose_t currentPose)
{
	this->invalidateCache();
	_currentPose = currentPose;
}

void geometry::PID::setLinearVelocity(double velocity)
{
	this->invalidateCache();
	_linearVelocity = velocity;
}

void geometry::PID::setTrajectory(point_arr trajectory)
{
	this->invalidateCache();
	_trajectory = trajectory;
}

void geometry::PID::setVelocityMultiplier(float velocityMultiplier)
{
	this->invalidateCache();
	_velocityMultiplier = velocityMultiplier;
}

void geometry::PID::setVelocityOffset(float velocityOffset)
{
	this->invalidateCache();
	_velocityOffset = velocityOffset;
}

//---------------- Private methods ------------------------------//

//Angle at point1 
double geometry::PID::getReferenceAngle(point_t point1, point_t point2, point_t point3)
{
	double edge1 = geometry::distanceBetweenPoints(point1, point3);
	double edge2 = geometry::distanceBetweenPoints(point1, point2);
	double edge3 = geometry::distanceBetweenPoints(point2, point3);
	return this->getReferenceAngle(edge1, edge2, edge3);
}

//Angle between edge1 and edge2
double geometry::PID::getReferenceAngle(double edge1, double edge2, double edge3)
{
	//Cosine rule
	return acos((pow(edge2,2)+pow(edge1,2)-pow(edge3,2))/(2*edge2*edge1));
}

void geometry::PID::invalidateCache()
{
	_referencePointIsValid = false;
	_closestTrajectoryPointIsValid = false;
	_localFrameIsValid = false;
}

void geometry::PID::updateLocalFrame()
{
	//Transform points to robot-centered coordinate frame
	//Depending on where (left or right) closest trajectory point is
	//make reference distance positive or negative
	
	//Transformation
	_closestTrajectoryPointInLocalFrame = this->getClosestTrajectoryPoint();
	_closestTrajectoryPointInLocalFrame.x -= _currentPose.position.x;
	_closestTrajectoryPointInLocalFrame.y -= _currentPose.position.y;
	_closestTrajectoryPointInLocalFrame = geometry::rotatePoint(_closestTrajectoryPointInLocalFrame, tf::getYaw(_currentPose.orientation), CW);
	
	_referencePointInLocalFrame = this->getReferencePoint();
	_referencePointInLocalFrame.x -= _currentPose.position.x;
	_referencePointInLocalFrame.y -= _currentPose.position.y;
	_referencePointInLocalFrame = geometry::rotatePoint(_referencePointInLocalFrame, tf::getYaw(_currentPose.orientation), CW);
	
	int multiplier = _closestTrajectoryPointInLocalFrame.y >= 0 ? -1 : 1;
	
	_referenceDistance = geometry::distanceBetweenPoints(_closestTrajectoryPointInLocalFrame, _referencePointInLocalFrame) * multiplier;
	
	_localFrameIsValid = true;
}
