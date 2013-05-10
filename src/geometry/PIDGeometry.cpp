#include "PIDGeometry.h"
#include "basic.h"
#include "tf/tf.h"

using namespace lunabotics;

//---------------- Contructors / Destructor ---------------------//

PIDGeometry::PIDGeometry(): _currentPose(), _referencePoint(), _referencePointIsValid(false), _trajectory(), _closestTrajectoryPointIsValid(false), _closestTrajectoryPoint(), _closestTrajectoryPointInLocalFrame(), _referencePointInLocalFrame(), _referenceDistance(0), _localFrameIsValid(false), _linearVelocity(0), _velocityOffset(0.05), _velocityMultiplier(0.25)
{
}

PIDGeometry::PIDGeometry(float velocityOffset, float velocityMultiplier): _currentPose(), _referencePoint(), _referencePointIsValid(false), _trajectory(), _closestTrajectoryPointIsValid(true), _closestTrajectoryPoint(), _closestTrajectoryPointInLocalFrame(), _referencePointInLocalFrame(), _referenceDistance(0), _localFrameIsValid(false), _linearVelocity(0), _velocityOffset(velocityOffset), _velocityMultiplier(velocityMultiplier)
{
}

PIDGeometry::~PIDGeometry()
{
}


//---------------- Getters / Setters ----------------------------//

Point PIDGeometry::getReferencePoint()
{
	if (!_referencePointIsValid) {
		double theta = _currentPose.orientation;
		double velocityVector = _velocityOffset + _linearVelocity * _velocityMultiplier;
		_referencePoint.x = _currentPose.position.x + velocityVector*cos(theta);
		_referencePoint.y = _currentPose.position.y + velocityVector*sin(theta);
	}
	return _referencePoint;
}

Point PIDGeometry::getReferencePointInLocalFrame()
{
	if (!_localFrameIsValid) {
		this->updateLocalFrame();
	}
	return _referencePointInLocalFrame;
}

Point PIDGeometry::getClosestTrajectoryPointInLocalFrame()
{
	if (!_localFrameIsValid) {
		this->updateLocalFrame();
	}
	return _closestTrajectoryPointInLocalFrame;
}

double PIDGeometry::getReferenceDistance()
{
	if (!_localFrameIsValid) {
		this->updateLocalFrame();
	}
	return _referenceDistance;
}

Point PIDGeometry::getClosestTrajectoryPoint()
{
	if (!_closestTrajectoryPointIsValid) {
		_closestTrajectoryPoint = this->getReferencePoint();
		if (_trajectory.size() >= 2) {
			PointArr::iterator closestWaypointIterator = _trajectory.begin();
			PointArr::iterator secondClosestWaypointIterator = _trajectory.begin()+1;
			double closestDistance = distance(this->getReferencePoint(), *closestWaypointIterator);
			double secondClosestDistance = distance(this->getReferencePoint(), *secondClosestWaypointIterator);
			if (secondClosestDistance < closestDistance) {
				//Swap values to keep closestWaypoint always the closest one
				double tmp_dist = closestDistance;
				closestDistance = secondClosestDistance;
				secondClosestDistance = tmp_dist;
				PointArr::iterator tmp_waypoint = closestWaypointIterator;
				closestWaypointIterator = secondClosestWaypointIterator;
				secondClosestWaypointIterator = tmp_waypoint;
			}
			for (PointArr::iterator it = _trajectory.begin()+2; it < _trajectory.end(); it++) {
				double dist = distance(this->getReferencePoint(), *it);
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

void PIDGeometry::setCurrentPose(Pose currentPose)
{
	this->invalidateCache();
	_currentPose = currentPose;
}

void PIDGeometry::setLinearVelocity(double velocity)
{
	this->invalidateCache();
	_linearVelocity = velocity;
}

void PIDGeometry::setTrajectory(PointArr trajectory)
{
	this->invalidateCache();
	_trajectory = trajectory;
}

void PIDGeometry::setVelocityMultiplier(float velocityMultiplier)
{
	this->invalidateCache();
	_velocityMultiplier = velocityMultiplier;
}

void PIDGeometry::setVelocityOffset(float velocityOffset)
{
	this->invalidateCache();
	_velocityOffset = velocityOffset;
}

//---------------- Private methods ------------------------------//

//Angle at point1 
double PIDGeometry::getReferenceAngle(Point point1, Point point2, Point point3)
{
	double edge1 = distance(point1, point3);
	double edge2 = distance(point1, point2);
	double edge3 = distance(point2, point3);
	return this->getReferenceAngle(edge1, edge2, edge3);
}

//Angle between edge1 and edge2
double PIDGeometry::getReferenceAngle(double edge1, double edge2, double edge3)
{
	//Cosine rule
	return acos((pow(edge2,2)+pow(edge1,2)-pow(edge3,2))/(2*edge2*edge1));
}

void PIDGeometry::invalidateCache()
{
	_referencePointIsValid = false;
	_closestTrajectoryPointIsValid = false;
	_localFrameIsValid = false;
}

void PIDGeometry::updateLocalFrame()
{
	//Transform points to robot-centered coordinate frame
	//Depending on where (left or right) closest trajectory point is
	//make reference distance positive or negative
	
	//Transformation
	_closestTrajectoryPointInLocalFrame = this->getClosestTrajectoryPoint();
	_closestTrajectoryPointInLocalFrame.x -= _currentPose.position.x;
	_closestTrajectoryPointInLocalFrame.y -= _currentPose.position.y;
	_closestTrajectoryPointInLocalFrame = rotatePoint(_closestTrajectoryPointInLocalFrame, _currentPose.orientation, CW);
	
	_referencePointInLocalFrame = this->getReferencePoint();
	_referencePointInLocalFrame.x -= _currentPose.position.x;
	_referencePointInLocalFrame.y -= _currentPose.position.y;
	_referencePointInLocalFrame = rotatePoint(_referencePointInLocalFrame, _currentPose.orientation, CW);
	
	int multiplier = _closestTrajectoryPointInLocalFrame.y >= 0 ? -1 : 1;
	
	_referenceDistance = distance(_closestTrajectoryPointInLocalFrame, _referencePointInLocalFrame) * multiplier;
	
	_localFrameIsValid = true;
}
