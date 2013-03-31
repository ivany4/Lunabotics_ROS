#include "pid.h"
#include "tf/tf.h"

//---------------- Contructors / Destructor ---------------------//

motion::PIDGeometry::PIDGeometry(): _currentPose(), _referencePoint(), _referencePointIsValid(true), _trajectory(), _closestTrajectoryPointIsValid(true), _closestTrajectoryPoint(), _linearVelocity(0), _velocityOffset(0.05), _velocityMultiplier(0.25)
{
}

motion::PIDGeometry::PIDGeometry(float velocityOffset, float velocityMultiplier): _currentPose(), _referencePoint(), _referencePointIsValid(true), _trajectory(), _closestTrajectoryPointIsValid(true), _closestTrajectoryPoint(), _linearVelocity(0), _velocityOffset(velocityOffset), _velocityMultiplier(velocityMultiplier)
{
}

motion::PIDGeometry::~PIDGeometry()
{
}

//---------------- Public methods -------------------------------//

point_t motion::PIDGeometry::getReferencePoint()
{
	if (!_referencePointIsValid) {
		double theta = tf::getYaw(_currentPose.orientation);
		double velocityVector = _velocityOffset + _linearVelocity * _velocityMultiplier;
		_referencePoint.x = _currentPose.position.x + velocityVector*cos(theta);
		_referencePoint.y = _currentPose.position.y + velocityVector*sin(theta);
	}
	return _referencePoint;
}

point_t motion::PIDGeometry::getClosestTrajectoryPoint()
{
	if (!_closestTrajectoryPointIsValid) {
		_closestTrajectoryPoint = this->getReferencePoint();
		if (_trajectory.size() >= 2) {
			point_arr::iterator closestWaypointIterator = _trajectory.begin();
			point_arr::iterator secondClosestWaypointIterator = _trajectory.begin()+1;
			double closestDistance = motion::point_distance(this->getReferencePoint(), *closestWaypointIterator);
			double secondClosestDistance = motion::point_distance(this->getReferencePoint(), *secondClosestWaypointIterator);
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
				double dist = motion::point_distance(this->getReferencePoint(), *it);
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
			
			double lengthBetweenWaypoints = motion::point_distance(closestPoint, secondClosestPoint);
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

double motion::PIDGeometry::getReferenceDistance(point_t &transformedTrajectoryPoint, point_t &transformedReferencePoint)
{
	//Transform points to robot-centered coordinate frame
	//Depending on where (left or right) closest trajectory point is
	//make reference distance positive or negative
	
	//Transformation
	point_t trajectoryPoint = this->getClosestTrajectoryPoint();
	trajectoryPoint.x -= _currentPose.position.x;
	trajectoryPoint.y -= _currentPose.position.y;
	trajectoryPoint = rotate_point(trajectoryPoint, tf::getYaw(_currentPose.orientation), CW);
	
	transformedReferencePoint = this->getReferencePoint();
	transformedReferencePoint.x -= _currentPose.position.x;
	transformedReferencePoint.y -= _currentPose.position.y;
	transformedReferencePoint = rotate_point(transformedReferencePoint, tf::getYaw(_currentPose.orientation), CW);
	
	int multiplier = trajectoryPoint.y >= 0 ? -1 : 1;
	transformedTrajectoryPoint = trajectoryPoint;
	
	return motion::point_distance(this->getClosestTrajectoryPoint(), this->getReferencePoint()) * multiplier;
}


//---------------- Getters / Setters ----------------------------//

void motion::PIDGeometry::setCurrentPose(pose_t currentPose)
{
	_referencePointIsValid = false;
	_closestTrajectoryPointIsValid = false;
	_currentPose = currentPose;
}

void motion::PIDGeometry::setLinearVelocity(double velocity)
{
	_referencePointIsValid = false;
	_closestTrajectoryPointIsValid = false;
	_linearVelocity = velocity;
}

void motion::PIDGeometry::setTrajectory(point_arr trajectory)
{
	_referencePointIsValid = false;
	_closestTrajectoryPointIsValid = false;
	_trajectory = trajectory;
}

void motion::PIDGeometry::setVelocityMultiplier(float velocityMultiplier)
{
	_referencePointIsValid = false;
	_closestTrajectoryPointIsValid = false;
	_velocityMultiplier = velocityMultiplier;
}

void motion::PIDGeometry::setVelocityOffset(float velocityOffset)
{
	_referencePointIsValid = false;
	_closestTrajectoryPointIsValid = false;
	_velocityOffset = velocityOffset;
}

//---------------- Private methods ------------------------------//

//Angle at point1 
double motion::PIDGeometry::getReferenceAngle(point_t point1, point_t point2, point_t point3)
{
	double edge1 = motion::point_distance(point1, point3);
	double edge2 = motion::point_distance(point1, point2);
	double edge3 = motion::point_distance(point2, point3);
	return this->getReferenceAngle(edge1, edge2, edge3);
}

//Angle between edge1 and edge2
double motion::PIDGeometry::getReferenceAngle(double edge1, double edge2, double edge3)
{
	//Cosine rule
	return acos((pow(edge2,2)+pow(edge1,2)-pow(edge3,2))/(2*edge2*edge1));
}

//---------------- Non Class methods ----------------------------//

double motion::point_distance(point_t p1, point_t p2)
{
	return sqrt(pow(p2.x-p1.x, 2)+pow(p2.y-p1.y, 2));
}
