#include "Trajectory.h"
using namespace lunabotics;

Trajectory::Trajectory():_cached_points(), _segments(), _cached_max_curvature(-1), _segments_per_curve(20)
{
}

Trajectory::Trajectory(int num_segments):_cached_points(), _segments(), _cached_max_curvature(-1), _segments_per_curve(num_segments)
{
}

Trajectory::~Trajectory()
{
}

TrajectorySegmentArr Trajectory::segments()
{
	return this->_segments;
}

void Trajectory::setSegments(TrajectorySegmentArr segments)
{
	this->_segments = segments;
}

void Trajectory::appendSegment(TrajectorySegment s)
{
	this->_segments.push_back(s);
}
	
PointArr Trajectory::points()
{
	if (this->_cached_points.empty()) {
		for (TrajectorySegmentArr::iterator it = _segments.begin(); it < _segments.end(); it++) {
			TrajectorySegment s = *it;
			PointArr arr = s.curve->getPoints();
			this->_cached_points.insert(this->_cached_points.end(), arr.begin(), arr.end());
		}
	}
	return this->_cached_points;
}

float Trajectory::maxCurvature()
{
	if (this->_cached_max_curvature < 0) {
		for (TrajectorySegmentArr::iterator it = _segments.begin(); it < _segments.end(); it++) {
			TrajectorySegment s = *it;
			double curvature = s.curve->maxCurvature();
			if (this->_cached_max_curvature < curvature) {
				this->_cached_max_curvature = curvature;
			}
		}		
	}
	return this->_cached_max_curvature;
}
