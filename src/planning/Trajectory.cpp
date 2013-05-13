#include "Trajectory.h"
#include "ros/ros.h"

using namespace lunabotics;

Trajectory::Trajectory():_cached_points(), _segments(), _cached_max_curvature(-1)
{
}

Trajectory::~Trajectory()
{
	this->freeSegments();
}

void Trajectory::freeSegments()
{
	for (TrajectorySegmentArr::iterator it = this->_segments.begin(); it < this->_segments.end(); it++) {
		TrajectorySegment segment = *it;
		delete segment.curve;
	}
	this->_segments.clear();
	this->_cached_points.clear();
}

TrajectorySegmentArr Trajectory::segments()
{
	return this->_segments;
}

void Trajectory::setSegments(TrajectorySegmentArr segments)
{
	this->freeSegments();
	for (TrajectorySegmentArr::iterator it = segments.begin(); it < segments.end(); it++) {
		this->appendSegment(*it);
	}
}

void Trajectory::appendSegment(TrajectorySegment &s)
{
	s.start_idx = this->_cached_points.size();
	PointArr arr = s.curve->getPoints();
	s.finish_idx = s.start_idx+arr.size();
	this->_cached_points.insert(this->_cached_points.end(), arr.begin(), arr.end());
	this->_segments.push_back(s);
}
	
PointArr Trajectory::getPoints()
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
