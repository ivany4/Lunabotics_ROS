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
	this->_segments.push_back(s);
	this->_cached_points.clear();
}
	
PointArr Trajectory::getPoints()
{
	if (this->_cached_points.empty()) {
		this->updateSegmentsMetaInfo(FLT_MAX);
	}
	return this->_cached_points;
}

void Trajectory::updateSegmentsMetaInfo(float max_curvature)
{
	this->_cached_points.clear();
	unsigned int i = 0;
	for (TrajectorySegmentArr::iterator it = _segments.begin(); it < _segments.end(); it++) {
		TrajectorySegment s = *it;
		PointArr arr;
		if (s.curve->maxCurvature() > max_curvature) {
			arr.push_back(s.curve->p0());
			arr.push_back(s.curve->p1());
			arr.push_back(s.curve->p2());
		}
		else {
			arr = s.curve->getPoints();
		}
	//	ROS_INFO("Setting start idx %d", s.start_idx);
		s.start_idx = this->_cached_points.size();
		s.finish_idx = s.start_idx+arr.size();
		this->_segments.at(i) = s;
		this->_cached_points.insert(this->_cached_points.end(), arr.begin(), arr.end());
		i++;
	}
}

void Trajectory::updateSegmentsMetaInfo()
{
	this->updateSegmentsMetaInfo(FLT_MAX);
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
