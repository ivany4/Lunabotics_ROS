#include "allwheel.h"
#include "ros/ros.h"
#include "basic.h"
using namespace lunabotics;


AllWheelGeometry::AllWheelGeometry(Point left_front, Point left_rear, Point right_front, Point right_rear):need_max_dimensions_update(false), geometryAcquired(false) {
	this->joints.left_front = left_front;
	this->joints.left_rear = left_rear;
	this->joints.right_front = right_front;
	this->joints.right_rear = right_rear;
}

AllWheelGeometry::AllWheelGeometry(AllWheelGeometry *copy) 
{
	this->joints = copy->getJoints();
	this->max_dimensions = copy->getMaxDimensions();
	this->wheel_offset = copy->getWheelOffset();
	this->wheel_radius = copy->getWheelRadius();
	this->geometryAcquired = copy->geometryAcquired;
	this->need_max_dimensions_update = false;
}

AllWheelGeometry::~AllWheelGeometry()
{
}

bool AllWheelGeometry::calculateAngles(Point ICR, float &left_front, float &right_front, float &left_rear, float &right_rear)
{
//	ROS_INFO("Joint positions (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)", this->joints.left_front.x, this->joints.left_front.y, this->joints.right_front.x, this->joints.right_front.y, this->joints.left_rear.x, this->joints.left_rear.y, this->joints.right_rear.x, this->joints.right_rear.y);
	
	bool ICROnRight = ICR.y < this->joints.right_front.y;
	bool ICROnLeft = ICR.y > this->joints.left_front.y;
	bool ICROnTop = ICR.x > this->joints.left_front.x;
	bool ICROnBottom = ICR.x < this->joints.left_rear.x;
	
	if ((ICROnTop || ICROnBottom) && !(ICROnRight || ICROnLeft)) {
		ROS_ERROR("Unhandled ICR position");
		return false;
	}
	else {
		if (ICROnRight) {
			//ICR is on the right from the robot
		//	ROS_INFO("ICR on the right");
			
			double offset = -ICR.y+this->joints.right_front.y;
		//	ROS_INFO("Right offset %f top %f bottom %f", offset, this->joints.right_front.x-ICR.y, -this->joints.right_rear.x+ICR.y);
			right_front = -atan2(this->joints.right_front.x-ICR.x, offset);
			right_rear = atan2(-this->joints.right_rear.x+ICR.x, offset);
			offset += (this->joints.left_front.y-this->joints.right_front.y);
		//	ROS_INFO("Left offset %f top %f bottom %f", offset, this->joints.left_front.x-ICR.y, -this->joints.left_rear.x+ICR.y);
			left_front = -atan2(this->joints.left_front.x-ICR.x, offset);
			left_rear = atan2(-this->joints.left_rear.x+ICR.x, offset);
		}
		else if (ICROnLeft) {
			//ICR is on the left from the robot
		//	ROS_INFO("ICR on the left");
			
			double offset = ICR.y-this->joints.left_front.y;
			left_front = atan2(this->joints.left_front.x-ICR.x, offset);
			left_rear = -atan2(-this->joints.left_rear.x+ICR.x, offset);
		//	ROS_INFO("Left offset %f top %f bottom %f", offset, this->joints.left_front.x-ICR.y, -this->joints.left_rear.x+ICR.y);
			offset += (this->joints.left_front.y-this->joints.right_front.y);
			right_front = atan2(this->joints.right_front.x-ICR.x, offset);
			right_rear = -atan2(-this->joints.right_rear.x+ICR.x, offset);
	//		ROS_INFO("Right offset %f top %f bottom %f", offset, this->joints.right_front.x-ICR.y, -this->joints.right_rear.x+ICR.y);
		}
		else {
			//ICR is underneath the robot
		//	ROS_INFO("ICR in between");
			
			double offset = this->joints.left_front.y-ICR.y;
			left_front = -atan2(this->joints.left_front.x-ICR.x, offset);
			left_rear = atan2(-this->joints.left_rear.x+ICR.x, offset);
	//		ROS_INFO("Left offset %f top %f bottom %f", offset, this->joints.left_front.x-ICR.y, -this->joints.left_rear.x+ICR.y);
			offset = -this->joints.right_front.y+ICR.y;
			right_front = atan2(this->joints.right_front.x-ICR.x, offset);
			right_rear = -atan2(-this->joints.right_rear.x+ICR.x, offset);
	//		ROS_INFO("Right offset %f top %f bottom %f", offset, this->joints.right_front.x-ICR.y, -this->joints.right_rear.x+ICR.y);
		}
	//	ROS_INFO("Calculated angles are %.2f | %.2f | %.2f | %.2f", left_front, right_front, left_rear, right_rear);
	
	}
	return true;
}

bool AllWheelGeometry::calculateVelocities(Point ICR, float center_velocity, float &left_front, float &right_front, float &left_rear, float &right_rear)
{
	//Coordinate frames are different
	
//	ROS_INFO("Joint positions (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)", this->joints.left_front.x, this->joints.left_front.y, this->joints.right_front.x, this->joints.right_front.y, this->joints.left_rear.x, this->joints.left_rear.y, this->joints.right_rear.x, this->joints.right_rear.y);
	bool ICROnRight = ICR.y < this->joints.right_front.y;
	bool ICROnLeft = ICR.y > this->joints.left_front.y;
	bool ICROnTop = ICR.x > this->joints.left_front.x;
	bool ICROnBottom = ICR.x < this->joints.left_rear.x;
	
	if ((ICROnTop || ICROnBottom) && !(ICROnRight || ICROnLeft)) {
		ROS_ERROR("Unhandled ICR position");
		return false;
	}
	else {		
		double left_front_shoulder = distance(ICR, this->joints.left_front);
		double right_front_shoulder = distance(ICR, this->joints.right_front);
		double left_rear_shoulder = distance(ICR, this->joints.left_rear);
		double right_rear_shoulder = distance(ICR, this->joints.right_rear);
		
	//	ROS_INFO("Shoulders are are %.2f | %.2f | %.2f | %.2f", left_front_shoulder, right_front_shoulder, left_rear_shoulder, right_rear_shoulder);
		
		if (ICROnRight) {
			//ROS_INFO("ICR on the right");
			right_front_shoulder -= this->wheel_offset;
			right_rear_shoulder -= this->wheel_offset;
			left_front_shoulder += this->wheel_offset;
			left_rear_shoulder += this->wheel_offset;
		}
		else if (ICROnLeft) {
			//ICR is on the left from the robot
			//ROS_INFO("ICR on the left");
			right_front_shoulder += this->wheel_offset;
			right_rear_shoulder += this->wheel_offset;
			left_front_shoulder -= this->wheel_offset;
			left_rear_shoulder -= this->wheel_offset;
		}
		else {
			//ICR is underneath the robot
			//ROS_INFO("ICR in between");
			right_front_shoulder += this->wheel_offset;
			right_rear_shoulder += this->wheel_offset;
			left_front_shoulder += this->wheel_offset;
			left_rear_shoulder += this->wheel_offset;
		}
		
		//ROS_INFO("Shoulders are are %.2f | %.2f | %.2f | %.2f", left_front_shoulder, right_front_shoulder, left_rear_shoulder, right_rear_shoulder);
		
		Point zeroPoint; zeroPoint.x = 0; zeroPoint.y = 0;
		double center_shoulder = distance(ICR, zeroPoint);
		double ang_vel = center_velocity/center_shoulder;
		
		double left_front_vel = left_front_shoulder*ang_vel;
		double right_front_vel = right_front_shoulder*ang_vel;
		double left_rear_vel = left_rear_shoulder*ang_vel;
		double right_rear_vel = right_rear_shoulder*ang_vel;
		
		left_front = left_front_vel/this->wheel_radius;
		right_front = right_front_vel/this->wheel_radius;
		left_rear = left_rear_vel/this->wheel_radius;
		right_rear = right_rear_vel/this->wheel_radius;
		
		if (!ICROnLeft && !ICROnRight) {
			if (ICR.y < 0) {
				right_front *= -1;
				right_rear *= -1;
			}
			else {
				left_front *= -1;
				left_rear *= -1;
			}
		}
		
		if (isinf(left_front) || isnan(left_front) || isinf(right_front) || isnan(right_front) || isinf(left_rear) || isnan(left_rear) || isinf(right_rear) || isnan(right_rear)) {
	//		ROS_INFO("INF or NAN");
			left_front = right_front = left_rear = right_rear = 0;	
		}
				
	//	ROS_INFO("Calculated velocities are %.2f | %.2f | %.2f | %.2f", left_front, right_front, left_rear, right_rear);
	}
	return true;
}

void AllWheelGeometry::set_left_front(Point new_point)
{
	this->joints.left_front = new_point;
	this->need_max_dimensions_update = true;
}

void AllWheelGeometry::set_left_rear(Point new_point)
{
	this->joints.left_rear = new_point;
	this->need_max_dimensions_update = true;
}

void AllWheelGeometry::set_right_front(Point new_point)
{
	this->joints.right_front = new_point;
	this->need_max_dimensions_update = true;
}

void AllWheelGeometry::set_right_rear(Point new_point)
{
	this->joints.right_rear = new_point;
	this->need_max_dimensions_update = true;
}

void AllWheelGeometry::set_wheel_offset(float new_offset)
{
	this->wheel_offset = new_offset;
	this->need_max_dimensions_update = true;
}

void AllWheelGeometry::set_wheel_radius(float new_radius)
{
	this->wheel_radius = new_radius;
}

void AllWheelGeometry::set_wheel_width(float new_width)
{
	this->wheel_width = new_width;
	this->need_max_dimensions_update = true;
}

Rect AllWheelGeometry::getJoints()
{
	return this->joints;
}

Rect AllWheelGeometry::getMaxDimensions()
{
	if (this->need_max_dimensions_update) {
		this->max_dimensions.left_front.x = this->joints.left_front.x+this->wheel_offset+this->wheel_width/2;
		this->max_dimensions.left_front.y = this->joints.left_front.y+this->wheel_offset+this->wheel_width/2;
		this->max_dimensions.right_front.x = this->joints.right_front.x+this->wheel_offset+this->wheel_width/2;
		this->max_dimensions.right_front.y = this->joints.right_front.y-this->wheel_offset-this->wheel_width/2;
		this->max_dimensions.left_rear.x = this->joints.left_rear.x-this->wheel_offset-this->wheel_width/2;
		this->max_dimensions.left_rear.y = this->joints.left_rear.y+this->wheel_offset+this->wheel_width/2;
		this->max_dimensions.right_rear.x = this->joints.right_rear.x-this->wheel_offset-this->wheel_width/2;
		this->max_dimensions.right_rear.y = this->joints.right_rear.y-this->wheel_offset-this->wheel_width/2;
		this->need_max_dimensions_update = false;
	}
	return this->max_dimensions;
}

float AllWheelGeometry::getWheelOffset()
{
	return this->wheel_offset;
}

float AllWheelGeometry::getWheelRadius()
{
	return this->wheel_radius;
}

float AllWheelGeometry::getWheelWidth()
{
	return this->wheel_width;
}

bool lunabotics::validateAngles(float &left_front, float &right_front, float &left_rear, float &right_rear)
{
	bool result = true;
	if (left_front > GEOMETRY_INNER_ANGLE_MAX) {
		result = false;
		left_front = GEOMETRY_INNER_ANGLE_MAX;
	}
	else if (left_front < -GEOMETRY_OUTER_ANGLE_MAX) {
		result = false;
		left_front = -GEOMETRY_OUTER_ANGLE_MAX;
	}
	
	if (right_front > GEOMETRY_OUTER_ANGLE_MAX) {
		result = false;
		right_front = GEOMETRY_OUTER_ANGLE_MAX;
	}
	else if (right_front < -GEOMETRY_INNER_ANGLE_MAX) {
		result = false;
		right_front = -GEOMETRY_INNER_ANGLE_MAX;
	}
	
	if (left_rear > GEOMETRY_OUTER_ANGLE_MAX) {
		result = false;
		left_rear = GEOMETRY_OUTER_ANGLE_MAX;
	}
	else if (left_rear < -GEOMETRY_INNER_ANGLE_MAX) {
		result = false;
		left_rear = -GEOMETRY_INNER_ANGLE_MAX;
	}
	
	if (right_rear > GEOMETRY_INNER_ANGLE_MAX) {
		result = false;
		right_rear = GEOMETRY_INNER_ANGLE_MAX;
	}
	else if (right_rear < -GEOMETRY_OUTER_ANGLE_MAX) {
		result = false;
		right_rear = -GEOMETRY_OUTER_ANGLE_MAX;
	}
	return result;
}

Point AllWheelGeometry::point_outside_base_link(Point ICR)
{
	if (ICR.y < 0 && ICR.y > this->joints.right_front.y) {
		ICR.y = this->joints.right_front.y-this->wheel_offset/2;
	}
	else if (ICR.y >= 0 && ICR.y < this->joints.left_front.y) {
		ICR.y = this->joints.left_front.y+this->wheel_offset/2;
	}
	return ICR;
}

float AllWheelGeometry::maxAvailableCurvature()
{
	return 1/(tan(GEOMETRY_INNER_ANGLE_MAX)*this->joints.right_front.x);
}
