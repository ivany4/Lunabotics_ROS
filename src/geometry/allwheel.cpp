#include "allwheel.h"
#include "ros/ros.h"

geometry::AllWheelGeometry::AllWheelGeometry(point_t left_front, point_t left_rear, point_t right_front, point_t right_rear): lf(left_front), lr(left_rear), rf(right_front), rr(right_rear) {
}

geometry::AllWheelGeometry::~AllWheelGeometry()
{
}

bool geometry::AllWheelGeometry::calculateAngles(point_t ICR, float &left_front, float &right_front, float &left_rear, float &right_rear)
{
	ROS_INFO("Joint positions (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)", this->lf.x, this->lf.y, this->rf.x, this->rf.y, this->lr.x, this->lr.y, this->rr.x, this->rr.y);
	
	if (ICR.y > this->lf.x || ICR.y < this->lr.x) {
		ROS_ERROR("Unhandled ICR position");
		return false;
	}
	else {
		if (-this->rf.y < ICR.x) {
			//ICR is on the right from the robot
			ROS_INFO("ICR on the right");
			
			double offset = ICR.x+this->rf.y;
			ROS_INFO("Right offset %f top %f bottom %f", offset, this->rf.x-ICR.y, -this->rr.x+ICR.y);
			right_front = -atan2(this->rf.x-ICR.y, offset);
			right_rear = atan2(-this->rr.x+ICR.y, offset);
			offset += (this->lf.y-this->rf.y);
			ROS_INFO("Left offset %f top %f bottom %f", offset, this->lf.x-ICR.y, -this->lr.x+ICR.y);
			left_front = -atan2(this->lf.x-ICR.y, offset);
			left_rear = atan2(-this->lr.x+ICR.y, offset);
		}
		else if (ICR.x < -this->lf.y) {
			//ICR is on the left from the robot
			ROS_INFO("ICR on the left");
			
			double offset = -this->lf.y-ICR.x;
			left_front = atan2(this->lf.x-ICR.y, offset);
			left_rear = -atan2(-this->lr.x+ICR.y, offset);
			ROS_INFO("Left offset %f top %f bottom %f", offset, this->lf.x-ICR.y, -this->lr.x+ICR.y);
			offset += (this->lf.y-this->rf.y);
			right_front = atan2(this->rf.x-ICR.y, offset);
			right_rear = -atan2(-this->rr.x+ICR.y, offset);
			ROS_INFO("Right offset %f top %f bottom %f", offset, this->rf.x-ICR.y, -this->rr.x+ICR.y);
		}
		else {
			//ICR is underneath the robot
			ROS_INFO("ICR in between");
			
			double offset = this->lf.y+ICR.x;
			left_front = -atan2(this->lf.x-ICR.y, offset);
			left_rear = atan2(-this->lr.x+ICR.y, offset);
			ROS_INFO("Left offset %f top %f bottom %f", offset, this->lf.x-ICR.y, -this->lr.x+ICR.y);
			offset = -this->rf.y+ICR.x;
			right_front = atan2(this->rf.x-ICR.y, offset);
			right_rear = -atan2(-this->rr.x+ICR.y, offset);
			ROS_INFO("Right offset %f top %f bottom %f", offset, this->rf.x-ICR.y, -this->rr.x+ICR.y);
		}
		ROS_INFO("Calculated angles are %.2f | %.2f | %.2f | %.2f", left_front, right_front, left_rear, right_rear);
	}
	return true;
}

void geometry::AllWheelGeometry::set_left_front(point_t new_point)
{
	this->lf = new_point;
}

void geometry::AllWheelGeometry::set_left_rear(point_t new_point)
{
	this->lr = new_point;
}

void geometry::AllWheelGeometry::set_right_front(point_t new_point)
{
	this->rf = new_point;
}

void geometry::AllWheelGeometry::set_right_rear(point_t new_point)
{
	this->rr = new_point;
}

point_t geometry::AllWheelGeometry::left_front()
{
	return this->lf;
}
point_t geometry::AllWheelGeometry::left_rear()
{
	return this->lr;
}
point_t geometry::AllWheelGeometry::right_front()
{
	return this->rf;
}
point_t geometry::AllWheelGeometry::right_rear()
{
	return this->rr;
}
