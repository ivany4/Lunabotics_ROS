#ifndef _TYPES_H_
#define _TYPES_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "../protos_gen/SteeringModeType.pb.h"
#include "tf/tf.h"
#include <iostream>

enum ROTATION_DIRECTION {
	CW = 1,
	CCW = -1
};

inline std::string steeringModeToString(lunabotics::proto::SteeringModeType type)
{
	switch (type) {
		case lunabotics::proto::ACKERMANN: return "Ackermann";
		case lunabotics::proto::POINT_TURN: return "Point-turn";
		case lunabotics::proto::AUTO: return "Automatic";
	}
	return "Undefined";
}

//New version
namespace lunabotics {


struct Point {
	float x;
	float y;
	bool operator==(const Point &b) const {return fabs(x-b.x) < 0.00001 && fabs(y-b.y) < 0.00001;}
	bool operator!=(const Point &b) const {return fabs(x-b.x) >= 0.00001 || fabs(y-b.y) >= 0.00001;}
	friend Point operator+(const Point &c1, const Point &c2);
	friend Point operator-(const Point &c1, const Point &c2);
	friend Point operator*(const double &c, const Point &p);
	friend Point operator*(const Point &p, const double &c);
	friend Point operator/(const double &c, const Point &p);
	friend Point operator/(const Point &p, const double &c);
	friend Point operator*(const int &c, const Point &p);
	friend Point operator*(const Point &p, const int &c);
	friend Point operator/(const int &c, const Point &p);
	friend Point operator/(const Point &p, const int &c);
	//std::ostream &operator<<(std::ostream &, const Point &p) { output << "(" << p.x << "," << p.y << ")"; return output; }
};

inline Point CreatePoint(float x, float y)
{
	Point result;
	result.x = x;
	result.y = y;
	return result;
}
inline Point CreateZeroPoint()
{
	return CreatePoint(0,0);
}

inline Point operator+(const Point &c1, const Point &c2){return CreatePoint(c1.x+c2.x, c1.y+c2.y);}
inline Point operator-(const Point &c1, const Point &c2){return CreatePoint(c1.x-c2.x, c1.y-c2.y);}
inline Point operator*(const double &c, const Point &p){return CreatePoint(p.x*c, p.y*c);}
inline Point operator*(const Point &p, const double &c){return CreatePoint(p.x*c, p.y*c);}
inline Point operator/(const double &c, const Point &p){return c==0?CreateZeroPoint():CreatePoint(p.x/c, p.y/c);}
inline Point operator/(const Point &p, const double &c){return c==0?CreateZeroPoint():CreatePoint(p.x/c, p.y/c);}
inline Point operator*(const int &c, const Point &p){return CreatePoint(p.x*c, p.y*c);}
inline Point operator*(const Point &p, const int &c){return CreatePoint(p.x*c, p.y*c);}
inline Point operator/(const int &c, const Point &p){return c==0?CreateZeroPoint():CreatePoint(p.x/c, p.y/c);}
inline Point operator/(const Point &p, const int &c){return c==0?CreateZeroPoint():CreatePoint(p.x/c, p.y/c);}

struct Rect {
	Point left_front;
	Point right_front;
	Point left_rear;
	Point right_rear;
};

struct Pose {
	Point position;
	float orientation;
};

struct IndexedPoint {
	Point point;
	int index;
	bool operator==(const IndexedPoint &b) const {return index == b.index;} //Used for 'find' function
};

inline IndexedPoint CreateIndexedPoint(Point p, int idx)
{
	IndexedPoint ip;
	ip.point = p;
	ip.index = idx;
	return ip;
}

inline Rect CreateRect(Point left_front, Point right_front, Point left_rear, Point right_rear)
{
	Rect result;
	result.left_front = left_front;
	result.right_front = right_front;
	result.left_rear = left_rear;
	result.right_rear = right_rear;
	return result;
}

inline geometry_msgs::Point geometry_msgs_Point_from_Point(Point p)
{
	geometry_msgs::Point result;
	result.x = p.x;
	result.y = p.y;
	return result;
}

inline Point Point_from_geometry_msgs_Point(geometry_msgs::Point p)
{
	Point result;
	result.x = p.x;
	result.y = p.y;
	return result;
}

inline geometry_msgs::Pose geometry_msgs_Pose_from_Pose(Pose p)
{
	geometry_msgs::Pose result;
	result.position = geometry_msgs_Point_from_Point(p.position);
	result.orientation = tf::createQuaternionMsgFromYaw(0);
	return result;
}

inline Pose Pose_from_geometry_msgs_Pose(geometry_msgs::Pose p)
{
	Pose result;
	result.position = Point_from_geometry_msgs_Point(p.position);
	result.orientation = tf::getYaw(p.orientation);
	return result;
}
	

struct Line {
	Point p1;
	Point p2;
	bool operator==(const Line &b) const {return p1 == b.p1 && p2 == b.p2;}
	bool operator!=(const Line &b) const {return p1 != b.p1 || p2 != b.p2;}
};

inline Line CreateLine(Point p1, Point p2)
{
	Line line;
	line.p1 = p1;
	line.p2 = p2;
	return line;
}

inline Line CreateLine(float x1, float y1, float x2, float y2)
{
	return CreateLine(CreatePoint(x1, y1), CreatePoint(x2, y2));
}


inline geometry_msgs::PoseStamped PoseStamped_from_Point(Point p, int &seq, std::string frame_id)
{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = seq++;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = frame_id;
	pose.pose.position = geometry_msgs_Point_from_Point(p);
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	return pose;
}


typedef std::vector<Point> PointArr;
typedef std::vector<Pose> PoseArr;
typedef std::vector<int8_t> OccupancyArr;
typedef std::vector<IndexedPoint> IndexedPointArr;
typedef PointArr::iterator PointArrIt;


inline bool pointAtIndex(IndexedPointArr arr, int idx, Point &p)
{
	IndexedPoint ip = CreateIndexedPoint(CreateZeroPoint(), idx);
	IndexedPointArr::iterator it = find(arr.begin(), arr.end(), ip);
	if (it != arr.end()) {
		p = (*it).point;
		//ROS_INFO("Extracting point at index %d = %.2f,%.2f", idx, p.x, p.y);
		return true;
	}
	//ROS_ERROR("Can't find point with index %d", idx);
	return false;
}

struct MapData {
	OccupancyArr cells;
	int width;
	int height;
	double resolution;
	int8_t at(int x, int y) {
		unsigned int idx = width*y+x;
		if (idx >= this->cells.size()) {
			ROS_ERROR("Trying to get cell beyond the map boundaries");
			return 0;
		}
		return this->cells.at(this->width*y+x);
	}
	int8_t at(Point p) {return this->at(p.x, p.y);}
};

inline int sign(double value, double accuracy) {
	if (value > accuracy) return 1;
	if (value < -accuracy) return -1;
	return 0;
}

inline int sign(double value) {
	return sign(value, 0.0001);
}

struct Triangle {
	Point p1;
	Point p2;
	Point p3;
};

inline Triangle CreateTriangle(Point p1, Point p2, Point p3) {
	Triangle t;
	t.p1 = p1;
	t.p2 = p2;
	t.p3 = p3;
	return t;
}

}


#endif //_TYPES_H_
