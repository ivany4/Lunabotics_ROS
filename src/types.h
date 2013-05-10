#ifndef _TYPES_H_
#define _TYPES_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "../protos_gen/SteeringModeType.pb.h"
#include "tf/tf.h"

enum ROTATION_DIRECTION {
	CW = 1,
	CCW = -1
};

inline std::string controlModeToString(lunabotics::proto::SteeringModeType type)
{
	switch (type) {
		case lunabotics::proto::ACKERMANN: return "Ackermann";
		case lunabotics::proto::TURN_IN_SPOT: return "'Turn in spot'";
		case lunabotics::proto::CRAB: return "Crab";
	}
	return "Undefined";
}

//New version
namespace lunabotics {


struct Point {
	float x;
	float y;
};

struct Pose {
	Point position;
	float orientation;
};

struct IndexedPoint {
	Point point;
	int index;
};

inline Point CreatePoint(float x, float y)
{
	Point result;
	result.x = x;
	result.y = y;
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

typedef std::vector<Point> PointArr;
typedef std::vector<Pose> PoseArr;
typedef std::vector<int8_t> OccupancyArr;
typedef std::vector<IndexedPoint> IndexedPointArr;

}


#endif //_TYPES_H_
