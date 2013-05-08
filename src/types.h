#ifndef _TYPES_H_
#define _TYPES_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "../protos_gen/SteeringModeType.pb.h"

typedef std::vector<int8_t> map_grid;
typedef	geometry_msgs::Point point_t;
typedef geometry_msgs::Pose pose_t;
typedef std::vector<point_t> point_arr;
typedef std::vector<pose_t> pose_arr;

struct point_indexed {
	point_t point;
	int index;
};

typedef std::vector<point_indexed> point_indexed_arr;

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

#endif //_TYPES_H_
