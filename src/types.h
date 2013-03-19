#ifndef _TYPES_H_
#define _TYPES_H_

#include "geometry_msgs/Point.h"

typedef std::vector<int8_t> map_grid;
typedef std::vector<geometry_msgs::Point> point_arr;


enum CTRL_MODE_TYPE {
    ACKERMANN 		 = 0,
    TURN_IN_SPOT     = 1,
    LATERAL   		 = 2
};

std::string controlModeToString(CTRL_MODE_TYPE type);

#endif //_TYPES_H_
