#include "ros/ros.h"
#include "MotionControlNode.h"

int main(int argc, char **argv)
{
	lunabotics::MotionControlNode *assistant = new lunabotics::MotionControlNode(argc, argv, "luna_driver", 200);
	assistant->run();
	delete assistant;
	
	return 0;
}
