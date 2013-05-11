#include "Cartographer.h"

using namespace lunabotics;

Cartographer::Cartographer(int argc, char **argv): NodeAssistant(argc, argv)
{
}

Cartographer::~Cartographer()
{
}

void Cartographer::exec()
{
	ROS_INFO("Beat!");
}
