#include "types.h"

std::string controlModeToString(lunabotics::SteeringModeType type)
{
	switch (type) {
		case lunabotics::ACKERMANN: return "Ackermann";
		case lunabotics::TURN_IN_SPOT: return "'Turn in spot'";
		case lunabotics::CRAB: return "Crab";
	}
	return "Undefined";
}
