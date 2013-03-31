#include "types.h"

std::string controlModeToString(CTRL_MODE_TYPE type)
{
	switch (type) {
		case ACKERMANN: return "Ackermann";
		case TURN_IN_SPOT: return "'Turn in spot'";
		case LATERAL: return "Lateral";
	}
	return "Undefined";
}
