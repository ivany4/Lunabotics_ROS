#ifndef _ALL_WHEEL_PREDEFINED_CMD_CONTROLLER_H_
#define _ALL_WHEEL_PREDEFINED_CMD_CONTROLLER_H_

#include "lunabotics/AllWheelState.h"
#include "../../protos_gen/AllWheelControl.pb.h"
#include "../geometry/allwheel.h"

namespace lunabotics {
	
#define DEFAULT_WHEEL_VELOCITY	3

enum AwaitingState {
	AwaitingStop,
	AwaitingSteer,
	AwaitingDrive,
	AwaitingNothing
};
	
class PredefinedCmdController {
	private:
		lunabotics::AllWheelState final_state; 
		bool has_final_state;
		lunabotics::proto::AllWheelControl::PredefinedControlType _current_cmd;
		bool state_reached;
		bool force_state;
		AllWheelGeometryPtr _geometry;
		lunabotics::AllWheelState previousState;
		double angular_velocity;
		AwaitingState awaitingState;
		bool needPreviousState;
		void setAwaitingState(AwaitingState newState);
		void setFinalState(lunabotics::AllWheelState finalState);
		void removeFinalState();
		void copySteeringAngles(lunabotics::AllWheelState source, lunabotics::AllWheelState &destination);
		void copyDrivingVelocities(lunabotics::AllWheelState source, lunabotics::AllWheelState &destination);
	public:
		PredefinedCmdController();
		~PredefinedCmdController();
		bool needsMoreControl();
		bool isFinalState();
		bool control(lunabotics::AllWheelState &signal);
		void abort();
		void setNewCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd);
		void setNewCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd, bool safeSwitch);
		void giveFeedback(lunabotics::AllWheelState state);
		void setGeometry(AllWheelGeometryPtr geometry);
		void setWheelVelocity(double velocity);
		lunabotics::AllWheelState stateForCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd);
		lunabotics::AllWheelState standByState();
};
typedef PredefinedCmdController * PredefinedCmdControllerPtr;

}



#endif
