#ifndef _ALL_WHEEL_PREDEFINED_CMD_CONTROLLER_H_
#define _ALL_WHEEL_PREDEFINED_CMD_CONTROLLER_H_

#include "lunabotics/AllWheelState.h"
#include "../../protos_gen/AllWheelControl.pb.h"
#include "../geometry/allwheel.h"

namespace lunabotics {
	
#define DEFAULT_WHEEL_VELOCITY	3

enum AwaitingState {
	AwaitingDriving,
	AwaitingSteering,
	AwaitingNone
};
	
class PredefinedCmdController {
	private:
		lunabotics::AllWheelState final_state; 
		bool has_final_state;
		lunabotics::proto::AllWheelControl::PredefinedControlType _current_cmd;
		bool is_final_state;
		bool state_reached;
		AllWheelGeometryPtr _geometry;
		lunabotics::AllWheelState previousState;
		double angular_velocity;
		AwaitingState awaitingState;
		bool needPreviousState;
		void setAwaitingState(AwaitingState newState);
		void setFinalState(lunabotics::AllWheelState finalState);
		void removeFinalState();
		
	public:
		PredefinedCmdController();
		~PredefinedCmdController();
		bool needsMoreControl();
		bool control(lunabotics::AllWheelState &signal);
		void setNewCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd);
		void giveFeedback(lunabotics::AllWheelState state);
		void setGeometry(AllWheelGeometryPtr geometry);
		void setWheelVelocity(double velocity);
};
typedef PredefinedCmdController * PredefinedCmdControllerPtr;

}



#endif
