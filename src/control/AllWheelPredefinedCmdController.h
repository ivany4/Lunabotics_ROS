#ifndef _ALL_WHEEL_PREDEFINED_CMD_CONTROLLER_H_
#define _ALL_WHEEL_PREDEFINED_CMD_CONTROLLER_H_

#include "lunabotics/AllWheelState.h"
#include "../../protos_gen/AllWheelControl.pb.h"
#include "../geometry/allwheel.h"

namespace lunabotics {
	namespace control {
		class PredefinedCmdController {
			private:
				int _cnt;
				bool _expecting_result;
				lunabotics::AllWheelState _expected_state; 
				lunabotics::proto::AllWheelControl::PredefinedControlType _current_cmd;
				bool _state_updated;
				geometry::AllWheelGeometryPtr _geometry;
				void incrementCnt();
			public:
				PredefinedCmdController();
				~PredefinedCmdController();
				bool needsMoreControl();
				bool control(lunabotics::AllWheelState &signal);
				void setNewCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd);
				void giveFeedback(lunabotics::AllWheelState state);
				void setGeometry(geometry::AllWheelGeometryPtr geometry);
		};
		typedef PredefinedCmdController * PredefinedCmdControllerPtr;
		
	}
}



#endif
