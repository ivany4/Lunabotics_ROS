#ifndef _ALL_WHEEL_PREDEFINED_CMD_CONTROLLER_H_
#define _ALL_WHEEL_PREDEFINED_CMD_CONTROLLER_H_

#include "lunabotics/AllWheelStateROS.h"
#include "../../protos_gen/AllWheelControl.pb.h"
#include "../geometry/allwheel.h"

namespace lunabotics {
	namespace control {
		class PredefinedCmdController {
			private:
				int _cnt;
				bool _expecting_result;
				lunabotics::AllWheelStateROS _expected_state; 
				lunabotics::AllWheelControl::PredefinedControlType _current_cmd;
				bool _state_updated;
				geometry::AllWheelGeometryPtr _geometry;
				void incrementCnt();
			public:
				PredefinedCmdController();
				~PredefinedCmdController();
				bool needsMoreControl();
				bool control(lunabotics::AllWheelStateROS &signal);
				void setNewCommand(lunabotics::AllWheelControl::PredefinedControlType cmd);
				void giveFeedback(lunabotics::AllWheelStateROS state);
				void setGeometry(geometry::AllWheelGeometryPtr geometry);
		};
		typedef PredefinedCmdController * PredefinedCmdControllerPtr;
		
	}
}



#endif
