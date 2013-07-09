#ifndef _TOPICS_H_
#define _TOPICS_H_

namespace lunabotics {

//Telemetry topics
#define TOPIC_TM_PATH					"tm/path"			//Transfer path points to show in GUI
#define TOPIC_TM_PATH_FOLLOWING 		"tm/path_following" //Path following details, including feedback and feedforward look-ahead point positions
#define TOPIC_TM_ROBOT_STATE			"tm/robot_state"	//Overall robot condition
#define TOPIC_TM_VISION					"tm/vision"			//Lidar and Kinect-related data collected from the sensors
#define TOPIC_TM_ICR					"tm/icr"			//ICR position to show in GUI
#define TOPIC_TM_ALL_WHEEL				"tm/all_wheel"		//Condition of steering and driving motors of all-wheeler
#define TOPIC_TM_ROBOT_GEOMETRY			"tm/robot_geometry" //Positions of steering joints and other info of all-wheeler
#define TOPIC_TM_ODOMETRY				"/odom"				//Odometry of a robot
#define TOPIC_TM_LIDAR					"/base_scan"		//Laser scan

//Command topics
#define TOPIC_CMD_UPDATE_MAP			"cmd/map_update"	//Request a new map
#define TOPIC_CMD_AUTONOMY				"cmd/autonomy"		//Disable autonomous driving
#define TOPIC_CMD_GOAL					"cmd/goal"			//Define a driving goal
#define	TOPIC_CMD_PATH_FOLLOWING 		"cmd/path_following" //Define PID gains and feedback,feedforward offsets
#define TOPIC_CMD_ICR					"cmd/icr"			//Define an ICR-centered motion
#define TOPIC_CMD_CRAB					"cmd/crab"			//Define a crab motion
#define TOPIC_CMD_ALL_WHEEL				"cmd/all_wheel" 	//Define a predefined driving command of an all-wheeler
#define TOPIC_CMD_EXPLICIT_ALL_WHEEL	"cmd/explicit_all_wheel" //Define an explicit command for each motor
#define TOPIC_CMD_TWIST					"/cmd_vel"			//Motion of a differential-drive robot
#define TOPIC_CMD_TELEOP				"cmd/teleop"		//Control robot from a joystick
#define TOPIC_CMD_CONN					"cmd/connection"			//Port of the GUI to reply to

//Common topics (work both ways)
#define TOPIC_EMERGENCY					"emergency"			//To trigger obstacle avoidance routine
#define TOPIC_STEERING_MODE 			"steering_mode"		//Commanded from GUI and reported to GUI as well



//Services
#define SERVICE_MAP						"map"	
#define SERVICE_MAP_GMAPPING			"/dynamic_map"
}


#endif 
