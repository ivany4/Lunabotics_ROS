#!/bin/bash 

elias() {
	CURRENT_PATH=`pwd`
	roscd lunabotics
	./lunabotics $1 $2 $3 $4 $5 $6 $7 $8
	cd $CURRENT_PATH
}

lunastage() {
	rosrun stage stageros `rospack find lunabotics`/stage.world
}

lunagazebo() {
	gazebo `rospack find lunabotics`/gazebo/worlds/cubic.sdf
}

lunamoon() {
	gazebo `rospack find lunabotics`/gazebo/worlds/moon.sdf
}

lunactrl() {
	rosrun teleop_base teleop_base_keyboard base_controller/command:=$1
}

export GAZEBO_PLUGIN_PATH=~/local/lib/gazebo_plugins:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=`rospack find lunabotics`/gazebo/models:$GAZEBO_MODEL_PATH
#export GAZEBO_MODEL_DATABASE_URI=""
