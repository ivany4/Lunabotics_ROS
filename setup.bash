#!/bin/bash 

elias() {
	roscd lunabotics
	./lunabotics $1 $2 $3 $4 $5 $6 $7 $8
}

lunastage() {
	rosrun stage stageros `rospack find lunabotics`/stage.world
}

lunagazebo() {
	gazebo `rospack find lunabotics`/gazebo/worlds/empty.world
}

lunactrl() {
	rosrun teleop_base teleop_base_keyboard base_controller/command:=$1
}
