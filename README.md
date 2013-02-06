Put following code to the ~/.bashrc file
elias() {
  cd ~/ROS/pkgs/lunabotics
	./lunabotics $1 $2 $3 $4 $5 $6 $7 $8
}

Run . ~/.bashrc

Now you can use commands for launching and building like "elias -n -p 1234"

Note:
If lunabotics package path is different change it in the following function and in lunabotics/lunabotics file
