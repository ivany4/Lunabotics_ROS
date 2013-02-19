Put following code to the ~/.bashrc file
elias() {
  cd ~/ROS/pkgs/lunabotics
	./lunabotics $1 $2 $3 $4 $5 $6 $7 $8
}

Run . ~/.bashrc

Now you can use commands for launching and building like "elias -n -p 1234"

Use profiles for different hardware. E.g, "elias -r Pioneer" will use the script for Pionner robot, whereas VMWare is the default profile and is intented to support generic Ubuntu 12.04 ros configuration
