This package uses Google Protocol Buffers for transmission. If you don't have Gazebo installed, likely you need to install protocol buffers.
run apt-cache search protobuf

and install packages like:
libprotobuf
libprotobuf-dev
libprotoc-dev

using: sudo apt-get install <package name> <additional package name>


run the following command:
echo "source `rospack find lunabotics`/setup.bash" >> ~/.bashrc
. ~/.bashrc



Now you can use commands for launching and building like "elias" as well as shortcuts for simulators and teleoperation:
lunastage
lunagazebo
lunactrl /cmd_vel

Use profiles for different hardware. E.g, "elias -r Pioneer" will use the script for Pionner robot, whereas VMWare is the default profile and is intented to support generic Ubuntu 12.04 ros configuration
