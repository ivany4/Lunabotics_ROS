# Robot State from Mech Interface

# Wheel rotation angles
float32 wheel_roll_angle_front_left
float32 wheel_roll_angle_front_right
float32 wheel_roll_angle_rear_left
float32 wheel_roll_angle_rear_right

# Wheel steering angles
float32 wheel_yaw_angle_front_left
float32 wheel_yaw_angle_front_right
float32 wheel_yaw_angle_rear_left
float32 wheel_yaw_angle_rear_right

# Currents of driving motors
float32 motor_driving_current_front_left
float32 motor_driving_current_front_right
float32 motor_driving_current_rear_left
float32 motor_driving_current_rear_right

# Currents of steering motors
float32 motor_steering_current_front_left
float32 motor_steering_current_front_right
float32 motor_steering_current_rear_left
float32 motor_steering_current_rear_right

float32 front_axis_tilt		# Tilt of the front wheel axis
float32 laser_tilt			# Laser tilt level

sensor_msgs/Imu imu_data	# Accelerometer + Gyro data (Orientation quaternion is not used)

std_msgs/Bool[] bumpers			# Triggered/Not triggered for each bumper

sensor_msgs/Range[] proximity_data	# Ranges for each of the proximity sensors

nav_msgs/Odometry odometry  # Odometry data

# Load information
float32[] weights			# Load info for each load sensor [kg/sensor]
float32 total_weight		# Total weight of the whole upper part
float32 regolith_weight		# Weight of the regolith in the bucket
float32 bucket_weight		# Weight of the bucket

# Battery information
float32[] voltages			# Voltage of each battery
float32[] remainings		# Remaining mA/h in each battery

# Actuators 
# TBD
