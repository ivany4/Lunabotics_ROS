# This contains information about emergency type

uint8 reason					# 0 - unknown, 1 - proximity, 2 - crater

# Emergency caused by proximity alert
float32 proximity_distance		# distance to closest point
float32 proximity_angle			# angle of closest point

# Emergency caused by detection of crater
uint8 is_crater_alert					# 1 if yes 0 if no
geometry_msgs/Point crater_position		# Position of a crater on the map
