def home():
	global x, y, yaw

	delta_x = 0 - x
	delta_y = 0 - y
	distance = abs(math.sqrt((delta_x ** 2) + (delta_y ** 2)))
	desired_angle = 0.0

	if (x<0 and y<0):
		desired_angle = math.asin(delta_y/distance)
		desired_angle = math.degree(desired_angle)
	elif (x<0 and y>0):
		desired_angle = math.asin(delta_y/distance)
		desired_angle = math.degree(desired_angle)
	elif (x>0 and y<0):
		angle = math.asin(delta_y/distance)
		angle = math.degree(desired_angle)
		desired_angle = 180 - angle
	elif (x>0 and y>0):
		angle = math.asin(delta_y/distance)
		angle = math.degree(desired_angle)
		desired_angle = 180 + angle
	elif (x== 0 and y<0):
		desired_angle = 90
	elif (x== 0 and y>0):
		desired_angle = 270
	elif (x< 0 and y==0):
		desired_angle = 0
	elif (x>0 and y==0):
		desired_angle = 270
	
	relative_angle = desired_angle - math.degrees(yaw)

	if (relative_angle > 180):
		relative_angle = relative_angle - 360

	if(relative_angle > 0):
		rotate(10, desired_angle, False)
	elif (desired_angle < 0):
		rotate(10, desired_angle, True)

	moveX(0.1, distance, True)
