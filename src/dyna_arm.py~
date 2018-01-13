#!/usr/bin/env python

#import necessary libraries
import pypot.dynamixel
import itertools
import rospy
from geometry_msgs.msg import Point
from time import sleep

#get all available ports your robot is connected to, assume the first one
ports = pypot.dynamixel.get_available_ports()
print('Available ports:', ports)
if not ports:
	raise IOError('No port available.')
port = ports[0]
print('Using the first on the list' + port)

#connect to the port
dxl_io = pypot.dynamixel.DxlIO(port)
print('Connected...')

#find servo_ids
found_ids = dxl_io.scan()
print('Found ids:', found_ids)

#there should be five servos, exit upon failure
if len(found_ids) < 5:
	raise IOError('You should connect at least five motors on the bus for this test.')

#start procedure
print('Ready!')

#save the state of each servo's upcoming movement
translations = [0] * 5

#global constants that define angular and linear speed
#for reference, original values were DTH = 5.0 and SPEED = 30
DTH = 5.0
SPEED = 30	

#our timer callback function, triggered when the ROS timer is published to
def timerCallback(data):

	#get and save the current values of each servo, in radians
	current = dxl_io.get_present_position(found_ids)

	#calculate goal position, given the latest change 
	pos = [0.0] * 5
	for i in range(0,5):
		pos[i] = float(current[i] + (translations[i]*DTH))

	#Enable servos and set the speed
	dxl_io.enable_torque(found_ids)

	#create a dictionary, mapping each servo to a speed
	#itertools.repeat(SPEED) will create a list the length of found_ids containing SPEED
	#if found_ids=[1, 2, 3, 4, 5] and SPEED=30, the dictionary would look like:
	#	{1: 30, 2: 30, 3: 30, 4: 30, 5: 30}
	speed = dict(zip(found_ids, itertools.repeat(SPEED)))
	dxl_io.set_moving_speed(speed)

	#create a dictionary, mapping each servo to its goal position
	pos = dict(zip(found_ids, pos))
	dxl_io.set_goal_position(pos)

	#sleep for a short amount of time to give the servos time to move to their position
	sleep(0.05*DTH)
	print(translations)

#our ps_controller callback, triggered when dyna_chatter is published to
def controllerCallback(data):

	#get and save the servo_id, left thumbstick direction, and button's activity
	servo_id  = int(data.x)
	direction = int(data.y)
	active    = int(data.z)

	#set the servo_id's activity in the translation array, to be read on a timer callback
	if (servo_id == 5):
		translations[4] = active
	elif (active == 0) or (direction == 0):
		translations[servo_id-1] = 0
	else:
		translations[servo_id-1] = (1 if (direction == 1) else -1)

#our main procedure
def listener():

	#initialize our node
	rospy.init_node('controller_to_point_listener', anonymous=True)

	#subscribe to dyna_chatter, listening for controller events
	rospy.Subscriber("dyna_chatter", Point, controllerCallback)

	#create a ROS timer that activates every 0.25 seconds
	rospy.Timer(rospy.Duration(0.25), timerCallback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

#run our procedure on startup
if __name__ == '__main__':

	listener()
