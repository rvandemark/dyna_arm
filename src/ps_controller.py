#!/usr/bin/env python

#import necessary libraries
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

#make our publisher a global variable so it's visible in the callback function
pub = rospy.Publisher('dyna_chatter', Point, queue_size=100)

#our callback function, triggered when "joy" has been published to
def callback(data):

	#vertical movement in the left thumbstick
	vaxis = int(data.axes[1])

	#buttons 1-4
	bttns = data.buttons[:4]

	#left and right triggers
	ltrig = data.buttons[4]
	rtrig = data.buttons[5]

	#build messages to be sent
	#"x" contains the servo_id (1, 2, 3, or 4)
	#"y" contains the direction of the left thumbstick (-1, 0, or 1)
	#"z" contains the state of the button (0 or 1)
	for i in range(0,4):
		point   = Point()
		point.x = i+1
		point.y = vaxis
		point.z = bttns[i]
		pub.publish(point)

	#build a message to be sent for our special case, the fifth servo (the gripper)
	#"x" and "y" are always 5 and 0, respectively
	#"z" is the direction the gripper should move:
	#	0 ---> if neither or both triggers are held
	#	-1 --> if the left trigger is held
	#	1 ---> if the right trigger is held
	point   = Point()
	point.x = 5
	point.y = 0
	point.z = rtrig - ltrig
	pub.publish(point)

#our main procedure
def listener():

	#initialize our node
	rospy.init_node('controller_to_point', anonymous=True)

	#subsribe to joy, listening for controller events
	rospy.Subscriber("joy", Joy, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


#run our procedure on startup
if __name__ == '__main__':

	listener()
