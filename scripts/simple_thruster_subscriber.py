#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np
from generic_reference.msg import generic_reference_vector
from std_msgs.msg import Float32

class thrusterKeyboard():
    
    # Variables to store the x and y co-ordinate of the generic reference vector
    x = 0
    y = 0

    # variables to store the magnitude and direction of the generic reference vector
    theta = 0
    magnitude = 0

    # variables to store the values published on the /wamv/thrusters topics
    thrust_left = Float32()
    angle_left = Float32()
    thrust_right = Float32()
    angle_right = Float32()

    # Constructor
    def __init__(self):
        rospy.init_node("thruster_keyboard_input", anonymous=False)
        rospy.Subscriber("/wamv/reference", generic_reference_vector, self.callback)

        # Define the publishers to the 4 /wamv/thrusters topics
        self.pub_thrust_left = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size=1)
        self.pub_angle_left = rospy.Publisher("/wamv/thrusters/left_thrust_angle", Float32, queue_size=1)
        self.pub_thrust_right = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size=1)
        self.pub_angle_right = rospy.Publisher("/wamv/thrusters/right_thrust_angle", Float32, queue_size=1)

        # Stops python from executing until the node is terminated.
        rospy.spin()

    def callback(self, data):
        self.x = data.x
        self.y = data.y
        self.magnitude = np.sqrt(self.x**2 + self.y**2)
        self.thrust_left = self.magnitude
        self.thrust_right = self.magnitude
        self.theta = np.arctan2(data.y, data.x)
        self.angle_left = self.theta
        self.angle_right = self.theta
        self.pub_thrust_left.publish(self.thrust_left)
        self.pub_thrust_right.publish(self.thrust_right)
        self.pub_angle_left.publish(self.angle_left)
        self.pub_angle_right.publish(self.angle_right)


if __name__ == '__main__':
    ThrusterSubscriber = thrusterKeyboard()