#!/usr/bin/env python

import rospy
# import math
# import angles
# from turtlesim.msg import Pose
# from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dialogflow_ros.msg import DialogflowResult


rospy.init_node('test_dialogflow')
vel = rospy.Publisher('/test_df/msg', DialogflowResult, queue_size=10)
insertString = rospy.Publisher('/dialogflow_client/requests/string_msg', String, queue_size=10)
for i in range(2):
    rospy.Rate(0.3).sleep()
    insertString.publish("Hello Tiago can you hear me?")


def publish_result(result):
    # command = Twist()
    #
    # diff = (goal[0] - current.x, goal[1] - current.y)
    # goal_orientation = math.atan2(diff[1], diff[0])
    #
    # angle_diff = angles.shortest_angular_distance(current.theta, goal_orientation)
    #
    # if angle_diff < -0.087:
    #     command.angular.z = -1
    # elif angle_diff > 0.087:
    #     command.angular.z = 1
    # elif math.hypot(*diff) > 0.3:
    #     command.linear.x = 1

    # rospy.Rate(10).sleep()  # Only send at a max of 10/second - this works but I have disabled
    vel.publish(result)


rospy.Subscriber('/dialogflow_client/results', DialogflowResult, publish_result, queue_size=10)
rospy.spin()
