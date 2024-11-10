#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from math import sqrt

# Threshold
DISTANCE_THRESHOLD = 1.0  # Minimum distance allowed between turtles

# Publisher for distance and stop command
distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)
pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

# Global variables to store current poses
current_pose_turtle1 = Pose()
current_pose_turtle2 = Pose()

# Callback functions to update the turtles' current poses
def pose_callback_turtle1(data):
    global current_pose_turtle1
    current_pose_turtle1 = data

def pose_callback_turtle2(data):
    global current_pose_turtle2
    current_pose_turtle2 = data

def calculate_distance():
    distance = sqrt(((current_pose_turtle2.x - current_pose_turtle1.x) ** 2) + ((current_pose_turtle2.y - current_pose_turtle1.y) ** 2))
    if distance <= DISTANCE_THRESHOLD:
        pub1.publish(Twist())
        pub2.publish(Twist())
        rospy.loginfo("Turtles stopped due to close proximity.")
    check_boundaries()
    return distance

def check_boundaries():
    if current_pose_turtle2.x < 1.0 or current_pose_turtle2.x > 10 or current_pose_turtle2.y < 1 or current_pose_turtle2.y > 10:
        pub2.publish(Twist())
        rospy.loginfo("Turtle stopped due to boundary limit.")
    if current_pose_turtle1.x < 1.0 or current_pose_turtle1.x > 10 or current_pose_turtle1.y < 1 or current_pose_turtle1.y > 10:
        pub1.publish(Twist())
        rospy.loginfo("Turtle stopped due to boundary limit.")

def main():
    rospy.init_node('distance', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback_turtle1)
    rospy.Subscriber('/turtle2/pose', Pose, pose_callback_turtle2)
    while not rospy.is_shutdown():
        # Calculating the distance
        distance = calculate_distance()
        rospy.loginfo(f"distance = {distance} ")
        distance_pub.publish(distance)

if __name__ == "__main__":
    main()
    