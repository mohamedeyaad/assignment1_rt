#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from turtlesim.msg import Pose
from math import sqrt

# Threshold
DISTANCE_THRESHOLD = rospy.get_param("distance_threshold")  # Minimum distance allowed between turtles
UPPER_BOUNDARY_LIMIT = rospy.get_param("upper_boundary_limit")
LOWER_BOUNDARY_LIMIT = rospy.get_param("lower_boundary_limit")

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
    if current_pose_turtle2.x < LOWER_BOUNDARY_LIMIT or current_pose_turtle2.x > UPPER_BOUNDARY_LIMIT or current_pose_turtle2.y < LOWER_BOUNDARY_LIMIT or current_pose_turtle2.y > UPPER_BOUNDARY_LIMIT:
        pub2.publish(Twist())
        rospy.loginfo("Turtle2 stopped due to boundary limit.")
    if current_pose_turtle1.x < LOWER_BOUNDARY_LIMIT or current_pose_turtle1.x > UPPER_BOUNDARY_LIMIT or current_pose_turtle1.y < LOWER_BOUNDARY_LIMIT or current_pose_turtle1.y > UPPER_BOUNDARY_LIMIT:
        pub1.publish(Twist())
        rospy.loginfo("Turtle1 stopped due to boundary limit.")

def main():
    rospy.init_node('distance', anonymous=True)
    rate = rospy.Rate(100)

    rospy.Subscriber('/turtle1/pose', Pose, pose_callback_turtle1)
    rospy.Subscriber('/turtle2/pose', Pose, pose_callback_turtle2)
    while not rospy.is_shutdown():
        # Calculating the distance
        distance = calculate_distance()
        rospy.loginfo(f"distance = {distance} ")
        distance_pub.publish(distance)
        rate.sleep()
                
if __name__ == "__main__":
    main()
    