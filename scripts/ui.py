#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

def spawn_turtle():
    x = rospy.get_param("/turtle2_pose/x")
    y = rospy.get_param("/turtle2_pose/y")
    theta = rospy.get_param("/turtle2_pose/theta")

    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        request = spawn_turtle(x, y, theta,"turtle2")
        rospy.loginfo("Successfully spawned turtle2")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def get_user_input():
    # Get user input for turtle selection and velocity 
    turtle = input("Enter turtle to control:\n"
                   "- turtle1\n"
                   "- turtle2\n")
    linear_vel = float(input("Enter linear velocity: "))
    angular_vel = float(input("Enter angular velocity: "))
    return turtle, linear_vel, angular_vel

def control_turtle(turtle, linear_vel, angular_vel):
    #pub = rospy.Publisher(f'/{turtle}/cmd_vel', Twist, queue_size=10)
    #while pub.get_num_connections() == 0:
    #    rospy.sleep(0.1)  # Wait for connection
    
    pub = pub1 if turtle == "turtle1" else pub2

    twist = Twist()
    twist.linear.x = linear_vel
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angular_vel
    pub.publish(twist)
    
    # Sleep for 1 second
    rospy.sleep(rospy.Duration(1)) 
    
    # Stop after duration
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

    # #rate = rospy.Rate(1)
    # elapsed_time = 0
    # start_time = rospy.get_rostime()
    # while elapsed_time < 1.0:
    #     # Calculate elapsed time in seconds
    #     pub.publish(twist)
    #     elapsed_time = (rospy.get_rostime() - start_time).to_sec()
    #     #rate.sleep()

def main():
    rospy.init_node('ui_node',anonymous=True)
    #rate = rospy.Rate(1)

    spawn_turtle()  # Spawn turtle2 at startup
    while not rospy.is_shutdown():
        turtle, linear_vel, angular_vel = get_user_input()
        if turtle in ["turtle1", "turtle2"]:
            control_turtle(turtle, linear_vel, angular_vel)
        else:
            rospy.logwarn("Invalid turtle name. Choose either 'turtle1' or 'turtle2'.")
        #rate.sleep()

if __name__ == "__main__":
    main()
