#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn

rospy.init_node('ui', anonymous=True)

def spawn_turtle():
    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        request = spawn_turtle(2.0, 3.0, 0.0,"turtle2")
        rospy.loginfo("Successfully spawned turtle2")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
 

if __name__ == "__main__":
    spawn_turtle()