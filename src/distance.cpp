#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <cmath>


const double DISTANCE_THRESHOLD = 1.0;  // Minimum distance allowed between turtles

// Global variables to store current poses
turtlesim::Pose current_pose_turtle1;
turtlesim::Pose current_pose_turtle2;

ros::Publisher distance_pub;
ros::Publisher pub1;
ros::Publisher pub2;

// Callback functions to update the turtles' current poses
void pose_callback_turtle1(const turtlesim::Pose::ConstPtr &msg)
{
    current_pose_turtle1 = *msg;
}
void pose_callback_turtle2(const turtlesim::Pose::ConstPtr &msg)
{
    current_pose_turtle2 = *msg;
}

void check_boundaries() {
    if (current_pose_turtle2.x < 1.0 || current_pose_turtle2.x > 10 || current_pose_turtle2.y < 1 || current_pose_turtle2.y > 10) {
        pub2.publish(geometry_msgs::Twist());
        ROS_INFO("Turtle stopped due to boundary limit.");
    }
    if (current_pose_turtle1.x < 1.0 || current_pose_turtle1.x > 10 || current_pose_turtle1.y < 1 || current_pose_turtle1.y > 10) {
        pub1.publish(geometry_msgs::Twist());
        ROS_INFO("Turtle stopped due to boundary limit.");
    }
}

double calculate_distance() {
    double distance = sqrt(pow(current_pose_turtle2.x - current_pose_turtle1.x, 2) + pow(current_pose_turtle2.y - current_pose_turtle1.y, 2));
    
    if (distance <= DISTANCE_THRESHOLD) {
        pub1.publish(geometry_msgs::Twist());
        pub2.publish(geometry_msgs::Twist());
        ROS_INFO("Turtles stopped due to close proximity.");
    }

    check_boundaries();
    return distance;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "distance");
    ros::NodeHandle nh;

    // Publisher for distance and stop command
    distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
    pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
    ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, pose_callback_turtle1);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, pose_callback_turtle2);
    ros::Rate loop_rate(7);

    std_msgs::Float32 distance_msg;

    while (ros::ok()) {
        // Calculating the distance
        double distance = calculate_distance();
        ROS_INFO("Distance = %f", distance);

        distance_msg.data = distance;
        distance_pub.publish(distance_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}