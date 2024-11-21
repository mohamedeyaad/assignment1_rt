#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "assignment1_rt/Vel.h"
#include <cmath>


const double DISTANCE_THRESHOLD = 1.5;  // Minimum distance allowed between turtles
const double LOWER_BOUNDARY_LIMIT = 1.0;
const double UPPER_BOUNDARY_LIMIT = 10.0;

// Global variables to store current poses
turtlesim::Pose current_pose_turtle1;
turtlesim::Pose current_pose_turtle2;
assignment1_rt::Vel current_vel_turtle1;
assignment1_rt::Vel current_vel_turtle2;

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
void cmd_vel_callback_turtle1(const assignment1_rt::Vel::ConstPtr &msg)
{
    current_vel_turtle1 = *msg;
}
void cmd_vel_callback_turtle2(const assignment1_rt::Vel::ConstPtr &msg)
{
    current_vel_turtle2 = *msg;
}



void check_boundaries(){
    if (current_pose_turtle2.x < LOWER_BOUNDARY_LIMIT || current_pose_turtle2.x > UPPER_BOUNDARY_LIMIT ||
        current_pose_turtle2.y < LOWER_BOUNDARY_LIMIT || current_pose_turtle2.y > UPPER_BOUNDARY_LIMIT)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = -current_vel_turtle2.linear_velocity;
        twist.angular.z = -current_vel_turtle2.angular_velocity;
        pub2.publish(twist);
        ros::Duration(0.25).sleep();
        twist.linear.x = 0;
        twist.angular.z = 0;
        pub2.publish(twist);
        ROS_INFO("Turtle2 stopped due to boundary limit.");
    }

    if (current_pose_turtle1.x < LOWER_BOUNDARY_LIMIT || current_pose_turtle1.x > UPPER_BOUNDARY_LIMIT ||
        current_pose_turtle1.y < LOWER_BOUNDARY_LIMIT || current_pose_turtle1.y > UPPER_BOUNDARY_LIMIT)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = -current_vel_turtle1.linear_velocity;
        twist.angular.z = -current_vel_turtle1.angular_velocity;
        pub1.publish(twist);
        ros::Duration(0.25).sleep();
        twist.linear.x = 0;
        twist.angular.z = 0;
        pub1.publish(twist);
        ROS_INFO("Turtle1 stopped due to boundary limit.");
    }
}

double calculate_distance()
{
    double distance = sqrt(pow(current_pose_turtle2.x - current_pose_turtle1.x, 2) +
                           pow(current_pose_turtle2.y - current_pose_turtle1.y, 2));

    if (distance <= DISTANCE_THRESHOLD)
    {
        // Stop the moving turtle (by checking which turtle has non-zero velocities)
        if (current_pose_turtle1.linear_velocity != 0.0 || current_pose_turtle1.angular_velocity != 0.0)
        {
            geometry_msgs::Twist twist;
            twist.linear.x = -current_vel_turtle1.linear_velocity;
            twist.angular.z = -current_vel_turtle1.angular_velocity;
            pub1.publish(twist);
            ros::Duration(0.25).sleep();
            twist.linear.x = 0;
            twist.angular.z = 0;
            pub1.publish(twist);
        }
        else if (current_pose_turtle2.linear_velocity != 0.0 || current_pose_turtle2.angular_velocity != 0.0)
        {
            geometry_msgs::Twist twist;
            twist.linear.x = -current_vel_turtle2.linear_velocity;
            twist.angular.z = -current_vel_turtle2.angular_velocity;
            pub2.publish(twist);
            ros::Duration(0.25).sleep();
            twist.linear.x = 0;
            twist.angular.z = 0;
            pub2.publish(twist);
        }
        ROS_INFO("Turtles stopped due to close proximity.");
    }
    check_boundaries();
    return distance;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "distance");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100); 

    // Publisher for distance and stop command
    distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
    pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
    ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, pose_callback_turtle1);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, pose_callback_turtle2);

    ros::Subscriber sub3 = nh.subscribe("/turtle1/vel", 10, cmd_vel_callback_turtle1);
    ros::Subscriber sub4 = nh.subscribe("/turtle2/vel", 10, cmd_vel_callback_turtle2); 
    
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