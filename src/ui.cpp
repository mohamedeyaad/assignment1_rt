#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"
#include "assignment1_rt/Vel.h"
#include <iostream>
#include <string>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;

struct TurtleInput {
    std::string turtle;
    float linear_vel;
    float angular_vel;
};

TurtleInput getUserInput() {
    // Get user input for turtle selection and velocity
	TurtleInput input;

	std::cout << "Enter turtle to control:\n"
              << "- turtle1\n"
              << "- turtle2\n";
    std::cin >> input.turtle;

    std::cout << "Enter linear velocity: ";
    std::cin >> input.linear_vel;

    std::cout << "Enter angular velocity: ";
    std::cin >> input.angular_vel;
    
    return input;
}

void control_turtle(const std::string turtle, float linear_vel,float angular_vel){
    // Choose the correct publisher based on the turtle
	ros::Publisher pub = (turtle == "turtle1") ? pub1 : pub2;
	ros::Publisher pub_vel = (turtle == "turtle1") ? pub3 : pub4;

	// Create a Twist message and set linear and angular velocities
	geometry_msgs::Twist twist;
	assignment1_rt::Vel vel;

	twist.linear.x = linear_vel;
	vel.linear_velocity = linear_vel;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = angular_vel;
	vel.angular_velocity = angular_vel;


	// Publish the Twist message
	pub.publish(twist);
	pub_vel.publish(vel);
	
	ros::Duration(1.0).sleep(); // sleep for 1 second
	
	// Stop after duration
	twist.linear.x = 0;
    twist.angular.z = 0;
	vel.linear_velocity = 0;
	vel.angular_velocity= 0;
    pub.publish(twist);
	pub_vel.publish(vel);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "ui");
    ros::NodeHandle nh;
	// ros::Rate loop_rate(1); 

	pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

	pub3 = nh.advertise<assignment1_rt::Vel>("/turtle1/vel", 10);
    pub4 = nh.advertise<assignment1_rt::Vel>("/turtle2/vel", 10);

    ros::ServiceClient spawn_turtle = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv1;
	srv1.request.x = 2.0;
	srv1.request.y = 3.0;
 	srv1.request.theta = 0.0;
 	srv1.request.name = "turtle2";
 	spawn_turtle.waitForExistence();
	if (spawn_turtle.call(srv1))
		ROS_INFO("Successfully spawned turtle2"); 
	else
	{
		ROS_ERROR("Failed to call service"); 
		return 1;
	}	 

	while (ros::ok())
	{
		TurtleInput input;
		input = getUserInput();
		if (input.turtle == "turtle1" || input.turtle == "turtle2") {
            control_turtle(input.turtle, input.linear_vel, input.angular_vel);
		}
        else{
			ROS_WARN("Invalid turtle name. Choose either 'turtle1' or 'turtle2'.");
		}
		ros::spinOnce();
        // loop_rate.sleep();
	}
	return 0;
}