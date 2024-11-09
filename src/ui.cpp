#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "ui");
    ros::NodeHandle nh;
    ros::ServiceClient spawn_turtle = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv1;
	srv1.request.x = 2.0;
	srv1.request.y = 3.0;
 	srv1.request.theta = 0.0;
 	srv1.request.name = "turtle2";
 	spawn_turtle.waitForExistence();
	spawn_turtle.call(srv1);
}