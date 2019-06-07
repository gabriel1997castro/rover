/*
 * Author: Gabriel Guimarães Almeida de Castro
 * 
 *
 */

#include "ros/ros.h"
#include "rover/WheelVel.h"

void WheelsVelocityCallback(const rover::WheelVel vel)
{
    ROS_INFO("[Listener] I heard: [%f]\n", vel.left_wheels);
    ROS_INFO("[Listener] I heard: [%f]\n", vel.right_wheels);
}

int main(int argc, char **argv)
{
    rover::WheelVel vel;
    rover::WheelVel *ptrVel;
    ptrVel = &vel

    // Initiate a new ROS node named "listener"
	ros::init(argc, argv, "listener_vel");
	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle node;


    // Subscribe to a given topic, in this case "chatter".
	//chatterCallback: is the name of the callback function that will be executed each time a message is received.
    ros::Subscriber sub = node.subscribe("wheels_velocity", 10, WheelsVelocityCallback);

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}
