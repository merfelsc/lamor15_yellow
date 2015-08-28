#include "ros/ros.h"
#include "std_msgs/String.h"

std_msgs::String attendee_name;



void chatterCallback(const std_msgs::String::ConstPtr& msg)
{

attendee_name.data = msg->data;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "conversate");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("attendee_name", 1000, chatterCallback);

  ros::Rate loop_rate(10);

while (ros::ok())

{

  ROS_INFO("I heard: [%s]", attendee_name.data.c_str());

  ros::spinOnce();

loop_rate.sleep();
}


  return 0;
}
