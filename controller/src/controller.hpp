#ifndef _CONTROLLER_NODE_HPP_
#define _CONTROLLER_NODE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tf/transform_listener.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mary_tts/maryttsAction.h>
#include <strands_gazing/GazeAtPoseAction.h>

#include <iostream>

class Controller
{
private:
	ros::NodeHandle n;
	std::string text;
	ros::Subscriber name_tag_sub;

	actionlib::SimpleActionClient<mary_tts::maryttsAction> ac_speak;
	actionlib::SimpleActionClient<strands_gazing::GazeAtPoseAction> ac_gaze;
	ros::ServiceClient rnd_walk_start;
	ros::ServiceClient rnd_walk_stop;
	bool new_task;

public:
	Controller();
	void startDialog();
	void startGaze();
	void tagSubscriber(const std_msgs::String::ConstPtr& _msg);
	void update();
	~Controller(){}
};

#endif
