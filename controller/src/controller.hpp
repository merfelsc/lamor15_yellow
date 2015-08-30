#ifndef _CONTROLLER_NODE_HPP_
#define _CONTROLLER_NODE_HPP_

#include <iostream>
#include <map>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "tf/transform_listener.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mary_tts/maryttsAction.h>
#include <strands_gazing/GazeAtPoseAction.h>
#include <facts/TellFacts.h>



class Controller
{
private:
	ros::NodeHandle n;
	int person_id;
	ros::Subscriber name_tag_sub;

	actionlib::SimpleActionClient<mary_tts::maryttsAction> ac_speak;
	actionlib::SimpleActionClient<strands_gazing::GazeAtPoseAction> ac_gaze;
	ros::ServiceClient rnd_walk_start;
	ros::ServiceClient rnd_walk_stop;
	ros::ServiceClient client_facts; // for asking for facts
	bool new_task;
	std::map<int,int> memory_ppl; // key: person_id, value: number of times seen
	std::map<int,std::string> name_dict; // dictionary for peoples' name
  bool initialized;

public:
	Controller();
	void startDialog();
	void startGaze();
	void tagSubscriber(const std_msgs::Int32::ConstPtr& _msg);
	void update();
	~Controller(){};

private:
	/** increase the number of times, that we have seen a specific person, by one.**/
	void updatePersonSeen(const int & _person_id);
	void fillDictionary();
};

#endif
