#include <sstream>
#include "controller.hpp"

Controller::Controller(): n("~"), new_task(false), ac_speak("/speak", true), ac_gaze("/gaze_at_pose", true), memory_ppl(), name_dict(), person_id(-1)
{
	name_tag_sub = n.subscribe<std_msgs::Int32>(/* "/" + robot +*/ "/tag_name_detected", 1000, &Controller::tagSubscriber, this);

	rnd_walk_start = n.serviceClient<std_srvs::Empty>("/start_random_walk");
	rnd_walk_stop = n.serviceClient<std_srvs::Empty>("/stop_random_walk");

	fillDictionary();

	ros::spinOnce();
}

void Controller::startDialog()
{
	std::cerr<<"I feel like I should talk more..."<<std::endl;
	ac_speak.waitForServer();

	std::stringstream ss;
	std::string name = "";

	// get the name for that person
	std::map<int,std::string>::iterator it;
	it = name_dict.find(person_id);
	if(it != name_dict.end()) {
		name = it->second;
	} else {
		name = "unknown person. I do not recognize you"; // start an alarm?
	}

	ss << "Hi " << name << ".";

	// check how often we have seen that person before
	std::map<int,int>::iterator it_count;
	it_count = memory_ppl.find(person_id);
	if(it_count != memory_ppl.end()) {
		ss << "We have already met " << it_count->second << " times before.";
		ss << "I hope we'll keep in touch." << std::endl;
	} else {
		ss << " Welcome to the E C M R.";
		ss << " Be aware of the other robots. They are plotting an evil plan";
		ss << " against you. Especially the green one.";
		ss << " I am looking forward to seeing you again.";
		ss << std::endl;
	}	

	mary_tts::maryttsGoal goal;
  	goal.text = ss.str();
  	ac_speak.sendGoal(goal);

	bool finished_before_timeout = ac_speak.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout)
	{
	  actionlib::SimpleClientGoalState state = ac_speak.getState();
	  ROS_INFO("Action finished: %s",state.toString().c_str());
	}
}

void Controller::startGaze()
{
	ac_gaze.waitForServer();

	strands_gazing::GazeAtPoseGoal goal;
  	goal.runtime_sec = 0;
	goal.topic_name = "/upper_body_detector/closest_bounding_box_centre";	
  	ac_gaze.sendGoal(goal);

	bool finished_before_timeout = ac_gaze.waitForResult(ros::Duration(10.0));
	if (finished_before_timeout)
	{
	  actionlib::SimpleClientGoalState state = ac_gaze.getState();
	  ROS_INFO("Action finished: %s",state.toString().c_str());
	}
}

void Controller::tagSubscriber(const std_msgs::Int32::ConstPtr& _msg)
{
	if( person_id != _msg->data )
	{
		new_task = true;
		person_id = _msg->data;
		std::cerr<<"This is another person with the id: "<<person_id<<std::endl;
	}
}

void Controller::update()
{	
	std_srvs::Empty srv;

	if (new_task)
	{
		new_task = false;
		// update our people memory
		updatePersonSeen(person_id);

		std::cerr<<"Let's stop here for a while..."<<std::endl;
		rnd_walk_stop.call(srv);

		startGaze();
		std::cerr<<"I feel active..."<<std::endl;
		startDialog();		
		
		ac_gaze.cancelAllGoals();
		std::cerr<<"I would like to start roaming again..."<<std::endl;
		rnd_walk_start.call(srv);
	}
}

void Controller::updatePersonSeen(const int & _person_id) {
	// check whether that person exists in the memory
	std::map<int,int>::iterator it;
	it = memory_ppl.find(_person_id);
	if(it != memory_ppl.end()) {
		// increase it for a known user
		std::cerr<<"I have seen the person already "<<it->second<<" times."<<std::endl;
		memory_ppl[_person_id] = it->second++;
	} else {
		// init new user
		std::cerr<<"This is a new person to me."<<std::endl;
		memory_ppl[_person_id] = 1;
	}
}

void Controller::fillDictionary() {
	// fill the name dictionary with peoples' names
	name_dict[0] = "Bob";
	name_dict[1] = "Betty";
	name_dict[2] = "Linda";
	name_dict[3] = "Lucie";
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller_bev");
	Controller theController;
	
	ros::Rate loop_rate(5); // [Hz]

    while(ros::ok())
    {
		theController.update();
    	
		ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
