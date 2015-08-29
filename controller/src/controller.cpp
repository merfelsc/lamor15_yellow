

#include "controller.hpp"

Controller::Controller(): n("~"), new_task(false), ac_speak("/speak", true), ac_gaze("/gaze_at_pose", true)
{
	name_tag_sub = n.subscribe<std_msgs::String>(/* "/" + robot +*/ "/tag_name_detected", 1000, &Controller::tagSubscriber, this);

	rnd_walk_start = n.serviceClient<std_srvs::Empty>("start_random_walk");
	rnd_walk_stop = n.serviceClient<std_srvs::Empty>("stop_random_walk");

	ros::spinOnce();
}

void Controller::startDialog()
{
	std::cerr<<"I feel like I should talk more..."<<std::endl;
	ac_speak.waitForServer();

	mary_tts::maryttsGoal goal;
  	goal.text = text;
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
  	goal.runtime_sec = 120;
	goal.topic_name = "/upper_body_detector/closest_bounding_box_centre";	
  	ac_gaze.sendGoal(goal);

	bool finished_before_timeout = ac_gaze.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout)
	{
	  actionlib::SimpleClientGoalState state = ac_gaze.getState();
	  ROS_INFO("Action finished: %s",state.toString().c_str());
	}
}

void Controller::tagSubscriber(const std_msgs::String::ConstPtr& _msg)
{
	if( (_msg->data).compare(text) != 0)
	{
		new_task = true;
		text = _msg->data;
		std::cerr<<"This is another person: " + text<<std::endl;
	}
}

void Controller::update()
{	
	std_srvs::Empty srv;

	if (new_task)
	{
		new_task = false;
		std::cerr<<"Let's stop here for a while..."<<std::endl;
		rnd_walk_stop.call(srv);

		std::cerr<<"I feel active..."<<std::endl;
		startDialog();
		startGaze();
		
		std::cerr<<"I would like to start roaming again..."<<std::endl;
		rnd_walk_start.call(srv);
	}
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
