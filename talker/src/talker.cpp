

#include "talker.hpp"

Talker::Talker(): n("~"), new_task(false)
{
	name_tag_sub = n.subscribe<std_msgs::String>("/tag_name_detected", 1000, &Talker::tagSubscriber, this);

	rnd_walk_start = n.serviceClient<std_srvs::Empty>("start_random_walk");
	rnd_walk_stop = n.serviceClient<std_srvs::Empty>("stop_random_walk");

	ros::spinOnce();
}

void Talker::startDialog()
{
	actionlib::SimpleActionClient<mary_tts::maryttsAction> ac("/speak", true);
	ac.waitForServer();

	mary_tts::maryttsGoal goal;
  	goal.text = text;
  	ac.sendGoal(goal);

	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout)
	{
	  actionlib::SimpleClientGoalState state = ac.getState();
	  ROS_INFO("Action finished: %s",state.toString().c_str());
	}
}

void Talker::tagSubscriber(const std_msgs::String::ConstPtr& _msg)
{
	if( (_msg->data).compare(text) != 0)
	{
		new_task = true;
		text = _msg->data;
		std::cerr<<"This is another person: " + text<<std::endl;
	}
}

void Talker::update()
{	
	std_srvs::Empty srv;

	if (new_task)
	{
		new_task = false;
		rnd_walk_stop.call(srv);
		startDialog();
		
		std::cerr<<"I would like to start roaming again..."<<std::endl;
		rnd_walk_start.call(srv);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "talker_bev");
	Talker theTalker;
	
	ros::Rate loop_rate(5); // [Hz]

    while(ros::ok())
    {
		theTalker.update();
    	
		ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
