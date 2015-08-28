

#include "talker.hpp"

Talker::Talker(): n("~")
{
	name_tag_sub = n.subscribe<std_msgs::String>(/* "/" + robot +*/ "/tag_name_detected", 1000, &Talker::tagSubscriber, this);

	rnd_walk_start = n.serviceClient<std_srvs::Empty>("start_random_walk");
	rnd_walk_stop = n.serviceClient<std_srvs::Empty>("stop_random_walk");

	ros::spinOnce();
}

void Talker::tagSubscriber(const std_msgs::String::ConstPtr& _msg)
{
	text = _msg->data;
}

void Talker::update()
{	
	std_srvs::Empty srv;

	// if I wanna start it
    rnd_walk_start.call(srv);

	// else i wanna stop
	rnd_walk_stop.call(srv);

	// if i need to talk
	actionlib::SimpleActionClient<mary_tts::maryttsAction> ac("/speak", true);
	ac.waitForServer();

	mary_tts::maryttsGoal goal;
  	goal.text = text;
  	ac.sendGoal(goal);

	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
	
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
