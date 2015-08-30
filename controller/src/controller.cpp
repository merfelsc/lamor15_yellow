#include <sstream>
#include "controller.hpp"

Controller::Controller(): n("~"), new_task(false), ac_speak("/speak", true), ac_gaze("/gaze_at_pose", true), memory_ppl(), name_dict(), person_id(-1), client_facts(n.serviceClient<facts::TellFacts>("/tell_facts")), initialized(false), client_weather(n.serviceClient<weather::TellWeather>("/tell_weather")), person_counter(0), person_last_id(-1), sleepStarted(ros::Time::now() - ros::Duration(30))
{
	name_tag_sub = n.subscribe<circle_detection::detection_results_array>("/circle_detection/results_array", 1000, &Controller::tagSubscriber, this);

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
		name = "you"; // start an alarm?
	}

	ss << "Hi " << name << ".";

	// check how often we have seen that person before
	std::map<int,int>::iterator it_count;
	it_count = memory_ppl.find(person_id);
	if(it_count != memory_ppl.end()) {
		ss << "We have already met " << it_count->second;
    if(it_count->second == 1) {
      ss<<"time";
    } else {
      ss<<"times";
    }
    ss << " before.";

    if(it_count->second == 3) {
      // tell him about the weather
      weather::TellWeather srv;
      if(client_weather.call(srv)) {
        // we got a new weather update
        std::cerr<<"Received a new weather update: " << srv.response.fact << std::endl;
        ss << srv.response.fact << std::endl;
      } else {
        std::cerr<<"Did not receive a new weather update."<<std::endl;
      }
    } else {
      // query the facts service
      facts::TellFacts srv;
      srv.request.person = person_id;
      if(client_facts.call(srv)) {
        // we got a new fact
        std::cerr<<"Received a new fact: " << srv.response.fact << std::endl;
        ss << "Fact: ";
        ss << srv.response.fact << std::endl;
      } else {
        std::cerr<<"Did not receive a new fact."<<std::endl;
      }
    }
	} else {
		ss << " Welcome to the E C M R.";
		ss << " Be aware of the other robots. They are plotting an evil plan";
		ss << " against you. Especially the green one.";
		ss << " I am looking forward to seeing you again.";
		ss << std::endl;
	}	

	mary_tts::maryttsGoal goal;
 	goal.text = ss.str();
  std::cerr<<"<speaking>...";
 	ac_speak.sendGoal(goal);

	bool finished_before_timeout = ac_speak.waitForResult(ros::Duration(15.0));
  std::cerr<<"</speaking>"<<std::endl;
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

	bool finished_before_timeout = ac_gaze.waitForResult(ros::Duration(1.0));
	if (finished_before_timeout)
	{
	  actionlib::SimpleClientGoalState state = ac_gaze.getState();
	  ROS_INFO("Action finished: %s",state.toString().c_str());
	}
}

void Controller::tagSubscriber(const circle_detection::detection_results_array::ConstPtr& _msg)
{
  // check whether we are sleeping
  if(ros::Time::now() - sleepStarted < ros::Duration(20)) {
    return;
  }

  if(person_last_id == _msg->personId) {
    person_counter++;
    std::cerr<<"Seen person " << person_last_id<<" now " <<person_counter<<" times."<<std::endl;
  } else {
    person_counter=1;
  }
  person_last_id = _msg->personId;

  if(person_counter > 3) {
  	if( person_id != _msg->personId )
	  {
		  new_task = true;
		  person_id = _msg->personId;
		  std::cerr<<"This is another person with the id: "<<person_id<<std::endl;
	  }
  }
}

void Controller::update()
{	
	std_srvs::Empty srv;

  // initialization: start with random walking
  if(!initialized) {
    rnd_walk_start.call(srv);
    initialized=true; // will never bet to false again
  }

	if (new_task)
	{
		new_task = false;
		std::cerr<<"Let's stop here for a while..."<<std::endl;
		rnd_walk_stop.call(srv);

    std::cerr<<"Look at this *gaze*!"<<std::endl;
		startGaze();
		std::cerr<<"I feel active...*talk*"<<std::endl;
		startDialog();		

		// update our people memory
		updatePersonSeen(person_id);
		
    std::cerr<<"Stop staring ... this is embarassing."<<std::endl;
		ac_gaze.cancelAllGoals();
		std::cerr<<"I would like to start roaming again!"<<std::endl;
		rnd_walk_start.call(srv);

    // discard perception from now on
    sleepStarted = ros::Time::now();
	}
}

void Controller::updatePersonSeen(const int & _person_id) {
  memory_ppl[_person_id]++; // initializes to 1 or increases by one

	// check whether that person exists in the memory
	/* std::map<int,int>::iterator it;
	it = memory_ppl.find(_person_id);
	if(it != memory_ppl.end()) {
		// increase it for a known user
		std::cerr<<"I have seen the person already "<<it->second<<" times."<<std::endl;
		memory_ppl[_person_id] = it->second++;
	} else {
		// init new user
		std::cerr<<"This is a new person to me."<<std::endl;
		memory_ppl[_person_id] = 1;
	} */
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
