#include <sstream>
#include "controller.hpp"

Controller::Controller(): n("~"), new_task(false), ac_speak("/speak", true), ac_gaze("/gaze_at_pose", true), memory_ppl(), name_dict(), person_id(-1), client_facts(n.serviceClient<facts::TellFacts>("/tell_facts")), initialized(false), client_weather(n.serviceClient<weather::TellWeather>("/tell_weather")), person_counter(0), person_last_id(-1), sleepStarted(ros::Time::now() - ros::Duration(30)), speek_rate_(1./10.)
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
    ss << " before. ";

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
        ss << "Did you know that ";
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

    speek_rate_.reset();
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
	
	speek_rate_.sleep();
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
    std::cerr<<"Recognized person " << person_last_id<<" now " <<person_counter<<" times in a row."<<std::endl;
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
    std::cerr<<"Look at this *gaze*!"<<std::endl;
		startGaze();
}

	if (new_task)
	{
		new_task = false;
		std::cerr<<"Let's stop here for a while..."<<std::endl;
		rnd_walk_stop.call(srv);


		std::cerr<<"I feel active...*talk*"<<std::endl;
		startDialog();		

		// update our people memory
		updatePersonSeen(person_id);
		
    std::cerr<<"Stop staring ... this is embarassing."<<std::endl;
		//ac_gaze.cancelAllGoals();
		std::cerr<<"I would like to start roaming again..."<<std::endl;
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
  name_dict[0] = "unknown person";
  name_dict[1] = "Primo Zingaretti";
  name_dict[2] = "Rafael Socas";
  name_dict[3] = "Kai Lingemann";
  name_dict[4] = "Pablo Urcola";
  name_dict[5] = "Adam Borkowski";
  name_dict[6] = "Karel Kosnar";
  name_dict[7] = "Christoph Rosmann";
  name_dict[8] = "Tomas Krajnik";
  name_dict[9] = "Ivan Petrovic";
  name_dict[10] = "Ignacio Perez-Hurtado";
  name_dict[11] = "Anssi Kemppainen";
  name_dict[12] = "Lino Marques";
  name_dict[13] = "Juhan Ernits";
  name_dict[14] = "Jan Faigl";
  name_dict[15] = "Andreu Corominas-Murtra";
  name_dict[16] = "Stefano Ghidoni";
  name_dict[17] = "Christoph Manss";
  name_dict[18] = "Giorgio Grisetti";
  name_dict[19] = "Aamir Ahmad";
  name_dict[20] = "Alessandro Saffiotti";
  name_dict[21] = "Owen McAree";
  name_dict[22] = "Andreas Hermann";
  name_dict[23] = "Thomas Harrison";
  name_dict[24] = "Henning Deeken";
  name_dict[25] = "Matthias Rapp";
  name_dict[26] = "Tristan Igelbrink";
  name_dict[27] = "Elias Mueggler";
  name_dict[28] = "Gert Kanter";
  name_dict[29] = "Vijaya Kumar Ghorpade";
  name_dict[30] = "Jose Raul Ruiz Sarmiento";
  name_dict[31] = "Joao Machado Santos";
  name_dict[32] = "Alejo Concha Belenguer";
  name_dict[33] = "Peer Neubert";
  name_dict[34] = "Igor Cvisic";
  name_dict[35] = "Johannes Lachele";
  name_dict[36] = "Nils Bore";
  name_dict[37] = "Tayyab Naseer";
  name_dict[38] = "Amit Kumar Mondal";
  name_dict[39] = "Federico Boniardi";
  name_dict[40] = "Robert Lukierski";
  name_dict[41] = "Stefano Di Lucia";
  name_dict[42] = "Steffen Mueller";
  name_dict[43] = "Marina Kollmitz";
  name_dict[44] = "David Droeschel";
  name_dict[45] = "Dirk Holz";
  name_dict[46] = "Marian Himstedt";
  name_dict[47] = "Lenka Mudrova";
  name_dict[48] = "Ali Abdul Khaliq";
  name_dict[49] = "Matthias Nieuwenhuisen";
  name_dict[50] = "Tomasz Kucner";
  name_dict[51] = "Arunkumar Ramaswamy";
  name_dict[52] = "Alexander May";
  name_dict[53] = "James Finnis";
  name_dict[54] = "Saeed Gholami Shahbandi";
  name_dict[55] = "Achim Lilienthal";
  name_dict[56] = "Beth Downing";
  name_dict[57] = "Christian Merfels";
  name_dict[58] = "Hadi Saoud";
  name_dict[59] = "Johan Ekekrantz";
  name_dict[60] = "Minlue Wang";
  name_dict[61] = "Raviteja Chadalavada";
  name_dict[62] = "Russell Harding";
  name_dict[63] = "Rami Alkhawaji";
  name_dict[64] = "Patrick Bechon";
  name_dict[65] = "Feng Zhao";
  name_dict[66] = "Serhan Cosar";
  name_dict[67] = "Ingmar Posner";
  name_dict[68] = "Roland Siegwart";
  name_dict[69] = "Maja Pantic";
  name_dict[70] = "Tom Duckett";
  name_dict[71] = "Betty";
  name_dict[72] = "Linda";
  name_dict[73] = "Lucie";
  name_dict[74] = "Bob";
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
