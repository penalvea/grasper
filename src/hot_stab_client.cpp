#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <grasper/unplugHotStabAction.h>
#include <grasper/plugHotStabAction.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "turn_valve_client");
	actionlib::SimpleActionClient<grasper::unplugHotStabAction> unplug_ac("hotStab", true);
	actionlib::SimpleActionClient<grasper::plugHotStabAction> plug_ac("hotStab", true);

	ROS_INFO("Waiting for action server to start");
	unplug_ac.waitForServer();
	plug_ac.waitForServer();


	grasper::unplugHotStabGoal goal_unplug;
	grasper::plugHotStabGoal goal_plug;
	goal_unplug.init=1;
	unplug_ac.sendGoal(goal_unplug);
	unplug_ac.waitForResult(ros::Duration(30));
	goal_plug.init=1;
	plug_ac.sendGoal(goal_plug);
	plug_ac.waitForResult(ros::Duration(30));

	ROS_INFO("Action finished");

}
