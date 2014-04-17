#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <grasper/turnValveAction.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "turn_valve_client");
	actionlib::SimpleActionClient<grasper::turnValveAction> ac("turnValve", true);

	ROS_INFO("Waiting for action server to start");
	ac.waitForServer();

	grasper::turnValveGoal goal;
	goal.init=1;
	ac.sendGoal(goal);
	ac.waitForResult(ros::Duration(30));
	ROS_INFO("Action finished");

}
