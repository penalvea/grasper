
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <grasper/plugHotStabAction.h>
#include <grasper/unplugHotStabAction.h>



#include <mar_robot_arm5e/ARM5Arm.h>
#include <grasper/joint_offset.h>
#include <mar_ros_bridge/mar_params.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <tf/tfMessage.h>
#include <mar_perception/VirtualImage.h>
#include <std_srvs/Empty.h>



class HotStab{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<grasper::unplugHotStabAction> unplug_as_;
	actionlib::SimpleActionServer<grasper::plugHotStabAction> plug_as_;



	grasper::unplugHotStabResult unplug_result_;
	grasper::plugHotStabResult plug_result_;
	grasper::unplugHotStabFeedback unplug_feedback_;
	grasper::plugHotStabFeedback plug_feedback_;
	std::string action_name_;

	vpColVector initial_posture_;
	vpHomogeneousMatrix waypoint_pre_, waypoint_man_, waypoint_ext_, waypoint_ins_;
	vpColVector waypoint_pre_pre_, waypoint_undock_;
	std::string joint_state_, joint_state_command_, joint_state_fixed_;
	JointOffset* joint_offset_;
	ARM5Arm* robot_;
	std::string det_start_, det_stop_;
	vpHomogeneousMatrix cMh_;
	double max_current_;
	double velocity_aperture_;
	double gripper_manipulation_;
	double gripper_closed_;


public:
	HotStab(std::string name): unplug_as_(nh_, name, boost::bind(&HotStab::executeCBUnplug, this, _1), false ), plug_as_(nh_, name, boost::bind(&HotStab::executeCBPlug, this, _1), false ), action_name_(name){
		unplug_as_.start();
		plug_as_.start();
		nh_.getParam("joint_state", joint_state_);
		nh_.getParam("joint_state_command", joint_state_command_);
		nh_.getParam("joint_state_fixed", joint_state_fixed_);
		waypoint_pre_=mar_params::paramToVispHomogeneousMatrix(&nh_, "waypoint_pre");
		waypoint_man_=mar_params::paramToVispHomogeneousMatrix(&nh_, "waypoint_man");
		waypoint_ext_=mar_params::paramToVispHomogeneousMatrix(&nh_, "waypoint_ext");
		waypoint_ins_=mar_params::paramToVispHomogeneousMatrix(&nh_, "waypoint_ins");
		waypoint_pre_pre_=mar_params::paramToVispColVector(&nh_, "waypoint_pre_pre");
		waypoint_undock_=mar_params::paramToVispColVector(&nh_, "waypoint_undock");
		initial_posture_=mar_params::paramToVispColVector(&nh_, "initial_posture");
		nh_.getParam("detector_Valvestart", det_start_);
		nh_.getParam("detector_stop", det_stop_);
		nh_.getParam("max_current", max_current_);
		nh_.getParam("velocity_aperture", velocity_aperture_);
		nh_.getParam("gripper_manipulation", gripper_manipulation_);
		nh_.getParam("gripper_closed", gripper_closed_);



		joint_offset_=new JointOffset(nh_, joint_state_, joint_state_command_, joint_state_fixed_);
		robot_=new ARM5Arm(nh_, joint_state_fixed_, joint_state_command_);

	}
	~HotStab(){}

	vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf){
		vpHomogeneousMatrix matrix_visp;
		matrix_visp[0][0]=matrix_tf.getBasis()[0][0]; matrix_visp[0][1]=matrix_tf.getBasis()[0][1]; matrix_visp[0][2]=matrix_tf.getBasis()[0][2]; matrix_visp[0][3]=matrix_tf.getOrigin().x();
		matrix_visp[1][0]=matrix_tf.getBasis()[1][0]; matrix_visp[1][1]=matrix_tf.getBasis()[1][1]; matrix_visp[1][2]=matrix_tf.getBasis()[1][2]; matrix_visp[1][3]=matrix_tf.getOrigin().y();
		matrix_visp[2][0]=matrix_tf.getBasis()[2][0]; matrix_visp[2][1]=matrix_tf.getBasis()[2][1]; matrix_visp[2][2]=matrix_tf.getBasis()[2][2]; matrix_visp[2][3]=matrix_tf.getOrigin().z();
		return matrix_visp;
	}
	void detectConnector(bool valve_detected){
		tf::StampedTransform cMh_tf;
		tf::TransformListener listener;
		do{
			try{
				listener.lookupTransform("/stereo_down_optical", "/connector", ros::Time(0), cMh_tf);
				valve_detected=true;
			}
			catch(tf::TransformException &ex){
			}
			ros::spinOnce();
		}while(!valve_detected && ros::ok());
		cMh_=HotStab::tfToVisp(cMh_tf);
		valve_detected=true;


	}
	void reachPosition(vpHomogeneousMatrix waypoint){
		vpHomogeneousMatrix bMc, bMe, cMe, cMgoal;
		joint_offset_->get_bMc(bMc);

		HotStab::detectConnector(true);
		cMgoal=cMh_*waypoint;

		robot_->getPosition(bMe);
		cMe=bMc.inverse()*bMe;
		while((cMe.column(4)-cMgoal.column(4)).euclideanNorm()>0.02 && ros::ok()){
			std::cout<<"Error: "<<(cMe.column(4)-cMgoal.column(4)).euclideanNorm()<<std::endl;
			vpColVector xdot(6);
			xdot=0;
			vpHomogeneousMatrix eMgoal=cMe.inverse()*cMgoal;
			xdot[0]=eMgoal[0][3]*0.4;
			xdot[1]=eMgoal[1][3]*0.4;
			xdot[2]=eMgoal[2][3]*0.4;
			robot_->setCartesianVelocity(xdot);
			ros::spinOnce();

			robot_->getPosition(bMe);
			cMe=bMc.inverse()*bMe;
			HotStab::detectConnector(true);
		}
	}

	void reachJointPosition(vpColVector desired_joints){
		vpColVector current_joints;
		robot_->getJointValues(current_joints);
		while((desired_joints-current_joints).euclideanNorm()>0.02 && ros::ok()){
			robot_->setJointVelocity(desired_joints-current_joints);
			robot_->getJointValues(current_joints);
			ros::spinOnce();
		}
	}

	void openGripper(double velocity, double aperture, double current){
			vpColVector vel(5), current_joints;
			vel=0;
			vel[4]=velocity;
			robot_->getJointValues(current_joints);
			if(velocity>0){
				while(current_joints[4]<aperture && ros::ok() && robot_->getCurrent()<current ){
					robot_->setJointVelocity(vel);
					ros::spinOnce();
					robot_->getJointValues(current_joints);
				}
			}
			else{
				while(current_joints[4]>aperture && ros::ok() && robot_->getCurrent()<current ){
					robot_->setJointVelocity(vel);
					ros::spinOnce();
					robot_->getJointValues(current_joints);
				}
			}
		}

	void executeCBUnplug(const grasper::unplugHotStabGoalConstPtr &goal){

		unplug_feedback_.action="Initializing Joint Offset (bMc)";
		unplug_as_.publishFeedback(unplug_feedback_);
		joint_offset_->reset_bMc(initial_posture_);

		unplug_feedback_.action="Initializing Valve detector Service";
		unplug_as_.publishFeedback(unplug_feedback_);
		std_srvs::Empty::Request req;
		std_srvs::Empty::Response res;
		//ros::service::call(det_start_, req, res);

		unplug_feedback_.action="Waiting for the first valve detection";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::detectConnector(false);

		unplug_feedback_.action="Reach the pre pre manipulation position";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::reachJointPosition(waypoint_pre_pre_);

		unplug_feedback_.action="Reach the pre-manipulation position";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::reachPosition(waypoint_pre_);

		unplug_feedback_.action="Open the gripper until manipulation aperture";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::openGripper(velocity_aperture_, gripper_manipulation_, max_current_);

		unplug_feedback_.action="Stopping Valve detector Service";
		unplug_as_.publishFeedback(unplug_feedback_);
		//ros::service::call(det_stop_, req, res);

		unplug_feedback_.action="Reach the manipulation position";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::reachPosition(waypoint_man_);

		unplug_feedback_.action="Close the gripper";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::openGripper(-velocity_aperture_, gripper_closed_, max_current_);

		unplug_feedback_.action="Reach the extraction position";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::reachPosition(waypoint_ext_);

		unplug_feedback_.action="Reach the pre pre manipulation position";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::reachJointPosition(waypoint_pre_pre_);

		unplug_feedback_.action="Reach the undocking position";
		unplug_as_.publishFeedback(unplug_feedback_);
		HotStab::reachJointPosition(waypoint_undock_);

		unplug_result_.succeed=1;
		unplug_as_.setSucceeded(unplug_result_);
	}
	void executeCBPlug(const grasper::plugHotStabGoalConstPtr &goal){
		plug_feedback_.action="Initializing Valve detector Service";
		plug_as_.publishFeedback(plug_feedback_);
		std_srvs::Empty::Request req;
		std_srvs::Empty::Response res;
		//ros::service::call(det_start_, req, res);

		plug_feedback_.action="Reach the pre pre manipulation position";
		plug_as_.publishFeedback(plug_feedback_);
		HotStab::reachJointPosition(waypoint_pre_pre_);

		plug_feedback_.action="Stopping Valve detector Service";
		plug_as_.publishFeedback(plug_feedback_);
		//ros::service::call(det_stop_, req, res);

		plug_feedback_.action="Reach the pre-manipulation position";
		plug_as_.publishFeedback(plug_feedback_);
		HotStab::reachPosition(waypoint_pre_);

		plug_feedback_.action="Reach the insertion position";
		plug_as_.publishFeedback(plug_feedback_);
		HotStab::reachPosition(waypoint_ins_);

		plug_feedback_.action="Open the gripper until manipulation aperture";
		plug_as_.publishFeedback(plug_feedback_);
		HotStab::openGripper(velocity_aperture_, gripper_manipulation_, max_current_);


		//Tal vez sea necesario mover la mano para entrar el conector

		plug_feedback_.action="Reach the extraction position";
		plug_as_.publishFeedback(plug_feedback_);
		HotStab::reachPosition(waypoint_ext_);

		plug_feedback_.action="Reach the pre pre manipulation position";
		plug_as_.publishFeedback(plug_feedback_);
		HotStab::reachJointPosition(waypoint_pre_pre_);

		plug_feedback_.action="Reach the undocking position";
		plug_as_.publishFeedback(plug_feedback_);
		HotStab::reachJointPosition(waypoint_undock_);

		plug_result_.succeed=1;
		plug_as_.setSucceeded(plug_result_);

	}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "hotStabServer");
	HotStab hotStab(ros::this_node::getName());
	ros::spin();
	return 0;
	}
