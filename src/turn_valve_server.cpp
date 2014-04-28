#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <grasper/turnValveAction.h>

#include <mar_robot_arm5e/ARM5Arm.h>
#include <grasper/joint_offset.h>
#include <mar_ros_bridge/mar_params.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <tf/tfMessage.h>
#include <mar_perception/VirtualImage.h>
#include <std_srvs/Empty.h>





class TurnValve{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<grasper::turnValveAction> as_;
	grasper::turnValveResult result_;
	grasper::turnValveFeedback feedback_;
	std::string action_name_;
	vpColVector initial_posture_;
	vpHomogeneousMatrix waypoint_pre_, waypoint_man_;
	std::string joint_state_, joint_state_command_, joint_state_fixed_;
	JointOffset* joint_offset_;
	ARM5Arm* robot_;
	std::string det_start_, det_stop_;
	vpHomogeneousMatrix cMv_;
	double max_current_;
	double velocity_turn_;
	double velocity_aperture_;
	double gripper_manipulation_;
	double gripper_opened_;
	double position_max_right_, position_max_left_;
	double position_return_right_, position_return_left_;



public:
	TurnValve(std::string name): as_(nh_, name, boost::bind(&TurnValve::executeCB, this, _1), false ), action_name_(name){
			as_.start();
			nh_.getParam("joint_state", joint_state_);
			nh_.getParam("joint_state_command", joint_state_command_);
			nh_.getParam("joint_state_fixed", joint_state_fixed_);
			waypoint_pre_=mar_params::paramToVispHomogeneousMatrix(&nh_, "waypoint_pre");
			waypoint_man_=mar_params::paramToVispHomogeneousMatrix(&nh_, "waypoint_man");
			initial_posture_=mar_params::paramToVispColVector(&nh_, "initial_posture");
			nh_.getParam("detector_start", det_start_);
			nh_.getParam("detector_stop", det_stop_);
			nh_.getParam("max_current", max_current_);
			nh_.getParam("velocity_turn", velocity_turn_);
			nh_.getParam("velocity_aperture", velocity_aperture_);
			nh_.getParam("gripper_manipulation", gripper_manipulation_);
			nh_.getParam("gripper_opened", gripper_opened_);
			nh_.getParam("position_max_right", position_max_right_);
			nh_.getParam("position_max_left", position_max_left_);
			nh_.getParam("position_return_right", position_return_right_);
			nh_.getParam("position_return_left", position_return_left_);


			joint_offset_=new JointOffset(nh_, joint_state_, joint_state_command_, joint_state_fixed_);
			robot_=new ARM5Arm(nh_, joint_state_fixed_, joint_state_command_);

		}
		~TurnValve(){}


	vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf){
		vpHomogeneousMatrix matrix_visp;
		matrix_visp[0][0]=matrix_tf.getBasis()[0][0]; matrix_visp[0][1]=matrix_tf.getBasis()[0][1]; matrix_visp[0][2]=matrix_tf.getBasis()[0][2]; matrix_visp[0][3]=matrix_tf.getOrigin().x();
		matrix_visp[1][0]=matrix_tf.getBasis()[1][0]; matrix_visp[1][1]=matrix_tf.getBasis()[1][1]; matrix_visp[1][2]=matrix_tf.getBasis()[1][2]; matrix_visp[1][3]=matrix_tf.getOrigin().y();
		matrix_visp[2][0]=matrix_tf.getBasis()[2][0]; matrix_visp[2][1]=matrix_tf.getBasis()[2][1]; matrix_visp[2][2]=matrix_tf.getBasis()[2][2]; matrix_visp[2][3]=matrix_tf.getOrigin().z();
		return matrix_visp;
	}
	void detectValve(bool valve_detected){
		/*tf::StampedTransform cMv_tf;
		tf::TransformListener listener;
		do{
			try{
				listener.lookupTransform("/stereo_down_optical", "/valve_filtered", ros::Time(0), cMv_tf);
				valve_detected=true;
			}
			catch(tf::TransformException &ex){
			}
			ros::spinOnce();
		}while(!valve_detected && ros::ok());*/
		//cMv_=TurnValve::tfToVisp(cMv_tf);
		cMv_[0][0]=0;  cMv_[0][1]=1;  cMv_[0][2]=0; cMv_[0][3]=-0.0;
		cMv_[1][0]=0;  cMv_[1][1]=0;  cMv_[1][2]=-1; cMv_[1][3]=-0.1;
		cMv_[2][0]=-1;  cMv_[2][1]=0;  cMv_[2][2]=0; cMv_[2][3]=0.5;
		valve_detected=true;


	}
	void reachPosition(vpHomogeneousMatrix waypoint){
		vpHomogeneousMatrix bMc, bMe, cMe, cMgoal;
		joint_offset_->get_bMc(bMc);
		TurnValve::detectValve(true);
		cMgoal=cMv_*waypoint;

		robot_->getPosition(bMe);
		cMe=bMc.inverse()*bMe;
		while((cMe.column(4)-cMgoal.column(4)).euclideanNorm()>0.02 && ros::ok()){
			std::cout<<"Error: "<<(cMe.column(4)-cMgoal.column(4)).euclideanNorm()<<std::endl;
			vpColVector xdot(6);
			xdot=0;
			vpHomogeneousMatrix eMgoal=cMe.inverse()*cMgoal;
			xdot[0]=eMgoal[0][3]*0.6;
			xdot[1]=eMgoal[1][3]*0.6;
			xdot[2]=eMgoal[2][3]*0.6;
			robot_->setCartesianVelocity(xdot);
			ros::spinOnce();

			robot_->getPosition(bMe);
			cMe=bMc.inverse()*bMe;
			TurnValve::detectValve(true);
		}
	}
	void reachPositionStraight(vpHomogeneousMatrix waypoint){
		vpHomogeneousMatrix bMc, bMe, cMe, cMgoal;
		joint_offset_->get_bMc(bMc);

		TurnValve::detectValve(true);
		cMgoal=cMv_*waypoint;

		vpColVector q(5);
		cMgoal[0][3]=cMe[0][3];
		while((cMe.column(4)-cMgoal.column(4)).euclideanNorm()>0.02 && ros::ok()){
			std::cout<<"Error: "<<(cMe.column(4)-cMgoal.column(4)).euclideanNorm()<<std::endl;
			vpColVector finalJoints(5), current_joints;
			vpHomogeneousMatrix bMv=bMc*cMgoal;
			robot_->getJointValues(current_joints);
			vpHomogeneousMatrix bMe;
			robot_->getPosition(bMe);
			bMv[1][3]=bMe[1][3];
			finalJoints=robot_->armIK(bMv);
			if(finalJoints[0]>-1.57 && finalJoints[0]<2.1195 && finalJoints[1]>0 && finalJoints[1]<1.58665 && finalJoints[2]>0 && finalJoints[2]<2.15294){
				q=finalJoints-current_joints;
				std::cout<<"q: "<<q<<std::endl;
				q[0]=0;
				q[3]=0;
				q[4]=0;
				robot_->setJointVelocity(q);
			}
			else{
				std::cout<<"Point no reachable final Joints: "<<finalJoints<<std::endl;
			}
			ros::spinOnce();
			robot_->getPosition(bMe);
			cMe=bMc.inverse()*bMe;
			TurnValve::detectValve(true);
			cMgoal[0][3]=cMe[0][3];
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
	void turnWrist(double velocity, double position_max, double  position_return, double current ){
		vpColVector vel(5), current_joints;
		vel=0;
		vel[3]=velocity;
		robot_->getJointValues(current_joints);

		if(velocity>0){
			while(current_joints[3]<position_max && robot_->getCurrent()<current && ros::ok()){
				robot_->setJointVelocity(vel);
				ros::spinOnce();
				robot_->getJointValues(current_joints);
			}
			vel[3]=-velocity;
			while(current_joints[3]>position_return && ros::ok()){
				robot_->setJointVelocity(vel);
				ros::spinOnce();
				robot_->getJointValues(current_joints);
			}
		}
		else{
			while(current_joints[3]>position_max && robot_->getCurrent()<current && ros::ok()){
				robot_->setJointVelocity(vel);
				ros::spinOnce();
				robot_->getJointValues(current_joints);
			}
			vel[3]=-velocity;
			while(current_joints[3]<position_return && ros::ok()){
				robot_->setJointVelocity(vel);
				ros::spinOnce();
				robot_->getJointValues(current_joints);
			}
		}
	}


	void executeCB(const grasper::turnValveGoalConstPtr &goal){


		feedback_.action="Initializing Joint Offset (bMc)";
		as_.publishFeedback(feedback_);
		joint_offset_->reset_bMc(initial_posture_);
		vpHomogeneousMatrix bMc;
		joint_offset_->get_bMc(bMc);

		feedback_.action="Initializing Valve detector Service";
		as_.publishFeedback(feedback_);
		std_srvs::Empty::Request req;
		std_srvs::Empty::Response res;
		//ros::service::call(det_start_, req, res);

		feedback_.action="Waiting for the first valve detection";
		as_.publishFeedback(feedback_);
		TurnValve::detectValve(false);

		feedback_.action="Reach the pre-manipulation position";
		as_.publishFeedback(feedback_);
		TurnValve::reachPosition(waypoint_pre_);

		feedback_.action="Open the gripper until manipulation aperture";
		as_.publishFeedback(feedback_);
		TurnValve::openGripper(velocity_aperture_, gripper_manipulation_, max_current_);

		feedback_.action="Stopping Valve detector Service";
		as_.publishFeedback(feedback_);
		//ros::service::call(det_stop_, req, res);

		feedback_.action="Reach the manipulation position";
		as_.publishFeedback(feedback_);
		TurnValve::reachPosition(waypoint_man_);

		feedback_.action="Turn right the valve";
		as_.publishFeedback(feedback_);
		TurnValve::turnWrist(velocity_turn_, position_max_right_, position_return_right_, max_current_);

		feedback_.action="Turn left the valve";
		as_.publishFeedback(feedback_);
		TurnValve::turnWrist(-velocity_turn_, position_max_left_, position_return_left_, max_current_);

		feedback_.action="Open the gripper completely";
		as_.publishFeedback(feedback_);
		TurnValve::openGripper(velocity_aperture_, gripper_opened_, max_current_);

		feedback_.action="Reach the pre-manipulation position";
		as_.publishFeedback(feedback_);
		TurnValve::reachPosition(waypoint_pre_);

		result_.succeed=1;
		as_.setSucceeded(result_);

	}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "turnValveServer");
	TurnValve turnValve(ros::this_node::getName());
	ros::spin();
	return 0;
}
