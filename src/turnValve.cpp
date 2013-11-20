#include <mar_robot_arm5e/ARM5Arm.h>
#include "grasper/JointOffset.h"
#include <mar_ros_bridge/mar_params.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <tf/tfMessage.h>
#include <mar_perception/VirtualImage.h>
#include <visp/vpDisplayX.h>
#include <std_srvs/Empty.h>




#define GRIPPER_CLOSED 0
#define GRIPPER_MANIPULATION 0.5
#define GRIPPER_OPENED 1.0

#define CURRENTTHRESHOLD 1.6

vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf){
	vpHomogeneousMatrix matrix_visp;
	matrix_visp[0][0]=matrix_tf.getBasis()[0][0]; matrix_visp[0][1]=matrix_tf.getBasis()[0][1]; matrix_visp[0][2]=matrix_tf.getBasis()[0][2]; matrix_visp[0][3]=matrix_tf.getOrigin().x();
	matrix_visp[1][0]=matrix_tf.getBasis()[1][0]; matrix_visp[1][1]=matrix_tf.getBasis()[1][1]; matrix_visp[1][2]=matrix_tf.getBasis()[1][2]; matrix_visp[1][3]=matrix_tf.getOrigin().y();
	matrix_visp[2][0]=matrix_tf.getBasis()[2][0]; matrix_visp[2][1]=matrix_tf.getBasis()[2][1]; matrix_visp[2][2]=matrix_tf.getBasis()[2][2]; matrix_visp[2][3]=matrix_tf.getOrigin().z();
	return matrix_visp;
}




int main(int argc, char **argv){
	ros::init(argc, argv, "turnValve");
	ros::NodeHandle nh;

	//get Params
	std::string joint_state("");
	nh.getParam("joint_state", joint_state);
	std::string joint_state_command("");
	nh.getParam("joint_state_command", joint_state_command);
	std::string joint_state_fixed("");
	nh.getParam("joint_state_fixed", joint_state_fixed);
	///////Launch the service to detect the valve
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;
	ros::service::call("/valve_tracker/start_valve_detection", req, res);


	////////
	vpColVector initial_posture=mar_params::paramToVispColVector(&nh, "initial_posture");

	vpHomogeneousMatrix waypoint_pre=mar_params::paramToVispHomogeneousMatrix(&nh, "waypoint_pre");//(0,0,-0.05,0,0,0);
	vpHomogeneousMatrix waypoint_man=mar_params::paramToVispHomogeneousMatrix(&nh, "waypoint_man");//(0,0,0.03, 0,0,0);


        std::cout<<"Joint_offset"<<std::endl;
	JointOffset joint_offset(nh, joint_state, joint_state_command, joint_state_fixed);
	std::cout<<"joint_offset iniciado"<<std::endl;
	ARM5Arm robot(nh, joint_state_fixed, joint_state_command);
	
	
	joint_offset.reset_bMc(initial_posture);




	//waiting for the first valve detection
	tf::TransformListener listener;
	bool valve_detected=false;
	tf::StampedTransform cMv_tf;
	std::cout<<"First Detection"<<std::endl;
	while(!valve_detected && ros::ok()){
		try{
			listener.lookupTransform("/stereo_down_optical", "/valve_filtered", ros::Time(0), cMv_tf);
			valve_detected=true;
		}
		catch(tf::TransformException &ex){
		}
		ros::spinOnce();
	}
	std::cout<<"First valve detection detected"<<std::endl;
	vpHomogeneousMatrix cMv=tfToVisp(cMv_tf);




	//Reach the pre-manipulation position
	std::cout<<"Pre Manipulation Joint Velocity"<<std::endl;
	vpHomogeneousMatrix object_turn(0,0,0,0,0, -1.57);
	vpHomogeneousMatrix cMv_pre=cMv*waypoint_pre;




	vpHomogeneousMatrix bMc, bMe;
	joint_offset.get_bMc(bMc);
	robot.getPosition(bMe);
	vpHomogeneousMatrix cMe=bMc.inverse()*bMe;
	vpColVector q(5);
	cMv_pre[0][3]=cMe[0][3];
	while((cMe.column(4)-cMv_pre.column(4)).euclideanNorm()>0.02 && ros::ok()){
		std::cout<<"Error: "<<(cMe.column(4)-cMv_pre.column(4)).euclideanNorm()<<std::endl;
		vpColVector finalJoints(5), current_joints;
		vpHomogeneousMatrix bMv=bMc*cMv_pre;
		robot.getJointValues(current_joints);
		vpHomogeneousMatrix bMe;
		robot.getPosition(bMe);
		bMv[1][3]=bMe[1][3];
		finalJoints=robot.armIK(bMv);
		if(finalJoints[0]>-1.57 && finalJoints[0]<2.1195 && finalJoints[1]>0 && finalJoints[1]<1.58665 && finalJoints[2]>0 && finalJoints[2]<2.15294){

			q=finalJoints-current_joints;
			std::cout<<"q: "<<q<<std::endl;
			q[0]=0;
			q[3]=0;
			q[4]=0;
			robot.setJointVelocity(q);
		}
		else{
			std::cout<<"Point no reachable final Joints: "<<finalJoints<<std::endl;
		}
		ros::spinOnce();
		robot.getPosition(bMe);
		cMe=bMc.inverse()*bMe;
		try{
			listener.lookupTransform("/stereo_down_optical", "/valve_filtered", ros::Time(0), cMv_tf);
			cMv=tfToVisp(cMv_tf);
			cMv_pre=cMv*waypoint_pre;
		}
		catch(tf::TransformException &ex){
		}
		cMv_pre[0][3]=cMe[0][3];
		
	}

	

	std::cout<<"Open Gripper"<<std::endl;
	//Open the gripper
	vpColVector vel(5);
	vel=0;
	vel[4]=0.2;
	vpColVector current_joints;
	robot.getJointValues(current_joints);
	while(current_joints[4]<GRIPPER_MANIPULATION && ros::ok()){
		robot.setJointVelocity(vel);
		ros::spinOnce();
		robot.getJointValues(current_joints);
		//robot.getPosition(cMe);
	}


	//stop valve detection

	ros::service::call("/valve_tracker/stop_valve_detection", req, res);


	std::cout<<"Manipulation Joint Velocity"<<std::endl;
	//Reach the manipualation position
	joint_offset.get_bMc(bMc);
	robot.getPosition(bMe);
	vpHomogeneousMatrix cMv_man=cMv*waypoint_man;
	cMe=bMc.inverse()*bMe;
	cMv_man[0][3]=cMe[0][3];
	while((cMe.column(4)-cMv_man.column(4)).euclideanNorm()>0.02 && ros::ok()){
		vpColVector finalJoints(5), current_joints;
		vpHomogeneousMatrix bMv=bMc*cMv_man;
		vpHomogeneousMatrix bMe;
		robot.getPosition(bMe);
		bMv[1][3]=bMe[1][3];
		finalJoints=robot.armIK(bMv);
		robot.getJointValues(current_joints);
		if(finalJoints[0]>-1.57 && finalJoints[0]<2.1195 && finalJoints[1]>0 && finalJoints[1]<1.58665 && finalJoints[2]>0 && finalJoints[2]<2.15294){
			q=finalJoints-current_joints;
			q[0]=0;
			q[3]=0;
			q[4]=0;
			robot.setJointVelocity(q);
		}
		else{
			std::cout<<"IK not solve"<<std::endl;
		}
		ros::spinOnce();
		robot.getPosition(bMe);
		cMe=bMc.inverse()*bMe;
		cMv_man[0][3]=cMe[0][3];
	}
	std::cout<<"turn right"<<std::endl;
	//Turn right the valve
	vel=0;
	vel[3]=0.7;
	robot.getJointValues(current_joints);

	while((current_joints[3]<2) && ros::ok() && robot.getCurrent()<CURRENTTHRESHOLD){
		robot.setJointVelocity(vel);
		ros::spinOnce();
		robot.getJointValues(current_joints);
	}
	std::cout<<"Current= "<<robot.getCurrent()<<std::endl;
	sleep(1);
	while(robot.getCurrent()>CURRENTTHRESHOLD)
	{
		ros::spinOnce();
	}



	std::cout<<"turn left"<<std::endl;
	//Turn left the valve
	vel=0;
	vel[3]=-0.7;
	robot.getJointValues(current_joints);

	while((current_joints[3]>1.5) && ros::ok()){
		robot.setJointVelocity(vel);
		ros::spinOnce();
		robot.getJointValues(current_joints);
	}

	while((current_joints[3]>-0.5) && ros::ok() && robot.getCurrent()<CURRENTTHRESHOLD){
		robot.setJointVelocity(vel);
		ros::spinOnce();
		robot.getJointValues(current_joints);
	}
	std::cout<<"Current= "<<robot.getCurrent()<<std::endl;

	while(robot.getCurrent()>CURRENTTHRESHOLD)
	{
		ros::spinOnce();
	}

	std::cout<<"open gripper"<<std::endl;
	//Open the gripper
	vel=0;
	vel[4]=0.4;
	robot.getJointValues(current_joints);
	while(current_joints[4]<GRIPPER_OPENED && ros::ok()){
		robot.setJointVelocity(vel);
		ros::spinOnce();
		robot.getJointValues(current_joints);	
	}
	std::cout<<"Current= "<<robot.getCurrent()<<std::endl;
	sleep(1);
	std::cout<<"wrist to 0"<<std::endl;
	// wrist to 0 position
	vel=0;
	vel[3]=0.4;
	robot.getJointValues(current_joints);

	while((current_joints[3]<0.0) && ros::ok() && robot.getCurrent()<CURRENTTHRESHOLD){
		robot.setJointVelocity(vel);
		ros::spinOnce();
		robot.getJointValues(current_joints);
	}
	std::cout<<"Current= "<<robot.getCurrent()<<std::endl;



	std::cout<<"Pre Manipulation Joint Velocity"<<std::endl;
	//Reach the pre_manipualtion position
	joint_offset.get_bMc(bMc);
	robot.getPosition(bMe);
	cMe=bMc.inverse()*bMe;
	cMv_pre[0][3]=cMe[0][3];
	while((cMe.column(4)-cMv_pre.column(4)).euclideanNorm()>0.02 && ros::ok()){
		vpColVector finalJoints(5), current_joints;
		vpHomogeneousMatrix bMv=bMc*cMv_pre;
		robot.getJointValues(current_joints);
		vpHomogeneousMatrix bMe;
		robot.getPosition(bMe);
		bMv[1][3]=bMe[1][3];
		finalJoints=robot.armIK(bMv);
		if(finalJoints[0]>-1.57 && finalJoints[0]<2.1195 && finalJoints[1]>0 && finalJoints[1]<1.58665 && finalJoints[2]>0 && finalJoints[2]<2.15294){
			q=finalJoints-current_joints;
			std::cout<<"q: "<<q<<std::endl;
			q[0]=0;
			q[3]=0;
			q[4]=0;
			robot.setJointVelocity(q);
		}
		else{
			std::cout<<"ik not solve"<<std::endl;
		}
		ros::spinOnce();
		robot.getPosition(bMe);
		cMe=bMc.inverse()*bMe;
	cMv_pre[0][3]=cMe[0][3];
	}
		std::cout<<"Salgo pre-mani"<<std::endl;

	return 0;

}






