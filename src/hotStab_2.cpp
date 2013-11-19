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
#define GRIPPER_MANIPULATION 0.8
#define GRIPPER_OPENED 1.0

#define CURRENTTHRESHOLD 1.4

vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf){
	vpHomogeneousMatrix matrix_visp;
	matrix_visp[0][0]=matrix_tf.getBasis()[0][0]; matrix_visp[0][1]=matrix_tf.getBasis()[0][1]; matrix_visp[0][2]=matrix_tf.getBasis()[0][2]; matrix_visp[0][3]=matrix_tf.getOrigin().x();
	matrix_visp[1][0]=matrix_tf.getBasis()[1][0]; matrix_visp[1][1]=matrix_tf.getBasis()[1][1]; matrix_visp[1][2]=matrix_tf.getBasis()[1][2]; matrix_visp[1][3]=matrix_tf.getOrigin().y();
	matrix_visp[2][0]=matrix_tf.getBasis()[2][0]; matrix_visp[2][1]=matrix_tf.getBasis()[2][1]; matrix_visp[2][2]=matrix_tf.getBasis()[2][2]; matrix_visp[2][3]=matrix_tf.getOrigin().z();
	return matrix_visp;
}




int main(int argc, char **argv){
	ros::init(argc, argv, "hotStab2");
	ros::NodeHandle nh;

	//get Params
	std::string joint_state("");
	nh.getParam("joint_state", joint_state);
	std::string joint_state_command("");
	nh.getParam("joint_state_command", joint_state_command);
	std::string joint_state_fixed("");
	nh.getParam("joint_state_fixed", joint_state_fixed);



	////////
	vpColVector initial_posture=mar_params::paramToVispColVector(&nh, "initial_posture");
	vpHomogeneousMatrix waypoint_ext=mar_params::paramToVispHomogeneousMatrix(&nh, "waypoint_extract");
	vpHomogeneousMatrix waypoint_ins=mar_params::paramToVispHomogeneousMatrix(&nh, "waypoint_insert");




	JointOffset joint_offset(nh, joint_state, joint_state_command, joint_state_fixed);
	std::cout<<"joint_offset iniciado"<<std::endl;
	ARM5Arm robot(nh, joint_state_fixed, joint_state_command);

	vpColVector current_joints(5);
	robot.getJointValues(current_joints);
	joint_offset.reset_bMc(current_joints);


	///////Launch the service to detect the valve
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;
	ros::service::call("/template_pose/start_detection", req, res);

	//waiting for the first valve detection
	tf::TransformListener listener;
	bool connector_detected=false;
	tf::StampedTransform cMh_tf;
	while(!connector_detected && ros::ok()){
		try{
			listener.lookupTransform("/stereo_down_optical", "/connector", ros::Time(0), cMh_tf);
			connector_detected=true;
		}
		catch(tf::TransformException &ex){
		}
		ros::spinOnce();
	}
	vpHomogeneousMatrix cMh=tfToVisp(cMh_tf);



	vpHomogeneousMatrix object_turn(0,0,0,0,0, -1.57);





	vpHomogeneousMatrix bMc, bMe;
	robot.getPosition(bMe);
	joint_offset.get_bMc(bMc);



	vpHomogeneousMatrix cMe=bMc.inverse()*bMe;

	//Reach the extract position
	vpHomogeneousMatrix cMh_ext=cMh*waypoint_ext;
	robot.getPosition(bMe);
	while((cMe.column(4)-cMh_ext.column(4)).euclideanNorm()>0.03 && ros::ok()){

		vpColVector xdot(6);
		xdot=0;
		vpHomogeneousMatrix eMv=cMe.inverse()*cMh_ext;
		xdot[0]=eMv[0][3]*0.4;
		xdot[1]=eMv[1][3]*0.4;
		xdot[2]=eMv[2][3]*0.4;
		robot.setCartesianVelocity(xdot);
		ros::spinOnce();
		robot.getPosition(bMe);
		cMe=bMc.inverse()*bMe;
	}

	//stop valve detection
	ros::service::call("/template_pose/start_detection", req, res);


	//Reach the insert position
	vpHomogeneousMatrix cMh_ins=cMh*waypoint_ins;
	robot.getPosition(bMe);
	while((cMe.column(4)-cMh_ins.column(4)).euclideanNorm()>0.015 && ros::ok()){
		vpColVector xdot(6);
		xdot=0;
		vpHomogeneousMatrix eMv=cMe.inverse()*cMh_ins;
		xdot[0]=eMv[0][3]*0.4;
		xdot[1]=eMv[1][3]*0.4;
		xdot[2]=eMv[2][3]*0.4;
		robot.setCartesianVelocity(xdot);
		ros::spinOnce();
		robot.getPosition(bMe);
		cMe=bMc.inverse()*bMe;
	}
	//Open the gripper
	vpColVector vel(5);
	vel=0;
	vel[4]=0.4;
	robot.getJointValues(current_joints);
	while(current_joints[4]<GRIPPER_OPENED && ros::ok() && robot.getCurrent()<CURRENTTHRESHOLD){
		robot.setJointVelocity(vel);
		ros::spinOnce();
		robot.getJointValues(current_joints);
	}


	//Reach the extract position
	cMh_ext=cMh*waypoint_ext;
	robot.getPosition(bMe);
	while((cMe.column(4)-cMh_ext.column(4)).euclideanNorm()>0.03 && ros::ok()){
		vpColVector xdot(6);
		xdot=0;
		vpHomogeneousMatrix eMv=cMe.inverse()*cMh_ext;
		xdot[0]=eMv[0][3]*0.4;
		xdot[1]=eMv[1][3]*0.4;
		xdot[2]=eMv[2][3]*0.4;
		robot.setCartesianVelocity(xdot);
		ros::spinOnce();
		robot.getPosition(bMe);
		cMe=bMc.inverse()*bMe;
	}
	//Park
	vel=0;
	vel[2]=-0.4;
	robot.getJointValues(current_joints);
	vpColVector final_joints(5);
	final_joints=current_joints;
	final_joints[2]-=0.4;
	while(current_joints[2]>GRIPPER_CLOSED && ros::ok() && robot.getCurrent()<CURRENTTHRESHOLD){
		robot.setJointVelocity(vel);
		ros::spinOnce();
		robot.getJointValues(current_joints);
	}

	return 0;

}






