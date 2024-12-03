#ifndef GOFA_SCHUNK_SERVICES_HPP
#define GOFA_SCHUNK_SERVICES_HPP

// -- Standard ROS STUFF --
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

// -- MoveIt! --
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

// -- shunk gripper STUFF --

// -- Services
#include <gofa_schunk_services/MoveTo.h>
#include <schunk_interfaces/JogTo.h>
#include <schunk_interfaces/SimpleGrip.h>

class Gofa_schunk_services
{
public:
	Gofa_schunk_services(ros::NodeHandle &nh);
	void spinner(void);

private:
	// -- Standard ROS STUFF --
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh;

	// subscribers and publishers
	ros::Subscriber sub_;
	ros::Publisher pub_;

	// services and clients
	ros::ServiceClient cl_openGripper;
	ros::ServiceClient cl_closeGripper;
	ros::ServiceServer srv_executeWaypoint;
	bool MoveToService(gofa_schunk_services::MoveTo::Request &req, gofa_schunk_services::MoveTo::Response &res);
	ros::ServiceServer srv_closeGripper;
	bool closeGripperService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool closeGripper(void);
	ros::ServiceServer srv_openGripper;
	bool openGripperService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool openGripper(void);

	// shunk services
	ros::ServiceClient cl_simpleRelease;
	ros::ServiceClient cl_simpleGrip;

	// -- MoveIt! STUFF --
	bool manual_move_confirm;
	std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
	std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
	std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
	const robot_state::JointModelGroup *joint_model_group;
	void initMoveit(void);
	bool moveJToPose(geometry_msgs::PoseStamped targetPose);
	bool moveLToPose(geometry_msgs::PoseStamped targetPose);
	bool moveLToContact(void);
	bool moveLToHome(void);

	void tryMove(void);

};

#endif // GOFA_SCHUNK_SERVICES_HPP
