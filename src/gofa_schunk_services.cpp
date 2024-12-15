#include "gofa_schunk_services.hpp"

Gofa_schunk_services::Gofa_schunk_services(ros::NodeHandle &nh) : nh_(nh) {
    // -- Usefull parameters --
    private_nh = ros::NodeHandle("~");
    private_nh.getParam("manual_move_confirm", this->manual_move_confirm);
    private_nh.getParam("simulated_robot", this->simulated_robot);

    ROS_INFO("Simulated robot: %s", this->simulated_robot ? "true" : "false");
    
    // services 
    srv_executeWaypoint = nh_.advertiseService("move_to", &Gofa_schunk_services::MoveToService, this);
    srv_closeGripper = nh_.advertiseService("close_gripper", &Gofa_schunk_services::closeGripperService, this);
    srv_openGripper = nh_.advertiseService("open_gripper", &Gofa_schunk_services::openGripperService, this);

    cl_simpleGrip = nh_.serviceClient<schunk_interfaces::SimpleGrip>("/schunk/egk_40/simple_grip");
    cl_simpleRelease = nh_.serviceClient<schunk_interfaces::JogTo>("/schunk/egk_40/jog_to");

    // Wait for the "save_contact" service to become available
    // ROS_INFO("Waiting for save_contact service to become available...");
    // cl_saveContactFirstFinger.waitForExistence();  
    // cl_saveContactSecondFinger.waitForExistence();
    // ROS_INFO("save_contact service is now available.");
    
    this->initMoveit();
}


bool Gofa_schunk_services::openGripperService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    // * This function will open the gripper
    // * The function will return a boolean value to confirm the execution of the skill
    std::cout << "Opening gripper" << std::endl;
    if (!this->openGripper())
    {
        res.success = false;
        return false;
    }
    res.success = true;
    return true;
}

bool Gofa_schunk_services::openGripper(void)
{
    // * This function will open the gripper
    // * The function will return a boolean value to confirm the execution of the skill

    if (this->simulated_robot)
    {
        ROS_INFO("Simulated robot, skipping the opening of the gripper.");
        return true;
    }

    schunk_interfaces::JogTo srv;
    srv.request.position = 0.0;
    srv.request.motion_type = 0;
    srv.request.velocity = 50.0;

    if (!this->cl_simpleRelease.call(srv))
    {
        ROS_ERROR("Failed to call service simple_release");
        return false;
    }

    return true;
}

bool Gofa_schunk_services::closeGripperService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    // * This function will close the gripper
    // * The function will return a boolean value to confirm the execution of the skill
    if (!this->closeGripper())
    {
        res.success = false;
        return false;
    }
    res.success = true;
    return true;
}

bool Gofa_schunk_services::closeGripper(void)
{
    // * This function will open the gripper
    // * The function will return a boolean value to confirm the execution of the skill

    if (this->simulated_robot)
    {
        ROS_INFO("Simulated robot, skipping the closing of the gripper.");
        return true;
    }

    schunk_interfaces::SimpleGrip srv;
    srv.request.gripping_force = 50.0;
    if (!this->cl_simpleGrip.call(srv))
    {
        ROS_ERROR("Failed to call service simple_grip");
        return false;
    }
    return true;
}

bool Gofa_schunk_services::MoveToService(gofa_schunk_services::MoveTo::Request& req, gofa_schunk_services::MoveTo::Response& res)
{
    // * This function will execute the one movement with the associated gripper skill
    // * So it want execute the overall skill
    // * The function will return a boolean value to confirm the execution of the skill
    geometry_msgs::PoseStamped targetPose = req.next_waypoint;
    int n_attemps = 3;

    for (int i = 0; i < n_attemps; i++)
    {
        ROS_INFO("Trying to move with a MOVEL iteration %d", i);
        
        if (moveLToPose(targetPose))
        {
            ROS_INFO("MOVEL succeeded");
            res.success = true;
            return true;
        }
        ROS_INFO("MOVEL failed, trying with a MOVEJ iteration %d", i);
        if (moveJToPose(targetPose))
        {
            ROS_INFO("MOVEJ succeeded");
            res.success = true;
            return true;
        }
        ROS_INFO("MOVEJ failed, trying again... with next iteration ");
        if (i==1)
        {
            //move to home position
        }
    }

    ROS_ERROR("!!!!!MOVEMENT FAILED!!!!!");
    res.success = false;
    return false;
    // Try to execute this with MoveL and MoveJ trying to execute it even when the first planning get wrong
    // bool move_success = this->moveLToPose(targetPose);
    // res.success = move_success;  
    // return true;
}

void Gofa_schunk_services::spinner(void)
{
    // We are usign the async spinner, so we don't need to call ros::spin()
    // ros::Rate rate(60);
    // while (ros::ok())
    // {
    //     rate.sleep();
    // }
}

// -- Moveit! stuff --
void Gofa_schunk_services::initMoveit(void)
{
    static const std::string PLANNING_GROUP = "gofa_arm";
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    planning_scene_interface = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // visual tools are needed to use the rviz visual gui
    visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("base");

    move_group->setMaxVelocityScalingFactor(0.15);
    move_group->setMaxAccelerationScalingFactor(0.15);

    ROS_INFO("Moveit! is ready.");
}

bool Gofa_schunk_services::moveJToPose(geometry_msgs::PoseStamped targetPose)
{
    // * The pose has its own referenfe frame visibile in targetPose.header.frame_id and it is used by the robot to plan!
    // * Please specify the frame id before calling this function!

    // Set the target pose
    this->move_group->setPoseTarget(targetPose);
    this->move_group->setPlannerId("PTP");
    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (this->move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("MoveJ gofa_arm", "Planning of Move J to (pose goal): %s", success ? "SUCCESS" : "FAILED");

    move_group->setMaxVelocityScalingFactor(0.1);
    move_group->setMaxAccelerationScalingFactor(0.1);

    if (success)
    {
        if (this->manual_move_confirm)
            this->visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to execute the MoveJ.");
        auto execute_result = this->move_group->execute(my_plan);

        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Movement executed successfully.");
            return true;
        }
        else
        {
            ROS_ERROR("Movement execution failed.");
            return false;
        }
    }
    else
    {
        ROS_INFO("Failed to execute path.");
        return false;
    }
}

bool Gofa_schunk_services::moveLToPose(geometry_msgs::PoseStamped targetPose)
{
    // * The pose has its own referenfe frame visibile in targetPose.header.frame_id and it is used by the robot to plan!
    // * Please specify the frame id before calling this function!

    // Set the target pose
    this->move_group->setPoseTarget(targetPose);
    this->move_group->setPlannerId("LIN");
    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (this->move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("MoveJ gofa_arm", "Planning of Move L to (pose goal): %s", success ? "SUCCESS" : "FAILED");

    if (success)
    {
        ROS_INFO("Executing plan...");
        if (this->manual_move_confirm)
            this->visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to execute the MoveL.");
        auto execute_result = this->move_group->execute(my_plan);
        ROS_INFO("Execution finished.");

        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Movement executed successfully.");
            return true;
        }
        else
        {
            ROS_ERROR("Movement execution failed.");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Planning failed.");
        return false;
    }
}

void Gofa_schunk_services::tryMove(void)
{
    geometry_msgs::PoseStamped pose1;
    pose1.header.frame_id = "world";
    pose1.header.stamp.sec = 0; // Secondi
    pose1.header.stamp.nsec = 235062599;
    pose1.pose.position.x = 0.5715697290668968;
    pose1.pose.position.y = -1.8997843242429832e-06;
    pose1.pose.position.z = 0.8990243749818855;
    pose1.pose.orientation.x = 1.6242014786707746e-06;
    pose1.pose.orientation.y = 0.7070951878466983;
    pose1.pose.orientation.z = -1.200541454943199e-06;
    pose1.pose.orientation.w = 0.7071183743334373;

    moveLToPose(pose1);
    ROS_INFO("Moved to pose 1");

    geometry_msgs::PoseStamped pose2;
    pose2.header.frame_id = "world";
    pose2.header.stamp.sec = 0; // Secondi
    pose2.header.stamp.nsec = 235062599;
    pose2.pose.position.x = 0.4082195807991848;
    pose2.pose.position.y = 0.378312148033888;
    pose2.pose.position.z = 0.531395219606578;
    pose2.pose.orientation.x = -0.3647267212241974;
    pose2.pose.orientation.y = 0.9302851683262353;
    pose2.pose.orientation.z = 0.014326030581843341;
    pose2.pose.orientation.w = 0.03658810278003838;
    // RPY corrispondenti: [-3.1415583692394815, 0.07860583526748141, -2.394307873844392]

    moveLToPose(pose2);
    ROS_INFO("Moved to pose 2");
    geometry_msgs::PoseStamped pose3;
    pose3.header.frame_id = "world";
    pose3.header.stamp.sec = 0; // Secondi
    pose3.header.stamp.nsec = 235062599;
    pose3.pose.position.x = 0.4082195807991848;
    pose3.pose.position.y = 0.378312148033888;
    pose3.pose.position.z = 0.531395219606578;
    pose3.pose.orientation.x = -0.3647267212241974;
    pose3.pose.orientation.y = 0.9302851683262353;
    pose3.pose.orientation.z = 0.014326030581843341;
    pose3.pose.orientation.w = 0.03658810278003838;
    // RPY corrispondenti: [-3.1415583692394815, 0.07860583526748141, -2.394307873844392]

    moveLToPose(pose3);
    ROS_INFO("Moved to pose 3");
    geometry_msgs::PoseStamped pose4;
    pose4.header.frame_id = "world";
    pose4.header.stamp.sec = 0; // Secondi
    pose4.header.stamp.nsec = 235062599;
    pose4.pose.position.x = 0.38340076688168345;
    pose4.pose.position.y = -0.4812257312299913;
    pose4.pose.position.z = 0.19260802345827582;
    pose4.pose.orientation.x = 0.4579499541988648;
    pose4.pose.orientation.y = 0.8884144599286377;
    pose4.pose.orientation.z = 0.0002630749418405672;
    pose4.pose.orientation.w = 0.031646763350139856;

    moveLToPose(pose4);
    ROS_INFO("Moved to pose 4");
}
