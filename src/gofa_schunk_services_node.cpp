#include "gofa_schunk_services.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Gofa_schunk_services_node");
	ros::AsyncSpinner spinner(2);
    spinner.start();
	
	ros::NodeHandle nh;
	Gofa_schunk_services Gofa_schunk_services(nh);

    ros::waitForShutdown(); 
	// poseEstimator.spinner();
	return 0;
}