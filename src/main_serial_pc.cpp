#include <iostream>
#include <ros/ros.h>
#include "serial_comm_ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "fmu_serial_pc_node");
    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("fmu_serial_pc_node - starts.");
	
	try{
		if(ros::ok()) {
			std::shared_ptr<SerialCommROS> serial_ros;
			serial_ros = std::make_shared<SerialCommROS>(nh);
		}
		else{
			throw std::runtime_error("ros not ok");
		}
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("fmu_serial_pc_node - terminated.");
	return 0;
}