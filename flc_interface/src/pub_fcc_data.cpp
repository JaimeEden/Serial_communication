#include <ros/ros.h>
#include "flc_interface/fcc_function.hpp"
#include <iostream>
#include <std_msgs/String.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fcc_node");
    ros::NodeHandle nh;
    int baudrate;
    std::string serial_channel;
    nh.param("baudrate", baudrate,115200);
    nh.param<std::string>("serial_port", serial_channel, "/dev/ttyUSB0");
    std::cout<< serial_channel<<std::endl;
    serial_interface_fun ob(nh, baudrate, serial_channel);
    // set publishe frequency
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ob.parse_data_and_pub();
        ros::spinOnce();
        //ob.sub_data_and_package(nh);

        loop_rate.sleep();
        // loop_rate.sleep();
        
    }
    return 0;

}
