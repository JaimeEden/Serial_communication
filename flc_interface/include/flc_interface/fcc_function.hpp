#ifndef FCC_FUNCTION
#define FCC_FUNCTION

#include <ros/ros.h>
#include <serial/serial.h>
#include "flc_interface/fcc.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
typedef unsigned int UINT32;
typedef unsigned short UINT16;
#define TRVS16(X) ((((UINT16)(X) & 0xff00)>> 8) | (((UINT16)(X) & 0x00ff) <<8))
#define TRVS32(X) ((((UINT32)(X) & 0xff000000) >> 24) | (((UINT32)(X) & 0x00ff0000) >>8) | (((UINT32)(X) & 0x0000ff00) << 8) | (((UINT32)(X) & 0x000000ff) << 24))

struct fcc_data_received
{
    char frame_header1 __attribute__((packed));
    char frame_header2 __attribute__((packed));
    char indicator_of_frame __attribute__((packed));
    char frame_count __attribute__((packed));
    char frame_length __attribute__((packed));
    char flag_for_launch __attribute__((packed));
    char valid_flag_for_gps __attribute__((packed));
    unsigned short gps_cycle __attribute__((packed));
    unsigned int tic_toc __attribute__((packed));
    int latitude __attribute__((packed));
    int longitude __attribute__((packed));
    short altitude __attribute__((packed));
    // short velocity_to_north __attribute__((packed));
    // short velocity_to_east __attribute__((packed));
    // short velocity_to_sky __attribute__((packed));
    unsigned short orientation __attribute__((packed));
    // short armX __attribute__((packed));
    // short armY __attribute__((packed));
    // short armZ __attribute__((packed));
    //unsigned short localization_accuracy_factor __attribute__((packed));
    int reserved_bit __attribute__((packed));
    unsigned short checksum __attribute__((packed));
};

struct fcc_data_sent
{
    char frame_header1 __attribute__((packed));
    char frame_header2 __attribute__((packed));
    char application_frame __attribute__((packed));
    // char frame_count;
    char frame_length __attribute__((packed));
    unsigned int time_frame __attribute__((packed));
    char flag_for_initialization __attribute__((packed));
    char flag_for_avalible_localization_data __attribute__((packed));
    // 存储位置信息的成员变量
    //double pos_x __attribute__((packed));
    //double pos_y __attribute__((packed));
    //double pos_z __attribute__((packed));

    // 存储四元数信息的成员变量
    //double pos_qx __attribute__((packed));
    //double pos_qy __attribute__((packed));
    //double pos_qz __attribute__((packed));
    //double pos_qw __attribute__((packed));

    // 存储线速度和角速度信息的成员变量
    //double linear_x __attribute__((packed));
    //double linear_y __attribute__((packed));
    //double linear_z __attribute__((packed));
    //double angular_x __attribute__((packed));
    //double angular_y __attribute__((packed));
    //double angular_z __attribute__((packed));

    short velocity_to_north __attribute__((packed));
    short velocity_to_east __attribute__((packed));
    short velocity_to_sky __attribute__((packed));
    int latitude __attribute__((packed));
    int longitude __attribute__((packed));
    short altitude __attribute__((packed));
    short roll __attribute__((packed));
    short pitch __attribute__((packed));
    unsigned short yaw __attribute__((packed));
    int reserved_bit __attribute__((packed));
    unsigned short checksum __attribute__((packed));
};


class serial_interface_fun{
    public:
    serial_interface_fun(ros::NodeHandle &nh, int baudrate, std::string serial_channel);
    ~serial_interface_fun(){ser_obj.close(); std::cout<< "ser closed"<<std::endl;};
    void parse_data_and_pub();
    void Small2BigEnd(int cMesNum, char cMesType[20], unsigned char* pMes);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    //void sub_data_and_package(ros::NodeHandle &nh);
    
    private:
    fcc_data_received fcc_data_obj;
    fcc_data_sent fcc_data_sent_obj;
    unsigned char buffer[40];//正常情况每次收到37个字节
    ros::NodeHandle nh_private;
    serial::Serial ser_obj;
    serial::Timeout time_out;
    ros::Publisher pub_obj;
    ros::Subscriber sub;
    ros::Publisher pub_nav;
};


#endif
