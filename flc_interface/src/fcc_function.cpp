#include "flc_interface/fcc_function.hpp"
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <cmath>

boost::posix_time::time_duration gps_to_date(int gps_week, int gps_wins) {
    // 定义GPS起始时间（1980年1月6日）
    // std::chrono::system_clock::time_point gps_start_time = std::chrono::system_clock::from_time_t(0);
    boost::posix_time::ptime gpsEpochStart(boost::gregorian::date(1980, 1, 6));
    boost::posix_time::ptime gpsTimePoint = gpsEpochStart + boost::posix_time::seconds(int(gps_week * 7 * 24 * 3600 + gps_wins*0.001)) ;
    boost::posix_time::time_duration duration = gpsTimePoint - boost::posix_time::from_time_t(0);

    // 转换为时间点
    // boost::posix_time::ptime final_time = boost::posix_time::from_gmtime(gpsTimePoint.time_of_day());
    // boost::posix_time::ptime final_time;
    // try {
    //     final_time = boost::posix_time::from_string(gpsTimePoint.time_of_day().str());
    // } catch (const std::exception& e) {
    //     std::cerr << "Error converting GPS time to UTC: " << e.what() << std::endl;
    //     // return ;
    // }
    // auto start_time = std::chrono::system_clock::to_time_t(gps_start_time);
    
    // 计算从GPS起始时间到给定GPS周和周内秒数的总秒数
    // auto total_seconds = gps_week * 604800 + gps_wins;
    
    // // 将总秒数转换为时间差
    
    // auto time_diff = std::chrono::seconds(total_seconds);
    
    // // 计算最终时间点
    // auto final_time = gps_start_time + time_diff;
    
    return duration;
}

typedef unsigned char INT8U;
typedef unsigned short INT16U;
static const INT16U CRC16Tab_CCITT[256] =
{
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

INT16U CheckCRC16(INT8U *pBuf, INT16U len)
{
INT16U i = 0;
INT16U crc = 0;
INT8U crcHi = 0; // CRC 结果高字节临时变量
for(i = 0; i < len; i++)
{
crcHi = (INT8U)((crc & 0xFF00) >> 8); // 以 8 位二进制数暂存CRC 的高 8 位 //crcHi = (INT8U)(crc/256);
crc <<= 8; // 左移 8 位
crc ^= CRC16Tab_CCITT[crcHi ^ (*pBuf)]; // 高字节和当前数据 XOR 再查表
pBuf++;
}
return crc;
}

void serial_interface_fun::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 从收到的消息中获取机器人的位置信息
        int x = msg->pose.pose.position.x*1e3;
        int y = msg->pose.pose.position.y*1e3;
        short z = (msg->pose.pose.position.z + 500) * 5;


        // 获取机器人的线速度和角速度信息
        short linear_x = msg->twist.twist.linear.x*100;
        short linear_y = msg->twist.twist.linear.y*100;
        short linear_z = msg->twist.twist.linear.z*100;

        short angular_x = msg->twist.twist.angular.x*18000/M_PI;
        short angular_y = msg->twist.twist.angular.y*18000/M_PI;
        unsigned short angular_z;
        if(msg->twist.twist.angular.z*180/M_PI >= 360)
        {
            angular_z = (msg->twist.twist.angular.z*180/M_PI - 360)*100;
        }
        else if(msg->twist.twist.angular.z*180/M_PI < 0)
        {
            angular_z = (msg->twist.twist.angular.z*180/M_PI + 360)*100;
        }
        else
        {
            angular_z = msg->twist.twist.angular.z*18000/M_PI;
        }

	std::cout<<sizeof(fcc_data_sent_obj)<<std::endl;
    	unsigned char temp[40];
    	fcc_data_sent_obj.frame_header1 =  0xeb;
    	fcc_data_sent_obj.frame_header2 =  0x90;
    	// fcc_data_sent_obj.application_frame =  0x10;
    	fcc_data_sent_obj.frame_length = 38;
    	fcc_data_sent_obj.time_frame = ros::Time::now().sec;
    	fcc_data_sent_obj.flag_for_initialization = 1;
    	fcc_data_sent_obj.flag_for_avalible_localization_data = 1;

    	fcc_data_sent_obj.velocity_to_north = linear_x;
    	fcc_data_sent_obj.velocity_to_east = linear_y;
    	fcc_data_sent_obj.velocity_to_sky = linear_z;
    	fcc_data_sent_obj.latitude = x;
    	fcc_data_sent_obj.longitude = y;
    	fcc_data_sent_obj.altitude = z;
    	fcc_data_sent_obj.roll = angular_x;
    	fcc_data_sent_obj.pitch = angular_y;
    	fcc_data_sent_obj.yaw = angular_z;
    	fcc_data_sent_obj.reserved_bit = 0;
    	// fcc_data_sent_obj.checksum = CheckCRC16(&fcc_data_sent_obj, 36);
    	memcpy(temp, &fcc_data_sent_obj, 36);
    	INT16U checksum_cur;
    	checksum_cur = CheckCRC16(temp, 36);
    	memcpy(temp+36, &checksum_cur, 2);
    	// std::cout << "sent_checksum:"<<checksum_cur<<std::endl;
    	ser_obj.write(temp, 38);
	std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    }

serial_interface_fun::serial_interface_fun(ros::NodeHandle &nh, int baudrate, std::string serial_channel)
{
    nh_private=nh;
    ser_obj.setPort(serial_channel);
    ser_obj.setBaudrate(baudrate);
    time_out = serial::Timeout::simpleTimeout(1000);
    ser_obj.setTimeout(time_out);
    pub_obj = nh_private.advertise<flc_interface::fcc>("FligtControler_info",10);
    sub = nh_private.subscribe("/Odometry", 100, &serial_interface_fun::odometryCallback, this);
    pub_nav = nh_private.advertise<sensor_msgs::NavSatFix>("gps",10);
    try
    {
        ser_obj.open();
    }
    catch(const std::exception& e)
    {
        ROS_INFO_STREAM("Open serail port failed!");

    }

    if(ser_obj.isOpen())
    {
        ROS_INFO_STREAM("Open serial port successfully.");
    }else
    {
        ROS_INFO_STREAM("Open serail port failed! Warn again!");
    }
    
}

// serial_interface_fun::~serial_interface_fun()
// {
//     ser_obj.close();
// }

void serial_interface_fun::parse_data_and_pub()
{
    
    bool control_placeholder = true;
    int data_size = ser_obj.available();
    int count = 0;
    std::cout << "data_size:"<< data_size<<std::endl;
    if(data_size==31) //记得修改
    {
        // std::cout<<"data_size:"<<data_size<<std::endl;
        memset(buffer,'\0', sizeof(buffer));
        ser_obj.read(buffer, data_size);
        //大端转小端
        char cMesType[18]={0};
        int cMesNum = 18;
        cMesType[0] = 1;
        cMesType[1] = 1;
        cMesType[2] = 1;
        cMesType[3] = 1;
        cMesType[4] = 1;
        cMesType[5] = 1;
        cMesType[6] = 1;
        cMesType[7] = 2;
        cMesType[8] = 4;
        cMesType[9] = 4;
        cMesType[10] = 4;
        cMesType[11] = 2;
        cMesType[12] = 2;
        cMesType[13] = 2;
        cMesType[14] = 2;
        cMesType[15] = 2;
        cMesType[16] = 4;
        cMesType[17] = 2;
        // cMesType[17] = 2;
        // cMesType[18] = 2;
        // cMesType[19] = 1;
        // char *buffer_converted = reinterpret_cast<char*>(buffer);
        // Small2BigEnd(cMesNum, cMesType, buffer);
        char check_sum_buffer[2];
        // unsigned short* check_test;
        INT16U checksum_cur;
        memcpy(check_sum_buffer, buffer+data_size-2, 2);
        checksum_cur = check_sum_buffer[0]<<8 | check_sum_buffer[1];
        // checksum_cur = *check_test;
        // if((checksum_cur ^ CheckCRC16(buffer, 35)) != 0) 
        // {
        //     ros::spinOnce();
        //     return;
        // }
        memcpy(&fcc_data_obj, buffer, sizeof(fcc_data_received));
        // std::cout<< "fcc_data_obj.navigation_indicator:" << fcc_data_obj.frame_header1 << std::endl;  
        // publish message
        flc_interface::fcc msg_need_pub;
        sensor_msgs::NavSatFix msg_gps;
        std::cout<< "num:"<< sizeof(fcc_data_received)<<std::endl;
        if(control_placeholder)
        {
            
            msg_need_pub.header_ros.seq = count;
            msg_need_pub.header_ros.frame_id = "fcu";
            msg_need_pub.header_ros.stamp = ros::Time::now();
            msg_need_pub.frame_header1.data = fcc_data_obj.frame_header1;
            msg_need_pub.frame_header2.data = fcc_data_obj.frame_header2;
            // msg_need_pub.indicator_of_frame.data = fcc_data_obj.indicator_of_frame;
            msg_need_pub.frame_count.data = fcc_data_obj.frame_count;
            msg_need_pub.frame_length.data = fcc_data_obj.frame_length;
            msg_need_pub.flag_for_launch.data = fcc_data_obj.flag_for_launch;
            msg_need_pub.valid_flag_for_gps.data = fcc_data_obj.valid_flag_for_gps;
            msg_need_pub.gps_cycle.data = fcc_data_obj.gps_cycle;
            msg_need_pub.tic_toc.data = fcc_data_obj.tic_toc;
            msg_need_pub.latitude.data = ((double)fcc_data_obj.latitude)/(1e7);
            msg_need_pub.longitude.data = ((double)fcc_data_obj.longitude)/(1e7);
            msg_need_pub.altitude.data = ((double)fcc_data_obj.altitude)*0.2 -500;
            msg_gps.header.seq = fcc_data_obj.frame_count;
            auto time_utc = gps_to_date(fcc_data_obj.gps_cycle, fcc_data_obj.tic_toc);
	    //ros::Time timeStart(1970,1,1);
	    //ros::Duration duration = ros::Time::fromBoost(time_utc);
 	    //ros::Time timeEnd =
	    //ros::Time end1 = ros::Time::fromBoost(time_utc);
	    //ros::Time end2 = end1.toSec();
	    uint32_t seconds = time_utc.total_seconds();
	    ros::Time finalTime(seconds-18,uint32_t(fcc_data_obj.tic_toc % 1000)*1e6);
            msg_gps.header.stamp = finalTime;
            msg_gps.status.status = 0;
            msg_gps.status.service = 1;
            msg_gps.latitude = msg_need_pub.latitude.data ;
            msg_gps.longitude = msg_need_pub.longitude.data;
            msg_gps.altitude = msg_need_pub.altitude.data ;
            if(msg_need_pub.valid_flag_for_gps.data ==1)
            {
                // float cov[9] = {0.1,0,0,0,0.1,0,0,0,0.1};
                msg_gps.position_covariance = {0.001,0,0,0,0.001,0,0,0,0.001};
            }else
            {
                msg_gps.position_covariance = {100,0,0,0,100,0,0,0,100};
            }
            msg_gps.position_covariance_type = 0;
            // time_utc
            // msg_need_pub.velocity_to_north.data = ((float)fcc_data_obj.velocity_to_north)/(100);
            // msg_need_pub.velocity_to_east.data = ((float)fcc_data_obj.velocity_to_east)/(100);
            // msg_need_pub.velocity_to_sky.data = ((float)fcc_data_obj.velocity_to_sky)/(100);
            // msg_need_pub.roll.data = ((float)fcc_data_obj.roll)/(100);
            // msg_need_pub.pitch.data = ((float)fcc_data_obj.pitch)/(100);
            // msg_need_pub.yaw.data = ((float)fcc_data_obj.yaw)/(10);
            // msg_need_pub.angular_velocity_x.data = ((float)fcc_data_obj.angular_velocity_x)/(10);
            // msg_need_pub.angular_velocity_y.data = ((float)fcc_data_obj.angular_velocity_y)/(10);
            // msg_need_pub.angular_velocity_z.data = ((float)fcc_data_obj.angular_velocity_z)/(10);
            // msg_need_pub.acceleration_x.data = ((float)fcc_data_obj.acceleration_x)/(100);
            // msg_need_pub.acceleration_y.data = ((float)fcc_data_obj.acceleration_y)/(100);
            // msg_need_pub.acceleration_z.data = ((float)fcc_data_obj.acceleration_z)/(100);
            msg_need_pub.orientation_uav.data = ((float)fcc_data_obj.orientation)/(100);
            // msg_need_pub.armX.data = ((float)fcc_data_obj.armX)/100;
            // msg_need_pub.armY.data = ((float)fcc_data_obj.armY)/100;
            // msg_need_pub.armZ.data = ((float)fcc_data_obj.armZ)/100;
            //msg_need_pub.localization_accuracy_factor.data = ((float)fcc_data_obj.localization_accuracy_factor)/10;
            msg_need_pub.checksum.data = fcc_data_obj.checksum;
            // if(fcc_data_obj.frame_header1==0xeb && fcc_data_obj.frame_header2 == 0x90)
            // {
            //     pub_obj.publish(msg_need_pub);
            // }
            if((fcc_data_obj.checksum ^ CheckCRC16(buffer, data_size-2)) == 0) 
            {
                pub_obj.publish(msg_need_pub);
                pub_nav.publish(msg_gps);
            }
            
            std::cout<<"checksum_cur:"<<checksum_cur<<"real:"<< fcc_data_obj.checksum<<std::endl;
            
        }    
        count += 1;
    }else
    {
        ser_obj.flushInput();
    }
    // else
    // {
    //     // std::cout<<"error_data_size:"<<data_size<<std::endl;
    //     // ROS_INFO_STREAM("no data received");
    // }

    ros::spinOnce();
}


 

   
        
void serial_interface_fun::Small2BigEnd(int cMesNum, char cMesType[18], unsigned char* pMes)
{
    short sData = 0;
    int nData = 0;
    int nCurLen = 0;
    for(int i = 0; i < cMesNum; i++)
    {
        if(cMesType[i] == 2)
        {
            memcpy(&sData, pMes + nCurLen, sizeof(short));
            sData = TRVS16(sData);
            memcpy(pMes + nCurLen, &sData, sizeof(short));
            nCurLen += 2;
        }
        else if(cMesType[i] == 4)
        {
            memcpy(&nData, pMes + nCurLen, sizeof(int));
            nData = TRVS32(nData);
            memcpy(pMes + nCurLen, &nData, sizeof(int));
            nCurLen += 4;
        }
        else 
        {
            nCurLen += cMesType[i];
        }
    }
}
