/*
 *                   江城子 . 程序员之歌
 * 
 *               十年生死两茫茫，写程序，到天亮。
 *                   千行代码，Bug何处藏。
 *               纵使上线又怎样，朝令改，夕断肠。
 * 
 *               领导每天新想法，天天改，日日忙。
 *                   相顾无言，惟有泪千行。
 *               每晚灯火阑珊处，夜难寐，加班狂。
 * 
 * 
 * @Author: Glory Huang
 * @Date: 2020-09-05 15:14:04
 * @LastEditors: Glory Huang
 * @LastEditTime: 2021-01-02 16:56:04
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ecal_to_ros/RadarDetectionImage.h>
#include <ecal_to_ros/RadarDetection.h>
#include <cmath>
#include <fstream>
#include <iostream>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <calibration.h>

#include <termio.h>
#include <stdio.h>
#include <thread>

#define THR_FAP 0.2 // thresthold of false alarm probability

using namespace message_filters;
using namespace std;

string pkg_loc;
bool sys_pause = false;
double systime;

// 获取键盘输入
int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    in = getchar();

    tcsetattr(0, TCSANOW, &stored_settings);
    return in;
}

void stop2calibration()
{
    while (ros::ok())
    {
        if (scanKeyboard() == 32)
        {
            sys_pause = true;
            if (sys_pause)
            {
                std::cout << "Start to calibrate!" << std::endl;
            }
            else
            {
                std::cout << "Calibrate end!" << std::endl;
            }
        }
    }
}

void calibration_callback(const sensor_msgs::PointCloud2ConstPtr lidar_msg, const ecal_to_ros::RadarDetectionImageConstPtr radar_msg)
{
    systime = ros::Time::now().toSec();
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

    uint32_t N = radar_msg->u_NofDetections;
    for (auto i = 0; i < N; i++)
    {
        if (radar_msg->a_RadarDetectionList[i].f_Pdh0 < THR_FAP)
        {
            pcl::PointXYZ p;
            float r = radar_msg->a_RadarDetectionList[i].f_Range;
            float v = radar_msg->a_RadarDetectionList[i].f_VrelRad;
            float phi = -radar_msg->a_RadarDetectionList[i].a_AzAng_hyp[0];
            p.x = r * sin(phi);
            p.y = r * cos(phi);
            p.z = 0;
            radar_cloud->push_back(p);
        }
    }

    RadarCalibration radar_cali(lidar_cloud, radar_cloud);
    // radar_cali.showLidar();
    // radar_cali.showRadar();
    if(sys_pause)
    {
        pcl::io::savePCDFileASCII(pkg_loc + "/data/radar_" + std::to_string(systime) + ".pcd", *radar_cloud);
        pcl::io::savePCDFileASCII(pkg_loc + "/data/lidar_" + std::to_string(systime) + ".pcd", *lidar_cloud);
        sys_pause = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_lidar_static_calibration");
    pkg_loc = ros::package::getPath("radar_lidar_static_calibration");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string lidar_topic, radar_topic;
    int sync_queue_size;

    private_nh.param<string>("lidar_topic", lidar_topic, "/lidar_PointCloud2");
    private_nh.param<string>("radar_topic", radar_topic, "/radar_PointCloud2");
    private_nh.param<int>("sync_queue_size", sync_queue_size, 500);

    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, lidar_topic, 10);
    message_filters::Subscriber<ecal_to_ros::RadarDetectionImage> radar_sub(nh, radar_topic, 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, ecal_to_ros::RadarDetectionImage> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(sync_queue_size), lidar_sub, radar_sub);
    ROS_INFO_STREAM("lidar topic required. Lidar topic: " << lidar_topic);
    ROS_INFO_STREAM("radar topic required. Radar topic: " << radar_topic);
    sync.registerCallback(boost::bind(&calibration_callback, _1, _2));

    std::thread stop_to_calibrate(stop2calibration);
    stop_to_calibrate.detach();

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
