#include <ros/package.h>
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <radar_match/match.h>

using namespace message_filters;
using namespace std;

string pkg_loc;
string file_path;
Eigen::Translation3d radar2odom_translation;
Eigen::AngleAxisd radar2odom_rotation;

RadarMatch radar_match;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
int v1(0);
int v2(0);

int count_number =0;

void RadarMatchCallback(const sensor_msgs::PointCloud2ConstPtr radar_msg,
                        const nav_msgs::OdometryConstPtr odometry_msg)
{
    count_number = count_number + 1;
    if(count_number % 3  != 0)
    {
        return;
    }
    ros::Time time_now = radar_msg->header.stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*radar_msg, *radar_cloud);
    for (int index = 0; index < radar_cloud->size();index++)
    {
        radar_cloud->points[index].x = -radar_cloud->points[index].x;
    }

    Eigen::Quaterniond odom_Quaternion(odometry_msg->pose.pose.orientation.w,odometry_msg->pose.pose.orientation.x,odometry_msg->pose.pose.orientation.y,odometry_msg->pose.pose.orientation.z);
    Eigen::Vector3d eulerAngle = odom_Quaternion.matrix().eulerAngles(2, 1, 0);

    if (!radar_match.inited)
    {
        radar_match.init(radar_cloud, time_now, odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, eulerAngle.x());
        return;
    }
    radar_match.update(radar_cloud, time_now, odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, eulerAngle.x());
    double IMU2Radar_angle, IMU2Radar_x, IMU2Radar_y;
    radar_match.getTrans(IMU2Radar_angle, IMU2Radar_x, IMU2Radar_y);

    radar_match.showCloud(viewer, v1, v2);
    viewer->spinOnce();
    // TODO: 更新平移和旋转，使结果收敛。

    if(radar_match.Odom_angle>M_PI/180*2)
    {
        fstream out(file_path, ios::app);
        cout << "file_path: " << endl;
        cout << file_path << endl;
        out << IMU2Radar_angle << "," << IMU2Radar_x << "," << IMU2Radar_y << std::endl;
        out.close();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_match");
    pkg_loc = ros::package::getPath("radar_lidar_static_calibration");
    file_path = pkg_loc + "/data/calibration.csv";

    fstream out(file_path, ios::out | ios::trunc);
    out.close();

    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 40, 160.0, 0,40,0, 0,0,0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Unregistered Cloud", 10, 10, "v1 text", v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.2, 0.2, 0.2, v2);
    viewer->addText("Registered Cloude", 10, 10, "v2 text", v2);

    // viewer->addCoordinateSystem(); // 添加坐标轴
    // viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string radar_topic;
    string Odometry_topic;
    int sync_queue_size;

    private_nh.param<string>("radar_topic", radar_topic, "/radar");
    private_nh.param<string>("Odometry_topic", Odometry_topic, "/localization/ackermanekf");
    private_nh.param<int>("sync_queue_size", sync_queue_size, 50);

    message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub(nh, radar_topic, 10);
    message_filters::Subscriber<nav_msgs::Odometry> Odometry_sub(nh, Odometry_topic, 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(sync_queue_size), radar_sub, Odometry_sub);
    ROS_INFO_STREAM("Radar topic required. Radar topic: " << radar_topic);
    ROS_INFO_STREAM("Odometry topic required. Odometry topic: " << Odometry_topic);
    sync.registerCallback(boost::bind(&RadarMatchCallback, _1, _2));

    ros::spin();
    return 0;
}