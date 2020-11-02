#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

class RadarMatch
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_time_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr this_time_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    ros::Time last_stamp, this_stamp; // 上帧和这帧的时间戳
    // Eigen::Translation3d odometry_translation, last_odometry_translation, radar_translation, radar2odom_translation;
    // Eigen::AngleAxisd odometry_rotation, last_odometry_rotation, radar_rotation, radar2odom_rotation;

    Eigen::Matrix4f last_trans;
    Eigen::Matrix4f now_trans;
    double Odom_angle, Odom_offset_x, Odom_offset_y;
    double Radar_angel, Radar_offset_x, Radar_offset_y;
    double Odom2radar_angle, Odom2radar_offset_x, Odom2radar_offset_y;
    double score;

public:
    bool inited = false;

public:
    RadarMatch(){};
    ~RadarMatch(){};
    void init(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, ros::Time stamp, double odom_x, double odom_y, double odom_angle);
    void update(pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud,
                ros::Time stamp, double odom_x, double odom_y, double odom_angle);
    void getTrans(double &IMU2Radar_angle, double &IMU2Radar_x, double &IMU2Radar_y);
    void showCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int &v1, int &v2);
};