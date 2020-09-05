#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <fstream>
#include <iostream>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace message_filters;
using namespace std;

string pkg_loc;

// Eigen::Matrix4f transformation;

void calibration_callback(const sensor_msgs::PointCloud2ConstPtr lidar_msg, const sensor_msgs::PointCloud2ConstPtr radar_msg)
{
    
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
    private_nh.param<string>("sync_queue_size", sync_queue_size, 50);

    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, lidar_topic, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub(nh, radar_topic, 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(sync_queue_size), lidar_sub, radar_sub);
    ROS_INFO_STREAM("lidar topic required. Lidar topic: " << lidar_topic);
    ROS_INFO_STREAM("radar topic required. Radar topic: " << radar_topic);
    sync.registerCallback(boost::bind(&calibration_callback, _1, _2));
    ros::spin();
    return 0;
}
