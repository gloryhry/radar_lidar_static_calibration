#pragma once
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class RadarCalibration
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud{new pcl::PointCloud<pcl::PointXYZ>()};

    int height; // point.y
    int width;  // point.x
    double resolution = 0.05;
    
    cv::Mat lidar_mat;
    cv::Mat radar_mat;

    cv::Mat lidar_img;
    cv::Mat radar_img;

public:
    RadarCalibration(pcl::PointCloud<pcl::PointXYZ>::Ptr &lidar_cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &radar_cloud);
    ~RadarCalibration(){};
    void showLidar();
    void showRadar();
};