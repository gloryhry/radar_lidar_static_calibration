/*
 * _______________#########_______________________ 
 * ______________############_____________________ 
 * ______________#############____________________ 
 * _____________##__###########___________________ 
 * ____________###__######_#####__________________ 
 * ____________###_#######___####_________________ 
 * ___________###__##########_####________________ 
 * __________####__###########_####_______________ 
 * ________#####___###########__#####_____________ 
 * _______######___###_########___#####___________ 
 * _______#####___###___########___######_________ 
 * ______######___###__###########___######_______ 
 * _____######___####_##############__######______ 
 * ____#######__#####################_#######_____ 
 * ____#######__##############################____ 
 * ___#######__######_#################_#######___ 
 * ___#######__######_######_#########___######___ 
 * ___#######____##__######___######_____######___ 
 * ___#######________######____#####_____#####____ 
 * ____######________#####_____#####_____####_____ 
 * _____#####________####______#####_____###______ 
 * ______#####______;###________###______#________ 
 * ________##_______####________####______________ 
 * 
 * @Author: Glory Huang
 * @Date: 2020-12-30 22:16:41
 * @LastEditors: Glory Huang
 * @LastEditTime: 2021-01-02 11:43:43
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/geometry.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::vector<PointT> calibrate_points;

struct callback_args
{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

template <typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD<_Matrix_Type_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

void pp_callback(const pcl::visualization::PointPickingEvent &event, void *args)
{
    struct callback_args *data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
    calibrate_points.push_back(current_point);
}

class CalibrationPair
{
private:
    string radar_path, lidar_path;
    double x, y, theta;
    Eigen::Matrix3d trans_marix{Eigen::Matrix3d::Identity()};

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointXYZ lidar_point, radar_point;
    bool valid;

public:
    CalibrationPair(string radar_file, string lidar_file, string PATH);
    ~CalibrationPair();
    void init(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);
    void quit(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);
    void set_pretrans_param(double x, double y, double theta);
    pcl::PointXYZ trans_radar(pcl::PointXYZ &raw_point);
    pcl::PointXYZ retrans_radar(pcl::PointXYZ &raw_point);
};

void CalibrationPair::set_pretrans_param(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
    trans_marix << cos(theta), sin(theta), x,
        -sin(theta), cos(theta), y,
        0, 0, 1;
}

pcl::PointXYZ CalibrationPair::trans_radar(pcl::PointXYZ &raw_point)
{

    Eigen::Vector3d raw_pos(raw_point.x, raw_point.y, 0);
    Eigen::Vector3d output_pos;
    output_pos = trans_marix * raw_pos;
    pcl::PointXYZ output_point(output_pos(0), output_pos(1), 0);
    return output_point;
}

pcl::PointXYZ CalibrationPair::retrans_radar(pcl::PointXYZ &raw_point)
{
    Eigen::Vector3d raw_pos(raw_point.x, raw_point.y, 0);
    Eigen::Vector3d output_pos;
    output_pos = trans_marix.inverse() * raw_pos;
    pcl::PointXYZ output_point(output_pos(0), output_pos(1), 0);
    return output_point;
}

CalibrationPair::CalibrationPair(string radar_file, string lidar_file, string PATH)
{
    radar_path = PATH + radar_file;
    lidar_path = PATH + lidar_file;

    if (pcl::io::loadPCDFile(radar_path, *radar_cloud))
    {
        std::cerr << "ERROR: Cannot open file " << radar_path << "! Aborting..." << std::endl;
        return;
    }
    if (pcl::io::loadPCDFile(lidar_path, *lidar_cloud))
    {
        std::cerr << "ERROR: Cannot open file " << radar_path << "! Aborting..." << std::endl;
        return;
    }

    std::cout << "Radar Points: " << radar_cloud->points.size() << std::endl;
    std::cout << "Lidar Points: " << lidar_cloud->points.size() << std::endl;
}

CalibrationPair::~CalibrationPair()
{
}

void CalibrationPair::init(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
{
    // 先将radar点云转一下，方便手动对准。
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_radar_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto pt : radar_cloud->points)
    {
        trans_radar_cloud->points.push_back(trans_radar(pt));
    }
    calibrate_points.clear();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(trans_radar_cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(trans_radar_cloud, green_color, "radar");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "radar");
    viewer->addPointCloud<pcl::PointXYZ>(lidar_cloud, "lidar");

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pp_callback, (void *)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
    // Spin until 'Q' is pressed:
    viewer->spin();
    std::cout << "done." << std::endl;
}

void CalibrationPair::quit(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
{
    viewer->resetCameraViewpoint();
    viewer->resetCamera();
    viewer->setCameraPosition(0, 0, 50, 1, 0, 0, 1, 0, 0, 0);
    viewer->removeAllPointClouds();
    pcl::PointXYZ temp_point(0, 0, 0);
    std::vector<pcl::PointXYZ> cali_points;
    for (auto temp_pt : calibrate_points)
    {
        pcl::PointXYZ temp_pt_(temp_pt.x, temp_pt.y, temp_pt.z);
        if (pow(temp_pt_.x - temp_point.x, 2) + pow(temp_pt_.y - temp_point.y, 2) != 0)
        {
            temp_point = temp_pt_;
            cali_points.push_back(temp_point);
        }
    }
    std::cout << "calibrate points number:" << cali_points.size() << std::endl;

    if (cali_points.size() == 2)
    {
        radar_point = cali_points[0];
        lidar_point = cali_points[1];
        valid = true;
        std::cout << "find a pair of calibration points" << std::endl;
    }
    else
    {
        valid = false;
    }
}
