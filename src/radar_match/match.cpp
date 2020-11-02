#include <radar_match/match.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

void RadarMatch::init(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                          ros::Time stamp,
                          double odom_x,double odom_y,double odom_angle)
{
    *this_time_cloud = *input_cloud;
    this_stamp = stamp;
    Odom_angle = 0;
    Odom_offset_x = 0;
    Odom_offset_y = 0;

    last_trans << cos(odom_angle), -sin(odom_angle), 0, odom_x,
                  sin(odom_angle), cos(odom_angle), 0, odom_y,
                  0, 0, 1, 0,
                  0, 0, 0, 1;
    inited = true;
}

void RadarMatch::update(pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud,
                        ros::Time stamp, double odom_x, double odom_y, double odom_angle)
{
    last_time_cloud->clear();
    *last_time_cloud = *this_time_cloud;
    this_time_cloud->clear();
    *this_time_cloud = *new_cloud;
    last_stamp = this_stamp;
    this_stamp = stamp;

    // 求IMU的位姿变换
    now_trans << cos(odom_angle), -sin(odom_angle), 0, odom_x,
                 sin(odom_angle), cos(odom_angle), 0, odom_y,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    Eigen::Matrix4f Odom_trans;
    Odom_trans = last_trans.inverse() * now_trans;

    // 更新IMU的位姿
    // last_odom_angle = odom_angle;
    // last_odom_x = odom_x;
    // last_odom_y = odom_y;
    Odom_angle = acos(Odom_trans(0, 0));
    Odom_offset_x = Odom_trans(0, 3);
    Odom_offset_y = Odom_trans(1, 3);
    last_trans = now_trans;

    // Eigen::Matrix4f Odom_trans;
    // Odom_trans << cos(Odom_angle), -sin(Odom_angle), 0, Odom_offset_x,
    //               sin(Odom_angle), cos(Odom_angle), 0, Odom_offset_y,
    //               0, 0, 1, 0,
    //               0, 0, 0, 1;

    /******************** PCL匹配方法 ***********************/
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the input source and target
    // icp.setInputCloud(last_time_cloud);
    icp.setInputSource(last_time_cloud);
    icp.setInputTarget(this_time_cloud);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(20);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(100);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(0.001);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(0.01);

    // Perform the alignment
    icp.align(*registered_cloud);
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    score = icp.getFitnessScore();
    

    /******************** NDT匹配方法 ***********************/
    // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // ndt.setInputCloud(last_time_cloud);
    // ndt.setInputTarget(this_time_cloud);
    // ndt.setStepSize(1);
    // ndt.setMaximumIterations(100);
    // ndt.setResolution(10);
    // ndt.setTransformationEpsilon(0.01);
    // ndt.align(*registered_cloud);
    // Eigen::Matrix4f transformation = ndt.getFinalTransformation();
    // score = ndt.getFitnessScore();

    if(transformation(0,0)>=1)
    {
        Radar_angel = 0;
    }
    else if(transformation(0,0)<= -1)
    {
        Radar_angel = M_PI;
    }
    Radar_angel = - acos(transformation(0, 0));
    Radar_offset_x = -transformation(0, 3);
    Radar_offset_y = -transformation(1, 3);

    Eigen::Matrix4f Radar_trans;
    // Radar_trans << cos(Radar_angel), -sin(Radar_angel), 0, Radar_offset_x,
    //               sin(Radar_angel), cos(Radar_angel), 0, Radar_offset_y,
    //               0, 0, 1, 0,
    //               0, 0, 0, 1;
    Radar_trans = transformation.inverse();

    cout << "Radar transformation:" << endl;
    cout << Radar_trans << endl;
    cout << "odom transformation:" << endl;
    cout << Odom_trans << endl;

    double a1,a2;
    a1 = (Odom_trans(0, 0) - Radar_trans(0, 0)) / (Odom_trans(1, 0) + Radar_trans(0, 1));
    a2 = (Odom_trans(0, 1) + Radar_trans(1, 0)) / (Odom_trans(0, 0) - Radar_trans(0, 0));
    cout << "a1,a2: " << a1 << "|" << a2 << endl;

    Eigen::Matrix4f trans;
    // F(IMU)= F(I2R)*F(Radar)*F(I2R)^T
    trans = Odom_trans.inverse() * Radar_trans;
    std::cout << "Odom2Radar translation: " << std::endl;
    std::cout << trans << std::endl;

    Odom2radar_angle = acos(trans(0, 0));
    Odom2radar_offset_x = trans(0, 3);
    Odom2radar_offset_y = trans(1, 3);


}

void RadarMatch::getTrans(double &IMU2Radar_angle,double &IMU2Radar_x,double &IMU2Radar_y)
{
    cout << "Odom2Radar angle offset:" << endl;
    cout << Odom2radar_angle << "    " << Odom2radar_offset_x << "    " << Odom2radar_offset_y << endl;
    IMU2Radar_angle = Odom2radar_angle;
    IMU2Radar_x = Odom2radar_offset_x;
    IMU2Radar_y = Odom2radar_offset_y;
}

void RadarMatch::showCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,int &v1,int &v2)
{
    // std::cout << "--------------------" << std::endl;
    // std::cout << "this point size:" << this_time_cloud->size() << std::endl;
    // std::cout << "last point size:" << last_time_cloud->size() << std::endl;

    viewer->removeAllPointClouds(v1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(this_time_cloud, 255, 255, 255);
    viewer->addPointCloud(this_time_cloud, white, "cloud1",v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1",v1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow(last_time_cloud, 255, 255, 0);
    viewer->addPointCloud(last_time_cloud, yellow, "cloud2", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2", v1);

    viewer->removeAllPointClouds(v2);
    viewer->addPointCloud(this_time_cloud, white, "cloud4", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud4", v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(registered_cloud, 0, 255, 0);
    viewer->addPointCloud(registered_cloud, green, "cloud3", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud3", v2);

    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    // std::cout << "camera pos: " << camera.pos[0] << "|" << camera.pos[1] << "|" << camera.pos[2] << std::endl;
    // std::cout << "camera view: " << camera.view[0] << "|" << camera.view[1] << "|" << camera.view[2] << std::endl;
}