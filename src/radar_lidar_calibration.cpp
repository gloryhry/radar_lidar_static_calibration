/*
 *           佛曰:  
 *                   写字楼里写字间，写字间里程序员；  
 *                   程序人员写程序，又拿程序换酒钱。  
 *                   酒醒只在网上坐，酒醉还来网下眠；  
 *                   酒醉酒醒日复日，网上网下年复年。  
 *                   但愿老死电脑间，不愿鞠躬老板前；  
 *                   奔驰宝马贵者趣，公交自行程序员。  
 *                   别人笑我忒疯癫，我笑自己命太贱；  
 *                   不见满街漂亮妹，哪个归得程序员？
 * 
 * @Author: Glory Huang
 * @Date: 2020-12-30 22:12:50
 * @LastEditors: Glory Huang
 * @LastEditTime: 2021-01-04 16:18:40
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>

#include <vector>
#include <string>
#include <dirent.h>

#include <radar_lidar_calibration.h>

using namespace std;

string pkg_loc;

// Mutex: //
boost::mutex cloud_mutex;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_lidar_calibration");
    pkg_loc = ros::package::getPath("radar_lidar_static_calibration");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double trans_x, trans_y, trans_theta;
    private_nh.param<double>("pre_trans_x", trans_x, 0.0);
    private_nh.param<double>("pre_trans_y", trans_y, 0.0);
    private_nh.param<double>("pre_trans_theta", trans_theta, 1.57);

    struct dirent *ptr;
    DIR *dir;
    string PATH = pkg_loc + "/data/";
    dir = opendir(PATH.c_str());
    vector<string> files;
    // cout << "文件列表: " << endl;
    while ((ptr = readdir(dir)) != NULL)
    {

        //跳过'.'和'..'两个目录
        if (ptr->d_name[0] == '.')
            continue;
        //cout << ptr->d_name << endl;
        files.push_back(ptr->d_name);
    }
    closedir(dir);
    // for (int i = 0; i < files.size(); ++i)
    // {
    //     cout << files[i] << endl;
    // }

    std::vector<string> radar_files, lidar_files;
    for (auto temp_file : files)
    {
        string::size_type lidar_pos = temp_file.find("lidar_");
        string::size_type radar_pos = temp_file.find("radar_");
        if (lidar_pos != temp_file.npos && radar_pos == temp_file.npos)
        {
            lidar_files.push_back(temp_file);
        }
        else if (lidar_pos == temp_file.npos && radar_pos != temp_file.npos)
        {
            radar_files.push_back(temp_file);
        }
    }

    vector<CalibrationPair> all_pairs;

    for (auto temp_radar_file : radar_files)
    {
        string file_date = temp_radar_file;
        file_date = file_date.erase(0, 6);
        for (auto temp_lidar_file : lidar_files)
        {
            string::size_type lidar_pos = temp_lidar_file.find(file_date);
            if (lidar_pos != temp_lidar_file.npos)
            {
                std::cout << "file pair:" << temp_radar_file << "|" << temp_lidar_file << std::endl;
                CalibrationPair temp_pair(temp_radar_file, temp_lidar_file, PATH);
                temp_pair.set_pretrans_param(trans_x, trans_y, trans_theta);
                all_pairs.push_back(temp_pair);
            }
        }
    }
    std::cout << "ALL Radar and Lidar Pairs: " << all_pairs.size() << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer(radar-lidar)"));
    viewer->addCoordinateSystem(3.0);
    viewer->setCameraPosition(0, 0, 50, 1, 0, 0, 1, 0, 0, 0);

    for (int i = 0; i < all_pairs.size();i++)
    {
        cloud_mutex.lock();
        all_pairs[i].init(viewer);
        all_pairs[i].quit(viewer);
        cloud_mutex.unlock();
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    std::cout << "Start to Calculate!" << std::endl;

    /***
    x,y: radar_points
    u,v: lidar_points

    ┌ x1 0      xn 0  ┐ T ┌ a1 ┐    ┌ u1  ┐   
    | y1 0      yn 0  |   | a1 |    | v1  |
    | 1  0      1  0  |   | a3 |    | ... |
    | 0  x1 ... 0  xn |   | a4 |  = | ... | 
    | 0  y1     0  yn |   | a5 |    | un  |
    └ 0  1      0  1  ┘   └ a6 ┘    └ vn  ┘

        ┌ x1 0      xn 0  ┐ T    ┌ u1  ┐
        | y1 0      yn 0  |      | v1  |
        | 1  0      1  0  |      | ... |    
    M = | 0  x1 ... 0  xn |  w = | ... |        
        | 0  y1     0  yn |      | un  |
        └ 0  1      0  1  ┘      └ vn  ┘
    
    pinv(M) = (M^T M)^(-1) M^T

    ┌ a1 ┐
    | a1 |
    | a3 |
    | a4 |  = pinv(M) w
    | a5 |
    └ a6 ┘

    pinv(M)为M的伪逆矩阵
    ***/
    int pair_number = all_pairs.size();
    std::cout << "number of pairs:" << pair_number << std::endl;
    std::vector<double> w_;
    std::vector<std::vector<double>> M_;

    

    std::cout << "begin" << std::endl;
    for (auto temp : all_pairs)
    {
        if(temp.valid == false)
        {
            continue;
        }
        pcl::PointXYZ temp_radar_point = temp.retrans_radar(temp.radar_point);
        std::cout <<"radar_point:"<<temp_radar_point.x <<"|"<<temp_radar_point.y << std::endl;
        std::cout << "lidar_point:" << temp.lidar_point.x << "|" << temp.lidar_point.y << std::endl;

        std::vector<double> temp_M(6,0);
        temp_M[0] = temp_radar_point.x;
        temp_M[1] = temp_radar_point.y;
        temp_M[2] = 1;
        M_.push_back(temp_M);
        temp_M = std::vector<double>(6, 0);
        temp_M[3] = temp_radar_point.x;
        temp_M[4] = temp_radar_point.y;
        temp_M[5] = 1;
        M_.push_back(temp_M);

        w_.push_back(temp.lidar_point.x);
        w_.push_back(temp.lidar_point.y);
    }

    std::cout << "M: "<<std::endl;
    for (int i = 0; i < M_.size();i++)
    {
        for (int j = 0; j < 6;j++)
        {
            std::cout << M_[i][j] << "    ";
        }
        std::cout << std::endl;
    }
    std::cout << "w: " << std::endl;
    for (int i = 0; i < w_.size(); i++)
    {
        std::cout << w_[i] << "    ";
    }
    std::cout << std::endl;

    Eigen::VectorXd param(6);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M(M_.size(), 6);
    Eigen::VectorXd w(w_.size());

    for (int i = 0; i < M_.size(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            M(i, j) = M_[i][j];
        }
    }
    for (int i = 0; i < w_.size(); i++)
    {
        w(i) = w_[i];
    }
    std::cout << " M :" << M << std::endl;
    std::cout << " w :" << w << std::endl;

    // param = pseudoInverse(M) * w;
    param = (M.transpose() * M).inverse() * M.transpose() * w;

    std::cout <<"param :"<<std::endl<< param << std::endl;

    return 0;
}