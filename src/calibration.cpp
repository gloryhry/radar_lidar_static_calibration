#include <calibration.h>

RadarCalibration::RadarCalibration(pcl::PointCloud<pcl::PointXYZ>::Ptr &lidar_msg,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &radar_msg)
{
    *lidar_cloud = *lidar_msg;
    *radar_cloud = *radar_msg;

    resolution = 0.1;
    height = 80 / resolution;
    width = 80 / resolution;

    // std::cout << height << " | " << width << std::endl;
    lidar_mat = cv::Mat::zeros(height, width, CV_8UC1);
    radar_mat = cv::Mat::zeros(height, width, CV_8UC1);
    lidar_img = cv::Mat::zeros(height, width, CV_8UC1);
    radar_img = cv::Mat::zeros(height, width, CV_8UC1);

    for (auto pt : lidar_cloud->points)
    {
        int hei, wid;
        wid = int(pt.x / resolution) + width / 2;
        hei = int(-pt.y / resolution) + height / 2;
        if (wid >= 0 && wid < width && hei >= 0 && hei < height)
        {
            lidar_mat.at<uchar>(hei, wid) = 255;
        }
    }
    for (auto pt : radar_cloud->points)
    {
        int hei, wid;
        wid = int(-pt.x / resolution) + width / 2;
        hei = int(-pt.y / resolution) + height / 2;
        if (wid >= 0 && wid < width && hei >= 0 && hei < height)
        {
            radar_mat.at<uchar>(hei, wid) = 255;
        }
    }

    cv::Mat lidar_line_img = lidar_mat.clone();
    cv::Mat radar_line_img = radar_mat.clone();
    std::vector<cv::Vec4d> lidar_lines,radar_lines;
    cv::HoughLinesP(lidar_line_img, lidar_lines, 0.5/resolution, CV_PI / 30, 70, 2 / resolution, 2 / resolution);
    std::vector<cv::Vec4d> lidar_corners;
    for (size_t i = 0; i < lidar_lines.size(); i++)
    {
        double theta = atan2((lidar_lines[i][1] - lidar_lines[i][3]), (lidar_lines[i][0] - lidar_lines[i][2])); // 直线角度
        for (size_t j = i; j < lidar_lines.size(); j++)
        {
            double temp_theta = atan2((lidar_lines[j][1] - lidar_lines[j][3]), (lidar_lines[j][0] - lidar_lines[j][2]));
            if (temp_theta > theta - CV_PI / 180 * 85 && temp_theta < theta - CV_PI / 180 * 95 || temp_theta > theta - CV_PI / 180 * 85 && temp_theta < theta + CV_PI / 180 * 95)
            {
                lidar_corners.push_back(lidar_lines[i]);
                lidar_corners.push_back(lidar_lines[j]);
            }
        }
    }
    for (size_t i = 0; i < lidar_corners.size();i++)
    {
        cv::Vec4i point = lidar_corners[i];
        cv::line(lidar_mat, cv::Point(point[0], point[1]), cv::Point(point[2], point[3]), cv::Scalar(255), 1, CV_AA);
    }
    cv::HoughLinesP(radar_line_img, radar_lines, 10 / resolution, CV_PI / 30, 0, 0.5 / resolution, 15 / resolution);
    for (size_t i = 0; i < radar_lines.size(); i++)
    {
        cv::Vec4i point = radar_lines[i];
        cv::line(radar_mat, cv::Point(point[0], point[1]), cv::Point(point[2], point[3]), cv::Scalar(255), 1, CV_AA);
    }

    // std::vector<cv::Vec2d> lidar_lines, radar_lines;
    // cv::HoughLines(lidar_line_img, lidar_lines, 0.5 / resolution, CV_PI / 30, 150);
    // std::vector<cv::Vec2d> lidar_corners;
    // for (size_t i = 0; i < lidar_lines.size(); i++)
    // {
    //     double rho = lidar_lines[i][0];   // 圆半径R
    //     double theta = lidar_lines[i][1]; // 直线角度
    //     for (size_t j = i; j < lidar_lines.size(); j++)
    //     {
    //         double temp_rho = lidar_lines[j][0];
    //         double temp_theta = lidar_lines[j][1];
    //         if (temp_theta > theta - CV_PI / 180 * 85 && temp_theta < theta - CV_PI / 180 * 95 || temp_theta > theta - CV_PI / 180 * 85 && temp_theta < theta + CV_PI / 180 * 95)
    //         {
    //             lidar_corners.push_back(lidar_lines[i]);
    //             lidar_corners.push_back(lidar_lines[j]);
    //         }
    //     }
    // }
    // for (size_t i = 0; i < lidar_corners.size();i++)
    // {
    //     double rho = lidar_corners[i][0]; // 圆半径R
    //     double theta = lidar_corners[i][1]; // 直线角度
    //     cv::Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a * rho, y0 = b * rho;
    //     pt1.x = cvRound(x0 + 1000 * (-b));
    //     pt1.y = cvRound(y0 + 1000 * (a));
    //     pt2.x = cvRound(x0 - 1000 * (-b));
    //     pt2.y = cvRound(y0 - 1000 * (a));
    //     cv::line(lidar_mat, pt1, pt2, cv::Scalar(255), 1);
    // }
    // cv::HoughLines(radar_line_img, radar_lines, 5 / resolution, CV_PI / 30, 15);
    // for (size_t i = 0; i < radar_lines.size(); i++)
    // {
    //     double rho = radar_lines[i][0];   // 圆半径R
    //     double theta = radar_lines[i][1]; // 直线角度
    //     cv::Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a * rho, y0 = b * rho;
    //     pt1.x = cvRound(x0 + 1000 * (-b));
    //     pt1.y = cvRound(y0 + 1000 * (a));
    //     pt2.x = cvRound(x0 - 1000 * (-b));
    //     pt2.y = cvRound(y0 - 1000 * (a));
    //     cv::line(radar_mat, pt1, pt2, cv::Scalar(255), 1);
    // }
}

void RadarCalibration::showLidar()
{
    cv::Mat show_mat;
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::dilate(lidar_mat, show_mat, element);
    cv::resize(show_mat, show_mat, cv::Size(800, 800));
    cv::imshow("Lidar img", show_mat);
    cv::waitKey(1);
}

void RadarCalibration::showRadar()
{
    cv::Mat show_mat;
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::dilate(radar_mat, show_mat, element);
    cv::resize(show_mat, show_mat, cv::Size(800, 800));
    cv::imshow("Radar img", show_mat);
    cv::waitKey(1);
}