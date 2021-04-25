/*
 *  ┌───┐   ┌───┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌───┬───┬───┐
 *  │Esc│   │ F1│ F2│ F3│ F4│ │ F5│ F6│ F7│ F8│ │ F9│F10│F11│F12│ │P/S│S L│P/B│  ┌┐    ┌┐    ┌┐
 *  └───┘   └───┴───┴───┴───┘ └───┴───┴───┴───┘ └───┴───┴───┴───┘ └───┴───┴───┘  └┘    └┘    └┘
 *  ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───────┐ ┌───┬───┬───┐ ┌───┬───┬───┬───┐
 *  │~ `│! 1│@ 2│# 3│$ 4│% 5│^ 6│& 7│* 8│( 9│) 0│_ -│+ =│ BacSp │ │Ins│Hom│PUp│ │N L│ / │ * │ - │
 *  ├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─────┤ ├───┼───┼───┤ ├───┼───┼───┼───┤
 *  │ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{ [│} ]│ | \ │ │Del│End│PDn│ │ 7 │ 8 │ 9 │   │
 *  ├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤ └───┴───┴───┘ ├───┼───┼───┤ + │
 *  │ Caps │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  │               │ 4 │ 5 │ 6 │   │
 *  ├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────────┤     ┌───┐     ├───┼───┼───┼───┤
 *  │ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│  Shift   │     │ ↑ │     │ 1 │ 2 │ 3 │   │
 *  ├─────┬──┴─┬─┴──┬┴───┴───┴───┴───┴───┴──┬┴───┼───┴┬────┬────┤ ┌───┼───┼───┐ ├───┴───┼───┤ E││
 *  │ Ctrl│    │Alt │         Space         │ Alt│    │    │Ctrl│ │ ← │ ↓ │ → │ │   0   │ . │←─┘│
 *  └─────┴────┴────┴───────────────────────┴────┴────┴────┴────┘ └───┴───┴───┘ └───────┴───┴───┘
 * 
 * @Author: Glory Huang
 * @Date: 2020-09-05 15:19:55
 * @LastEditors: Glory Huang
 * @LastEditTime: 2020-12-30 21:08:41
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <calibration.h>

RadarCalibration::RadarCalibration(pcl::PointCloud<pcl::PointXYZ>::Ptr &lidar_msg,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &radar_msg)
{
    *lidar_cloud = *lidar_msg;
    *radar_cloud = *radar_msg;

    resolution = 0.05;
    height = 60 / resolution;
    width = 60 / resolution;

    lidar_mat = cv::Mat::zeros(height, width, CV_8UC1);
    radar_mat = cv::Mat::zeros(height, width, CV_8UC1);
    lidar_img = cv::Mat::zeros(height, width, CV_8UC1);
    radar_img = cv::Mat::zeros(height, width, CV_8UC1);

    for (auto pt : lidar_cloud->points)
    {
        int hei, wid;
        wid = int(pt.x / resolution);
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
        hei = int(-pt.y / resolution) + height ;
        if (wid >= 0 && wid < width && hei >= 0 && hei < height)
        {
            radar_mat.at<uchar>(hei, wid) = 255;
        }
    }

}

void RadarCalibration::showLidar()
{
    cv::imshow("Lidar img", lidar_mat);
    cv::waitKey(1);
}

void RadarCalibration::showRadar()
{
    cv::imshow("Radar img", radar_mat);
    cv::waitKey(1);
}