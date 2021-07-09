#include <mutex>
#include <thread>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "point_os.h"
std::queue<sensor_msgs::ImageConstPtr> rgb_buf;
std::queue<sensor_msgs::ImageConstPtr> thermal_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> laser_buf;
std::mutex m_buf;
std::string rgbDir, thermalDir, laserDir;
double max_intensity, min_intensity;

void thermal_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    // ROS_INFO("thermal received");
    m_buf.lock();
    thermal_buf.push(image_msg);
    m_buf.unlock();
}
void rgb_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    // ROS_INFO("rgb received");
    m_buf.lock();
    rgb_buf.push(image_msg);
    m_buf.unlock();
}
void laser_callback(const sensor_msgs::PointCloud2ConstPtr &laser_msg)
{
    // ROS_INFO("lidar received");
    m_buf.lock();
    laser_buf.push(laser_msg);
    m_buf.unlock();
}
void process()
{
    unsigned int idx=0;
    while (true)
    {
        sensor_msgs::ImageConstPtr rgb_msg = NULL;
        sensor_msgs::ImageConstPtr thermal_msg = NULL;
        sensor_msgs::PointCloud2ConstPtr laser_msg=NULL;
        if (!rgb_buf.empty() && !thermal_buf.empty() && !laser_buf.empty())
        {
            if (laser_buf.front()->header.stamp.toSec() > thermal_buf.front()->header.stamp.toSec())
            {
                thermal_buf.pop();
                // printf("throw pose at beginning\n");
            }
            else if (laser_buf.front()->header.stamp.toSec() > rgb_buf.front()->header.stamp.toSec())
            {
                laser_buf.pop();
            }
            else
            {
                rgb_msg = rgb_buf.front();
                rgb_buf.pop();
                thermal_msg = thermal_buf.front();
                thermal_buf.pop();
                laser_msg = laser_buf.front();
                laser_buf.pop();
            }
        }

        if(rgb_msg != NULL && thermal_msg!= NULL && laser_msg!= NULL)
        {
            // ROS_INFO("rgb time %lf, thermal time %lf", rgb_msg->header.stamp.toSec(), thermal_msg->header.stamp.toSec());
            cv_bridge::CvImageConstPtr ptr_thermal;
            ptr_thermal = cv_bridge::toCvCopy(thermal_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat IR16 = ptr_thermal->image;
            double minI, maxI;
            cv::minMaxIdx(IR16, &minI, &maxI);

            // auto pixel=IR16.at<ushort>(10,10); //21705
            // std::cout<<"IR16 type: "<<IR16.type()<<"[10,10]="<<pixel<<std::endl;
            // cv::Mat IR8, temp;
            // cv::threshold(IR16, IR8, max_intensity, max_intensity, cv::THRESH_TRUNC);
            // cv::threshold(-IR8, temp, -min_intensity, -min_intensity, cv::THRESH_TRUNC);
            // IR8=(-temp);
            cv::Mat IR8(IR16.rows, IR16.cols, 0);
            for (int i = 0; i < IR8.rows * IR8.cols; i++)
            {
                auto pixel = ((uint16_t *)ptr_thermal->image.data)[i];
                pixel = pixel > maxI ? maxI : pixel;
                pixel = pixel < minI ? minI : pixel;
                IR8.data[i] = uchar((pixel - minI) * 255.0 / (maxI - minI));
            }
            std::string thermal_file = thermalDir + std::to_string(idx) + ".jpg";
            cv::imwrite(thermal_file, IR8);

            cv_bridge::CvImageConstPtr ptr_rgb;
            ptr_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat rgb = ptr_rgb->image;
            std::string rgb_file = rgbDir + std::to_string(idx) + ".jpg";
            cv::imwrite(rgb_file, rgb);

            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(new pcl::PointCloud<point_os::PointcloudXYZITR>());
            pcl::fromROSMsg(*laser_msg, *pointCloud);
            std::string cloud_file = laserDir + std::to_string(idx) + ".ply";
            pcl::io::savePLYFileASCII(cloud_file, *pointCloud);
            idx++;
        }  
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "thermal-rgb");
    ros::NodeHandle n("~");

    std::string rgb_topic, thermal_topic, laser_topic;
    n.param("rgbDir",   rgbDir,   std::string(""));
    n.param("thermalDir",   thermalDir,   std::string(""));
    n.param("laserDir",   laserDir,   std::string(""));
    n.param("rgb_topic",   rgb_topic,   std::string(""));
    n.param("thermal_topic",   thermal_topic,   std::string(""));
    n.param("laser_topic",   laser_topic,   std::string(""));
    n.getParam("min_intensity", min_intensity);
    n.getParam("max_intensity", max_intensity);
    
    ros::Subscriber sub_thermal_image = n.subscribe(thermal_topic, 1000, thermal_callback);
    ros::Subscriber sub_rgb_image = n.subscribe(rgb_topic, 1000, rgb_callback);
    ros::Subscriber sub_laser_image = n.subscribe<sensor_msgs::PointCloud2>(laser_topic, 1000, laser_callback);
    
    ros::Rate r(100);
    std::thread joint_process;
    joint_process = std::thread(process);
    ros::spin();
    
    return 0;
}
