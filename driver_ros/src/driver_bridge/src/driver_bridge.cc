#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// pcl 
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>


int close_sub_count = 0;
int far_sub_count = 0;
int lidar_sub_count = 0;

pcl::PointCloud<pcl::PointXYZ> cloud;
cv::Mat close_frame;
cv::Mat far_frame;


void far_image_callback(const sensor_msgs::Image::ConstPtr& msg){
    printf("far image receive\n");
    far_sub_count++;
    far_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
}

void close_image_callback(const sensor_msgs::Image::ConstPtr& msg){
    printf("close image receive\n");
    close_sub_count++;
    close_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &temp_cloud)
{
    printf("lidar receive\n");
    lidar_sub_count++;
    pcl::fromROSMsg(*temp_cloud, cloud);
}


int main(int agrc, char *argv[]){

    setlocale(LC_ALL, "");
    ros::init(agrc, argv, "driver_bridge_node");
    ros::NodeHandle nh;

    //ros循环频率
    ros::Rate loop_rate(30);
    
    // 订阅image&radar话题
    ros::Subscriber close_image_sub = nh.subscribe<sensor_msgs::Image>("/rgb/image_raw",1,close_image_callback);
    ros::Subscriber far_image_sub = nh.subscribe<sensor_msgs::Image>("/hikrobot_camera/rgb",1,far_image_callback);
    ros::Subscriber lidar_sub = nh.subscribe("/livox/lidar", 1, &lidar_callback);
    
    //ros循环延时
    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}