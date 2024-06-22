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

// openrm
#include <structure/shm.hpp>
#include <structure/swapbuffer.hpp>

#include "data_manager/param.h"


void *close_buffer, *far_buffer, *radar_buffer;

int height_close, width_close, height_far, width_far, num_ladar;


//将点云数据转换为矩阵
cv::Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    cv::Mat output_matrix = cv::Mat::zeros(4, (int)cloud->size(), CV_32F);
    for (int i = 0; i < output_matrix.cols; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            output_matrix.at<float>(j, i) = cloud->points[i].data[j];
        }
    }
    return output_matrix;
}

void far_image_callback(const sensor_msgs::Image::ConstPtr& msg){
    printf("far image receive\n");
    cv::Mat far_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    memcpy(far_buffer, far_frame.data, height_far * width_far * 3);
}


void close_image_callback(const sensor_msgs::Image::ConstPtr& msg){
    printf("close image receive\n");
    cv::Mat close_frame  = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    memcpy(close_buffer, close_frame.data, height_close * width_close * 3);
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &temp_cloud)
{
    printf("lidar receive\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*temp_cloud, *cloud);
    std::shared_ptr<cv::Mat> cloud_matrix = std::make_shared<cv::Mat>(Cloud2Mat(cloud));
    memcpy(radar_buffer, cloud_matrix->data, num_ladar * 4);
}


int main(int agrc, char *argv[]){

    setlocale(LC_ALL, "");
    ros::init(agrc, argv, "driver_bridge_node");
    ros::NodeHandle nh;
    auto param = Param::get_instance();

    // 睡眠1s  FIXME: 不加会出问题，暂时不清楚为什么????
    sleep(1);
    
    // close_buffer
    height_close = (*param)["Camera"]["Close"]["Height"];
    width_close = (*param)["Camera"]["Close"]["Width"];
    std::string close_buffer_name =  (*param)["Camera"]["Close"]["Name"];
    close_buffer = (void *)rm::__shm_alloc__(rm::__gen_hash_key__(close_buffer_name), height_close * width_close * 3);
    memset(close_buffer, 0, height_close * width_close * 3);
    ros::Subscriber close_image_sub = nh.subscribe<sensor_msgs::Image>("/rgb/image_raw",1,close_image_callback);
    

    // far_buffer
    height_far = (*param)["Camera"]["Far"]["Height"];
    width_far = (*param)["Camera"]["Far"]["Width"];
    std::string far_buffer_name =  (*param)["Camera"]["Far"]["Name"];
    far_buffer = (void *)rm::__shm_alloc__(rm::__gen_hash_key__(far_buffer_name), height_far * width_far * 3);
    memset(far_buffer, 0, height_far * width_far * 3);
    ros::Subscriber far_image_sub = nh.subscribe<sensor_msgs::Image>("/hikrobot_camera/rgb",1,far_image_callback);


    // radar
    num_ladar = (*param)["Radar"]["Num"];
    std::string radar_buffer_name = (*param)["Radar"]["Name"];
    radar_buffer = (void *)rm::__shm_alloc__(rm::__gen_hash_key__(radar_buffer_name), num_ladar * 4);
    memset(radar_buffer, 0, num_ladar * 4);
    ros::Subscriber lidar_sub = nh.subscribe("/livox/lidar", 1, &lidar_callback);

    ros::spin();

    
    
    return 0;
}