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
#include <structure/radar.hpp>

#include "data_manager/param.h"

#include <radar_msgs/points.h>
#include <radar_msgs/point.h>


void *close_buffer, *far_buffer, *radar_buffer, *extrinsic_buffer;

int height_close, width_close, height_far, width_far, num_ladar;

auto param = Param::get_instance();

// 获取相机参数矩阵json
nlohmann::json camlens;


//将点云数据转换为矩阵
cv::Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    cv::Mat output_matrix = cv::Mat::zeros(4, (int)cloud->size(), CV_32F);
    for (int i = 0; i < output_matrix.cols; i++) 
    {
        for (int j = 0; j < 3; j++) 
        {
            output_matrix.at<float>(j, i) = cloud->points[i].data[j];
        }
        output_matrix.at<float>(3, i) = cloud->points[i].intensity;
    }
    return output_matrix;
}


cv::Mat undistort_image_close;


void far_image_callback(const sensor_msgs::Image::ConstPtr& msg){
    printf("far image receive\n");
    cv::Mat far_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    // 对图像去畸变
    // cv::Mat intrinsic_matrix;                               
    // cv::Mat distortion_coeffs;    
    // Param::from_json(camlens["Far"]["Intrinsic"], intrinsic_matrix);
    // Param::from_json(camlens["Far"]["Distortion"], distortion_coeffs);
    // cv::undistort(far_frame, undistort_image_close, intrinsic_matrix, distortion_coeffs);
    memcpy(far_buffer, far_frame.data, height_far * width_far * 3);
}

cv::Mat undistort_image_far;

void close_image_callback(const sensor_msgs::Image::ConstPtr& msg){
    printf("close image receive\n");
    cv::Mat close_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    // 对图像去畸变
    // cv::Mat intrinsic_matrix;
    // cv::Mat distortion_coeffs;
    // Param::from_json(camlens["Close"]["Intrinsic"], intrinsic_matrix);
    // Param::from_json(camlens["Close"]["Distortion"], distortion_coeffs);
    // cv::undistort(close_frame, undistort_image_far, intrinsic_matrix, distortion_coeffs);
    memcpy(close_buffer, close_frame.data, height_close * width_close * 3);
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &temp_cloud)
{
    printf("lidar receive\n");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*temp_cloud, *cloud);
    std::shared_ptr<cv::Mat> cloud_matrix = std::make_shared<cv::Mat>(Cloud2Mat(cloud));
    memcpy(radar_buffer, cloud_matrix->data, num_ladar * 4 * 4);
}


void close_extrinsic_callback(const radar_msgs::points &points){
    printf("close_extrinsic receive\n");
    rm::RadarData *extrinsic = (rm::RadarData *)extrinsic_buffer;
    for(int i = 0; i < 4; i++){
        extrinsic->image_zed_calib[0][i].x = points.data[i].x;
        extrinsic->image_zed_calib[0][i].y = points.data[i].y;
    }
}


void far_extrinsic_callback(const radar_msgs::points &points){
    printf("far_extrinsic receive\n");
    rm::RadarData *extrinsic = (rm::RadarData *)extrinsic_buffer;
    for(int i = 0; i < 4; i++){
        extrinsic->image_zed_calib[1][i].x = points.data[i].x;
        extrinsic->image_zed_calib[1][i].y = points.data[i].y;
    }
    extrinsic->is_valid = true;
}



int main(int agrc, char *argv[]){

    setlocale(LC_ALL, "");
    ros::init(agrc, argv, "driver_bridge_node");
    ros::NodeHandle nh;
    auto param = Param::get_instance();

    // 睡眠1s  FIXME: 不加会出问题，暂时不清楚为什么????
    sleep(1);

    std::string camlen_path = (*param)["Camera"]["CamLensDir"];
    try {
        std::ifstream camlens_json(camlen_path);
        camlens_json >> camlens;
        camlens_json.close();
    } catch (std::exception& e) {
        std::string err_str = "Failed to load CamLens json: " + std::string(e.what());
        std::cout << err_str << std::endl;
        return false;
    }
    
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
    radar_buffer = (void *)rm::__shm_alloc__(rm::__gen_hash_key__(radar_buffer_name), num_ladar * 4 * 4);
    memset(radar_buffer, 0, num_ladar * 4);
    ros::Subscriber lidar_sub = nh.subscribe("/livox/lidar", 1, &lidar_callback);


    // 外参标定结果
    std::string extrinsic_name = (*param)["Extrinsic"]["Name"];
    extrinsic_buffer = (void *)rm::__shm_alloc__(rm::__gen_hash_key__(extrinsic_name), sizeof(rm::RadarData));
    memset(extrinsic_buffer, 0, sizeof(rm::RadarData));
    ros::Subscriber close_extrinsic_sub = nh.subscribe("/sensor_close/calibration", 1, &close_extrinsic_callback);
    ros::Subscriber far_extrinsic_sub = nh.subscribe("/sensor_far/calibration", 1, &far_extrinsic_callback);



    ros::spin();

    
    
    return 0;
}