#include "data_manager/param.h"
#include "data_manager/base.h"
#include "fuse/fuse.h"
#include <openrm/cudatools.h>
#include <unistd.h>




cv::Mat PointCloud2Depth(rm::Radar* radar, rm::Camera* camera){
    cv::Mat close_image = cv::Mat(camera->height, camera->width, CV_8UC3, camera->image_buffer);
    cv::Mat radar_image = cv::Mat(4, radar->num_point, CV_32FC1, radar->point_cloud_buffer);

    // 1. 为了方便，先将点云数据转换Matrix格式
    Eigen::Matrix<double, 4, Eigen::Dynamic> radar_matrix = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, radar->num_point);
    for(int i = 0; i < radar->num_point; i++){
        radar_matrix(0, i) = radar_image.at<float>(0, i);
        radar_matrix(1, i) = radar_image.at<float>(1, i);
        radar_matrix(2, i) = radar_image.at<float>(2, i);
        radar_matrix(3, i) = 1;
    }

    // 2. 将点云经过close_camera的外参转换到close_camera坐标系下
    Eigen::Matrix<double, 4, Eigen::Dynamic> radar_matrix_in_close = camera->Trans_pnp2head * radar_matrix;

    // 3. 将点云投影到close_camera的图像坐标系下
    Eigen::Matrix<double, 3, 3> intrinsic_matrix;
    rm::tf_Mat3d(camera->intrinsic_matrix, intrinsic_matrix);
    Eigen::Matrix<double, 3, Eigen::Dynamic> radar_matrix_in_close_3d = intrinsic_matrix * radar_matrix_in_close.topRows(3);

    // 4. 将点云投影到图像坐标系下，得到深度图
    cv::Mat depth_image = cv::Mat::zeros(camera->height, camera->width, CV_32FC1);
    for(int i = 0; i < radar->num_point; i++){
        int x = radar_matrix_in_close_3d(0, i) / radar_matrix_in_close_3d(2, i);
        int y = radar_matrix_in_close_3d(1, i) / radar_matrix_in_close_3d(2, i);
        if(x >= 0 && x < camera->width && y >= 0 && y < camera->height){
            float depth = radar_matrix_in_close_3d(2, i);
            depth_image.at<float>(y, x) = depth;
        }
    }
    return depth_image;
}


