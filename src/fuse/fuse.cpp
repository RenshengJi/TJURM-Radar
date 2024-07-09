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


bool extrinsic_calib(){
    auto param = Param::get_instance();
    
    
    if(!Data::extrinsic->is_valid){
        return false;
    }
    Data::radar2place = Eigen::Matrix<double, 4, 4>::Identity();

    // TODO: 目前选择前场相机进行外参标定
    // 1. 获取世界坐标（从配置文件里读取）
    std::vector<cv::Point3d> zed_cali(4);
    zed_cali[0].x = (*param)["PlacePoint"]["1x"];
    zed_cali[0].y = (*param)["PlacePoint"]["1y"];
    zed_cali[0].z = (*param)["PlacePoint"]["1z"];
    zed_cali[1].x = (*param)["PlacePoint"]["2x"];
    zed_cali[1].y = (*param)["PlacePoint"]["2y"];
    zed_cali[1].z = (*param)["PlacePoint"]["2z"];
    zed_cali[2].x = (*param)["PlacePoint"]["3x"];
    zed_cali[2].y = (*param)["PlacePoint"]["3y"];
    zed_cali[2].z = (*param)["PlacePoint"]["3z"];
    zed_cali[3].x = (*param)["PlacePoint"]["4x"];
    zed_cali[3].y = (*param)["PlacePoint"]["4y"];
    zed_cali[3].z = (*param)["PlacePoint"]["4z"];
    

    // 2. 获取相机坐标
    std::vector<cv::Point2d> radar_cali(4);
    for(int i = 0; i < 4; i++){
        radar_cali[i].x = Data::extrinsic->image_zed_calib[i].x * Data::camera[0]->width;
        radar_cali[i].y = Data::extrinsic->image_zed_calib[i].y * Data::camera[0]->height;
    }

    // 3. 使用solvepnp求解相机与场地坐标系的外参
    cv::Mat rvec, tvec;
    cv::solvePnP(zed_cali, radar_cali, Data::camera[0]->intrinsic_matrix, Data::camera[0]->distortion_coeffs, rvec, tvec, 0, cv::SOLVEPNP_ITERATIVE);

    // 4. 将rvec和tvec转换为4x4的矩阵place2camera
    Eigen::Matrix<double, 4, 4> place2camera = Eigen::Matrix<double, 4, 4>::Identity();
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix<double, 3, 3> rotate;
    rm::tf_Mat3d(rmat, rotate);
    Eigen::Matrix<double, 4, 1> pose;
    rm::tf_Vec4d(tvec, pose);
    rm::tf_rt2trans(pose, rotate, place2camera);

    // 5. 得到radar2place
    Data::radar2place = place2camera.inverse() * Data::camera[0]->Trans_pnp2head;

    // 输出tvec 
    std::cout << "tvec: " << tvec << std::endl;

    return true;
}


