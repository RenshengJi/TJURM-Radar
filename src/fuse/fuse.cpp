#include "data_manager/param.h"
#include "data_manager/base.h"
#include "fuse/fuse.h"
#include <openrm/cudatools.h>
#include <unistd.h>

// pcl库
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


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
    
    // 等待人工选点结果
    if(!Data::extrinsic->is_valid){
        return false;
    }
    Data::radar2place = Eigen::Matrix<double, 4, 4>::Identity();


    // 对每个相机进行外参标定，得到外参标定结果
    for(int i = 0; i < Data::camera.size(); i++){
        // 1. 获取世界坐标（从配置文件里读取）
        std::vector<cv::Point3d> zed_cali(4);
        std::string key = "PlacePoint" + std::to_string(i);
        zed_cali[0].x = (*param)[key]["1x"];
        zed_cali[0].y = (*param)[key]["1y"];
        zed_cali[0].z = (*param)[key]["1z"];
        zed_cali[1].x = (*param)[key]["2x"];
        zed_cali[1].y = (*param)[key]["2y"];
        zed_cali[1].z = (*param)[key]["2z"];
        zed_cali[2].x = (*param)[key]["3x"];
        zed_cali[2].y = (*param)[key]["3y"];
        zed_cali[2].z = (*param)[key]["3z"];
        zed_cali[3].x = (*param)[key]["4x"];
        zed_cali[3].y = (*param)[key]["4y"];
        zed_cali[3].z = (*param)[key]["4z"];

        // 2. 获取相机坐标
        std::vector<cv::Point2d> radar_cali(4);
        for(int j = 0; j < 4; j++){
            radar_cali[j].x = Data::extrinsic->image_zed_calib[j].x * Data::camera[i]->width;
            radar_cali[j].y = Data::extrinsic->image_zed_calib[j].y * Data::camera[i]->height;
        }
        // 3. 使用solvepnp求解相机与场地坐标系的外参
        cv::Mat rvec, tvec;
        cv::solvePnP(zed_cali, radar_cali, Data::camera[i]->intrinsic_matrix, Data::camera[i]->distortion_coeffs, rvec, tvec, 0, cv::SOLVEPNP_EPNP);

        // 4. 将rvec和tvec转换为4x4的矩阵place2camera
        Eigen::Matrix<double, 4, 4> place2camera = Eigen::Matrix<double, 4, 4>::Identity();
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix<double, 3, 3> rotate;
        rm::tf_Mat3d(rmat, rotate);
        Eigen::Matrix<double, 4, 1> pose;
        rm::tf_Vec4d(tvec, pose);
        rm::tf_rt2trans(pose, rotate, place2camera);
        Data::camera2place.push_back(place2camera.inverse());

        // // 5. 得到radar2place
        Data::radar2place = place2camera.inverse() * Data::camera[0]->Trans_pnp2head;
    }


    // 输出tvec 
    std::cout << "x: " << Data::camera2place[0](0, 3) << " y: " << Data::camera2place[0](1, 3) << " z: " << Data::camera2place[0](2, 3) << std::endl;

    
    return true;
}



void init_depth(){
    // 读取pcl文件，拿到PointCloud格式的点云数据
    auto param = Param::get_instance();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string pcd_path = (*param)["PointCloud"]["Dir"];
    pcl::io::loadPCDFile(pcd_path, *cloud);


    // 根据雷达站和场地的外参以及雷达站和对应相机的外参，将点云转换到相机坐标系下
    // 1. 先将点云数据转换为Matrix格式
    Eigen::Matrix<double, 4, Eigen::Dynamic> cloud_matrix = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, cloud->width);
    for(int i = 0; i < cloud->width; i++){
        cloud_matrix(0, i) = cloud->points[i].x;
        cloud_matrix(1, i) = cloud->points[i].y;
        cloud_matrix(2, i) = cloud->points[i].z;
        cloud_matrix(3, i) = 1;
    }

    // 2. 将点云投影到雷达站的图像坐标系下，根据各相机的外参，将点云转换到相机坐标系下
    for(int i = 0; i < Data::camera.size(); i++){

        Eigen::Matrix<double, 4, Eigen::Dynamic> cloud_matrix_in_camera = Data::camera2place[i].inverse() * cloud_matrix;
        Eigen::Matrix<double, 3, 3> intrinsic_matrix;
        rm::tf_Mat3d(Data::camera[i]->intrinsic_matrix, intrinsic_matrix);
        Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_matrix_in_camera_3d = intrinsic_matrix * cloud_matrix_in_camera.topRows(3);


        // 3. 将点云投影到图像坐标系下，得到深度图  FIXME: 采样bug,在后面解决了,但理论上在前面解决更好一点
        cv::Mat depth_image = cv::Mat::zeros(Data::camera[i]->height, Data::camera[i]->width, CV_64FC1);
        for(int j = 0; j < cloud->width; j++){
            int x = cloud_matrix_in_camera_3d(0, j) / cloud_matrix_in_camera_3d(2, j);
            int y = cloud_matrix_in_camera_3d(1, j) / cloud_matrix_in_camera_3d(2, j);
            if(x >= 0 && x < Data::camera[i]->width && y >= 0 && y < Data::camera[i]->height){
                double depth = cloud_matrix_in_camera_3d(2, j)/1000.0;
                if(depth < 1) continue;
                if(depth_image.at<double>(y, x) == 0 || depth_image.at<double>(y, x) > depth)
                    depth_image.at<double>(y, x) = depth;
            }
        }
        Data::depth.push_back(depth_image);
    }
}