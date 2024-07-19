#include "data_manager/param.h"
#include "data_manager/base.h"
#include "model/init.h"
#include "fuse/fuse.h"
#include <serial/serial.h>
#include "serial/serial_.h"
#include <thread>


int main(int argc, char* argv[]) {
    auto param = Param::get_instance();

    // 初始化所有设备
    while(true){
        if(init_driver()) break;
    }

    // 等待外参标定结果
    while(true){
        if(extrinsic_calib()) break;
    }

    // 初始化模型
    auto rgb_model = RGB_MODEL::get_instance();

    // 初始化串口，建立接收数据线程并进入
    serial_port_init();
    std::thread serial_thread(serial_port_recv);

    // 初始化深度图(需要外参支持)
    init_depth();


    // 主循环
    while(true){

        // FIXME: 首先copy一份出来
        // TODO: 对Data::camera[i]->image_buffer去畸变(便于和点云深度图叠加)
        for(int i = 0; i < Data::camera.size(); i++){
            memcpy(Data::camera[i]->image, Data::camera[i]->image_buffer, Data::camera[i]->height * Data::camera[i]->width * 3);
            cv::Mat image = cv::Mat(Data::camera[i]->height, Data::camera[i]->width, CV_8UC3, Data::camera[i]->image);
            cv::Mat undistort_image;
            cv::undistort(image, undistort_image, Data::camera[i]->intrinsic_matrix, Data::camera[i]->distortion_coeffs);
            memcpy(Data::camera[i]->image, undistort_image.data, Data::camera[i]->height * Data::camera[i]->width * 3);
        }

        // 神经网络推理，得到装甲板的2D位置，同时注意记录相机id
        std::shared_ptr<std::vector<rm::YoloRectWithCamera>> yolo_list = std::make_shared<std::vector<rm::YoloRectWithCamera>>();
        for(int i = 0; i < Data::camera.size(); i++){
            auto yolo_list_single = rgb_model->detect_armor(i);
            yolo_list->insert(yolo_list->end(), yolo_list_single->begin(), yolo_list_single->end());
        }

        // 激光雷达与RGB相机信息融合
        for(int i = 0; i < Data::camera.size(); i++){
            Data::radar_depth[i] = PointCloud2Depth(Data::radar, Data::camera[i]);
        }

        cv::Mat map = Data::map.clone();

        // 获取装甲板在场地坐标系下的3D坐标
        for(auto& yolo : *yolo_list){

            // 只处理敌方类别 TODO:
            // if(yolo.color == Data::self_color)
            //     continue;
            // 不处理建筑物
            if(yolo.class_id % 9 >= 6)
                continue;
            int camera_id = yolo.camera_id;
            std::vector<cv::Point3f> armor_3d;
            // 遍历当前矩形框(yolo)内所有点，计算出装甲板在场地坐标系下的坐标
            int x = yolo.box.x, y = yolo.box.y, w = yolo.box.width, h = yolo.box.height;
            // TODO: 往下操作一波(不然不是车的中心)
            if(y + 3*h > Data::camera[camera_id]->height)
                continue;
            y += 2*h;
            for(int i = x; i < x + w; i++){
                for(int j = y; j < y + h; j++){
                    // TODO: 联合标定法深度获取
                    // // 如果深度图中该点深度值不为0，可以参与计算
                    // if(Data::radar_depth[camera_id].at<float>(j, i) != 0){
                    //     // 1. 计算该点在相机坐标系下的坐标（利用当前像素坐标，深度信息，以及内参矩阵）
                    //     float z = Data::radar_depth[camera_id].at<float>(j, i);
                    //     cv::Mat point_pixel = (cv::Mat_<float>(3, 1) << i*z, j*z, z);
                    //     cv::Mat camera_cor_mat = Data::camera[camera_id]->intrinsic_matrix.inv() * point_pixel;
                    //     Eigen::Vector4d camera_cor(camera_cor_mat.at<float>(0), camera_cor_mat.at<float>(1), camera_cor_mat.at<float>(2), 1);

                    //     // 2. 计算该点在云台坐标系下的坐标（利用相机坐标系下的坐标，以及联合标定矩阵）
                    //     Eigen::Vector4d head_cor = Data::camera[camera_id]->Trans_pnp2head.inverse() * camera_cor;

                    //     // 3. 计算该点在场地坐标系下的坐标（利用云台坐标系下的坐标，以及外参矩阵）
                    //     Eigen::Vector4d world_cor = Data::radar2place.inverse() * head_cor;

                    //     armor_3d.push_back(cv::Point3f(world_cor(0), world_cor(1), world_cor(2)));
                    // }

                    // 单目法深度获取
                    if(Data::depth[camera_id].at<double>(j, i) != 0){
                        std::cout <<  Data::depth[camera_id].at<double>(j, i) << " ";
                        // 1. 计算该点在相机坐标系下的坐标（利用当前像素坐标，深度信息，以及内参矩阵）
                        double z = Data::depth[camera_id].at<double>(j, i) * 1000;
                        cv::Mat point_pixel = (cv::Mat_<double>(3, 1) << i*z, j*z, z);
                        cv::Mat camera_cor_mat = Data::camera[camera_id]->intrinsic_matrix.inv() * point_pixel;
                        Eigen::Vector4d camera_cor(camera_cor_mat.at<double>(0), camera_cor_mat.at<double>(1), camera_cor_mat.at<double>(2), 1);

                        // 2. 计算该点在场地坐标系下的坐标（利用相机坐标系下的坐标，以及外参矩阵）
                        Eigen::Vector4d world_cor = Data::camera2place[camera_id] * camera_cor;

                        armor_3d.push_back(cv::Point3f(world_cor(0), world_cor(1), world_cor(2)));
                    }
                }
            }
            std::cout << std::endl;
            std::cout << "-------------------" <<   yolo.class_id%9 <<   "-------------------" << std::endl;

            if(armor_3d.size() == 0)
                continue;
            // 对取到的3D坐标直接取平均 TODO: 更好的方法
            cv::Point3f armor_3d_mean(0, 0, 0);
            for(auto& point : armor_3d){
                armor_3d_mean.x += point.x;
                armor_3d_mean.y += point.y;
                armor_3d_mean.z += point.z;
            }
            armor_3d_mean.x /= armor_3d.size();
            armor_3d_mean.y /= armor_3d.size();
            armor_3d_mean.z /= armor_3d.size();

            // 将检测到的点绘制到小地图上
            int scale = 3 * 10;
            cv::Point2f armor_2d = cv::Point2f(armor_3d_mean.x/scale, Data::map.rows - armor_3d_mean.y/scale);
            cv::circle(map, armor_2d, 10, cv::Scalar(0, 0, 255), -1);
            // 写出类别(class_id)
            cv::putText(map, std::to_string(yolo.class_id % 9), armor_2d, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }


        // TODO: 绘制图像和点云深度叠加图，检测外参标定效果
        for(int i = 0; i < Data::camera.size(); i++){
            cv::Mat image = cv::Mat(Data::camera[i]->height, Data::camera[i]->width, CV_8UC3, Data::camera[i]->image).clone();
            cv::Mat depth_image = Data::depth[i];
            for(int i = 0; i < depth_image.rows; i++){
                for(int j = 0; j < depth_image.cols; j++){
                    double pixel = depth_image.at<double>(i, j);
                    // pixel, 从0到1, 红近蓝远
                    pixel = pixel < 1 ? 0 : pixel > 30 ? 1 : pixel/30.0;
                    if(pixel != 0){
                        image.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(i, j) * 0.5 + cv::Vec3b(255*pixel, 0, 255*(1-pixel)) * 0.5;
                    }
                }
            }
            cv::resize(image, image, cv::Size(1280, 960));
            cv::imshow("image" + std::to_string(i), image);
        }
        
        cv::imshow("map", map);
        cv::waitKey(1);
    }

    return 0;
}

