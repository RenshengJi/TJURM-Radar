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

    

        // 获取装甲板在场地坐标系下的3D坐标
        for(auto& yolo : *yolo_list){

            // 只处理敌方类别 TODO:
            // if(yolo.color == Data::self_color)
            //     continue;
            int camera_id = yolo.camera_id;
            std::vector<cv::Point3f> armor_3d;
            // 遍历当前矩形框(yolo)内所有点，计算出装甲板在场地坐标系下的坐标
            int x = yolo.box.x, y = yolo.box.y, w = yolo.box.width, h = yolo.box.height;
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
                        // 1. 计算该点在相机坐标系下的坐标（利用当前像素坐标，深度信息，以及内参矩阵）
                        double z = Data::depth[camera_id].at<double>(j, i);
                        cv::Mat point_pixel = (cv::Mat_<double>(3, 1) << i*z, j*z, z);
                        cv::Mat camera_cor_mat = Data::camera[camera_id]->intrinsic_matrix.inv() * point_pixel;
                        Eigen::Vector4d camera_cor(camera_cor_mat.at<float>(0), camera_cor_mat.at<float>(1), camera_cor_mat.at<float>(2), 1);

                        // 2. 计算该点在云台坐标系下的坐标（利用相机坐标系下的坐标，以及联合标定矩阵）
                        Eigen::Vector4d head_cor = Data::camera[camera_id]->Trans_pnp2head.inverse() * camera_cor;

                        // 3. 计算该点在场地坐标系下的坐标（利用云台坐标系下的坐标，以及外参矩阵）
                        Eigen::Vector4d world_cor = Data::radar2place.inverse() * head_cor;

                        armor_3d.push_back(cv::Point3f(world_cor(0), world_cor(1), world_cor(2)));
                    }
                }
            }

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

            std::cout << "armor_3d_mean: " << armor_3d_mean << std::endl;
        }


        // serial


        


    }

    return 0;
}

