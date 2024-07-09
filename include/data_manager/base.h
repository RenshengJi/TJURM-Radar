#ifndef RM2024_DATA_MANAGER_BASE_H_
#define RM2024_DATA_MANAGER_BASE_H_

#include <opencv2/opencv.hpp>
#include <openrm.h>
#include <Eigen/Dense>
#include <cstdint>
#include <serial/serial.h>

namespace Data {
    // 颜色
    extern rm::ArmorColor self_color;
    extern rm::ArmorColor enemy_color;

    // 相机的参数记录
    extern std::vector<rm::Camera*> camera;
    extern rm::Radar* radar;
    
    // 点云转换得到的深度图(便于信息融合)
    extern std::vector<cv::Mat> radar_depth;

    // 外参标定结果
    extern Eigen::Matrix<double, 4, 4> radar2place;  
    extern rm::RadarData* extrinsic;

    // 敌方6辆车的位置
    extern std::vector<cv::Point3f> enemy_pos;

    // 串口
    extern serial::Serial ser;
}

bool init_driver();

#endif