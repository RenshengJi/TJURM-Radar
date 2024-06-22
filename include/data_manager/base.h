#ifndef RM2024_DATA_MANAGER_BASE_H_
#define RM2024_DATA_MANAGER_BASE_H_

#include <opencv2/opencv.hpp>
#include <openrm.h>
#include <Eigen/Dense>
#include <cstdint>

namespace Data {
    // 颜色
    extern rm::ArmorColor self_color;
    extern rm::ArmorColor enemy_color;

    // 相机的参数记录
    extern rm::Camera* close_camera;
    extern rm::Camera* far_camera;
    extern rm::Radar* radar;
}

bool init_driver();

#endif