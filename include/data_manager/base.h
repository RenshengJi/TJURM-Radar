#ifndef RM2024_DATA_MANAGER_BASE_H_
#define RM2024_DATA_MANAGER_BASE_H_

#include <opencv2/opencv.hpp>
#include <openrm.h>
#include <Eigen/Dense>
#include <cstdint>
#include <serial/serial.h>
#include "serial/serial_.h"
// pcl库
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>


// 存储地敌方车辆信息 0哨兵 1-5同车辆贴纸数字
struct Car {
    cv::Point3f pos;   // 场地坐标系下的坐标
    bool is_debuff;     // 是否正在debuff状态
    bool is_dehealth;   // 是否正在掉血
};

namespace Data {
    // time 
    extern game_status_t game_status;

    // 颜色
    extern rm::ArmorColor self_color;
    extern rm::ArmorColor enemy_color;

    // 敌方车信息
    extern std::vector<Car> enemy_info;

    // 雷达标记进度数据
    extern radar_mark_data_t radar_mark_data;
    extern radar_info_t radar_info;
    extern robot_interaction_data_t robot_interaction_data;
    extern game_robot_HP_t game_robot_HP;

    // 雷达标记数据
    extern map_robot_data_t map_robot_data;
    extern radar_cmd_t radar_cmd;

    // 相机的参数记录
    extern std::vector<rm::Camera*> camera;
    extern rm::Radar* radar;
    
    // 点云转换得到的深度图(便于信息融合)
    extern std::vector<cv::Mat> radar_depth;
    extern std::vector<cv::Mat> depth;

    // 外参标定结果(雷达站到场地坐标系的，以及每个相机到场地坐标系的)
    extern rm::RadarData* extrinsic;
    extern Eigen::Matrix<double, 4, 4> radar2place;  
    extern std::vector<Eigen::Matrix<double, 4, 4>> camera2place;

    // 敌方6辆车的位置
    extern std::vector<cv::Point3f> enemy_pos;

    // 串口
    extern serial::Serial ser;

    // 小地图map
    extern cv::Mat map;

    // KD树+八叉树
    extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    extern pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // extern pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;

}

bool init_driver();

#endif