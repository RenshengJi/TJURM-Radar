#include "data_manager/base.h"

// 颜色
rm::ArmorColor Data::self_color;
rm::ArmorColor Data::enemy_color;

// 雷达标记进度数据
radar_mark_data_t Data::radar_mark_data;
radar_info_t Data::radar_info;
robot_interaction_data_t Data::robot_interaction_data;
game_robot_HP_t Data::game_robot_HP;

// 雷达标记数据
map_robot_data_t Data::map_robot_data;
radar_cmd_t Data::radar_cmd;

// 设备
std::vector<rm::Camera*> Data::camera;
rm::Radar* Data::radar;

// 点云转换得到的深度图(便于信息融合)
std::vector<cv::Mat> Data::radar_depth;
std::vector<cv::Mat> Data::depth;

// 外参标定结果
rm::RadarData* Data::extrinsic;
Eigen::Matrix<double, 4, 4> Data::radar2place;
std::vector<Eigen::Matrix<double, 4, 4>> Data::camera2place;

// 敌方6辆车的位置
std::vector<Car> Data::enemy_info;

// 串口
serial::Serial Data::ser;

// 小地图map
cv::Mat Data::map;





