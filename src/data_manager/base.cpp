#include "data_manager/base.h"

// 颜色
rm::ArmorColor Data::self_color;
rm::ArmorColor Data::enemy_color;


// 设备
std::vector<rm::Camera*> Data::camera;
rm::Radar* Data::radar;

// 点云转换得到的深度图(便于信息融合)
std::vector<cv::Mat> Data::radar_depth;





