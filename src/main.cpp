#include "data_manager/param.h"
#include "data_manager/base.h"
#include "model/init.h"
#include "fuse/fuse.h"


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

        
        


        // serial


    }

    return 0;
}

