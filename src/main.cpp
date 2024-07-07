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

    // 等待外参标定结果 TODO: 
    // while(true){
    //     if(extrinsic_calib()) break;
    // }

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
        // cv::Mat close_image = cv::Mat(Data::camera[0]->height, Data::camera[0]->width, CV_8UC3, Data::camera[0]->image_buffer);
        // cv::Mat far_image = cv::Mat(Data::camera[1]->height, Data::camera[1]->width, CV_8UC3, Data::camera[1]->image_buffer);
        // for(auto& yolo : *yolo_list){
        //     if(yolo.camera_id == 0){
        //         rm::displaySingleArmorID(close_image, yolo);
        //         rm::displaySingleArmorRect(close_image, yolo, 3);
        //     }
        //     else{
        //         rm::displaySingleArmorID(far_image, yolo);
        //         rm::displaySingleArmorRect(far_image, yolo, 3);
        //     }    
        // }
        // cv::resize(close_image, close_image, cv::Size(1280, 720));
        // cv::resize(far_image, far_image, cv::Size(1280, 720));
        // cv::imshow("close", close_image);
        // cv::imshow("far", far_image);
        // cv::waitKey(1);


        // fuse
        for(int i = 0; i < Data::camera.size(); i++){
            Data::radar_depth[i] = PointCloud2Depth(Data::radar, Data::camera[i]);
        }


        cv::resize(Data::radar_depth[0], Data::radar_depth[0], cv::Size(1280, 720));
        cv::resize(Data::radar_depth[1], Data::radar_depth[1], cv::Size(1280, 720));
        cv::imshow("close_depth", Data::radar_depth[0]);
        cv::imshow("far_depth", Data::radar_depth[1]);
        cv::waitKey(1);



        // serial


    }

    return 0;
}

