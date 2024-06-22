#include "data_manager/param.h"
#include "data_manager/base.h"

uchar* close_buffer;


int main(int argc, char* argv[]) {
    auto param = Param::get_instance();

    // 挂载所有设备的缓冲区
    while(true){
        if(init_driver()) break;
    }
    
    // 读取摄像头数据
    while(true){
        // 读取close_camera数据
        cv::Mat close_image = cv::Mat(Data::close_camera->height, Data::close_camera->width, CV_8UC3, Data::close_camera->image_buffer);
        cv::resize(close_image, close_image, cv::Size(640, 480));
        cv::imshow("close_image", close_image);
        cv::waitKey(1);

        // 读取far_camera数据
        cv::Mat far_image = cv::Mat(Data::far_camera->height, Data::far_camera->width, CV_8UC3, Data::far_camera->image_buffer);
        cv::resize(far_image, far_image, cv::Size(640, 480));
        cv::imshow("far_image", far_image);
        cv::waitKey(1);

        // 读取radar数据
        cv::Mat radar_image = cv::Mat(4, Data::radar->num_point, CV_32FC1, Data::radar->point_cloud_buffer);
    }

    return 0;
}

