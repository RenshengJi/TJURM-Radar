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
        // cv::imshow("close_image", close_image);
        // cv::waitKey(1);

        // 读取far_camera数据
        cv::Mat far_image = cv::Mat(Data::far_camera->height, Data::far_camera->width, CV_8UC3, Data::far_camera->image_buffer);
        cv::resize(far_image, far_image, cv::Size(640, 480));
        // cv::imshow("far_image", far_image);
        // cv::waitKey(1);

        // 读取radar数据
        cv::Mat radar_image = cv::Mat(4, Data::radar->num_point, CV_32FC1, Data::radar->point_cloud_buffer);


        // 将点云数据转换为和close_camera对齐的深度图（深度用颜色表示，红近蓝远)
        // 1. 为了方便，先将点云数据转换Matrix格式
        Eigen::Matrix<double, 4, Eigen::Dynamic> radar_matrix = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, Data::radar->num_point);
        for(int i = 0; i < Data::radar->num_point; i++){
            radar_matrix(0, i) = radar_image.at<float>(0, i);
            radar_matrix(1, i) = radar_image.at<float>(1, i);
            radar_matrix(2, i) = radar_image.at<float>(2, i);
            radar_matrix(3, i) = 1;
        }
        // 2. 将点云经过close_camera的外参转换到close_camera坐标系下
        // 外参: Data::close_camera->Trans_pnp2head
        Eigen::Matrix<double, 4, Eigen::Dynamic> radar_matrix_in_close = Data::close_camera->Trans_pnp2head * radar_matrix;
        

        // 3. 将点云投影到close_camera的图像坐标系下
        // 内参: Data::close_camera->intrinsic_matrix 但是为Mat格式，首先转换为Matrix格式
        Eigen::Matrix<double, 3, 3> intrinsic_matrix;
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                intrinsic_matrix(i, j) = Data::close_camera->intrinsic_matrix.at<double>(i, j);
            }
        }
        Eigen::Matrix<double, 3, Eigen::Dynamic> radar_matrix_in_close_3d = intrinsic_matrix * radar_matrix_in_close.topRows(3);

        // 4. 将点云投影到图像坐标系下，得到深度图
        cv::Mat depth_image = cv::Mat(Data::close_camera->height, Data::close_camera->width, CV_64FC3, cv::Scalar(0));
        for(int i = 0; i < Data::radar->num_point; i++){
            int x = radar_matrix_in_close_3d(0, i) / radar_matrix_in_close_3d(2, i);
            int y = radar_matrix_in_close_3d(1, i) / radar_matrix_in_close_3d(2, i);
            if(x >= 0 && x < Data::close_camera->width && y >= 0 && y < Data::close_camera->height){
                // 为了显示效果，将深度值映射为伪彩色（红近蓝远）
                double depth = radar_matrix_in_close_3d(2, i);
                if(depth < 0) depth = 0;
                if(depth > 10) depth = 10;
                depth_image.at<cv::Vec3d>(y, x) = cv::Vec3d(255 * depth / 10, 0, 255 * (10 - depth) / 10);
            }
        }

        // 5. 显示深度图
        cv::resize(depth_image, depth_image, cv::Size(640, 480));


        cv::imshow("depth_image", depth_image);
        cv::waitKey(1);






    }

    return 0;
}

