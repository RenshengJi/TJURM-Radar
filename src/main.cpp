#include "data_manager/param.h"
#include "data_manager/base.h"

uchar* close_buffer;


void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) {
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) {
    v = vmin;
  }

  if (v > vmax) {
    v = vmax;
  }

  double dr, dg, db;

  if (v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if (v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}


int main(int argc, char* argv[]) {
    auto param = Param::get_instance();

    // 初始化所有设备
    while(true){
        if(init_driver()) break;
    }
    
    // 读取摄像头数据
    while(true){
        // 读取close_camera数据
        cv::Mat close_image = cv::Mat(Data::close_camera->height, Data::close_camera->width, CV_8UC3, Data::close_camera->image_buffer);
        // cv::resize(close_image, close_image, cv::Size(640, 480));
        // cv::imshow("close_image", close_image);
        // cv::waitKey(1);

        // 读取far_camera数据
        cv::Mat far_image = cv::Mat(Data::far_camera->height, Data::far_camera->width, CV_8UC3, Data::far_camera->image_buffer);
        // cv::resize(far_image, far_image, cv::Size(640, 480));
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
        rm::tf_Mat3d(Data::close_camera->intrinsic_matrix, intrinsic_matrix);
        Eigen::Matrix<double, 3, Eigen::Dynamic> radar_matrix_in_close_3d = intrinsic_matrix * radar_matrix_in_close.topRows(3);



        // 4. 将点云投影到图像坐标系下，得到深度图，强度图
        cv::Mat depth_image = cv::Mat(Data::close_camera->height, Data::close_camera->width, CV_8UC3, Data::close_camera->image_buffer);
        cv::Mat intensity_image = cv::Mat(Data::close_camera->height, Data::close_camera->width, CV_8UC3, Data::close_camera->image_buffer);
        for(int i = 0; i < Data::radar->num_point; i++){
            int x = radar_matrix_in_close_3d(0, i) / radar_matrix_in_close_3d(2, i);
            int y = radar_matrix_in_close_3d(1, i) / radar_matrix_in_close_3d(2, i);
            if(x >= 0 && x < Data::close_camera->width && y >= 0 && y < Data::close_camera->height){
                // 强度值映射为伪彩色
                float intensity = radar_image.at<float>(3, i) / 255.0;
                uint8_t r, g, b;
                mapJet(intensity, 0, 1, r, g, b);
                intensity_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
        }

        cv::resize(intensity_image, intensity_image, cv::Size(640, 480));
        intensity_image = 0.5 * intensity_image + 0.8 * close_image;

        // 5. 显示强度图
        cv::imshow("intensity_image", intensity_image);
        cv::waitKey(1);

    }

    return 0;
}

