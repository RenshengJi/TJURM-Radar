#include "data_manager/param.h"
#include "data_manager/base.h"

bool init_driver() {
    auto param = Param::get_instance();

    // 获取相机参数矩阵json
    nlohmann::json camlens;
    std::string camlen_path = (*param)["Camera"]["CamLensDir"];
    try {
        std::ifstream camlens_json(camlen_path);
        camlens_json >> camlens;
        camlens_json.close();
    } catch (std::exception& e) {
        std::string err_str = "Failed to load CamLens json: " + std::string(e.what());
        rm::message(err_str, rm::MSG_ERROR);
        return false;
    }


    // close_camera
    Data::close_camera = new rm::Camera();
    Data::close_camera->width = (*param)["Camera"]["Close"]["Width"];
    Data::close_camera->height = (*param)["Camera"]["Close"]["Height"];
    Param::from_json(camlens["Close"]["Intrinsic"], Data::close_camera->intrinsic_matrix);
    Param::from_json(camlens["Close"]["Distortion"], Data::close_camera->distortion_coeffs);
    cv::Mat extrinsic_matrix(4, 4, CV_64F);
    Param::from_json(camlens["Close"]["Extrinsic"], extrinsic_matrix);
    Eigen::Matrix<double, 4, 4>  extrinsic_matrix_eigen = Eigen::Matrix<double, 4, 4>::Identity();
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            extrinsic_matrix_eigen(i, j) = extrinsic_matrix.at<double>(i, j);
        }
    }
    Data::close_camera->Trans_pnp2head = extrinsic_matrix_eigen;
    Data::close_camera->image_buffer = (uint8_t *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Camera"]["Close"]["Name"]), Data::close_camera->height * Data::close_camera->width * 3);

    // far_camera
    Data::far_camera = new rm::Camera();
    Data::far_camera->width = (*param)["Camera"]["Far"]["Width"];
    Data::far_camera->height = (*param)["Camera"]["Far"]["Height"];
    Data::far_camera->image_buffer = (uint8_t *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Camera"]["Far"]["Name"]), Data::far_camera->height * Data::far_camera->width * 3);

    // radar
    Data::radar = new rm::Radar();
    Data::radar->num_point = (*param)["Radar"]["Num"];
    Data::radar->point_cloud_buffer = (float *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Radar"]["Name"]), Data::radar->num_point * 4 * 4);
    

    return true;
}