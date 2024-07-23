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


    // camera[0] Close
    Data::camera.push_back(new rm::Camera());
    Data::camera[0]->camera_id = 0;
    Data::camera[0]->width = (*param)["Camera"]["Close"]["Width"];
    Data::camera[0]->height = (*param)["Camera"]["Close"]["Height"];
    Param::from_json(camlens["Close"]["Intrinsic"], Data::camera[0]->intrinsic_matrix);
    Param::from_json(camlens["Close"]["Distortion"], Data::camera[0]->distortion_coeffs);
    cv::Mat extrinsic_matrix(4, 4, CV_64F);
    Param::from_json(camlens["Close"]["Extrinsic"], extrinsic_matrix);
    rm::tf_Mat4d(extrinsic_matrix, Data::camera[0]->Trans_pnp2head);
    Data::camera[0]->image_buffer = (uint8_t *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Camera"]["Close"]["Name"]), Data::camera[0]->height * Data::camera[0]->width * 3);
    rm::mallocYoloCameraBuffer(&Data::camera[0]->rgb_host_buffer, &Data::camera[0]->rgb_device_buffer, Data::camera[0]->width, Data::camera[0]->height);

    // camera[1] Far-left
    Data::camera.push_back(new rm::Camera());
    Data::camera[1]->camera_id = 1;
    Data::camera[1]->width = (*param)["Camera"]["Far"]["Width"];
    Data::camera[1]->height = (*param)["Camera"]["Far"]["Height"];
    Param::from_json(camlens["Far"]["Intrinsic"], Data::camera[1]->intrinsic_matrix);
    Param::from_json(camlens["Far"]["Distortion"], Data::camera[1]->distortion_coeffs);
    Param::from_json(camlens["Far"]["Extrinsic"], extrinsic_matrix);
    rm::tf_Mat4d(extrinsic_matrix, Data::camera[1]->Trans_pnp2head);
    Data::camera[1]->image_buffer = (uint8_t *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Camera"]["Far"]["Name"]), Data::camera[1]->height * Data::camera[1]->width * 3);
    rm::mallocYoloCameraBuffer(&Data::camera[1]->rgb_host_buffer, &Data::camera[1]->rgb_device_buffer, Data::camera[1]->width, Data::camera[1]->height);

    // radar
    Data::radar = new rm::Radar();
    Data::radar->num_point = (*param)["Radar"]["Num"];
    Data::radar->point_cloud_buffer = (float *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Radar"]["Name"]), Data::radar->num_point * 4 * 4);

    // radar_depth
    for(int i = 0; i < Data::camera.size(); i++){
        Data::radar_depth.push_back(cv::Mat(Data::camera[i]->height, Data::camera[i]->width, CV_32FC1));
    }

    // extrinsic
    std::string extrinsic_name = (*param)["Extrinsic"]["Name"];
    Data::extrinsic = (rm::RadarData *)rm::__shm_alloc__(rm::__gen_hash_key__(extrinsic_name), sizeof(rm::RadarData));


    // 小地图map
    std::string map_dir = (*param)["Map"]["Dir"];
    Data::map = cv::imread(map_dir);
    cv::resize(Data::map, Data::map, cv::Size(Data::map.cols/3, Data::map.rows/3));

    // 初始化Data::camera[i]->image
    for(int i = 0; i < Data::camera.size(); i++){
        Data::camera[i]->image = (uint8_t *)malloc(Data::camera[i]->width * Data::camera[i]->height * 3);
    }

    // 初始化map_robot_data和radar_cmd
    memset(&Data::map_robot_data, 0, sizeof(map_robot_data_t));
    memset(&Data::radar_cmd, 0, sizeof(radar_cmd_t));
    memset(&Data::game_robot_HP, 0, sizeof(game_robot_HP_t));

    // 初始化敌方车信息
    for(int i = 0; i < 6; i++){
        Car car;
        car.pos = cv::Point3f(0, 0, 0);
        car.is_debuff = false;
        car.is_dehealth = false;
        Data::enemy_info.push_back(car);
    }

    return true;
}