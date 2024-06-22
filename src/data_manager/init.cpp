#include "data_manager/param.h"
#include "data_manager/base.h"

bool init_driver() {
    auto param = Param::get_instance();


    // close_camera
    Data::close_camera = new rm::Camera();
    Data::close_camera->width = (*param)["Camera"]["Close"]["Width"];
    Data::close_camera->height = (*param)["Camera"]["Close"]["Height"];
    Data::close_camera->image_buffer = (uint8_t *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Camera"]["Close"]["Name"]), Data::close_camera->height * Data::close_camera->width * 3);

    // far_camera
    Data::far_camera = new rm::Camera();
    Data::far_camera->width = (*param)["Camera"]["Far"]["Width"];
    Data::far_camera->height = (*param)["Camera"]["Far"]["Height"];
    Data::far_camera->image_buffer = (uint8_t *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Camera"]["Far"]["Name"]), Data::far_camera->height * Data::far_camera->width * 3);

    // radar
    Data::radar = new rm::Radar();
    Data::radar->num_point = (*param)["Radar"]["Num"];
    Data::radar->point_cloud_buffer = (uint8_t *)rm::__shm_alloc__(rm::__gen_hash_key__((*param)["Radar"]["Name"]), Data::radar->num_point * 4);
    

    return true;
}