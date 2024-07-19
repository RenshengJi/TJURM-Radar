#include "data_manager/param.h"
#include "data_manager/base.h"
#include "model/init.h"
#include <openrm/cudatools.h>
#include <unistd.h>


std::shared_ptr<std::vector<rm::YoloRectWithCamera>> RGB_MODEL::detect_armor(int camera_id) {
    rm::memcpyYoloCameraBuffer(
            Data::camera[camera_id]->image,
            Data::camera[camera_id]->rgb_host_buffer,
            Data::camera[camera_id]->rgb_device_buffer,
            Data::camera[camera_id]->width,
            Data::camera[camera_id]->height);
        
        rm::resize(
            Data::camera[camera_id]->rgb_device_buffer,
            Data::camera[camera_id]->width,
            Data::camera[camera_id]->height,
            armor_input_device_buffer_,
            infer_width,
            infer_height,
            (void*)resize_stream_);

        rm::detectEnqueue(
            armor_input_device_buffer_,
            armor_output_device_buffer_,
            &armor_context_,
            &detect_stream_
        );
        rm::detectOutput(
            armor_output_host_buffer_,
            armor_output_device_buffer_,
            &detect_stream_,
            yolo_struct_size,
            bboxes_num
        );
        std::shared_ptr<std::vector<rm::YoloRect>> yolo_list = std::make_shared<std::vector<rm::YoloRect>>(
            rm::yoloArmorNMS_V5(
                armor_output_host_buffer_,
                bboxes_num,
                class_num,
                confidence_thresh,
                nms_thresh,
                Data::camera[camera_id]->width,
                Data::camera[camera_id]->height,
                infer_width,
                infer_height
            )
        );

        std::shared_ptr<std::vector<rm::YoloRectWithCamera>> yolo_list_with_camera = std::make_shared<std::vector<rm::YoloRectWithCamera>>();
        for(auto& yolo : *yolo_list){
            rm::YoloRectWithCamera yolo_with_camera;
            yolo_with_camera.box = yolo.box;
            yolo_with_camera.class_id = yolo.class_id;
            yolo_with_camera.confidence = yolo.confidence;
            yolo_with_camera.four_points = yolo.four_points;
            yolo_with_camera.camera_id = camera_id;
            yolo_list_with_camera->push_back(yolo_with_camera);
        }

    return yolo_list_with_camera;
}


RGB_MODEL::RGB_MODEL() {
    auto param = Param::get_instance();
    bool cuda_status = rm::initCudaStream(&detect_stream_);
    cuda_status = rm::initCudaStream(&resize_stream_);
    if (!cuda_status) {
        rm::message("Failed to initialize CUDA stream", rm::MSG_ERROR);
        exit(-1);
    }
    onnx_file = (*param)["Model"]["YoloArmor"]["DirONNX"];
    engine_file = (*param)["Model"]["YoloArmor"]["DirEngine"];
    infer_width = (*param)["Model"]["YoloArmor"]["InferWidth"];
    infer_height = (*param)["Model"]["YoloArmor"]["InferHeight"];
    class_num = (*param)["Model"]["YoloArmor"]["ClassNum"];
    bboxes_num = (*param)["Model"]["YoloArmor"]["BboxesNum"];
    confidence_thresh = (*param)["Model"]["YoloArmor"]["ConfThresh"];
    nms_thresh = (*param)["Model"]["YoloArmor"]["NMSThresh"];
    yolo_struct_size = sizeof(float) * static_cast<size_t>(class_num + 5);
    if (access(engine_file.c_str(), F_OK) == 0) {
        if (!rm::initTrtEngine(engine_file, &armor_context_)) exit(-1);
    } else if (access(onnx_file.c_str(), F_OK) == 0){
        if (!rm::initTrtOnnx(onnx_file, engine_file, &armor_context_, 1U)) exit(-1);
    } else {
        rm::message("No model file found!", rm::MSG_ERROR);
        exit(-1);
    }
    rm::mallocYoloDetectBuffer(
        &armor_input_device_buffer_, 
        &armor_output_device_buffer_, 
        &armor_output_host_buffer_, 
        infer_width, 
        infer_height, 
        yolo_struct_size,
        bboxes_num);
}