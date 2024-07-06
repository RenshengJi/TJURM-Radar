#ifndef RM2024_MODEL_INIT_H_
#define RM2024_MODEL_INIT_H_

#include "data_manager/param.h"
#include "data_manager/base.h"
#include <openrm/cudatools.h>
#include <unistd.h>


class RGB_MODEL{
public:
    RGB_MODEL();
    static std::shared_ptr<RGB_MODEL> get_instance() {
        static std::shared_ptr<RGB_MODEL> instance(new RGB_MODEL());
        return instance;
    }
    std::shared_ptr<std::vector<rm::YoloRectWithCamera>> detect_armor(int camera_id);
    
private:
    float* armor_input_device_buffer_ = nullptr;
    float* armor_output_device_buffer_ = nullptr;
    float* armor_output_host_buffer_ = nullptr;
    cudaStream_t resize_stream_;
    cudaStream_t detect_stream_;
    nvinfer1::IExecutionContext* armor_context_;
    std::string onnx_file;
    std::string engine_file;
    int infer_width;
    int infer_height;
    int class_num;
    int bboxes_num;
    double confidence_thresh;
    double nms_thresh;
    size_t yolo_struct_size;
};




#endif