#include "data_manager/param.h"
#include "data_manager/base.h"

int main(int argc, char* argv[]) {
    auto param = Param::get_instance();

    // 启动前场相机
    while(true) if(init_camera()) break;

    // 启动
    // while(true) if(init_camera()) break;

    // 启动
    // while(true) if(init_camera()) break;

    return 0;
}

