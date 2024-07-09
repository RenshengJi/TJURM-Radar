#include "data_manager/param.h"
#include "data_manager/base.h"
#include "serial/serial_.h"
#include <openrm/cudatools.h>
#include <unistd.h>
#include "serial/protocol.h"


// 初始化串口
void serial_port_init(){

    auto param = Param::get_instance();
    std::string serial_port = (*param)["serial"]["Port"];
    Data::ser.setPort(serial_port);
    Data::ser.setBaudrate(115200);

    Data::ser.setBytesize(serial::bytesize_t::eightbits); // 8 位数据位
    Data::ser.setStopbits(serial::stopbits_t::stopbits_one); //  1位停止位
    Data::ser.setFlowcontrol(serial::flowcontrol_t::flowcontrol_none); // 无硬件流控

    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    Data::ser.setTimeout(to);
    Data::ser.open();

}


// 串口接收数据（单开一个线程循环)
void serial_port_recv(){

    int buffer_size = 0;

    while(true){
        uint8_t buffer[100];
        if (Data::ser.available() >= 40) {
            
            size_t size = Data::ser.available();
            uint8_t receiveData[buffer_size + size];
            // 先将缓冲区数据读取出来
            for(int i = 0; i < buffer_size; i++){
                receiveData[i] = buffer[i];
            }
            Data::ser.read(receiveData + buffer_size, size);
            size = buffer_size + size;
            buffer_size = 0;
            for(int i = 0; i < size;){
                // 寻找帧头
                if(receiveData[i] == 0xA5){
                    if(i + 3 > size){
                        buffer_size = size - i;
                        for(int j = 0; j < buffer_size; j++){
                            buffer[j] = receiveData[i + j];
                        }
                        break;
                    }
                    int data_length = (receiveData[i + 2] << 8) | receiveData[i + 1] + 9;
                    if(i + data_length > size){
                        buffer_size = size - i;
                        for(int j = 0; j < buffer_size; j++){
                            buffer[j] = receiveData[i + j];
                        }
                        break;
                    }
                    uint8_t data[data_length];
                    for(int j = 0; j < data_length; j++){
                        data[j] = receiveData[i + j];
                    }
                    // crc校验
                    if(verify_crc8_check_sum(data, 5) && verify_crc16_check_sum(data, data_length)){
                        if(data_length == 22 ||  data_length == 25)
                            std::cout << data_length << std::endl;
                    }
                    i += data_length;
                }
                else
                    i++;
            }
        }
    }

}