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
    if(Data::ser.isOpen()){
        std::cout << "Serial Port initialized" << std::endl;
    }
    else{
        std::cout << "Serial Port failed to initialize" << std::endl;
    }

}


// 串口接收数据（单开一个线程循环)
void serial_port_recv(){

    int buffer_size = 0;

    while(true){
        uint8_t buffer[100];
        if (Data::ser.available() >= 10) {
            
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
                        // 处理数据
                        data_process(data, data_length);
                    }
                    i += data_length;
                }
                else
                    i++;
            }
        }
    }

}


// 串口发送数据
void serial_port_send(){

    // 注意控制发送频率
    while(true){
        // 睡眠200ms
        usleep(200000);
        send_info();
        send_map();
    }

}


void send_map(){
    packet_map_robot_data_t packet;
    packet.header.SOF = 0xA5;
    packet.header.data_length = 24;
    packet.header.seq = 0;
    append_crc8_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet.header));
    memcpy(&packet.data, &Data::map_robot_data, sizeof(map_robot_data_t));
    append_crc16_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet));
    Data::ser.write((uint8_t *)&packet, sizeof(packet));
}


void send_info(){
    radar_info_msgs packet;
    packet.header.SOF = 0xA5;
    packet.header.data_length = 1;
    packet.header.seq = 0;
    (&packet.cmd_id)[0] = 0x21;
    (&packet.cmd_id)[1] = 0x01;
    append_crc8_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet.header));
    memcpy(&packet.data, &Data::radar_cmd, sizeof(radar_cmd_t));
    append_crc16_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet));
    Data::ser.write((uint8_t *)&packet, sizeof(packet));
}




// 数据处理
void data_process(uint8_t* data, int size){
    // 根据cmd_id确定包的类型
    // 比赛状态数据
    if(data[5] == 0x01 && data[6] == 0x00){
        game_status_t game_status;
        memcpy(&game_status, data + 7, sizeof(game_status_t));
    }
    // 机器人性能体系数据(可以拿到本机器人ID，进而得到红蓝方信息)
    else if(data[5] == 0x01 && data[6] == 0x02){
        robot_status_t robot_status;
        memcpy(&robot_status, data + 7, sizeof(robot_status_t));
        if(robot_status.robot_id > 100){
            Data::self_color = rm::ArmorColor::ARMOR_COLOR_BLUE;
            Data::enemy_color = rm::ArmorColor::ARMOR_COLOR_RED;
        }
        else{
            Data::self_color = rm::ArmorColor::ARMOR_COLOR_RED;
            Data::enemy_color = rm::ArmorColor::ARMOR_COLOR_BLUE;
        }
        
    }
    // 雷达标记进度数据
    else if(data[5] == 0x0C && data[6] == 0x02){
        memcpy(&Data::radar_mark_data, data + 7, sizeof(radar_mark_data_t));
    }
    // 雷达自主决策信息同步
    else if(data[5] == 0x0E && data[6] == 0x02){
        memcpy(&Data::radar_info, data + 7, sizeof(radar_info_t));
    }
    // 机器人交互数据(等待启用)
    else if(data[5] == 0x01 && data[6] == 0x03){
        std::cout << "机器人交互数据" << std::endl;
    }
}
