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
        send_map();
        send_sentry();
    }

}

void send_cmd(){
    packet_robot_interaction_data_t packet;
    packet.header.SOF = 0xA5;
    packet.header.data_length = 7;
    packet.header.seq = 0;
    append_crc8_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet.header));
    robot_interaction_data_t robot_interaction_data;
    robot_interaction_data.sender_id = Data::self_color == rm::ArmorColor::ARMOR_COLOR_RED? 9 : 109;
    robot_interaction_data.receiver_id = 0x8080;
    robot_interaction_data.data_cmd_id = 0x0121;
    robot_interaction_data.user_data = Data::radar_cmd;
    memcpy(&packet.data, &robot_interaction_data, sizeof(robot_interaction_data_t));
    append_crc16_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet));
    Data::ser.write((uint8_t *)&packet, sizeof(packet));
    memset(&Data::robot_interaction_data, 0, sizeof(robot_interaction_data_t));
}

void send_sentry(){
    packet_robot_interaction_data_t packet;
    packet.header.SOF = 0xA5;
    packet.header.data_length = 7;
    packet.header.seq = 0;
    append_crc8_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet.header));
    robot_interaction_data_t robot_interaction_data;
    robot_interaction_data.sender_id = Data::self_color == rm::ArmorColor::ARMOR_COLOR_RED? 9 : 109;
    robot_interaction_data.receiver_id = Data::self_color == rm::ArmorColor::ARMOR_COLOR_RED? 7 : 107;
    robot_interaction_data.data_cmd_id = 0x0200;
    if(Data::map_robot_data.hero_position_x != 0 && Data::map_robot_data.hero_position_y != 0){
        int is_hero_base;
        if(Data::self_color == rm::ArmorColor::ARMOR_COLOR_RED)
            is_hero_base = Data::map_robot_data.hero_position_x <= 850;
        else
            is_hero_base = Data::map_robot_data.hero_position_x >= 2000;
        robot_interaction_data.user_data.radar_cmd = is_hero_base? 0x01 : 0x00;
    }
    else
        robot_interaction_data.user_data.radar_cmd = 0x00;
    memcpy(&packet.data, &robot_interaction_data, sizeof(robot_interaction_data_t));
    append_crc16_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet));
    Data::ser.write((uint8_t *)&packet, sizeof(packet));
    memset(&Data::robot_interaction_data, 0, sizeof(robot_interaction_data_t));
}


void send_map(){
    packet_map_robot_data_t packet;
    packet.header.SOF = 0xA5;
    packet.header.data_length = 24;
    packet.header.seq = 0;
    append_crc8_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet.header));
    // 循环Data::enemy_info，将信息写入Data::map_robot_data
    Data::map_robot_data.sentry_position_x = Data::enemy_info[0].pos.x;
    Data::map_robot_data.sentry_position_y = Data::enemy_info[0].pos.y;
    Data::map_robot_data.hero_position_x = Data::enemy_info[1].pos.x;
    Data::map_robot_data.hero_position_y = Data::enemy_info[1].pos.y;
    Data::map_robot_data.engineer_position_x = Data::enemy_info[2].pos.x;
    Data::map_robot_data.engineer_position_y = Data::enemy_info[2].pos.y;
    Data::map_robot_data.infantry_3_position_x = Data::enemy_info[3].pos.x;
    Data::map_robot_data.infantry_3_position_y = Data::enemy_info[3].pos.y;
    Data::map_robot_data.infantry_4_position_x = Data::enemy_info[4].pos.x;
    Data::map_robot_data.infantry_4_position_y = Data::enemy_info[4].pos.y;
    Data::map_robot_data.infantry_5_position_x = Data::enemy_info[5].pos.x;
    Data::map_robot_data.infantry_5_position_y = Data::enemy_info[5].pos.y;
    memcpy(&packet.data, &Data::map_robot_data, sizeof(map_robot_data_t));
    append_crc16_check_sum((uint8_t *)&packet.header.SOF, sizeof(packet));
    Data::ser.write((uint8_t *)&packet, sizeof(packet));
    memset(&Data::map_robot_data, 0, sizeof(map_robot_data_t));
}




// 数据处理
void data_process(uint8_t* data, int size){
    // 根据cmd_id确定包的类型
    // 比赛状态数据
    if(data[5] == 0x01 && data[6] == 0x00){
        memcpy(&Data::game_status, data + 7, sizeof(game_status_t));
        // std::cout << "stage_remain_time:" << int(Data::game_status.stage_remain_time) << std::endl;
        // std::cout << "game_progress:" << int(Data::game_status.game_progress) << std::endl;
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
        // 初始化给哨兵发信息的交互数据
        Data::robot_interaction_data.sender_id = int(robot_status.robot_id);
        Data::robot_interaction_data.receiver_id = int(robot_status.robot_id) - 2;
        Data::robot_interaction_data.data_cmd_id = 0x0200;
    }
    // 雷达标记进度数据
    else if(data[5] == 0x0C && data[6] == 0x02){
        memcpy(&Data::radar_mark_data, data + 7, sizeof(radar_mark_data_t));
        Data::enemy_info[0].is_debuff = Data::radar_mark_data.mark_sentry_progress >= 100;
        Data::enemy_info[1].is_debuff = Data::radar_mark_data.mark_hero_progress >= 100;
        Data::enemy_info[2].is_debuff = Data::radar_mark_data.mark_engineer_progress >= 100;
        Data::enemy_info[3].is_debuff = Data::radar_mark_data.mark_standard_3_progress >= 100;
        Data::enemy_info[4].is_debuff = Data::radar_mark_data.mark_standard_4_progress >= 100;
        Data::enemy_info[5].is_debuff = Data::radar_mark_data.mark_standard_5_progress >= 100;
    }
    // 雷达自主决策信息同步
    else if(data[5] == 0x0E && data[6] == 0x02){
        memcpy(&Data::radar_info, data + 7, sizeof(radar_info_t));
        // std::cout << "is_douing: " << int(Data::radar_info.is_double_ing) << std::endl;
        // std::cout << "is_have_chance: " << int(Data::radar_info.is_have_chance) << std::endl;
    }
    // 机器人血量数据
    else if(data[5] == 0x03 && data[6] == 0x00){
        game_robot_HP_t game_robot_HP;
        memcpy(&game_robot_HP, data + 7, sizeof(game_robot_HP_t));
        if(Data::self_color == rm::ArmorColor::ARMOR_COLOR_RED){
            Data::enemy_info[0].is_dehealth = game_robot_HP.blue_7_robot_HP < Data::game_robot_HP.blue_7_robot_HP;
            Data::enemy_info[1].is_dehealth = game_robot_HP.blue_1_robot_HP < Data::game_robot_HP.blue_1_robot_HP;
            Data::enemy_info[2].is_dehealth = game_robot_HP.blue_2_robot_HP < Data::game_robot_HP.blue_2_robot_HP;
            Data::enemy_info[3].is_dehealth = game_robot_HP.blue_3_robot_HP < Data::game_robot_HP.blue_3_robot_HP;
            Data::enemy_info[4].is_dehealth = game_robot_HP.blue_4_robot_HP < Data::game_robot_HP.blue_4_robot_HP;
            Data::enemy_info[5].is_dehealth = game_robot_HP.blue_5_robot_HP < Data::game_robot_HP.blue_5_robot_HP;
        }
        else{
            Data::enemy_info[0].is_dehealth = game_robot_HP.red_7_robot_HP < Data::game_robot_HP.red_7_robot_HP;
            Data::enemy_info[1].is_dehealth = game_robot_HP.red_1_robot_HP < Data::game_robot_HP.red_1_robot_HP;
            Data::enemy_info[2].is_dehealth = game_robot_HP.red_2_robot_HP < Data::game_robot_HP.red_2_robot_HP;
            Data::enemy_info[3].is_dehealth = game_robot_HP.red_3_robot_HP < Data::game_robot_HP.red_3_robot_HP;
            Data::enemy_info[4].is_dehealth = game_robot_HP.red_4_robot_HP < Data::game_robot_HP.red_4_robot_HP;
            Data::enemy_info[5].is_dehealth = game_robot_HP.red_5_robot_HP < Data::game_robot_HP.red_5_robot_HP;
        }
        Data::game_robot_HP = game_robot_HP;
    }
}
