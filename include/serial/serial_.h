#ifndef RM2024_SERIAL_SERIAL__H_
#define RM2024_SERIAL_SERIAL__H_

#include "data_manager/param.h"
#include "data_manager/base.h"
#include <openrm/cudatools.h>
#include <unistd.h>


void serial_port_init();
void serial_port_recv();
void serial_port_send();
void data_process(uint8_t* data, int size);
void send_map();
void send_map_old();
void send_info();


// 以下是一大堆包的结构体
// 0x0001 比赛状态
struct game_status_t
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
}__attribute__((packed));

// 0x0201 机器人性能体系数据
struct robot_status_t
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
}__attribute__((packed));

// 0x020C 雷达标记进度数据
struct radar_mark_data_t
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
}__attribute__((packed));

// 0x020E 雷达自主决策信息同步
struct radar_info_t
{
    uint8_t is_have_chance : 2;
    uint8_t is_double_ing : 1;
}__attribute__((packed));

// 0x0301 机器人交互数据(暂无)

// 0x0305 选手端小地图接收雷达数据
struct map_robot_data_t
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
}__attribute__((packed));

// 帧头
struct frame_header // 1 + 2 + 1 + 1 = 5
{
    uint8_t SOF = 0xA5;      // 固定值
    uint16_t data_length = 10; // 数据长度
    uint8_t seq;             // 序列号
    uint8_t crc8;            // crc8校验
} __attribute__((packed));


///////////////
struct map_data
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} __attribute__((packed));
struct map_msg // 19
{
    frame_header header; // 5
    uint16_t cmd_id = 0x0305; // 2
    map_data data; // 10
    uint16_t crc16; // 2
} __attribute__((packed));
///////////////


// 0x0121 雷达自主决策指令
struct radar_cmd_t
{
    uint8_t radar_cmd ;
}__attribute__((packed));


// TODO: 大小端？
struct packet_map_robot_data_t
{
    frame_header header;
    uint16_t cmd_id = 0x0305;
    map_robot_data_t data;
    uint16_t crc16;
}__attribute__((packed));


struct radar_info_msgs {
    frame_header header;
    uint16_t cmd_id = 0x0121;
    radar_cmd_t data;
    uint16_t crc16;
} __attribute__((packed));






#endif