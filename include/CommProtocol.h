#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <cstdint>

// 避障方向枚举
enum AvoidDirection : uint8_t {
    AVOID_NONE = 0,
    AVOID_LEFT = 1,
    AVOID_RIGHT = 2
};

// 控制状态枚举（仅定义，当前实现中不使用接收功能）
enum ControlState : uint8_t {
    STATE_TRACKING = 0,
    STATE_AVOIDING = 1,
    STATE_AVOID_COMPLETE = 2
};

// 运动状态枚举（仅定义，当前实现中不使用接收功能）
enum MotionState : uint8_t {
    MOTION_STOP = 0,
    MOTION_FORWARD = 1,
    MOTION_TURN_LEFT = 2,
    MOTION_TURN_RIGHT = 3
};

#pragma pack(push, 1)
// 视觉系统发送给电控系统的数据结构
struct VisionData {
    uint8_t header = 0xCD;         // 帧头
    float line_offset = 0.0f;      // 盲道偏差值（负：左，正：右；单位：像素或归一化）
    uint8_t line_found = 0;        // 是否检测到盲道：0=未检测到，1=检测到
    uint8_t obstacle_detected = 0; // 是否检测到障碍物：0=无，1=有
    uint8_t avoid_direction = 0;   // 避障方向：0=无，1=左，2=右（基于深度判断）
    float timestamp = 0.0f;        // 时间戳或帧编号
    uint8_t tail = 0xDC;           // 帧尾
};

// 电控系统发送给视觉系统的数据结构（仅定义，当前实现中不使用接收功能）
struct Stm32Data {
    uint8_t header = 0xCD;       // 帧头
    uint8_t control_state = 0;   // 控制状态：0=正常循迹，1=进入避障，2=已避障完成
    uint8_t motion_state = 0;    // 当前运动状态：0=停止，1=前进，2=左转，3=右转
    float timestamp = 0.0f;      // 时间戳，用于同步
    uint8_t tail = 0xDC;         // 帧尾
};
#pragma pack(pop)

#endif // COMM_PROTOCOL_H 