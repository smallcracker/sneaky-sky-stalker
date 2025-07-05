/**
 * @file siyi_zr10_protocol.h
 * @brief SIYI ZR10云台相机通信协议头文件
 * @copyright Copyright 2025 SIYI 思翼科技All Rights Reserved.
 */

#ifndef SIYI_ZR10_PROTOCOL_H
#define SIYI_ZR10_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 协议包头定义
 */
#define SIYI_PROTOCOL_HEAD 0x6655  // 低字节在前

#define MAX_BAG_LENGTH 64  // 数据包最大长度

/**
 * @brief 命令ID定义
 */
typedef enum {
    CMD_TCP_HEARTBEAT              = 0x00,  // TCP心跳
    CMD_GET_FIRMWARE_VERSION       = 0x01,  // 请求云台相机固件版本号
    CMD_GET_HARDWARE_ID            = 0x02,  // 请求云台相机硬件ID
    CMD_AUTO_FOCUS                 = 0x04,  // 自动对焦
    CMD_MANUAL_ZOOM_AUTO_FOCUS     = 0x05,  // 手动变倍自动对焦
    CMD_MANUAL_FOCUS               = 0x06,  // 手动对焦
    CMD_GIMBAL_ROTATION            = 0x07,  // 云台转向
    CMD_GIMBAL_CENTER              = 0x08,  // 一键回中
    CMD_GET_GIMBAL_CONFIG          = 0x0A,  // 请求云台配置信息
    CMD_FUNCTION_FEEDBACK          = 0x0B,  // 回传功能反馈信息
    CMD_PHOTO_VIDEO_CONTROL        = 0x0C,  // 拍照、录像等
    CMD_CONTROL_ANGLE              = 0x0E,  // 发送控制角度到云台
    CMD_ABSOLUTE_ZOOM              = 0x0F,  // 绝对变倍自动对焦
    CMD_GET_MAX_ZOOM               = 0x16,  // 请求当前状态最大变倍值
    CMD_GET_CURRENT_ZOOM           = 0x18,  // 请求当前变倍值
    CMD_GET_GIMBAL_MODE            = 0x19,  // 请求云台相机当前工作模式
    CMD_REQUEST_GIMBAL_DATA_STREAM = 0x25,  // 请求云台发送数据流
} SiyiCmdId;

/**
 *  @brief 云台数据包
 */
typedef struct{
    uint16_t STX;
    uint8_t  CTRL;      // 控制字段
    uint16_t data_len;  // 数据域字节长度
    uint16_t seq;       // 帧的序列
    uint8_t  cmd_id;    // 命令ID
    uint8_t  data[MAX_BAG_LENGTH];    // 数据域
    uint16_t CRC16;     // CRC16校验
} SiyiProtocolPacket;

/**
 * @brief 云台工作模式
 */
typedef enum {
    GIMBAL_MODE_LOCK     = 0x00,  // 锁定模式
    GIMBAL_MODE_FOLLOW   = 0x01,  // 跟随模式
    GIMBAL_MODE_FPV      = 0x02,  // FPV模式
} GimbalMode;

/**
 * @brief 功能类型
 */
typedef enum {
    FUNC_TAKE_PHOTO          = 0x00,  // 拍照
    FUNC_RECORD_VIDEO        = 0x02,  // 录像
    FUNC_MODE_LOCK           = 0x03,  // 运动模式: 锁定模式
    FUNC_MODE_FOLLOW         = 0x04,  // 运动模式: 跟随模式
    FUNC_MODE_FPV            = 0x05,  // 运动模式: FPV模式
} FunctionType;

/**
 * @brief 数据流类型
 */
typedef enum {
    DATA_STREAM_ATTITUDE = 0x01,  // 姿态数据
} DataStreamType;

/**
 * @brief 数据流频率
 */
typedef enum {
    DATA_FREQ_STOP = 0x00,  // 关闭发送
    DATA_FREQ_2HZ  = 0x01,  // 2Hz
    DATA_FREQ_4HZ  = 0x02,  // 4Hz
    DATA_FREQ_5HZ  = 0x03,  // 5Hz
    DATA_FREQ_10HZ = 0x04,  // 10Hz
    DATA_FREQ_20HZ = 0x05,  // 20Hz
    DATA_FREQ_50HZ = 0x06,  // 50Hz
    DATA_FREQ_100HZ= 0x07,  // 100Hz
} DataFrequency;

/**
 * @brief 协议包头结构体
 */
typedef struct {
    uint16_t stx;       // 起始标志 0x6655 (低字节在前)
    uint8_t  ctrl;      // 控制字段 0:need_ack 1:ack_pack
    uint16_t data_len;  // 数据域字节长度 (低字节在前)
    uint16_t seq;       // 帧的序列 (低字节在前)
    uint8_t  cmd_id;    // 命令ID
} __attribute__((packed)) SiyiProtocolHeader;

/**
 * @brief 固件版本号响应结构体
 */
typedef struct {
    uint32_t code_board_ver;      // 相机固件版本号
    uint32_t gimbal_firmware_ver; // 云台固件版本号
    uint32_t zoom_firmware_ver;   // 变焦固件版本号
} __attribute__((packed)) FirmwareVersionResponse;

/**
 * @brief 硬件ID响应结构体
 */
typedef struct {
    uint8_t hardware_id[12];  // 硬件ID字符串(10位数)
} __attribute__((packed)) HardwareIdResponse;

/**
 * @brief 云台工作模式响应结构体
 */
typedef struct {
    uint8_t gimbal_mode;  // 云台工作模式
} __attribute__((packed)) GimbalModeResponse;

/**
 * @brief 自动对焦请求结构体
 */
typedef struct {
    uint8_t  auto_focus;  // 1:启动一次自动对焦
    uint16_t touch_x;     // x坐标
    uint16_t touch_y;     // y坐标
} __attribute__((packed)) AutoFocusRequest;

/**
 * @brief 通用状态响应结构体
 */
typedef struct {
    uint8_t sta;  // 1:设置成功 0:设置出错
} __attribute__((packed)) StatusResponse;

/**
 * @brief 手动变倍自动对焦请求结构体
 */
typedef struct {
    int8_t zoom;  // 1:放大 0:停止缩放 -1:缩小
} __attribute__((packed)) ManualZoomRequest;

/**
 * @brief 变焦倍数响应结构体
 */
typedef struct {
    uint16_t zoom_multiple;  // 当前变焦倍数 (zoom_multiple/10倍)
} __attribute__((packed)) ZoomMultipleResponse;

/**
 * @brief 绝对变倍自动对焦请求结构体
 */
typedef struct {
    uint8_t absolute_movement_int;    // 指定倍数的整数部分(0x1~0x1E)
    uint8_t absolute_movement_float;  // 指定倍数的小数部分(0x0~0x9)
} __attribute__((packed)) AbsoluteZoomRequest;

/**
 * @brief 绝对变倍自动对焦响应结构体
 */
typedef struct {
    uint8_t absolute_movement_ask;  // 成功返回1
} __attribute__((packed)) AbsoluteZoomResponse;

/**
 * @brief 最大变倍值响应结构体
 */
typedef struct {
    uint8_t zoom_max_int;    // 最大变倍整数
    uint8_t zoom_max_float;  // 最大变倍小数
} __attribute__((packed)) MaxZoomResponse;

/**
 * @brief 当前变倍值响应结构体
 */
typedef struct {
    uint8_t zoom_int;    // 当前变倍整数
    uint8_t zoom_float;  // 当前变倍小数
} __attribute__((packed)) CurrentZoomResponse;

/**
 * @brief 手动对焦请求结构体
 */
typedef struct {
    int8_t focus;  // 1:远景 0:停止对焦 -1:近景
} __attribute__((packed)) ManualFocusRequest;

/**
 * @brief 云台转向请求结构体
 */
typedef struct {
    int8_t turn_yaw;    // -100~0~100
    int8_t turn_pitch;  // -100~0~100
} __attribute__((packed)) GimbalRotationRequest;

/**
 * @brief 一键回中请求结构体
 */
typedef struct {
    uint8_t center_pos;  // 1:触发回中
} __attribute__((packed)) GimbalCenterRequest;

/**
 * @brief 云台配置信息响应结构体
 */
typedef struct {
    uint8_t reserved1;            // 保留
    uint8_t reserved2;            // 保留
    uint8_t record_sta;           // 录像状态
    uint8_t gimbal_motion_mode;   // 云台运动模式
    uint8_t gimbal_mounting_dir;  // 云台安装方向
    uint8_t video_hdmi_or_cvbs;   // 视频输出状态
} __attribute__((packed)) GimbalConfigResponse;

/**
 * @brief 功能反馈信息响应结构体
 */
typedef struct {
    uint8_t info_type;  // 功能反馈类型
} __attribute__((packed)) FunctionFeedbackResponse;

/**
 * @brief 拍照、录像等请求结构体
 */
typedef struct {
    uint8_t func_type;  // 功能类型
} __attribute__((packed)) FunctionControlRequest;

/**
 * @brief 控制角度请求结构体
 */
typedef struct {
    int16_t yaw;    // 目标偏航角度
    int16_t pitch;  // 目标俯仰角度
} __attribute__((packed)) ControlAngleRequest;

/**
 * @brief 角度响应结构体
 */
typedef struct {
    int16_t yaw;    // 当前偏航角度
    int16_t pitch;  // 当前俯仰角度
    int16_t roll;   // 当前横滚角度
} __attribute__((packed)) AngleResponse;

/**
 * @brief 姿态数据响应结构体
 */
typedef struct {
    int16_t yaw;            // 转向角度
    int16_t pitch;          // 俯仰角度
    int16_t roll;           // 横滚角度
    int16_t yaw_velocity;   // 转向角速度
    int16_t pitch_velocity; // 俯仰角速度
    int16_t roll_velocity;  // 横滚角速度
} __attribute__((packed)) AttitudeDataResponse;

/**
 * @brief 请求云台发送数据流请求结构体
 */
typedef struct {
    uint8_t data_type;  // 数据类型
    uint8_t data_freq;  // 数据频率
} __attribute__((packed)) RequestDataStreamRequest;

/**
 * @brief 请求云台发送数据流响应结构体
 */
typedef struct {
    uint8_t data_type;  // 数据类型
} __attribute__((packed)) RequestDataStreamResponse;

// 计算CRC16校验值
uint16_t CRC16_cal(uint8_t *data, uint32_t len);

// 封装协议数据包
uint16_t siyi_pack_data(uint8_t cmd_id, bool need_ack, const uint8_t *data, uint16_t data_len, 
                        uint16_t seq, uint8_t *bag, uint16_t max_bag_len);

// 解析协议数据包
bool siyi_unpack_data(const uint8_t *bag, uint16_t bag_len, uint8_t *cmd_id, 
                     bool *is_ack, uint16_t *seq, uint8_t *data, uint16_t *data_len);

// 封装请求云台相机固件版本号命令
uint16_t siyi_pack_get_firmware_version(uint8_t *bag, uint16_t max_bag_len, uint16_t seq);

// 解析固件版本号响应
bool siyi_unpack_firmware_version_response(const uint8_t *bag, uint16_t bag_len, FirmwareVersionResponse *response);

// 封装请求云台相机硬件ID命令
uint16_t siyi_pack_get_hardware_id(uint8_t *bag, uint16_t max_bag_len, uint16_t seq);

// 解析硬件ID响应
bool siyi_unpack_hardware_id_response(const uint8_t *bag, uint16_t bag_len, HardwareIdResponse *response);

// 封装请求云台相机当前工作模式命令
uint16_t siyi_pack_get_gimbal_mode(uint8_t *bag, uint16_t max_bag_len, uint16_t seq);

// 解析云台工作模式响应
bool siyi_unpack_gimbal_mode_response(const uint8_t *bag, uint16_t bag_len, GimbalModeResponse *response);

// 封装自动对焦命令
uint16_t siyi_pack_auto_focus(uint8_t *bag, uint16_t max_bag_len, uint8_t auto_focus, 
                              uint16_t touch_x, uint16_t touch_y, uint16_t seq);

// 解析通用状态响应
bool siyi_unpack_status_response(const uint8_t *bag, uint16_t bag_len, StatusResponse *response);

// 封装手动变倍自动对焦命令
uint16_t siyi_pack_manual_zoom(uint8_t *bag, uint16_t max_bag_len, int8_t zoom, uint16_t seq);

// 解析变焦倍数响应
bool siyi_unpack_zoom_multiple_response(const uint8_t *bag, uint16_t bag_len, ZoomMultipleResponse *response);

// 封装绝对变倍自动对焦命令
uint16_t siyi_pack_absolute_zoom(uint8_t *bag, uint16_t max_bag_len, uint8_t int_part, 
                                 uint8_t float_part, uint16_t seq);

// 解析绝对变倍自动对焦响应
bool siyi_unpack_absolute_zoom_response(const uint8_t *bag, uint16_t bag_len, AbsoluteZoomResponse *response);

// 封装请求当前状态最大变倍值命令
uint16_t siyi_pack_get_max_zoom(uint8_t *bag, uint16_t max_bag_len, uint16_t seq);

// 解析最大变倍值响应
bool siyi_unpack_max_zoom_response(const uint8_t *bag, uint16_t bag_len, MaxZoomResponse *response);

// 封装请求当前变倍值命令
uint16_t siyi_pack_get_current_zoom(uint8_t *bag, uint16_t max_bag_len, uint16_t seq);

// 解析当前变倍值响应
bool siyi_unpack_current_zoom_response(const uint8_t *bag, uint16_t bag_len, CurrentZoomResponse *response);

// 封装手动对焦命令
uint16_t siyi_pack_manual_focus(uint8_t *bag, uint16_t max_bag_len, int8_t focus, uint16_t seq);

// 封装云台转向命令
uint16_t siyi_pack_gimbal_rotation(uint8_t *bag, uint16_t max_bag_len, int8_t turn_yaw, 
                                   int8_t turn_pitch, uint16_t seq);

// 封装一键回中命令
uint16_t siyi_pack_gimbal_center(uint8_t *bag, uint16_t max_bag_len, uint16_t seq);

// 封装请求云台配置信息命令
uint16_t siyi_pack_get_gimbal_config(uint8_t *bag, uint16_t max_bag_len, uint16_t seq);

// 解析云台配置信息响应
bool siyi_unpack_gimbal_config_response(const uint8_t *bag, uint16_t bag_len, GimbalConfigResponse *response);

// 解析功能反馈信息响应
bool siyi_unpack_function_feedback_response(const uint8_t *bag, uint16_t bag_len, FunctionFeedbackResponse *response);

// 封装拍照、录像等命令
uint16_t siyi_pack_function_control(uint8_t *bag, uint16_t max_bag_len, uint8_t func_type, uint16_t seq);

// 封装发送控制角度到云台命令
uint16_t siyi_pack_control_angle(uint8_t *bag, uint16_t max_bag_len, int16_t yaw, int16_t pitch, uint16_t seq);

// 解析角度响应
bool siyi_unpack_angle_response(const uint8_t *bag, uint16_t bag_len, AngleResponse *response);

// 解析姿态数据响应
bool siyi_unpack_attitude_data_response(const uint8_t *bag, uint16_t bag_len, AttitudeDataResponse *response);

// 封装请求云台发送数据流命令
uint16_t siyi_pack_request_data_stream(uint8_t *bag, uint16_t max_bag_len, uint8_t data_type, 
                                       uint8_t data_freq, uint16_t seq);

// 解析请求云台发送数据流响应
bool siyi_unpack_request_data_stream_response(const uint8_t *bag, uint16_t bag_len, RequestDataStreamResponse *response);

#ifdef __cplusplus
}
#endif

#endif /* SIYI_ZR10_PROTOCOL_H */