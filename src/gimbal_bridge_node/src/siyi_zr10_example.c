/**
 * @file siyi_zr10_example.c
 * @brief SIYI ZR10云台相机通信协议示例程序
 * @copyright Copyright 2025 SIYI 思翼科技All Rights Reserved.
 */

#include "siyi_zr10_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 定义bag大小
#define BUFFER_SIZE 256

/**
 * @brief 打印十六进制数据
 * @param data 数据bag
 * @param len 数据长度
 */
void print_hex_data(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

/**
 * @brief 示例：请求云台相机固件版本号
 */
void example_get_firmware_version(void) {
    uint8_t tx_buffer[BUFFER_SIZE];
    uint8_t rx_buffer[BUFFER_SIZE] = {0x55, 0x66, 0x02, 0x0C, 0x00, 0x00, 0x00, 0x01, 
                                     0x03, 0x02, 0x03, 0x6E, 0x03, 0x02, 0x03, 0x6E, 
                                     0x03, 0x02, 0x03, 0x6E, 0x12, 0x34}; // 模拟接收数据
    uint16_t tx_len;
    uint8_t cmd_id;
    bool is_ack;
    uint16_t seq;
    uint8_t data[BUFFER_SIZE];
    uint16_t data_len;
    FirmwareVersionResponse response;
    
    printf("\n=== 示例：请求云台相机固件版本号 ===\n");
    
    // 封装请求云台相机固件版本号命令
    tx_len = siyi_pack_get_firmware_version(tx_buffer, BUFFER_SIZE, 0);
    if (tx_len == 0) {
        printf("封装请求云台相机固件版本号命令失败\n");
        return;
    }
    
    printf("发送数据：\n");
    print_hex_data(tx_buffer, tx_len);
    
    // 模拟发送数据
    printf("模拟发送数据...\n");
    
    // 模拟接收数据
    printf("模拟接收数据：\n");
    print_hex_data(rx_buffer, sizeof(rx_buffer));
    
    // 解析接收数据
    if (!siyi_unpack_data(rx_buffer, sizeof(rx_buffer), &cmd_id, &is_ack, &seq, data, &data_len)) {
        printf("解析接收数据失败\n");
        return;
    }
    
    printf("解析接收数据成功：\n");
    printf("命令ID：0x%02X\n", cmd_id);
    printf("是否为应答包：%s\n", is_ack ? "是" : "否");
    printf("序列号：%u\n", seq);
    printf("数据长度：%u\n", data_len);
    
    // 解析固件版本号响应
    if (!siyi_unpack_firmware_version_response(data, data_len, &response)) {
        printf("解析固件版本号响应失败\n");
        return;
    }
    
    printf("解析固件版本号响应成功：\n");
    printf("相机固件版本号：v%d.%d.%d\n", 
           (response.code_board_ver >> 16) & 0xFF,
           (response.code_board_ver >> 8) & 0xFF,
           response.code_board_ver & 0xFF);
    printf("云台固件版本号：v%d.%d.%d\n", 
           (response.gimbal_firmware_ver >> 16) & 0xFF,
           (response.gimbal_firmware_ver >> 8) & 0xFF,
           response.gimbal_firmware_ver & 0xFF);
    printf("变焦固件版本号：v%d.%d.%d\n", 
           (response.zoom_firmware_ver >> 16) & 0xFF,
           (response.zoom_firmware_ver >> 8) & 0xFF,
           response.zoom_firmware_ver & 0xFF);
}

/**
 * @brief 示例：手动变倍自动对焦
 */
void example_manual_zoom(void) {
    uint8_t tx_buffer[BUFFER_SIZE];
    uint8_t rx_buffer[BUFFER_SIZE] = {0x55, 0x66, 0x02, 0x02, 0x00, 0x00, 0x00, 0x05, 
                                     0x64, 0x00, 0x12, 0x34}; // 模拟接收数据
    uint16_t tx_len;
    uint8_t cmd_id;
    bool is_ack;
    uint16_t seq;
    uint8_t data[BUFFER_SIZE];
    uint16_t data_len;
    ZoomMultipleResponse response;
    
    printf("\n=== 示例：手动变倍自动对焦 ===\n");
    
    // 封装手动变倍自动对焦命令（放大）
    tx_len = siyi_pack_manual_zoom(tx_buffer, BUFFER_SIZE, 1, 0);
    if (tx_len == 0) {
        printf("封装手动变倍自动对焦命令失败\n");
        return;
    }
    
    printf("发送数据（放大）：\n");
    print_hex_data(tx_buffer, tx_len);
    
    // 模拟发送数据
    printf("模拟发送数据...\n");
    
    // 模拟接收数据
    printf("模拟接收数据：\n");
    print_hex_data(rx_buffer, sizeof(rx_buffer));
    
    // 解析接收数据
    if (!siyi_unpack_data(rx_buffer, sizeof(rx_buffer), &cmd_id, &is_ack, &seq, data, &data_len)) {
        printf("解析接收数据失败\n");
        return;
    }
    
    printf("解析接收数据成功：\n");
    printf("命令ID：0x%02X\n", cmd_id);
    printf("是否为应答包：%s\n", is_ack ? "是" : "否");
    printf("序列号：%u\n", seq);
    printf("数据长度：%u\n", data_len);
    
    // 解析变焦倍数响应
    if (!siyi_unpack_zoom_multiple_response(data, data_len, &response)) {
        printf("解析变焦倍数响应失败\n");
        return;
    }
    
    printf("解析变焦倍数响应成功：\n");
    printf("当前变焦倍数：%.1f倍\n", response.zoom_multiple / 10.0f);
    
    // 封装手动变倍自动对焦命令（停止缩放）
    tx_len = siyi_pack_manual_zoom(tx_buffer, BUFFER_SIZE, 0, 1);
    if (tx_len == 0) {
        printf("封装手动变倍自动对焦命令失败\n");
        return;
    }
    
    printf("发送数据（停止缩放）：\n");
    print_hex_data(tx_buffer, tx_len);
}

/**
 * @brief 示例：云台转向
 */
void example_gimbal_rotation(void) {
    uint8_t tx_buffer[BUFFER_SIZE];
    uint8_t rx_buffer[BUFFER_SIZE] = {0x55, 0x66, 0x02, 0x01, 0x00, 0x00, 0x00, 0x07, 
                                     0x01, 0x12, 0x34}; // 模拟接收数据
    uint16_t tx_len;
    uint8_t cmd_id;
    bool is_ack;
    uint16_t seq;
    uint8_t data[BUFFER_SIZE];
    uint16_t data_len;
    StatusResponse response;
    
    printf("\n=== 示例：云台转向 ===\n");
    
    // 封装云台转向命令（向右转）
    tx_len = siyi_pack_gimbal_rotation(tx_buffer, BUFFER_SIZE, 50, 0, 0);
    if (tx_len == 0) {
        printf("封装云台转向命令失败\n");
        return;
    }
    
    printf("发送数据（向右转）：\n");
    print_hex_data(tx_buffer, tx_len);
    
    // 模拟发送数据
    printf("模拟发送数据...\n");
    
    // 模拟接收数据
    printf("模拟接收数据：\n");
    print_hex_data(rx_buffer, sizeof(rx_buffer));
    
    // 解析接收数据
    if (!siyi_unpack_data(rx_buffer, sizeof(rx_buffer), &cmd_id, &is_ack, &seq, data, &data_len)) {
        printf("解析接收数据失败\n");
        return;
    }
    
    printf("解析接收数据成功：\n");
    printf("命令ID：0x%02X\n", cmd_id);
    printf("是否为应答包：%s\n", is_ack ? "是" : "否");
    printf("序列号：%u\n", seq);
    printf("数据长度：%u\n", data_len);
    
    // 解析通用状态响应
    if (!siyi_unpack_status_response(data, data_len, &response)) {
        printf("解析通用状态响应失败\n");
        return;
    }
    
    printf("解析通用状态响应成功：\n");
    printf("状态：%s\n", response.sta ? "设置成功" : "设置出错");
    
    // 封装云台转向命令（停止转向）
    tx_len = siyi_pack_gimbal_rotation(tx_buffer, BUFFER_SIZE, 0, 0, 1);
    if (tx_len == 0) {
        printf("封装云台转向命令失败\n");
        return;
    }
    
    printf("发送数据（停止转向）：\n");
    print_hex_data(tx_buffer, tx_len);
}

/**
 * @brief 示例：发送控制角度到云台
 */
void example_control_angle(void) {
    uint8_t tx_buffer[BUFFER_SIZE];
    uint8_t rx_buffer[BUFFER_SIZE] = {0x55, 0x66, 0x02, 0x06, 0x00, 0x00, 0x00, 0x0E, 
                                     0x2C, 0x01, 0xF4, 0xFE, 0x00, 0x00, 0x12, 0x34}; // 模拟接收数据
    uint16_t tx_len;
    uint8_t cmd_id;
    bool is_ack;
    uint16_t seq;
    uint8_t data[BUFFER_SIZE];
    uint16_t data_len;
    AngleResponse response;
    
    printf("\n=== 示例：发送控制角度到云台 ===\n");
    
    // 封装发送控制角度到云台命令（偏航30度，俯仰-10度）
    tx_len = siyi_pack_control_angle(tx_buffer, BUFFER_SIZE, 300, -100, 0);
    if (tx_len == 0) {
        printf("封装发送控制角度到云台命令失败\n");
        return;
    }
    
    printf("发送数据（偏航30度，俯仰-10度）：\n");
    print_hex_data(tx_buffer, tx_len);
    
    // 模拟发送数据
    printf("模拟发送数据...\n");
    
    // 模拟接收数据
    printf("模拟接收数据：\n");
    print_hex_data(rx_buffer, sizeof(rx_buffer));
    
    // 解析接收数据
    if (!siyi_unpack_data(rx_buffer, sizeof(rx_buffer), &cmd_id, &is_ack, &seq, data, &data_len)) {
        printf("解析接收数据失败\n");
        return;
    }
    
    printf("解析接收数据成功：\n");
    printf("命令ID：0x%02X\n", cmd_id);
    printf("是否为应答包：%s\n", is_ack ? "是" : "否");
    printf("序列号：%u\n", seq);
    printf("数据长度：%u\n", data_len);
    
    // 解析角度响应
    if (!siyi_unpack_angle_response(data, data_len, &response)) {
        printf("解析角度响应失败\n");
        return;
    }
    
    printf("解析角度响应成功：\n");
    printf("当前偏航角度：%.1f度\n", response.yaw / 10.0f);
    printf("当前俯仰角度：%.1f度\n", response.pitch / 10.0f);
    printf("当前横滚角度：%.1f度\n", response.roll / 10.0f);
}

/**
 * @brief 主函数
 */
int main(void) {
    printf("SIYI ZR10云台相机通信协议示例程序\n");
    printf("================================\n");
    
    // 示例：请求云台相机固件版本号
    example_get_firmware_version();
    
    // 示例：手动变倍自动对焦
    example_manual_zoom();
    
    // 示例：云台转向
    example_gimbal_rotation();
    
    // 示例：发送控制角度到云台
    example_control_angle();
    
    return 0;
}