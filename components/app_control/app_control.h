/*
 * @Author: fzh
 * @Date: 2026-07-14 18:19:35
 * @LastEditors: fzh 
 * @LastEditTime: 2026-07-14 18:19:35
 */
#ifndef APP_CONTROL_H_
#define APP_CONTROL_H_
#include <stdio.h>
#include <stdlib.h>
#include "common.h"

/* 原接口无法指定传感器；新增分侧及目标掩码，支持左、右或双侧控制。 */
#define BC_SENSOR_SIDE_LEFT  0U
#define BC_SENSOR_SIDE_RIGHT 1U
#define BC_SENSOR_TARGET_LEFT  0x01U
#define BC_SENSOR_TARGET_RIGHT 0x02U
#define BC_SENSOR_TARGET_BOTH  (BC_SENSOR_TARGET_LEFT | BC_SENSOR_TARGET_RIGHT)

int set_bc_side(uint8_t side, const char *cmd, char *response,
                size_t response_size, uint16_t timeout_ms);
void app_control_server(void);
int set_bc(uint32_t time_stamp, char *value, uint8_t switch_return, uint8_t switch_to_aliyun, char *return_value, uint16_t time_ms);
uint8_t get_sleep_up_flag(void);
void set_sleep_up_flag(uint8_t data);
void set_ota_now_flag(uint8_t data);
uint8_t get_ota_now_flag(void);
void set_report_cli(uint8_t data1,uint8_t data2);
/* 原报告请求默认发往固定侧；新增 target 参数按请求 ID 选择目标。 */
void set_report_cli_target(uint8_t data1, uint8_t data2, uint8_t target);
void set_cli_report_name(const char *data, size_t len);
uint8_t get_devic_id_flag(void);
void sensor_reboot_config(void);
/** 传感器 OTA 预留：内部转 su3_sensor_ota，当前返回 -1 */
int sensor_ota_bc(char *value);
int mqtt_send_data(uint32_t time_stamp, char *value, uint8_t switch_return, uint8_t switch_to_aliyun, char *return_value, uint16_t time_ms);
void check_report_and_up_to_aliyun(void);   //xinzeng
void set_mode_flag_config(uint8_t data);
uint8_t get_mode_flag_config(void);
extern bool mqtt_send_mutex;
int mqtt_ble_data_parser_cb(uint8_t *data);
void check_stack_space(void);
#endif
