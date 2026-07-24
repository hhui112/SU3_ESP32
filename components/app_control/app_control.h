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

void app_control_server(void);
int set_bc(uint32_t time_stamp, char *value, uint8_t switch_return, uint8_t switch_to_aliyun, char *return_value, uint16_t time_ms);
/** MQTT id 是否属于本机（仅侧别码 00/03/06 可不同） */
bool su3_mqtt_id_belong(const char *mqtt_id);
/** sensorCli：MQTT 回调只入队；cli_worker 内执行并 cli/put */
void su3_mqtt_handle_sensor_cli(const char *mqtt_id, const char *cmd);
/** SU3 协议栈是否已启动（可下发 CLI） */
bool su3_stack_ready(void);
void set_ota_now_flag(uint8_t data);
uint8_t get_ota_now_flag(void);
/** 请求睡眠报告同步（入队 cli_worker；周期任务 / MQTT 重连 / list updata 调用） */
void su3_report_sync_request(void);
uint8_t get_devic_id_flag(void); /* 兼容：任一 setup_done */
void sensor_reboot_config(void);
int sensor_ota_bc(char *value);
extern bool mqtt_send_mutex;
int mqtt_ble_data_parser_cb(uint8_t *data);
void check_stack_space(void);
#endif
