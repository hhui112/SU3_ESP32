/*
 * @Author: fzh
 * @Date: 2026-07-14 18:19:35
 * @LastEditors: fzh 
 * @LastEditTime: 2026-07-14 18:19:35
 */
#include "app_control.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include "cJSON.h"
#include "use_wifi.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include <string.h>
#include "use_wifi.h"
#include "esp_wifi.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "sntp.h"
#include <time.h>
#include "use_uart.h"
#include "driver/uart.h"
#include "pb_decode.h"
#include "keesoncloud.pb.h"
#include "qs_protobuf.h"
#include "mqtt_client.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "use_mfp.h"
#include "su3_client.h"

// ==================== 调试配置开关 ====================
// 量产时将下面的宏改为 0，调试时改为 1
#define ENABLE_DEBUG_DISPLAY    0   // 1=启用调试显示任务  0=关闭（量产）
#define ENABLE_PERFORMANCE_MON  0   // 1=启用性能监控任务  0=关闭（量产）
#define ENABLE_MFP_DETAIL_LOG   0   // 1=启用MFP详细日志  0=仅错误日志（量产）
// ======================================================

#define UP_RATIO_60S    12   // 60s上报一次需改为12

static const char *TAG = "control";
extern device_info_t *device_info;
extern esp_mqtt_client_handle_t client;
extern char user_5s_data_publish_topic[64];
extern char user_60s_data_publish_topic[64];
extern char user_sa_data_publish_topic[64];
extern char user_sleep_data_publish_topic[64];
// extern char user_cli_data_subscribe_topic[64];
extern char mc_cli_data_publish_topic[64];

extern qs_pb_msg_sensor_1min_info *user_60s_sensor_info;
extern qs_pb_msg_sensor_5sec_info *user_5s_sensor_info;
extern qs_pb_msg_sleep_apnea_info *user_sa_sensor_info;
time_t now;
static uint16_t human_year;
static uint8_t human_mon;
struct tm ti;
bool get_5s_flag = false;
bool mqtt_send_mutex = true;


char json_report_name[32] = {0};

static char device_id[12] = {0},device_version[32] = {0},set_rtc_flag = 0,devic_id_flag = 0;   //bc重启后将两个标志位置零，重新设置addr与rtc 
static char sleep_up_flag = 0,ota_now_flag = 0;//sleep_up睡眠报告上传进行中、esp32固件升级
static uint8_t set_mode_flag = 2;   // xinzeng:set_mode_flag 强制生成报告
static char report_cli_data[2]={0},cli_report_name[32]={0};
static snore_parameters_t snore_parameters_demo = {0};    //打鼾干预延时流程参数
static uint32_t snore_blocked_until_s = 0;                //闹钟指令后 120s 内禁止打鼾干预（开机秒数）

#define ECHO_TEST_TXD (19)
#define ECHO_TEST_RXD (25)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (2)
#define ECHO_UART_BAUD_RATE     (38400)
#define ECHO_TASK_STACK_SIZE    (4096)

#define BUF_SIZE (512)

/* SU2(UART1)：0=关 1=每 chunk 打印 ring 前后长度 2=另打本 chunk 十六进制(最多 SU2_UART_HEX_DUMP_MAX) */
#define DEBUG_SU2_UART_RX_LOG   1
#define SU2_UART_HEX_DUMP_MAX   64
/* MFP(UART2) 事件接收：1=每次 UART_DATA 打印 read 长度（流量大，默认 0） */
#define DEBUG_MFP_UART_RX_LOG   0

static QueueHandle_t uart2_queue;

union SyncCommunicationData_t g_Sync_TX;
union SyncCommunicationData_t g_Sync_RX;
union keys_t   g_keys;
g_system_flag_t g_system_flag;

// MFP解析缓冲区 - 独立于接收缓冲区,避免数据被清零
static uint8_t g_MFP_Parsed_Frame[256];      // 存储最近一次完整解析的帧
static uint16_t g_MFP_Parsed_Len = 0;        // 已解析帧的长度
static SemaphoreHandle_t g_MFP_Parsed_Mutex = NULL;  // 线程安全保护

void set_cli_report_name(char* data,char len)
{
    memset(cli_report_name, 0, 32);
    memcpy(cli_report_name,data,len);
}
static void report_cli_up(void)   //指令下发
{
    // return_value[1024] = {0};
    // char get_report_cmd1[]="list";
    char get_report_cmd2[12]={0};
    switch (report_cli_data[0])
    {
    // case 1:
    //     printf("get_report_cmd1");
    //     set_bc(device_info->utc.time_stamp, get_report_cmd1, 1, 1, return_value, 50);  //获取list列表
    //     printf("ble_test list %s",return_value);
    //     report_cli_data[0] = 0;
    //     break;
    case 2:   //睡眠报告上传指令
        sprintf(get_report_cmd2, "report %d",report_cli_data[1]);
        printf("get_report_cmd2 %s\n", get_report_cmd2);
        // sleep_up_flag = 1;
        memset(json_report_name, 0, 32);
        strcpy(json_report_name,cli_report_name);
        set_bc(device_info->utc.time_stamp, get_report_cmd2, 0, 0, NULL, 50);   //睡眠报告上传云端
        
        //vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
        sleep_up_flag = 0;
        report_cli_data[0] = 0;
        break;
  
    default:
        break;
    }
}
/*
    static void bochuang_test(uint8_t data1,uint8_t data2)
    {
        char return_value[1024] = {0};
        char get_report_cmd1[]="list";

        switch (data1)
        {
        case 1:
            printf("get_report_cmd1");
            set_bc(device_info->utc.time_stamp, get_report_cmd1, 1, 1, return_value, 50);  //获取list列表
            printf("ble_test list %s",return_value);
            data1 = 0;
            break;
        case 2:
            if(sleep_up_flag == 0)
            {
                sleep_up_flag = 1;
                report_cli_data[0]=data1;
                report_cli_data[1]=data2;
            }
            else
            {
                printf("sleep_uping now/n");
            }
            data1 = 0;
            break;
    
        default:
            break;
        }
    }
*/
//蓝牙数据解析
int ble_data_parser_cb(uint8_t *data)
{

#if BLE_TEST
if(data[0] == 0x11 && data[1]==0x22 && data[2]==0x33)
{
    //bochuang_test(data[3],data[4]);
}
else if (data[0] == 0x22 && data[1]==0x33 && data[2]==0x44)
{
    memset(device_info->wifi.one_key_config.ssid, 0, 32);
    memcpy(device_info->wifi.one_key_config.ssid, MY_WIFI_SSID, strlen(MY_WIFI_SSID));
    printf("ssid = %s \n",device_info->wifi.one_key_config.ssid);

    memset(device_info->wifi.one_key_config.passwd, 0, 64);
    memcpy(device_info->wifi.one_key_config.passwd, MY_WIFI_PASSWD, strlen(MY_WIFI_PASSWD));
    printf("passwd = %s \n",device_info->wifi.one_key_config.passwd);

    if(get_mqtt_status())
    {
        esp_mqtt_client_stop(client);
        set_mqtt_status(0);
    }
    if (get_wifi_status())
    {
        ESP_ERROR_CHECK(esp_wifi_disconnect());
    }
    set_wifi_status(WIFI_STATUS_WAITING );  // 等待wifi连接成功

    set_one_key_config_wifi_status(1);
    ESP_ERROR_CHECK(esp_wifi_stop());
    memset(wifi_config.sta.ssid, 0, 32);
    memset(wifi_config.sta.password, 0, 64);
    memcpy(wifi_config.sta.ssid, (uint8_t *)device_info->wifi.one_key_config.ssid, strlen(device_info->wifi.one_key_config.ssid));
    memcpy(wifi_config.sta.password, (uint8_t *)device_info->wifi.one_key_config.passwd, strlen(device_info->wifi.one_key_config.passwd));
    printf("ssid = %s \n",wifi_config.sta.ssid);
    printf("passwd = %s \n",wifi_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    s_retry_num = 0;
    ESP_ERROR_CHECK(esp_wifi_start());
}
else if (data[0] == 0x33 && data[1]==0x44 && data[2]==0x55)
{
    char temp[100] = {0};
    /* 传感器 OTA 暂未实现，仅回告 */
    (void)sensor_ota_bc(NULL);
    sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":5,\"sdate\":\"sensor OTA not supported yet\"}",
            device_info->id, device_info->utc.time_stamp);
    printf("sensorUpgrade = %s \n",temp);
    esp_ble_gatts_send_indicate(device_info->ble->gatts_if,
                                device_info->ble->conn_id,
                                device_info->ble->handle,
                                strlen((char *)temp), (uint8_t *)temp, false);
}

#endif

    {
        cJSON *firstItem = NULL;
        cJSON *sencondItem = NULL;
        cJSON *thirdItem = NULL;
        firstItem = cJSON_Parse((char *)data);
    if (firstItem)
    {
        sencondItem = cJSON_GetObjectItem(firstItem, "id");
        if(!sencondItem) return 0;
        // printf("id = %s\n", sencondItem->valuestring);
        if(strcmp(sencondItem->valuestring, device_info->id) != 0) return 0;
        sencondItem = cJSON_GetObjectItem(firstItem, "type");
        if(!sencondItem) return 0;
        if(sencondItem->valueint == 3)  // 配网
        {
            sencondItem = cJSON_GetObjectItem(firstItem, "cmd");
            if(!sencondItem) return 0;

            thirdItem = cJSON_GetObjectItem(sencondItem, "ssid");
            if(!thirdItem) return 0;
            memset(device_info->wifi.one_key_config.ssid, 0, 32);
            memcpy(device_info->wifi.one_key_config.ssid, thirdItem->valuestring, strlen(thirdItem->valuestring));
            printf("ssid = %s \n",device_info->wifi.one_key_config.ssid);

            thirdItem = cJSON_GetObjectItem(sencondItem, "passwd");
            if(!thirdItem) return 0;
            memset(device_info->wifi.one_key_config.passwd, 0, 64);
            memcpy(device_info->wifi.one_key_config.passwd, thirdItem->valuestring, strlen(thirdItem->valuestring));
            printf("passwd = %s \n",device_info->wifi.one_key_config.passwd);
            if(get_mqtt_status())
            {
                esp_mqtt_client_stop(client);
                set_mqtt_status(0);
            }
            if (get_wifi_status())
            {
                ESP_ERROR_CHECK(esp_wifi_disconnect());
            }

            set_wifi_status(WIFI_STATUS_WAITING);
            memset(wifi_config.sta.ssid, 0, 32);
            memset(wifi_config.sta.password, 0, 64);
            strncpy((char *)wifi_config.sta.ssid, device_info->wifi.one_key_config.ssid, sizeof(wifi_config.sta.ssid) - 1);
            strncpy((char *)wifi_config.sta.password, device_info->wifi.one_key_config.passwd, sizeof(wifi_config.sta.password) - 1);
            printf("ble set ssid = %s \n",wifi_config.sta.ssid);
            printf("ble set passwd = %s \n",wifi_config.sta.password);
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

            set_one_key_config_wifi_status(1);          // 参数已设置完成后再标记改变
            ESP_ERROR_CHECK(esp_wifi_stop());           // 停 WiFi 前，确保 config 数据已更新

            s_retry_num = 0;
            ESP_ERROR_CHECK(esp_wifi_start());          // 重启wifi模块
        }
        else if(sencondItem->valueint == 4)
        {
            sencondItem = cJSON_GetObjectItem(firstItem, "cmd");
            if(!sencondItem) return 0;

            
            char temp[100] = {0}; 
            if(strcmp(sencondItem->valuestring, "sensorVersion") == 0) 
            {
                
                // if(strlen(device_version) < 2)
                // {
                //     sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":4,\"version\":\"version not get yet,pleas wait\"}",
                //                                         device_info->id,
                //                                         device_info->utc.time_stamp);
                // }
                // else
                // {
                //     sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":4,\"version\":\"%s\"}",
                //                                         device_info->id,
                //                                         device_info->utc.time_stamp,
                //                                         device_version);
                // }

                sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":4,\"version\":\"%s\"}",
                                                        device_info->id,
                                                        device_info->utc.time_stamp,
                                                        device_version);
                
                printf("sensorVersion = %s \n",temp);
                esp_ble_gatts_send_indicate(device_info->ble->gatts_if,
                                            device_info->ble->conn_id,
                                            device_info->ble->handle,
                                            strlen((char *)temp), (uint8_t *)temp, false);
            }
        }
        else if(sencondItem->valueint == 5)
        {
            sencondItem = cJSON_GetObjectItem(firstItem, "cmd");
            if(!sencondItem) return 0;

            char temp[100] = {0};
            if(strcmp(sencondItem->valuestring, "sensorUpgrade") == 0)
            {
                (void)sensor_ota_bc(NULL);
                sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":5,\"sdate\":\"sensor OTA not supported yet\"}",
                        device_info->id, device_info->utc.time_stamp);
                printf("sensorUpgrade = %s \n",temp);
                esp_ble_gatts_send_indicate(device_info->ble->gatts_if,
                                            device_info->ble->conn_id,
                                            device_info->ble->handle,
                                            strlen((char *)temp), (uint8_t *)temp, false);
            }
        }else if(sencondItem->valueint == 12)
        {
            uint8_t cmd_bin[64] = {0}; 
            // printf("BLE: 12\n");
            sencondItem = cJSON_GetObjectItem(firstItem, "cmd");
            if(!sencondItem) return 0;
            // printf("BLE: sencondItem->valuestring = %s\n",sencondItem->valuestring);

            const char *cmd_str = sencondItem->valuestring;  
            int cmd_len = 0;
            // 每2个字符转换为一个字节
            while (*cmd_str && *(cmd_str + 1) && (cmd_len < 64)) {
                char byte_str[3] = { cmd_str[0], cmd_str[1], '\0' };
                cmd_bin[cmd_len++] = (uint8_t)strtol(byte_str, NULL, 16);
                cmd_str += 2;
            }
            // 将转换后的数据通过队列发送出去
            if (cmd_len > 0 && cmd_bin[0] == 0xAA) 
            {
                // printf("\nble_Rev:"); for(int i=0;i<cmd_len;i++){printf("%02X ",cmd_bin[i]);}printf("\n");

                mqtt_ble_data_parser_cb(cmd_bin);
            }else{
                printf("head error\n");
            }
        }

        
    }
    cJSON_Delete(firstItem);
    }
    return 0;
}

void one_key_config_wifi_task(void *pv)
{
    uint8_t enevt_id;
    char temp[100] = {0};
    while (1)
    {
        if (xQueueReceive(device_info->wifi.one_key_config.xQueue, &enevt_id, portMAX_DELAY))
        {
            switch (enevt_id)
            {
            case wifi_ok:
                sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":3,\"state\":\"wifiOk\"}",
                                                                                    device_info->id,
                                                                                    device_info->utc.time_stamp);
                break;
            case wifi_fail:
                sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":3,\"state\":\"wifiFail\"}",
                                                                                    device_info->id,
                                                                                    device_info->utc.time_stamp);
                break;
            case mqtt_ok:
                sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":3,\"state\":\"mqttOk\"}",
                                                                                    device_info->id,
                                                                                    device_info->utc.time_stamp);                                                               
                break;
            case mqtt_fail:
                sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"type\":3,\"state\":\"mqttFail\"}",
                                                                                    device_info->id,
                                                                                    device_info->utc.time_stamp);
                break;
            default:
                break;
            }

            printf("%s\n",temp);
            if(get_ble_status())
            {
                esp_ble_gatts_send_indicate(device_info->ble->gatts_if,
                                                device_info->ble->conn_id,
                                                device_info->ble->handle,
                                                strlen(temp), (uint8_t *)temp, false);
            }
        }
    }
    vTaskDelete(NULL);
}

void ble_data_parser_task(void *pv)
{
    while (1)
    {
        if (xQueueReceive(device_info->ble->xQueue, device_info->ble->data_rec.value, portMAX_DELAY))
        {
            // printf("ble:%s\n",device_info->ble->data_rec.value);
            ble_data_parser_cb(device_info->ble->data_rec.value);
        }
    }
    vTaskDelete(NULL);
}

/*
#define BLE_RECV_BUF_SIZE 512

void ble_data_parser_task(void *pv)
{
    static char ble_recv_buf[BLE_RECV_BUF_SIZE];
    static size_t ble_recv_len = 0;

    uint8_t temp_data[32]; // 临时存放20字节以内的分段
    int brace_count = 0;

    while (1)
    {
        if (xQueueReceive(device_info->ble->xQueue, temp_data, portMAX_DELAY))
        {
            size_t seg_len = strlen((char *)temp_data);

            // 防止溢出
            if (ble_recv_len + seg_len >= BLE_RECV_BUF_SIZE)
            {
                ble_recv_len = 0;
                brace_count = 0;
                memset(ble_recv_buf, 0, sizeof(ble_recv_buf));
                continue;
            }

            // 追加到缓存
            memcpy(ble_recv_buf + ble_recv_len, temp_data, seg_len);
            ble_recv_len += seg_len;
            ble_recv_buf[ble_recv_len] = '\0';

            // 统计大括号匹配数量
            for (size_t i = 0; i < seg_len; i++)
            {
                if (temp_data[i] == '{') brace_count++;
                else if (temp_data[i] == '}') brace_count--;
            }

            // 如果 brace_count == 0 并且最后一个是 '}'
            if (brace_count == 0 && ble_recv_len > 0 && ble_recv_buf[ble_recv_len - 1] == '}')
            {
                // 完整JSON
                ble_data_parser_cb((uint8_t *)ble_recv_buf);

                // 清空缓冲
                ble_recv_len = 0;
                memset(ble_recv_buf, 0, sizeof(ble_recv_buf));
            }
        }
    }
    vTaskDelete(NULL);
}
*/

/*
//mqtt配置解析
int user_mqtt_data_parser_cb(mmqtt_msg_t *msg)
{
    ESP_LOGI(TAG,"user get data success! from_topic=[%d][%s], msg=[%d][%s].", msg->topic_len, msg->topic, msg->data_len, msg->data);
    if(strstr(msg->topic, user_cli_data_subscribe_topic))
    {
        printf("1111111111111\n");
        cJSON *firstItem = cJSON_Parse((char *)msg->data);
        printf("%s\n",msg->data);
        if(firstItem)
        {
            printf("22222222222222\n");
            cJSON *secondItem = cJSON_GetObjectItem(firstItem, "id");
            if(strstr(secondItem->valuestring, device_info->id))
            {
                printf("33333333333333\n");
                secondItem = cJSON_GetObjectItem(firstItem, "cmd");
                if(strstr(secondItem->valuestring, "queryReport"))
                {
                    printf("444444444444\n");
                    // char get_report_cmd2[] = "report 0";
                    // set_bc(device_info->utc.time_stamp, get_report_cmd2, 0, NULL, NULL, NULL);
                }
            }
            cJSON_Delete(firstItem);
        }
    }
    return 0;
}

//mqtt接收topic
void mqtt_data_parser_task(void *pv)
{
    mmqtt_msg_t *recv_msg = (mmqtt_msg_t *)malloc(sizeof(mmqtt_msg_t));
        
    while (1)
    {   
        memset(recv_msg, 0, sizeof(mmqtt_msg_t));
        if (xQueueReceive(device_info->aliyun.xQueue, recv_msg, portMAX_DELAY))
        {
            // printf("device_info->aliyun.msg.data = %s\n",(char *)device_info->aliyun.msg.data);
            printf("topic len: %d  data len: %d\r\n", recv_msg->topic_len, recv_msg->data_len);
            user_mqtt_data_parser_cb(recv_msg);
        }
    }

    free(recv_msg);
    vTaskDelete(NULL);
}

*/

void report_to_aliyun(uint8_t type, uint8_t *value, uint16_t len)
{
    if (!value || len == 0) return;
    uint16_t one_packet_len = 200;   //256;  //128;    //64;   //修改
    uint16_t packet_num = len / one_packet_len;
    uint16_t i,j;
    cJSON *firstItem = NULL, *secondItem = NULL, *thirdItem = NULL;
    char *p_str = NULL;
    char *json_buff = malloc(1200);     //1024  //修改
    if((len % one_packet_len) != 0)
        packet_num ++;
    printf("%d packet to report_to_aliyun\n", packet_num);
    for(i = 0; i < packet_num; i++)
    {
        printf("i = %d\n",i);
        if((i == (packet_num - 1)) && ((len % one_packet_len) != 0))
        {
            printf("ii = %d\n",i);
            firstItem = cJSON_CreateObject();
            cJSON_AddItemToObject(firstItem,"id",cJSON_CreateString(device_info->id));
            cJSON_AddItemToObject(firstItem,"ts",cJSON_CreateNumber(device_info->utc.time_stamp));
            cJSON_AddItemToObject(firstItem,"type",cJSON_CreateNumber(type));
            cJSON_AddItemToObject(firstItem,"report",cJSON_CreateString(json_report_name));

            secondItem = cJSON_CreateObject();
            cJSON_AddItemToObject(secondItem,"len",cJSON_CreateNumber(len));
            cJSON_AddItemToObject(secondItem,"allPacket", cJSON_CreateNumber(packet_num));
            cJSON_AddItemToObject(secondItem,"packet", cJSON_CreateNumber(i));
            cJSON_AddItemToObject(secondItem,"packetLen",cJSON_CreateNumber(len % one_packet_len));
            
            thirdItem = cJSON_CreateArray();
            
            for(j = 0; j < len % one_packet_len; j ++)
            {
                cJSON_AddItemToArray(thirdItem, cJSON_CreateNumber(value[i*one_packet_len + j]));
            }
            cJSON_AddItemToObject(secondItem,"value",thirdItem);
            cJSON_AddItemToObject(firstItem,"data",secondItem);
            p_str = cJSON_Print(firstItem);
            if(p_str)
            {
                cJSON_Minify(p_str);
                printf("%s\n", p_str); //打印创建的字符串
                if(get_mqtt_status())
                {   
                    
                    if(mqtt_send_mutex == true)
                    {
                        mqtt_send_mutex = false;
                        esp_mqtt_client_publish(client, user_sleep_data_publish_topic, (char *)p_str, strlen((char *)p_str), 0, 0);
                        mqtt_send_mutex = true;                        
                    }
                    else
                    {
                        printf("mqtt_send_mutex1\n");
                    }
                }
                free(p_str);                      //一定要记得释放,不然会导致内存泄漏
                p_str = NULL;
            }
            else printf("p_str fail\n");
            cJSON_Delete(firstItem); 
            continue;
        }
        
        printf("i = %d\n",i);
        firstItem = cJSON_CreateObject();
        cJSON_AddItemToObject(firstItem,"id",cJSON_CreateString(device_info->id));
        cJSON_AddItemToObject(firstItem,"ts",cJSON_CreateNumber(device_info->utc.time_stamp));
        cJSON_AddItemToObject(firstItem,"type",cJSON_CreateNumber(type));
        cJSON_AddItemToObject(firstItem,"report",cJSON_CreateString(json_report_name));

        secondItem = cJSON_CreateObject();
        cJSON_AddItemToObject(secondItem,"len",cJSON_CreateNumber(len));
        cJSON_AddItemToObject(secondItem,"allPacket", cJSON_CreateNumber(packet_num));
        cJSON_AddItemToObject(secondItem,"packet", cJSON_CreateNumber(i));
        cJSON_AddItemToObject(secondItem,"packetLen",cJSON_CreateNumber(one_packet_len));
        
        thirdItem = cJSON_CreateArray();
        
        for(j = 0; j < one_packet_len; j ++)
        {
            cJSON_AddItemToArray(thirdItem, cJSON_CreateNumber(value[i*one_packet_len + j]));
        }

        cJSON_AddItemToObject(secondItem,"value",thirdItem);
        cJSON_AddItemToObject(firstItem,"data",secondItem);
        memset(json_buff, 0, 1200);     //1024
        cJSON_PrintPreallocated(firstItem, json_buff, 1200, 1);   //1024
        printf("%s\n", json_buff); //打印创建的字符串
        if(get_mqtt_status())
        {   
            if(mqtt_send_mutex == true)
            {
                mqtt_send_mutex = false;
                esp_mqtt_client_publish(client, user_sleep_data_publish_topic, (char *)json_buff, strlen((char *)json_buff), 0, 0);
                mqtt_send_mutex = true;                        
            }
            else
            {
                printf("mqtt_send_mutex2\n");
            }
        }
        
        cJSON_Delete(firstItem); 
    }
    free(json_buff);
}
void snoring_detection_func(void)
{
    uint8_t count_in_block = 0;
    
    // 统计当前5秒数据中，体动值为4的次数
    for (int i = 0; i < 5; i++) {
        if (user_5s_sensor_info->status[i] == 4) {
            count_in_block++;
        }
    }

    // 如果5秒内体动值为4的次数大于等于4，则认为本次数据块为打鼾包，记为1；否则为0
    uint8_t block_value = (count_in_block >= device_info->snore->snore_parameters.threshold_5s) ? 1 : 0;
    
    // 更新循环缓冲区：记录本次5秒数据块的打鼾状态
    // printf("block_index : %d, block_value : %d\n", device_info->snore->snore_state.block_index, block_value);
    device_info->snore->snore_state.snoring_block[device_info->snore->snore_state.block_index] = block_value;
    device_info->snore->snore_state.block_index = (device_info->snore->snore_state.block_index + 1) % device_info->snore->snore_state.block_size;
    
    
    // 计算最近2分钟内（24个数据块）打鼾包的数量
    uint16_t total_count = 0;
    for (int i = 0; i < device_info->snore->snore_state.block_size; i++) {
        total_count += device_info->snore->snore_state.snoring_block[i];
    }
    
    // 若2分钟内打鼾包数量达到阈值，则触发打鼾事件
    if (total_count >= device_info->snore->snore_parameters.threshold) {
        device_info->snore->snore_state.triggered_flag = true;
    }

}

// 系统控制任务（LED指示灯 + 打鼾检测）
// 这个任务独立于调试显示，量产时必须保留
void system_control_task(void *pv)
{
    while(1)
    {
        // LED指示灯控制：WiFi连接时红灯亮，绿灯灭；未连接时相反
        gpio_set_level(LED_RED, (device_info->wifi.flag == 1) ? 1 : 0);
        gpio_set_level(LED_GREEN, (device_info->wifi.flag == 1) ? 0 : 1);

        // 打鼾检测逻辑
        snoring_detection_func();

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// 参数显示任务（仅用于调试，量产时可注释）
void data_display_task(void *pv)
{
    while(1)
    {
        ESP_LOGI(TAG, "/*======111=====================data display===============%d===========*/",0);

        ESP_LOGI(TAG, "heap_size:%d,version:%s,id:%s,dataUpOn %d,sleepUping %d,sensorOta %d,current_report:%s",esp_get_free_heap_size(),device_info->ota.running_version,
                                                                                                device_info->id,device_info->data_up_switch,
                                                                                                sleep_up_flag,
                                                                                                0,
                                                                                                device_info->report);
        ESP_LOGI(TAG, "5s-->num:%d\tstatus:[%d][%d][%d][%d][%d]\theart:%d\tbreath:%d dev-time:%d id:%s",user_5s_sensor_info->sequence, user_5s_sensor_info->status[0],
                                                                                                user_5s_sensor_info->status[1],
                                                                                                user_5s_sensor_info->status[2],
                                                                                                user_5s_sensor_info->status[3],
                                                                                                user_5s_sensor_info->status[4],
                                                                                                user_5s_sensor_info->heartbeat,
                                                                                                user_5s_sensor_info->breathRate,user_5s_sensor_info->timestamp,user_5s_sensor_info->device_id); //test 
        ESP_LOGI(TAG, "60s-->num:%d\ton_off_bed:%d\theart:%d\tbreath:%d\tMmin:%d\tMmean:%d\tNSD:%d\tNpd:%d\tSBP:%d\tDBP:%d", 
                                                                                                user_60s_sensor_info->sequence, 
                                                                                                user_60s_sensor_info->on_off_bed,
                                                                                                user_60s_sensor_info->heartbeat,
                                                                                                user_60s_sensor_info->breath_rate,
                                                                                                user_60s_sensor_info->Mmin,
                                                                                                user_60s_sensor_info->Mmean,
                                                                                                user_60s_sensor_info->NSD,
                                                                                                user_60s_sensor_info->Npd,
                                                                                                user_60s_sensor_info->SBP,
                                                                                                user_60s_sensor_info->DBP);

        ESP_LOGI(TAG, "wifi:%d\tmqtt:%d\tble:%d\ttime:%d\tbc_device_id:%d",device_info->wifi.flag, device_info->aliyun.flag,device_info->ble->flag,device_info->utc.flag,devic_id_flag);
        if(device_info->utc.flag)
        {
            ESP_LOGI(TAG, "%d-%02d-%02d %02d:%02d:%02d  %02d %d", human_year ,human_mon, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec , ti.tm_wday, device_info->utc.time_stamp);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/* 需左右双侧执行的命令（对时/模式等） */
static bool su3_cmd_need_both_sides(const char *value)
{
    if (value == NULL) {
        return false;
    }
    if (strncmp(value, "set rtc", 7) == 0) {
        return true;
    }
    if (strncmp(value, "set mode", 8) == 0) {
        return true;
    }
    if (strcmp(value, "version") == 0 || strcmp(value, "reboot") == 0) {
        return true;
    }
    return false;
}

int set_bc(uint32_t time_stamp, char *value, uint8_t switch_return, uint8_t switch_to_aliyun, char *return_value, uint16_t time_ms)
{
    char tmp[256];
    char *rsp = (return_value != NULL) ? return_value : tmp;
    size_t rsp_cap = sizeof(tmp);
    esp_err_t err;
    su3_addr_t left = su3_side_to_addr(SU3_SIDE_LEFT);
    su3_addr_t right = su3_side_to_addr(SU3_SIDE_RIGHT);

    (void)time_stamp;
    (void)switch_to_aliyun;

    if (!su3_is_ready() || value == NULL) {
        return -1;
    }

    if (switch_return == 0) {
        err = su3_cli_fire(left, value);
        if (su3_cmd_need_both_sides(value)) {
            (void)su3_cli_fire(right, value);
        }
        return (err == ESP_OK) ? 0 : -1;
    }

    err = su3_cli_exec(left, value, rsp, rsp_cap, time_ms);
    if (su3_cmd_need_both_sides(value)) {
        char rsp2[256];
        (void)su3_cli_exec(right, value, rsp2, sizeof(rsp2), time_ms);
    }
    return (err == ESP_OK) ? 0 : -1;
}

int sensor_ota_bc(char *value)
{
    (void)value;
    /* 预留：转发 su3_sensor_ota，当前未实现 */
    esp_err_t err = su3_sensor_ota(su3_side_to_addr(SU3_SIDE_LEFT), NULL, 0);
    return (err == ESP_OK) ? 0 : -1;
}

uint32_t find_report_time(char *report_name, uint8_t len)
{
    uint32_t report_tim;
    // printf("%s\n", report_name);
    struct tm stm;  
    int iY, iM, iD, iH, iMin, iS;  
    if (len < 23) {
        printf("Invalid report format: Length is too short.\n");
        return 0;           // 返回默认值，表示格式错误
    }
    memset(&stm, 0, sizeof(stm));  
    iY =    atoi(report_name + len - 23);
    // printf("iY = %d\n",iY);  
    iM =    atoi(report_name + len - 18); 
    // printf("iM = %d\n",iM);  
    iD =    atoi(report_name + len - 15);
    // printf("iD = %d\n",iD);   
    iH =    atoi(report_name + len - 12);
    // printf("iH = %d\n",iH);   
    iMin =  atoi(report_name + len - 9); 
    // printf("iMin = %d\n",iMin);  
    iS =    atoi(report_name + len - 6);  
    // printf("iS = %d\n",iS);  
        // 简单校验：确保日期和时间的字段有效
    if (iY < 1900 || iM < 1 || iM > 12 || iD < 1 || iD > 31 || 
        iH < 0 || iH > 23 || iMin < 0 || iMin > 59 || iS < 0 || iS > 59) {
        printf("Invalid time values in report: %d-%d-%d %d:%d:%d\n", iY, iM, iD, iH, iMin, iS);
        return 0; // 返回默认值，表示解析失败
    }
    memset(&stm, 0, sizeof(stm));
    stm.tm_year=iY-1900;  
    stm.tm_mon=iM-1;  
    stm.tm_mday=iD;  
    stm.tm_hour=iH;  
    stm.tm_min=iMin;  
    stm.tm_sec=iS;  

    /*printf("%d-%0d-%0d %0d:%0d:%0d\n", iY, iM, iD, iH, iMin, iS);*/   //标准时间格式例如：2016:08:02 12:12:30
    report_tim = (uint32_t)mktime(&stm);
    if (report_tim <= 1262304000UL || report_tim >= 2147483647UL){
        return 0; 
    }
    
    return report_tim;
}

void check_report_and_up_to_aliyun(void)
{
    //uint64_t start_time = esp_timer_get_time(); 
    char return_value[1024] = {0};
    char json_buff[512] = {0};
    uint16_t offset = 0;
    char temp[3] = {0};
    char get_report_cmd1[] = "list";
    char get_report_cmd2[] = "report 0";
    uint8_t report_num = 0;
    char report_name[30][30] = {0};  //报告列表
    uint32_t report_time[30] = {0};   //报告对应的时间
    uint8_t report_len[30] = {0};   //报告名长度
    uint32_t current_time;
    nvs_handle nvs_config_handler;

    memset(return_value, 0, 1024);
    printf("get_report_cmd1\n");
    set_bc(device_info->utc.time_stamp, get_report_cmd1, 1, 1, return_value, 1000);
    // printf("\n \n \n cil = list :return_value =\n%s\nstrlen(return_value) = %d\n", return_value, strlen(return_value));

    if(strstr(return_value,"NONE"))
    {
        printf("no report, need't to up to aliyun\n");
    }
    else
    { 
        uint16_t len = strlen(return_value);
        for(uint16_t i=0; i<len; i++)
        {
            report_len[report_num] ++ ;
            // printf("report_len[%d] = %d, return_value[%d] = %c\n", report_num, report_len[report_num], i, return_value[i]);
            if(return_value[i] == '\n')
            {
                report_num++;
            } 
            if (report_len[report_num] >= 28)
            {
                printf("report_len[%d] = %d, return_value[%d] = %c\n", report_num, report_len[report_num], i, return_value[i]);
                report_len[report_num] = 28;
                return_value[i] = '\n';
                report_num++;
                printf("Warning: %d 报告名超过了28个字节 设置为28\n", report_num);
            }
            if(report_num > 30)
            {
                report_num = 30;
                printf("Warning: %d 报告数超过了30个 设置为0\n", report_num);
            }
        }

        printf("report has %d total\n", report_num);

        //分割报告名
        for(uint16_t i=0; i<report_num; i++)
        {
            memcpy(report_name[i], return_value + offset, report_len[i] - 2);
            offset += report_len[i] ;
            //printf("report_name[%d] = %s\n", i, report_name[i]);
            report_time[i] = find_report_time(report_name[i], strlen(report_name[i]));
            //printf("report_time[%d] = %d\n",i, report_time[i]);
        }
        
        printf("current report = %s\n",device_info->report);
        //初始报告全部上报
        if(strstr(device_info->report, "NONE"))
        {
            current_time = 0;
            printf("Report is being updated ...\n");
            
            //发送具体数据
            for(uint16_t i=0; i<report_num; i++)
            {
                printf("%s\n",report_name[i]);
                memset(json_buff, 0, 512);
                //发送报告名
                sprintf(json_buff, "{\"id\":\"%s\",\"ts\":%d,\"type\":4,\"report\":\"%s\",\"data\":\"%s\"}",
                                                                                        device_info->id,
                                                                                        device_info->utc.time_stamp,
                                                                                        report_name[i],
                                                                                        report_name[i]);
                printf("%s\n",json_buff);                                                                       
                //
                if(get_mqtt_status() && device_info->data_up_switch)
                {
                    if(mqtt_send_mutex == true)
                    {
                        mqtt_send_mutex = false;
                        esp_mqtt_client_publish(client, user_sleep_data_publish_topic, (char *)json_buff, strlen((char *)json_buff), 0, 0);
                        //vTaskDelay(1000 / portTICK_PERIOD_MS);
                        mqtt_send_mutex = true;
                    }
                }
                memcpy(temp, report_name[i], 2); 
                //printf("%d\n",atoi(temp));
                sprintf(get_report_cmd2, "report %d",atoi(temp));
                printf("初始化全部上报111111.%s\n",get_report_cmd2);
                if(current_time < report_time[i])
                {
                    //存储最新的报告名
                    ESP_ERROR_CHECK(nvs_open("config_cfg", NVS_READWRITE, &nvs_config_handler));
                    ESP_ERROR_CHECK(nvs_set_str(nvs_config_handler, "report", report_name[i]));
                    ESP_ERROR_CHECK(nvs_commit(nvs_config_handler));
                    nvs_close(nvs_config_handler);
                    memset(device_info->report,0,128);
                    memcpy(device_info->report, report_name[i], strlen(report_name[i]));
                    printf("recent report= %s\n", device_info->report);
                    current_time = find_report_time(device_info->report, strlen(device_info->report));
                    //current_time = report_time[i];
                }

                if(device_info->data_up_switch)
                {
                    if(sleep_up_flag == 1)
                    {
                        vTaskDelay(5 * 1000 / portTICK_PERIOD_MS);   //??隐患：在延时期间，报告上传指令1分钟结束，sleep_up_flag回到0，此时又接到云端上传指令
                    }
                    sleep_up_flag = 1;
                    memset(json_report_name, 0, 32);
                    strcpy(json_report_name, report_name[i]);
                    set_bc(device_info->utc.time_stamp, get_report_cmd2, 0, 0, return_value, 100);   //发送读取report指令
                    vTaskDelay(15*1000 / portTICK_PERIOD_MS);
                    sleep_up_flag = 0;
                }

                //printf("current_time = %d\n",current_time);
                //printf("report_time[%d] = %d\n",i,report_time[i]);
            }  
        }
        //检测有无新报告，并上报
        else
        {
            //printf("为什么没有进入新报告上传！ report_num = %d\n",report_num);
            for(uint16_t i=0; i<report_num; i++)
            {  
                current_time = find_report_time(device_info->report, strlen(device_info->report));
                printf("current_time = %d\n",current_time);
                //printf("report_time[%d] = %d\n",i,report_time[i]);
                if(current_time < report_time[i])
                {
                    sprintf(json_buff, "{\"id\":\"%s\",\"ts\":%d,\"type\":4,\"report\":\"%s\",\"data\":\"%s\"}",
                                                                                        device_info->id,
                                                                                        device_info->utc.time_stamp,
                                                                                        report_name[i],
                                                                                        report_name[i]);
                    printf("%s\n",json_buff);
                    //                                                                    
                    if(get_mqtt_status()&& device_info->data_up_switch)
                    {
                        //if(mqtt_send_mutex == true)
                        {
                            mqtt_send_mutex = false;
                            esp_mqtt_client_publish(client, user_sleep_data_publish_topic, (char *)json_buff, strlen((char *)json_buff), 0, 0);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            mqtt_send_mutex = true;
                        }
                    }

                    memcpy(temp, report_name[i], 2);
                    printf("uploading : %s\n",report_name[i]);
                    //printf("%d\n",atoi(temp));
                    sprintf(get_report_cmd2, "report %d",atoi(temp));
                    printf("存储最新报告名2222222.%s\n",get_report_cmd2);

                    // //存储最新的报告名
                    ESP_ERROR_CHECK(nvs_open("config_cfg", NVS_READWRITE, &nvs_config_handler));
                    ESP_ERROR_CHECK(nvs_set_str(nvs_config_handler, "report", report_name[i]));
                    ESP_ERROR_CHECK(nvs_commit(nvs_config_handler));
                    nvs_close(nvs_config_handler);
                    memset(device_info->report,0,128);
                    memcpy(device_info->report, report_name[i], strlen(report_name[i]));

                    if(device_info->data_up_switch)
                    {
                        if(sleep_up_flag == 1)
                        {
                            vTaskDelay(5* 1000 / portTICK_PERIOD_MS);
                        }
                        sleep_up_flag = 1;
                        memset(json_report_name, 0, 32);
                        strcpy(json_report_name, report_name[i]);
                        set_bc(device_info->utc.time_stamp, get_report_cmd2, 0, 0, return_value, 200);
                        vTaskDelay(5 * 1000 / portTICK_PERIOD_MS);
                        set_bc(device_info->utc.time_stamp, get_report_cmd2, 0, 0, return_value, 1000);   //发送读取report指令
                        sleep_up_flag = 0;
                    }

                } 
            }
        }
    }
    //int64_t end_time = esp_timer_get_time();
    //printf("check_report_and_up_to_aliyun time = %lld ms\n", (end_time - start_time)/1000);
}
//utc 获取任务
void utc_get_task(void *pv)
{
    static uint8_t utc_first_flag = 1;
    static char time_stamp_to_string[20] = {0};
    static char set_rtc_cmd[100] = {0};
    static char return_value[100] = {0};  
    static char get_version_cmd[] = "version";  
    if(utc_first_flag)
    {
        utc_first_flag=0;

        /* SNTP/MQTT 已在 use_wifi GOT_IP → start_sntp_once → on_sntp_synced 中完成 */
        while (device_info->utc.flag == false) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        if (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) {
            printf("[UTC] SNTP 同步失败 or 未完成\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            esp_restart();
        } else {
            printf("[UTC] SNTP 同步成功\n");
        }

        /* 等待博创 device_id（串口通信正常） */
        while (devic_id_flag == 0) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        printf("[UTC] devic_id_flag 同步成功\n");

        time(&now);
        localtime_r(&now, &ti);
        device_info->utc.time_stamp = (uint32_t)(mktime(&ti));
        printf("[UTC] device_info->utc.time_stamp = %d\n", device_info->utc.time_stamp);

        itoa(device_info->utc.time_stamp, time_stamp_to_string, 10);
        sprintf(set_rtc_cmd, "set rtc %s", time_stamp_to_string);
        printf("[UTC] set_rtc_cmd = %s\n", set_rtc_cmd);

        set_bc(device_info->utc.time_stamp, set_rtc_cmd, 1, 0, return_value, 100);
        set_rtc_flag = 1;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        set_bc(device_info->utc.time_stamp, get_version_cmd, 1, 0, device_version, 100);
        printf("[UTC]  get_version_cmd= %s\n",device_version);
    }
    while(1)
    {
/**
调用sntp_init()会立刻请求服务器同步一次时间。
因此，我们需要主动同步时：
先调用sntp_stop()、再调用sntp_init() 即可立刻同步一次时间。
*/
        
        time(&now);
		
		localtime_r(&now, &ti);
        device_info->utc.time_stamp = (uint32_t)(mktime(&ti));

        human_year = ti.tm_year + 1900;
        human_mon  = ti.tm_mon + 1;

		// ti.tm_year = ti.tm_year + 1900;
		// ti.tm_mon = ti.tm_mon + 1;
		device_info->utc.time[0] = human_year;			//年
		device_info->utc.time[1] = human_mon;			//月
		device_info->utc.time[2] = ti.tm_mday ;			//日
		device_info->utc.time[3] = ti.tm_hour ;			//时
		device_info->utc.time[4] = ti.tm_min;			//分
		device_info->utc.time[5] = ti.tm_sec ;			//秒
/*
        // 异常检查
        if (user_5s_sensor_info->timestamp <= 1262304000UL ||  user_5s_sensor_info->timestamp >= 2147483647UL){
            set_rtc_flag = 0;
            printf("[UTC]  重新校正 user_5s_sensor_info->timestamp = %d\n",user_5s_sensor_info->timestamp);
        }
*/
        if(set_rtc_flag == 0)
        {
            while(devic_id_flag == 0)      //等待device_id更新赋值
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            time(&now);
            localtime_r(&now, &ti);
            device_info->utc.time_stamp = (uint32_t)(mktime(&ti));
            itoa(device_info->utc.time_stamp, time_stamp_to_string, 10);
            memset(set_rtc_cmd,0,100);
            sprintf(set_rtc_cmd, "set rtc %s", time_stamp_to_string);
            printf("set_rtc_cmd\n");
            set_bc(device_info->utc.time_stamp, set_rtc_cmd, 1, 0, return_value, 20);
            set_rtc_flag = 1;
            vTaskDelay(100 / portTICK_PERIOD_MS);

            memset(device_version,0,32);
            set_bc(device_info->utc.time_stamp, get_version_cmd, 1, 0, device_version, 20);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            printf("bc version = %s\n",device_version);

        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
// static uint8_t real_data_up_task_flag = 0;

void real_data_up_task(void *pv)
{
    uint8_t* temp = (uint8_t*) malloc(512);  //缩减1024
    if (temp == NULL) {
        vTaskDelete(NULL);
        return;
    }
    static uint8_t cnt_5s = 0;
    static int32_t ap_min_max = 0;
    static int32_t ap_min_min = 0;
    int32_t ap_min_temp = 0;
    static uint8_t s_5s_cnt=20;
    //while(1)
    {
        if(s_5s_cnt<20)
        {
            s_5s_cnt++;
        }
        if(sleep_up_flag == 1)
        {
            if(s_5s_cnt>11)
            {
                report_cli_up();
                s_5s_cnt=0;
            }
        }
        else
        {
            /* 1s/1min already published in su3_on_topic */
            (void)temp;
            (void)cnt_5s;
            (void)ap_min_max;
            (void)ap_min_min;
            (void)ap_min_temp;

        
        }      
        //vTaskDelay(5000 / portTICK_PERIOD_MS);

    }
    free(temp);
    vTaskDelete(NULL); 
}

void report_data_up_task(void *pv)
{
// #if 0//BLE_TEST
//     while(1)
//     {
//         bochuang_test();
//         vTaskDelay(3000 / portTICK_PERIOD_MS);
//     }
// #else        
    //while(device_info->utc.flag == false || get_5s_flag == false)
    {
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    //vTaskDelay(30*1000 / portTICK_PERIOD_MS);
    //while(1)
    {
        if(get_wifi_status() && get_mqtt_status() && ota_now_flag == 0)
        {

            check_report_and_up_to_aliyun();
        }

    }
// #endif
    vTaskDelete(NULL); 
}


void twd_task(void *arg)
{
    // 为TWDT添加任务，并检查dwt状态看是否添加
    // esp_task_wdt_add(NULL);
    // esp_task_wdt_status(NULL);
 
    while(1){
        // 每2秒重置一次看门狗
        //CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);  // 喂狗。注释此行以测试触发TWDT超时
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL); 
}
extern char user_cli_data_publish_topic[100];
void test_task(void *pv)
{
    char device_data[100]={0};
    char temp[2048] = {1};
    while(0)
    {
        char return_value[1024] = {0};
        set_bc(device_info->utc.time_stamp, "report", 1, 0, return_value, 1000);
        // printf("%s\n",return_value);
        sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"cmd\":%s,\"back\":\"%s\"}", 
            device_info->id,
            device_info->utc.time_stamp,
            "list",
            return_value);
        printf("%s\n",temp);
        //esp_mqtt_client_publish(client, user_cli_data_publish_topic, (char *)temp, strlen((char *)temp), 0, 0);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void test_task1(void)
{
    char device_data[100]={0};
    char temp[2048] = {0};
    //while(1)
    {
        char return_value[1024] = {0};
        set_bc(device_info->utc.time_stamp, "list", 1, 0, return_value, 1000);
        // printf("%s\n",return_value);
        sprintf(temp,"{\"id\":\"%s\",\"ts\":%d,\"cmd\":%s,\"back\":\"%s\"}", 
            device_info->id,
            device_info->utc.time_stamp,
            "list",
            return_value);
        printf("%s\n",temp);
        //esp_mqtt_client_publish(client, user_cli_data_publish_topic, (char *)temp, strlen((char *)temp), 0, 0);
    }
    //vTaskDelete(NULL);
}

TaskHandle_t utc_get_task_handle = NULL;
TaskHandle_t real_data_up_task_handle = NULL;
TaskHandle_t report_data_up_task_handle = NULL;
void Task_scheduling(void *pv)
{
    //static uint8_t s_real_data_up_task_flag = 0;
    //static uint8_t s_report_data_up_task = 0;
    static uint8_t s_5s_Cnt = 115;
    static uint8_t s_1s_Cnt = 5;
    //xTaskCreatePinnedToCore(utc_get_task, "utc_get", 1024*5, NULL, 3, &utc_get_task_handle, 1);
    vTaskDelay(10000 / portTICK_PERIOD_MS);     //10s 初始化时间、版本号、传感器 ：old：5s
    while(1)
    {
        while(get_ota_now_flag()==1)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    //实时数据上报
    if(s_1s_Cnt>4)
    {
        xTaskCreatePinnedToCore(real_data_up_task, "real_data_up", 1024*6, NULL, 4, &real_data_up_task_handle, 1);   
        s_1s_Cnt=0;
        s_5s_Cnt++;
    }
    
    //报告数据上报
    if(get_mode_flag_config() == 0 || s_5s_Cnt>119)
    {
        if(device_info->utc.flag == true && get_5s_flag == true)
        {
            set_mode_flag_config(2);
            s_5s_Cnt=0;
            xTaskCreatePinnedToCore(report_data_up_task, "report_data_up", 1024*10, NULL, 10, &report_data_up_task_handle, 1);//缩减2048*7
            
        }else{
            s_5s_Cnt = 115; // 延迟30s
        }
 
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    s_1s_Cnt++;
    //printf("s_5s_Cnt:%d\n",s_5s_Cnt);
    }
}

void M_Ctr(uint8_t data);
#include "driver/gpio.h"
void key_task(void)
{
    static uint8_t s_oldkey_data = 0xff;
    uint8_t key_data;
    while(1)
    {
        key_data = gpio_get_level(13);
        if(s_oldkey_data!=key_data)
        {
            if(key_data)
            {
                M_Ctr(0);
            }else
            {
                M_Ctr(50);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}



/*
// 函数：根据当前 g_Sync_TX.BcPlugInPacket 设置填充发送的数据包（如键值等）
// 参数：type：数据包类型
// 参数：keys：键值 pwm：缓启动速度 tmr：缓启动时间
// 返回：无
*/
void prepare_mfp_command(uint8_t type, uint32_t keys, uint8_t pwm, uint8_t tmr)
{
    // 清零发送数据包缓存
    memset(g_Sync_TX.rawData, 0, sizeof(g_Sync_TX.rawData));

    switch(type) {
        case KEYS_TYPE_NORMAL:
            printf("KEYS_TYPE_NORMAL\n");
            g_Sync_TX.PlugInPacket_Normal.length   = 0x05;  // 数据部分长度
            g_Sync_TX.PlugInPacket_Normal.type     = 0x01;  // 固定类型
            g_Sync_TX.PlugInPacket_Normal.keys     = keys;  // 设置不同的命令键值
            g_Sync_TX.PlugInPacket_Normal.ctrlMode = 0x00;
            g_Sync_TX.PlugInPacket_Normal.checksum = syncCalcCheckSum();
            break;

        case KEYS_TYPE_SOFT_START:
            printf("KEYS_TYPE_SOFT_START\n");
            g_Sync_TX.PlugInPacket_SlowStart.length   = 0x07;  // 数据部分长度
            g_Sync_TX.PlugInPacket_SlowStart.type     = 0x01;  // 固定类型
            g_Sync_TX.PlugInPacket_SlowStart.keys     = keys;  // 设置不同的命令键值
            g_Sync_TX.PlugInPacket_SlowStart.cmd      = 0x00;
            g_Sync_TX.PlugInPacket_SlowStart.pwm      = pwm;   // 设置传入的 pwm
            if(keys == KEY_HEAD_LOWER){
                g_Sync_TX.PlugInPacket_SlowStart.tmr = (tmr + 5 > 0xFF) ? 0xFF : (tmr + 5);  // 头部放下，时间增加5秒
            }else{
                g_Sync_TX.PlugInPacket_SlowStart.tmr  = tmr;  // 使用传入的 tmr
            }
            g_Sync_TX.PlugInPacket_SlowStart.checksum = syncCalcCheckSum();
            break;

        case KEYS_TYPE_MASSAGE:
            printf("KEYS_TYPE_MASSAGE\n");
            g_Sync_TX.PlugInPacket_MASSAGE.length   = 0x0a;     // 数据部分长度
            g_Sync_TX.PlugInPacket_MASSAGE.type     = 0x01;     // 固定类型
            g_Sync_TX.PlugInPacket_MASSAGE.keys     = keys;     // 设置不同的命令键值
            g_Sync_TX.PlugInPacket_MASSAGE.reserve1 = 0x0000;
            g_Sync_TX.PlugInPacket_MASSAGE.cmd      = 0x00;
            g_Sync_TX.PlugInPacket_MASSAGE.ctrcmd   = 0x00;
            g_Sync_TX.PlugInPacket_MASSAGE.reserve2 = 0x0000;
            g_Sync_TX.PlugInPacket_MASSAGE.checksum = syncCalcCheckSum();
            break;

        default:
            return;
    }
}


/*
* 打鼾干预/演示 MQTT上报
*/
void handle_snore_trigger(bool is_demo)
{
    static uint8_t send_attempts = 0;
    snore_parameters_t *params;
    if (is_demo) {
        params = &snore_parameters_demo;  // 使用演示参数
    } else {
        params = &device_info->snore->snore_parameters;  // 使用正常参数
    }

    // 如果未在干预中，则立即发送缓启动抬升命令,并记录时间
    if (!device_info->snore->snore_state.is_intervening) {
        /* 闹钟指令后 120s 内不执行新打鼾干预 */
        if (snore_blocked_until_s != 0) {
            uint32_t now_s = (uint32_t)(xTaskGetTickCount() / pdMS_TO_TICKS(1000));
            if (now_s < snore_blocked_until_s) {
                if (is_demo) {
                    device_info->snore->snore_state.triggered_flag_demo = false;
                } else {
                    device_info->snore->snore_state.triggered_flag = false;
                }
                return;
            }
        }

        printf("触发打鼾干预%s: triggered = %d, 正在干预: in_progress = %d \r\n", 
               is_demo ? "(演示)" : "", 
               is_demo ? device_info->snore->snore_state.triggered_flag_demo : device_info->snore->snore_state.triggered_flag, 
               device_info->snore->snore_state.is_intervening);

        prepare_mfp_command(KEYS_TYPE_SOFT_START, KEY_MEMORY4, params->pwm, params->tmr);
        mfp_dateSend();
        send_attempts++;
        if (send_attempts >= 3) {
            send_attempts = 0;
            device_info->snore->snore_state.is_intervening = true;
            device_info->snore->snore_state.last_triggered_time_s = device_info->utc.time_stamp;


            // 打鼾干预触发时，上报 MQTT 数据
            char send_json_value[512];
            memset(send_json_value, 0, sizeof(send_json_value));
            
            // 构建 JSON 字符串
            sprintf(send_json_value, "{\"id\":\"%s\",\"ts\":%d,\"type\":16,\"report\":\"snore_param\",\"data\":{"
                                      "\"triggered_flag\":%d,"
                                      "\"triggered_flag_demo\":%d,"
                                      "\"upHoldTime\":%d,"
                                      "\"threshold\":%d,"
                                      "\"threshold5s\":%d,"
                                      "\"pwm\":%d,"
                                      "\"tmr\":%d,"
                                      "\"lastTriggeredTime\":%d,"
                                      "\"isIntervening\":%d}}",             
                                        device_info->id,
                                        device_info->utc.time_stamp,
                                        device_info->snore->snore_state.triggered_flag,                  
                                        device_info->snore->snore_state.triggered_flag_demo,
                                        params->up_hold_time_s,
                                        params->threshold,
                                        params->threshold_5s,
                                        params->pwm,
                                        params->tmr,
                                        device_info->snore->snore_state.last_triggered_time_s,
                                        device_info->snore->snore_state.is_intervening);
            
            printf("上报打鼾干预数据: %s\n", send_json_value);

            // MQTT 发送
            if (get_mqtt_status()) {
                if (mqtt_send_mutex == true) {
                    mqtt_send_mutex = false;
                    esp_mqtt_client_publish(client, mc_cli_data_publish_topic, (char *)send_json_value, strlen((char *)send_json_value), 0, 0);
                    mqtt_send_mutex = true;
                }
            }
        }
    } else {
        // 如果已经触发，则等到冷却时间后再发送缓启动降下命令
        uint32_t up_hold_time = is_demo ? snore_parameters_demo.up_hold_time_s : device_info->snore->snore_parameters.up_hold_time_s;

        if ((device_info->utc.time_stamp - device_info->snore->snore_state.last_triggered_time_s) >= up_hold_time) {
            printf("打鼾干预结束%s: triggered = %d, 正在干预: in_progress = %d \r\n", 
                is_demo ? "(演示)" : "", 
                is_demo ? device_info->snore->snore_state.triggered_flag_demo : device_info->snore->snore_state.triggered_flag, 
                device_info->snore->snore_state.is_intervening);

            prepare_mfp_command(KEYS_TYPE_SOFT_START, KEY_ALLFATE, params->pwm, params->tmr);
            mfp_dateSend();

            // 清除打鼾事件标志，结束本次打鼾干预
            send_attempts++;
            if (send_attempts >= 3) {
                device_info->snore->snore_state.is_intervening = false;
                if (is_demo) {
                    device_info->snore->snore_state.triggered_flag_demo = false;
                } else {
                    device_info->snore->snore_state.triggered_flag = false;
                }
                send_attempts = 0;

                // 打鼾干预触发时，上报 MQTT 数据
                char send_json_value[512];
                memset(send_json_value, 0, sizeof(send_json_value));
                
                // 构建 JSON 字符串
                sprintf(send_json_value, "{\"id\":\"%s\",\"ts\":%d,\"type\":16,\"report\":\"snore_param\",\"data\":{"
                                        "\"triggered_flag\":%d,"
                                        "\"triggered_flag_demo\":%d,"
                                        "\"upHoldTime\":%d,"
                                        "\"threshold\":%d,"
                                        "\"threshold5s\":%d,"
                                        "\"pwm\":%d,"
                                        "\"tmr\":%d,"
                                        "\"lastTriggeredTime\":%d,"
                                        "\"isIntervening\":%d}}",             
                                        device_info->id,
                                        device_info->utc.time_stamp,
                                        device_info->snore->snore_state.triggered_flag,                  
                                        device_info->snore->snore_state.triggered_flag_demo,
                                        params->up_hold_time_s,
                                        params->threshold,
                                        params->threshold_5s,
                                        params->pwm,
                                        params->tmr,
                                        device_info->snore->snore_state.last_triggered_time_s,
                                        device_info->snore->snore_state.is_intervening);
                
                printf("上报打鼾干预数据(结束): %s\n", send_json_value);

                // MQTT 发送
                if (get_mqtt_status()) {
                    if (mqtt_send_mutex == true) {
                        mqtt_send_mutex = false;
                        esp_mqtt_client_publish(client, mc_cli_data_publish_topic, (char *)send_json_value, strlen((char *)send_json_value), 0, 0);
                        mqtt_send_mutex = true;
                    }
                }

            }
        }
    }
}

void syncPacket_to_hex_str(const void* packet, size_t size, char* out_buf, size_t buf_size) {
    const uint8_t* data = (const uint8_t*)packet;
    char* p = out_buf;

    for (size_t i = 0; i < size && (p - out_buf) < buf_size - 4; i++) {
        p += snprintf(p, buf_size - (p - out_buf), "%02X ", data[i]);
    }
    *p = '\0';
/*
        const uint8_t* data = (const uint8_t*)packet;
    char* p = out_buf;

    for (size_t i = 0; i < size && (p - out_buf) < buf_size - 3; i++) {
        p += snprintf(p, buf_size - (p - out_buf), "%02X", data[i]);  // 去掉 "%02X " 中的空格
    }
    *p = '\0';
*/
}

/*
* <设备-APP> 主控盒数据上报函数（蓝牙）
*/
void ble_up_controlbox_data(uint8_t up_type) {
    static char json_buf[512];
    static char hex_str[256];
    uint16_t raw_len = 0;
    
    // 从独立的解析缓冲区读取数据（带线程安全保护）
    if (g_MFP_Parsed_Mutex != NULL && 
        xSemaphoreTake(g_MFP_Parsed_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        
        raw_len = g_MFP_Parsed_Len;
        
        if (raw_len > 0 && raw_len <= sizeof(g_MFP_Parsed_Frame)) {
            syncPacket_to_hex_str(g_MFP_Parsed_Frame, raw_len, hex_str, sizeof(hex_str));
        } else {
            ESP_LOGW(TAG, "Invalid parsed packet length: %d", raw_len);
            xSemaphoreGive(g_MFP_Parsed_Mutex);
            return;
        }
        
        xSemaphoreGive(g_MFP_Parsed_Mutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex for reading parsed data");
        return;
    }

    int len = snprintf(json_buf, sizeof(json_buf),
        "{"
            "\"id\":\"%s\",\"ts\":%d,\"type\":12,"
            "\"data_hex\":\"%s\""
        "}",
        device_info->id, device_info->utc.time_stamp, hex_str);
    
    if (len < 0 || (size_t)len >= sizeof(json_buf)) {
        printf("JSON 构建失败或缓冲区不足\n");
        return;
    }

    // BLE上报
    if(get_ble_status()) {
        esp_ble_gatts_send_indicate(device_info->ble->gatts_if,
                                    device_info->ble->conn_id,
                                    device_info->ble->handle,
                                    strlen((char *)json_buf), (uint8_t *)json_buf, false);
    }

    // MQTT上报（只在查询的时候上报）
    if(up_type == 1 && get_mqtt_status()) {
        if(mqtt_send_mutex == true) {
            mqtt_send_mutex = false;
            esp_mqtt_client_publish(client, mc_cli_data_publish_topic, (char *)json_buf, strlen((char *)json_buf), 0, 0);
            mqtt_send_mutex = true;
        }
        // printf("mqtt put hex_str: %s\n", hex_str);
    }
}

/*
* <主控盒-设备> 主控盒数据解析函数 （区分不同主控盒、收到主控盒数据后解析）
*/
void MFP_Cmd(uint8_t cmd, uint8_t up_type)
{
    //static uint32_t last_time = 0;
    //static uint32_t now = 0;
    //now = xTaskGetTickCount();
    // for(int i = 0;i<g_Sync_RX.syncPacket.length +3;i++){printf("%x ",g_Sync_RX.rawData[i]);} printf("\r\n");
    // printf("帧头: %d ,长度: %d ,sizeof(g_Sync_RX.syncPacket): %d \r \n",cmd,g_Sync_RX.syncPacket.length,sizeof(g_Sync_RX.syncPacket));
   
    // if((cmd == 0x07) && ((now - last_time >= pdMS_TO_TICKS(2000))))   // 200ms返回一次主控盒数据                // && (g_Sync_RX.syncPacket.length ==	sizeof(g_Sync_TX.syncPacket) - 3) // 不清楚这里为什么 34 ！= 39-3？
	if(cmd == 0x07) // 收到一帧后直接发送
    {
		ble_up_controlbox_data(up_type);// 蓝牙上报函数
        //last_time = now;
	}else if(cmd == 0x02)                   //  蓝牙查询主控盒数据：0x02  立即响应
    {   
        ble_up_controlbox_data(up_type);  // 蓝牙/mqtt都上报
        //last_time = now;
    }
}

/*
*    <主控盒-设备> MFP接收数据解析函数：帧头、和校验正确则传入主控盒数据解析函数，并在接收到数据后通知发送线程


void mfp_datareceive_handler(uint8_t *p,uint16_t len)
{
    static uint32_t count = 0;
    uint8_t checkSum = 0,checkSum1 = 0;
    memcpy(g_Sync_RX.rawData+count,p,len);
	count += len;  
    //printf("count: %d len: %d g_Sync_RX.Syncdata.length =%x \r\n",count,len,g_Sync_RX.Syncdata.length);
    if(g_Sync_RX.Syncdata.length != 0x3C && g_Sync_RX.Syncdata.length != 0x2C && g_Sync_RX.Syncdata.length != 0x2E && g_Sync_RX.Syncdata.length != 0x36) // 3c 2c
    {           
        count = 0;
    }
    // for(int i = 0;i<count;i++){printf("count:%d: %x ",i,g_Sync_RX.rawData[i]);} printf("\r\n");
    // printf("checkSum: %x checkSum1: %x \r\n",rxCalcCheckSum(),g_Sync_RX.Syncdata.data[g_Sync_RX.Syncdata.length]);
    // printf("count: %d g_Sync_RX.Syncdata.length+3=%d\r\n",count,g_Sync_RX.Syncdata.length+3);
    if( (count != 0) && (count>=g_Sync_RX.Syncdata.length+3))
	{	
		checkSum = rxCalcCheckSum();
		checkSum1 = g_Sync_RX.Syncdata.data[g_Sync_RX.Syncdata.length];
        
		if(checkSum == checkSum1 && g_Sync_RX.rawData[0] != 0)
		{	
            printf("crc ok mfp_send-ready \r\n");
		    count = 0;	
            mfp_queue_pop_send();                     // 接收完成后 发送队列
                // MFP_Cmd(g_Sync_RX.syncPacket.type,0);     // 不主动发送
		}
		else
		{	
            // g_MFPData_t.sync_error.usartCheck_error++;
			count = 0;
		}
	}
}

*/
void mfp_datareceive_handler(uint8_t *p, uint16_t len)
{
    static uint32_t count = 0;
    uint8_t checkSum = 0, checkSum1 = 0;
    const uint32_t MAX_BUF_SIZE = sizeof(g_Sync_RX.rawData);  // 1024字节

#if ENABLE_MFP_DETAIL_LOG
    static uint32_t rx_count = 0;
    printf("[RX#%u] recv %d bytes, total=%d\n", ++rx_count, len, count + len);
#endif

    // 缓冲区溢出保护（防御性编程）
    if (count + len > MAX_BUF_SIZE) {
        printf("[MFP_RX] ERROR: Buffer overflow! count=%d + len=%d > %d, reset\n", 
               count, len, MAX_BUF_SIZE);
        count = 0;
        memset(g_Sync_RX.rawData, 0, sizeof(g_Sync_RX.rawData));
        return;
    }

    memcpy(g_Sync_RX.rawData + count, p, len);
    count += len;

    // 判断是否至少收到了 "帧头+长度字段"
    if (count >= 2) {
        uint16_t frame_len = g_Sync_RX.Syncdata.length;

        // 长度范围判断，防止乱包
        if (frame_len > 80 || frame_len < 30) {
            printf("[MFP_RX] ERROR: Invalid frame_len=%d! Reset (probable stray byte)\n", frame_len);
            count = 0;
            memset(g_Sync_RX.rawData, 0, sizeof(g_Sync_RX.rawData));
            return;
        }

        // 收够了（长度+校验+帧头等）
        if (count >= frame_len + 3) {
            checkSum  = rxCalcCheckSum();
            checkSum1 = g_Sync_RX.Syncdata.data[frame_len];

            if (checkSum == checkSum1 && g_Sync_RX.rawData[0] != 0) {
                // 校验通过，保存到独立的解析缓冲区（带线程安全保护）
                if (g_MFP_Parsed_Mutex != NULL && 
                    xSemaphoreTake(g_MFP_Parsed_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    g_MFP_Parsed_Len = frame_len + 3;
                    memcpy(g_MFP_Parsed_Frame, g_Sync_RX.rawData, g_MFP_Parsed_Len);
                    xSemaphoreGive(g_MFP_Parsed_Mutex);
#if ENABLE_MFP_DETAIL_LOG
                   //  printf("[RX#%u] CRC OK, saved to parsed buffer, len=%d\n", rx_count, g_MFP_Parsed_Len);
#endif
                } else {
                    printf("[MFP_RX] WARNING: Failed to acquire mutex or mutex not initialized\n");
                }
                
                mfp_queue_pop_send();
            } else {
                printf("[MFP_RX] CRC FAIL! calc=%02X recv=%02X\n", checkSum, checkSum1);
            }
            
            // 清空接收缓冲区，为下一帧做准备
            count = 0;
            memset(g_Sync_RX.rawData, 0, sizeof(g_Sync_RX.rawData));
        }
#if ENABLE_MFP_DETAIL_LOG
        else {
            printf("[RX#%u] Wait more: %d < %d\n", rx_count, count, frame_len + 3);
        }
#endif
    }
}

#define MFP_RX_BUF_SIZE 256

typedef struct {
    uint8_t buf[MFP_RX_BUF_SIZE];
    uint16_t count;
} mfp_rx_buffer_t;

static mfp_rx_buffer_t rx_buf = {0};

// 查找有效帧头
static int find_valid_header(const uint8_t *buf, uint16_t count, uint16_t *header_pos)
{
    for (uint16_t i = 0; i < count - 1; i++) {
        uint8_t length = buf[i];
        uint8_t type = buf[i + 1];
        
        // 检查是否为有效的帧头
        if (length >= 5 && length <= 100 && 
            (type == 0x01 || type == 0x02 || type == 0x07)) {
            *header_pos = i;
            return 1; // 找到有效帧头
        }
    }
    return 0; // 未找到有效帧头
}

static uint8_t calc_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0xFF;
    for (int i = 0; i < len; i++) sum -= data[i];
    return sum;
}

static void MFP_DataReceive_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config={
        .baud_rate = 38400,                // 波特率38400
        .data_bits = UART_DATA_8_BITS,                  // 8位数据位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,          // 无硬件流控制
        .parity = UART_PARITY_EVEN,                     // 偶校验
        .stop_bits = UART_STOP_BITS_1                   // 1位停止位
    };

    uart_event_t event;

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 512, 20, &uart2_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));    // 设置接收超时，减少数据分帧（主控盒49字节需约12ms，设置为20个字符时间约5ms）
    ESP_ERROR_CHECK(uart_set_rx_timeout(ECHO_UART_PORT_NUM, 20));
    // Configure a temporary buffer for the incoming data
    uint8_t *dtmp  = (uint8_t *) malloc(BUF_SIZE);

    while (1)
    {
        //Waiting for UART event.
        if (xQueueReceive(uart2_queue, (void *)&event, (TickType_t)portMAX_DELAY)) 
        {
            // ESP_LOGI("UART", "UART event received: %d", event.type);
            memset(dtmp, 0, BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", ECHO_UART_PORT_NUM);
            switch (event.type)
            {
            //Event of UART receving data
            case UART_DATA:
#if DEBUG_MFP_UART_RX_LOG
                printf("MFP_U2 UART_DATA read_bytes=%d (buf_max=%d)\n", (int)event.size, BUF_SIZE);
#endif
                uart_read_bytes(ECHO_UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);
                mfp_datareceive_handler(dtmp,event.size);   // 一次性收完一帧
               // ESP_LOGI(TAG, "[DATA EVT]:");
                //uart_write_bytes(ECHO_UART_PORT_NUM, (const char*) dtmp, event.size);
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(ECHO_UART_PORT_NUM);
                xQueueReset(uart2_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(ECHO_UART_PORT_NUM);
                xQueueReset(uart2_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                // uart_flush_input(ECHO_UART_PORT_NUM);  // 清空当前错误帧
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
            /*
                uart_get_buffered_data_len(ECHO_UART_PORT_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(ECHO_UART_PORT_NUM);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(ECHO_UART_PORT_NUM);
                } else {
                    uart_read_bytes(ECHO_UART_PORT_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[ECHO_UART_PORT_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(ECHO_UART_PORT_NUM, pat, ECHO_UART_PORT_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "read data: %s", dtmp);
                    ESP_LOGI(TAG, "read pat : %s", pat);
                }
            */
                ESP_LOGI(TAG, "uart pattern detected");
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }            
    }
}

/*
*  10ms 定时器 

void my_timer_callback(TimerHandle_t xTimer)
{
    if(g_MFPData_t.syncModeSendInterval > 0)
		g_MFPData_t.syncModeSendInterval--;
    
}

void start_10ms_timer()
{
    my_timer = xTimerCreate("myTimer", pdMS_TO_TICKS(TIMER_PERIOD_MS), pdTRUE, NULL, my_timer_callback);
    xTimerStart(my_timer, 0);
}
*/
static void handle_snore_trigger_task(void *arg)
{
    while (1) 
    {       
 
        if (device_info->snore->snore_state.triggered_flag || device_info->snore->snore_state.triggered_flag_demo) // 处理打鼾干预（不重复发就不清 mfp_tx_ready） 
        {
            if (device_info->snore->snore_state.triggered_flag) {
                handle_snore_trigger(false);
            } else {
                // printf("demo triggerd \r\n");
                handle_snore_trigger(true);
            }
            g_system_flag.mfp_tx_ready = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}


/*
 *  解析MQTT/BLE 数据包
 *  data: AA010805010100000000f8XX
 */
int mqtt_ble_data_parser_cb(uint8_t *data)
{
    uint32_t key_value = 0;
    if (data[0] == 0xAA)    // 帧头判断
    {
        // 要加入和校验的（现在还没加）
        nvs_handle nvs_config_handler;
        char *send_json_value = NULL;
        // 解析
        switch (data[1])                            
        {
            case 0x00:
                if(data[4] == 0x01)     // 打鼾干预参数查询
                {
                    send_json_value = (char *)malloc(512);
                    if (send_json_value == NULL) {
                        printf("内存分配失败！\n");
                        return -1;
                    }
                    memset(send_json_value, 0, 512);
                    check_stack_space();
                    // 构建 JSON 字符串
                    sprintf(send_json_value, "{\"id\":\"%s\",\"ts\":%d,\"type\":16,\"report\":\"snore_param\",\"data\":{"
                                            "\"triggered_flag\":%d,"
                                            "\"triggered_flag_demo\":%d,"
                                            "\"upHoldTime\":%d,"
                                            "\"threshold\":%d,"
                                            "\"threshold5s\":%d,"
                                            "\"pwm\":%d,"
                                            "\"tmr\":%d,"
                                            "\"lastTriggeredTime\":%d,"
                                            "\"isIntervening\":%d}}",             
                                            device_info->id,
                                            device_info->utc.time_stamp,
                                            device_info->snore->snore_state.triggered_flag,                  
                                            device_info->snore->snore_state.triggered_flag_demo,
                                            device_info->snore->snore_parameters.up_hold_time_s,
                                            device_info->snore->snore_parameters.threshold,
                                            device_info->snore->snore_parameters.threshold_5s,
                                            device_info->snore->snore_parameters.pwm,
                                            device_info->snore->snore_parameters.tmr,
                                            device_info->snore->snore_state.last_triggered_time_s,
                                            device_info->snore->snore_state.is_intervening);
                    
                    printf("mqtt put打鼾干预参数: %s\n", send_json_value);

                    // MQTT 发送
                    if (get_mqtt_status()) {
                        if (mqtt_send_mutex == true) {
                            mqtt_send_mutex = false;
                            esp_mqtt_client_publish(client, mc_cli_data_publish_topic, (char *)send_json_value, strlen((char *)send_json_value), 0, 0);
                            mqtt_send_mutex = true;
                        }
                    }
                    if (send_json_value != NULL) {
                        free(send_json_value);
                    }
                }else if(data[4] == 0x02) // 主控盒参数查询
                {
                    // printf("查询主控盒参数\n");
                    MFP_Cmd(data[4],1);
                }else if(data[4] == 0x04) // 闹钟指令控制
                {
                    printf("闹钟指令控制 \r\n");
                    /* APP 下发闹钟指令后，120s 内禁止新的打鼾干预触发（不打断当前干预） */
                    snore_blocked_until_s = (uint32_t)(xTaskGetTickCount() / pdMS_TO_TICKS(1000)) + 120;
                }
                break;

            case 0x01:
                // printf("按键控制 \r\n");
                // 提取键值（注意是低字节在前）
                key_value = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8];
                // printf("键值=0x%08X\n", key_value);
                // 判断是否为指定键值
                if (key_value == 0x00000040 || key_value == 0x00000008)
                {
                    // 清除打鼾干预状态
                    device_info->snore->snore_state.triggered_flag = false;
                    device_info->snore->snore_state.triggered_flag_demo = false;
                    device_info->snore->snore_state.is_intervening = false;
                    device_info->snore->snore_state.last_triggered_time_s = 0;
                    // printf("清除打鼾干预状态\n");
                }
                mfp_queue_push(&data[3], data[2], 2);       // 将数据放入队列
                break;

            case 0x02:
                printf("演示流程 \r\n");
                snore_parameters_demo.up_hold_time_s = ((uint32_t)data[5] << 24) | ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 8)  | ((uint32_t)data[8]);
                snore_parameters_demo.threshold_5s = data[9];
                snore_parameters_demo.threshold_5s = data[10];
                snore_parameters_demo.pwm = data[11];
                snore_parameters_demo.tmr = data[12];
                
                device_info->snore->snore_state.triggered_flag_demo = true;  // 演示触发标志
                printf("snore_demo: up_hold_time_s = %d, threshold_5s = %d, threshold = %d, pwm = %d, tmr = %d\n",snore_parameters_demo.up_hold_time_s,
                                                                                                snore_parameters_demo.threshold_5s,
                                                                                                snore_parameters_demo.threshold,
                                                                                                snore_parameters_demo.pwm,
                                                                                                snore_parameters_demo.tmr);
                            
                // handle_snore_trigger(device_info->snore->snore_state.triggered_flag_demo);
                break;
                
            case 0x03:
                printf("参数设置 \r\n");
                device_info->snore->snore_parameters.up_hold_time_s = ((uint32_t)data[5] << 24) | ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 8) | ((uint32_t)data[8]);
                device_info->snore->snore_parameters.threshold_5s  = data[9];
                device_info->snore->snore_parameters.threshold     = data[10];       //打鼾包阈值
                device_info->snore->snore_parameters.pwm           = data[11];
                device_info->snore->snore_parameters.tmr           = data[12];

                printf("snore: up_hold_time_s = %d, threshold_5s = %d, threshold = %d, pwm = %d, tmr = %d\n",device_info->snore->snore_parameters.up_hold_time_s,
                                                                                                            device_info->snore->snore_parameters.threshold_5s,
                                                                                                            device_info->snore->snore_parameters.threshold,
                                                                                                            device_info->snore->snore_parameters.pwm,
                                                                                                            device_info->snore->snore_parameters.tmr);
                                                                                                            
                ESP_ERROR_CHECK(nvs_open("config_cfg", NVS_READWRITE, &nvs_config_handler));                                                                         
                ESP_ERROR_CHECK(nvs_set_blob(nvs_config_handler, "snore_param",  &device_info->snore->snore_parameters, sizeof(snore_parameters_t)));
                ESP_ERROR_CHECK(nvs_commit(nvs_config_handler));
                nvs_close(nvs_config_handler);
                break;

            case 0x04:
            printf("停止命令 \r\n");
                mfp_tx_queue_clear();                       // 为了避免溢出导致未收到停止，清空队列
                mfp_queue_push(&data[3], data[2], 2);       // 将数据放入队列
                break;
            case 0x05:
            printf("停止命令 \r\n");
                mfp_tx_queue_clear();                       // 为了避免溢出导致未收到停止，清空队列
                // mfp_queue_push(&data[3], data[2], 2);       // 将数据放入队列
                break;
            default:
                return -2;
        }
    }
    // check_stack_space();
    return 0;
}




void mqtt_key_parser_task(void *pv)
{
    while (1)
    {
        if (xQueueReceive(device_info->mqtt_key->xQueue, device_info->mqtt_key->data_rec.value, portMAX_DELAY))
        {
            mqtt_ble_data_parser_cb(device_info->mqtt_key->data_rec.value);
        }
    }
    vTaskDelete(NULL);
}

void wifi_check_task(void *pvParameters)
{
     bool need_reconnect = false;

    while (1)
    {
        need_reconnect = false;

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
            ESP_LOGW(TAG, "WiFi not connected.");
            need_reconnect = true;
        }

        tcpip_adapter_ip_info_t ip_info;
        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
        if (ip_info.ip.addr == 0) {
            ESP_LOGW(TAG, "No IP address.");
            need_reconnect = true;
        }

        if (need_reconnect) {
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(1000));  // 给点延时避免频繁重连
            esp_wifi_connect();
        }
        vTaskDelay(pdMS_TO_TICKS(30000)); // 每 30 秒检查一次
    }
    vTaskDelete(NULL);
}



void vlsit_task(void *pv){

    static char task_list_buf[1024];
    static char task_runtime_buf[1024];
    while(1)
    {

        vTaskList(task_list_buf);
        printf("Task Name\tStatus\tPrio\tStack\tTaskNum\n");
        printf("%s\n", task_list_buf);


        vTaskGetRunTimeStats(task_runtime_buf);
        printf("Task Name\tTime\t\tPercent\n");
        printf("%s\n", task_runtime_buf);
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

#if SU3_USE_NEW_STACK
#define SU3_SETUP_CMD_TIMEOUT_MS 3000U
#define SU3_LIST_TEST_ENABLE     1
#define SU3_LIST_TIMEOUT_MS      3000U
#define SU3_LIST_POLL_MS         (60U * 1000U)
#define SU3_REPORT_ZERO_TEST_ENABLE 1
#define SU3_REPORT_RX_SETTLE_MS  5000U

static char s_su3_devic_id_flag;

static int su3_side_index(su3_addr_t src)
{
    su3_side_t side = su3_addr_to_side(src);
    if (side < SU3_SIDE_COUNT) {
        return (int)side;
    }
    /* 设址前：按 pending_src 反查 */
    for (int i = 0; i < SU3_SENSOR_SIDE_MAX; i++) {
        if (g_su3_sensor[i].hello_seen && g_su3_sensor[i].pending_src == src) {
            return i;
        }
    }
    return -1;
}

static void su3_on_hello(su3_addr_t src, void *user)
{
    int idx = su3_side_index(src);
    (void)user;
    if (idx < 0) {
        /* 上电未设地址时 hello 可能来自非 03/06：按空闲槽分配 */
        for (int i = 0; i < SU3_SENSOR_SIDE_MAX; i++) {
            if (!g_su3_sensor[i].hello_seen) {
                idx = i;
                break;
            }
        }
    }
    if (idx < 0) {
        ESP_LOGW(TAG, "hello from unknown addr=0x%02X (no free side)", src);
        return;
    }
    g_su3_sensor[idx].hello_seen = true;
    g_su3_sensor[idx].need_setup = true;
    g_su3_sensor[idx].pending_src = src;
    ESP_LOGI(TAG, "hello side=%d src=0x%02X", idx, src);
}

static void su3_on_cli_push(su3_addr_t src, const char *text, const char *dev_id, void *user)
{
    int idx = su3_side_index(src);
    (void)user;
    if (idx < 0) {
        return;
    }
    if (dev_id != NULL && dev_id[0] != '\0') {
        strncpy(g_su3_sensor[idx].device_id, dev_id, sizeof(g_su3_sensor[idx].device_id) - 1);
        /* 兼容旧逻辑：写入全局 device_id 供调试显示 */
        if (device_id[0] == '\0') {
            strncpy(device_id, dev_id, sizeof(device_id) - 1);
            device_id[sizeof(device_id) - 1] = '\0';
        }
    }
    if (text != NULL && strstr(text, "list updata") != NULL) {
        ESP_LOGI(TAG, "list updata side=%d: %s", idx, text);
        set_mode_flag_config(0);
    }
}

static void su3_publish_json(const char *topic, const char *json)
{
    if (topic == NULL || json == NULL) {
        return;
    }
    if (get_mqtt_status() && device_info->data_up_switch && sleep_up_flag == 0) {
        if (mqtt_send_mutex) {
            mqtt_send_mutex = false;
            esp_mqtt_client_publish(client, topic, json, strlen(json), 0, 0);
            mqtt_send_mutex = true;
        }
    }
    if (get_ble_status() && device_info->data_up_switch) {
        esp_ble_gatts_send_indicate(device_info->ble->gatts_if,
                                    device_info->ble->conn_id,
                                    device_info->ble->handle,
                                    strlen(json), (uint8_t *)json, false);
    }
}

static bool su3_log_report_raw(int side, uint8_t topic, const char *name,
                               const char *device_id, int32_t timestamp,
                               const uint8_t *data, int32_t data_len)
{
    if (data_len < 0 || data_len > QS_PB_RAW_DATA_LEGNTH) {
        ESP_LOGE(TAG, "[SU3_REPORT] invalid side=%d topic=%u name=%s len=%d",
                 side, (unsigned)topic, name, (int)data_len);
        return false;
    }

    ESP_LOGI(TAG, "[SU3_REPORT] side=%d topic=%u name=%s id=%s ts=%d len=%d",
             side, (unsigned)topic, name, device_id, (int)timestamp, (int)data_len);
    if (data_len > 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, (size_t)data_len, ESP_LOG_INFO);
    }
    return true;
}

static void su3_on_topic(su3_addr_t src, uint8_t topic,
                         const uint8_t *pb_data, size_t pb_len, void *user)
{
    int idx = su3_side_index(src);
    char side_ch;
    char json[512];
    (void)user;

    if (idx < 0 || pb_data == NULL) {
        return;
    }
    side_ch = (idx == SU3_SIDE_LEFT) ? 'L' : 'R';

    if (topic == SU3_TOPIC_SENSOR_1SEC) {
        qs_pb_msg_sensor_1sec_info msg;
        memset(&msg, 0, sizeof(msg));
        // if (qs_pb_sensor_1sec_info_decode((char *)pb_data, pb_len, &msg) != QS_SUCCESS) {
        //     return;
        // }
        /* 解析1秒实时数据 */
        /* 解析1秒实时数据 */
        qs_ret_code_t ret = qs_pb_sensor_1sec_info_decode(
            (char *)pb_data,
            pb_len,
            &msg
        );

        // if (ret != QS_SUCCESS) {
        //     ESP_LOGE(TAG,
        //             "SU3 1s decode failed: src=0x%02X len=%u ret=%d",
        //             src,
        //             (unsigned)pb_len,
        //             (int)ret);
        //     return;
        // }

        /* 解析成功后打印字段 */
        // ESP_LOGI(TAG,
        //      "SU3 1s decode ok: id=%s ts=%d seq=%d "
        //      "status=0x%X heart=%d breath=%d "
        //      "sdata=%d pdata=%d sign=0x%X",
        //      msg.device_id,
        //      (int)msg.timestamp,
        //      (int)msg.sequence,
        //      (unsigned)msg.status,
        //      (int)msg.heartbeat,
        //      (int)msg.breath_rate,
        //      (int)msg.sdata,
        //      (int)msg.pdata,
        //      (unsigned)msg.sign);
        g_su3_sensor[idx].info_1s = msg;
        g_su3_sensor[idx].fresh_1s = true;
        if (!get_5s_flag) {
            get_5s_flag = true;
        }
        /* 兼容旧打鼾/显示：左路同步到 user_5s（status 展平为近似） */
        if (idx == SU3_SIDE_LEFT && user_5s_sensor_info != NULL) {
            strncpy(user_5s_sensor_info->device_id, msg.device_id, sizeof(user_5s_sensor_info->device_id) - 1);
            user_5s_sensor_info->timestamp = msg.timestamp;
            user_5s_sensor_info->sequence = msg.sequence;
            user_5s_sensor_info->heartbeat = msg.heartbeat;
            user_5s_sensor_info->breathRate = msg.breath_rate;
            memset(user_5s_sensor_info->status, 0, sizeof(user_5s_sensor_info->status));
            /* bit2=打鼾 → 旧逻辑用 status[]==4；此处用单点近似 */
            user_5s_sensor_info->status[0] = (msg.status & 0x04) ? 4 : ((msg.status & 0x01) ? 1 : 0);
        }
        snprintf(json, sizeof(json),
                 "{\"id\":\"%s\",\"ts\":%d,\"type\":1,\"side\":\"%c\",\"sensor\":%u,"
                 "\"data\":{\"sensor_ts\":%d,\"heart\":%d,\"breath\":%d,\"status\":%d,"
                 "\"sdata\":%d,\"pdata\":%d,\"sign\":%d}}",
                 device_info->id, device_info->utc.time_stamp, side_ch, (unsigned)src,
                 (int)msg.timestamp, msg.heartbeat, msg.breath_rate, msg.status,
                 msg.sdata, msg.pdata, msg.sign);
        su3_publish_json(user_5s_data_publish_topic, json);
        g_su3_sensor[idx].fresh_1s = false;
        return;
    }

    if (topic == SU3_TOPIC_SENSOR_1MIN) {
        qs_pb_msg_sensor_1min_info msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_sensor_1min_info_decode((char *)pb_data, pb_len, &msg) != QS_SUCCESS) {
            return;
        }
        g_su3_sensor[idx].info_1min = msg;
        g_su3_sensor[idx].fresh_1min = true;
        if (idx == SU3_SIDE_LEFT && user_60s_sensor_info != NULL) {
            memcpy(user_60s_sensor_info, &msg, sizeof(msg));
        }
        snprintf(json, sizeof(json),
                 "{\"id\":\"%s\",\"ts\":%d,\"type\":2,\"side\":\"%c\",\"sensor\":%u,"
                 "\"data\":{\"sensor_ts\":%d,\"bed\":%d,\"heart\":%d,\"breath\":%d,"
                 "\"Mmin\":%d,\"Mmean\":%d,\"NSD\":%d,\"NPD\":%d,\"SBP\":%d,\"DBP\":%d}}",
                 device_info->id, device_info->utc.time_stamp, side_ch, (unsigned)src,
                 (int)msg.timestamp, msg.on_off_bed, msg.heartbeat, msg.breath_rate,
                 msg.Mmin, msg.Mmean, msg.NSD, msg.Npd, msg.SBP, msg.DBP);
        su3_publish_json(user_60s_data_publish_topic, json);
        g_su3_sensor[idx].fresh_1min = false;
        return;
    }

    if (topic == SU3_TOPIC_SENSOR_SAE) {
        qs_pb_msg_sleep_apnea_info msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_sleep_apnea_info_decode((char *)pb_data, pb_len, &msg) != QS_SUCCESS) {
            return;
        }
        g_su3_sensor[idx].info_sa = msg;
        snprintf(json, sizeof(json),
                 "{\"id\":\"%s\",\"ts\":%d,\"type\":15,\"side\":\"%c\",\"sensor\":%u,"
                 "\"data\":{\"status_flag\":%d}}",
                 device_info->id, device_info->utc.time_stamp, side_ch, (unsigned)src,
                 msg.status_flag);
        su3_publish_json(user_sa_data_publish_topic, json);
        return;
    }

    /* 睡眠报告 topic 5~12（原 single_cmd_parse） */
    if (topic == SU3_TOPIC_SLEEP_CYCLE) {
        qs_pb_msg_sleep_cycle_repo msg;
        char send_json_value[1024];
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_sleep_cycle_repo_decode((char *)pb_data, pb_len, &msg) != QS_SUCCESS) {
            ESP_LOGE(TAG, "[SU3_REPORT] topic=5 decode failed side=%d len=%u",
                     idx, (unsigned)pb_len);
            return;
        }
        ESP_LOGI(TAG, "[SU3_REPORT] side=%d topic=5 id=%s ts=%d calResult=%d "
                      "startTime=%d totalSleepTime=%d efficiency=%d quality=%d",
                 idx, msg.device_id, (int)msg.timestamp, (int)msg.cal_result,
                 (int)msg.start_time, (int)msg.total_sleep_time,
                 (int)msg.sleep_efficiency, (int)msg.sleep_quality);
        ESP_LOGI(TAG, "[SU3_REPORT] side=%d topic=5 turnover=%d latency=%d "
                      "offBed=%d cRSD=%d slop1=%d slop2=%d osa=%d avgSA=%d maxSA=%d",
                 idx, (int)msg.turnover_times, (int)msg.sleep_latency,
                 (int)msg.off_bed_times, (int)msg.cRSD, (int)msg.slop1,
                 (int)msg.slop2, (int)msg.oSA_times, (int)msg.Ave_SA_time,
                 (int)msg.Longest_SA_time);
        snprintf(send_json_value, sizeof(send_json_value),
                 "{\"id\":\"%s\",\"ts\":%d,\"type\":5,\"report\":\"%s\",\"side\":\"%c\","
                 "\"data\":{\"calResult\":%d,\"startTime\":%d,\"totalSleepTime\":%d,"
                 "\"sleepEfficiency\":%d,\"sleepQuality\":%d,\"turnoverTimes\":%d,"
                 "\"sleepLatency\":%d,\"offBedTimes\":%d,\"cRSD\":%d,\"slop1\":%d,"
                 "\"slop2\":%d,\"osaTimes\":%d,\"avgSA\":%d,\"maxSA\":%d}}",
                 device_info->id, device_info->utc.time_stamp, json_report_name, side_ch,
                 msg.cal_result, msg.start_time, msg.total_sleep_time,
                 msg.sleep_efficiency, msg.sleep_quality, msg.turnover_times,
                 msg.sleep_latency, msg.off_bed_times, msg.cRSD, msg.slop1,
                 msg.slop2, msg.oSA_times, msg.Ave_SA_time, msg.Longest_SA_time);
        su3_publish_json(user_sleep_data_publish_topic, send_json_value);
        return;
    }

    if (topic == SU3_TOPIC_STATE_RAW) {
        qs_pb_msg_state_raw_data msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_state_raw_data_decode((char *)pb_data, pb_len, &msg) == QS_SUCCESS) {
            if (su3_log_report_raw(idx, topic, "sleep_state", msg.device_id,
                                   msg.timestamp, msg.data, msg.data_len)) {
                report_to_aliyun(topic, msg.data, (uint16_t)msg.data_len);
            }
        } else {
            ESP_LOGE(TAG, "[SU3_REPORT] topic=6 decode failed side=%d len=%u",
                     idx, (unsigned)pb_len);
        }
        return;
    }
    if (topic == SU3_TOPIC_HEART_RAW) {
        qs_pb_msg_heartbeat_raw_data msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_heartbeat_raw_data_decode((char *)pb_data, pb_len, &msg) == QS_SUCCESS) {
            if (su3_log_report_raw(idx, topic, "heart_rate", msg.device_id,
                                   msg.timestamp, msg.data, msg.data_len)) {
                report_to_aliyun(topic, msg.data, (uint16_t)msg.data_len);
            }
        } else {
            ESP_LOGE(TAG, "[SU3_REPORT] topic=7 decode failed side=%d len=%u",
                     idx, (unsigned)pb_len);
        }
        return;
    }
    if (topic == SU3_TOPIC_BREATH_RAW) {
        qs_pb_msg_breathrate_raw_data msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_breathrate_raw_data_decode((char *)pb_data, pb_len, &msg) == QS_SUCCESS) {
            if (su3_log_report_raw(idx, topic, "breath_rate_x10", msg.device_id,
                                   msg.timestamp, msg.data, msg.data_len)) {
                report_to_aliyun(topic, msg.data, (uint16_t)msg.data_len);
            }
        } else {
            ESP_LOGE(TAG, "[SU3_REPORT] topic=8 decode failed side=%d len=%u",
                     idx, (unsigned)pb_len);
        }
        return;
    }
    if (topic == SU3_TOPIC_MOVE_RAW) {
        qs_pb_msg_movement_raw_data msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_movement_raw_data_decode((char *)pb_data, pb_len, &msg) == QS_SUCCESS) {
            if (su3_log_report_raw(idx, topic, "movement", msg.device_id,
                                   msg.timestamp, msg.data, msg.data_len)) {
                report_to_aliyun(topic, msg.data, (uint16_t)msg.data_len);
            }
        } else {
            ESP_LOGE(TAG, "[SU3_REPORT] topic=9 decode failed side=%d len=%u",
                     idx, (unsigned)pb_len);
        }
        return;
    }
    if (topic == SU3_TOPIC_SNORE_RAW) {
        qs_pb_msg_snore_raw_data msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_snore_raw_data_decode((char *)pb_data, pb_len, &msg) == QS_SUCCESS) {
            if (su3_log_report_raw(idx, topic, "snore", msg.device_id,
                                   msg.timestamp, msg.data, msg.data_len)) {
                report_to_aliyun(topic, msg.data, (uint16_t)msg.data_len);
            }
        } else {
            ESP_LOGE(TAG, "[SU3_REPORT] topic=10 decode failed side=%d len=%u",
                     idx, (unsigned)pb_len);
        }
        return;
    }
    if (topic == SU3_TOPIC_SBP_RAW) {
        qs_pb_msg_sbp_raw_data msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_sbp_raw_data_decode((char *)pb_data, pb_len, &msg) == QS_SUCCESS) {
            if (su3_log_report_raw(idx, topic, "sbp", msg.device_id,
                                   msg.timestamp, msg.data, msg.data_len)) {
                report_to_aliyun(topic, msg.data, (uint16_t)msg.data_len);
            }
        } else {
            ESP_LOGE(TAG, "[SU3_REPORT] topic=11 decode failed side=%d len=%u",
                     idx, (unsigned)pb_len);
        }
        return;
    }
    if (topic == SU3_TOPIC_DBP_RAW) {
        qs_pb_msg_dbp_raw_data msg;
        memset(&msg, 0, sizeof(msg));
        if (qs_pb_dbp_raw_data_decode((char *)pb_data, pb_len, &msg) == QS_SUCCESS) {
            if (su3_log_report_raw(idx, topic, "dbp", msg.device_id,
                                   msg.timestamp, msg.data, msg.data_len)) {
                report_to_aliyun(topic, msg.data, (uint16_t)msg.data_len);
            }
        } else {
            ESP_LOGE(TAG, "[SU3_REPORT] topic=12 decode failed side=%d len=%u",
                     idx, (unsigned)pb_len);
        }
        return;
    }

    ESP_LOGD(TAG, "topic=%u side=%d pb_len=%u ignored", topic, idx, (unsigned)pb_len);
}

/** hello 后串行 set addr / set mode（禁止在 RX 回调里 su3_cli_exec） */
static void su3_setup_task(void *pv)
{
    (void)pv;
    while (1) {
        for (int i = 0; i < SU3_SENSOR_SIDE_MAX; i++) {
            if (!g_su3_sensor[i].need_setup) {
                continue;
            }
            su3_addr_t target = su3_side_to_addr((su3_side_t)i);
            su3_addr_t dest = g_su3_sensor[i].pending_src
                                  ? g_su3_sensor[i].pending_src
                                  : target;
            char rsp[64];
            char cmd[32];
            esp_err_t addr_err;
            esp_err_t mode_err;
            /* CLI 文本用逻辑地址 3/6，不是帧字节 0x33/0x36 */
            snprintf(cmd, sizeof(cmd), "set addr %u", (unsigned)SU3_ADDR_OF(target));
            ESP_LOGI(TAG, "setup side=%d dest=0x%02X: %s", i, dest, cmd);
            addr_err = su3_cli_exec(dest, cmd, rsp, sizeof(rsp), SU3_SETUP_CMD_TIMEOUT_MS);
            if (addr_err == ESP_OK) {
                ESP_LOGI(TAG, "setup addr result=0x%x (%s), rsp=%s",
                         (unsigned)addr_err, esp_err_to_name(addr_err), rsp);
            } else {
                ESP_LOGW(TAG, "setup addr result=0x%x (%s), side=%d dest=0x%02X",
                         (unsigned)addr_err, esp_err_to_name(addr_err), i, dest);
            }
            /* 设址后后续命令发往目标地址 */
            mode_err = su3_cli_exec(target, "set mode 4", rsp, sizeof(rsp),
                                    SU3_SETUP_CMD_TIMEOUT_MS);
            if (mode_err == ESP_OK) {
                ESP_LOGI(TAG, "setup mode result=0x%x (%s), rsp=%s",
                         (unsigned)mode_err, esp_err_to_name(mode_err), rsp);
            } else {
                ESP_LOGW(TAG, "setup mode result=0x%x (%s), side=%d dest=0x%02X",
                         (unsigned)mode_err, esp_err_to_name(mode_err), i, target);
            }

            if (addr_err == ESP_OK && mode_err == ESP_OK) {
                const char *test_cmd = "version";
                char test_rsp[128] = {0};

                ESP_LOGI(TAG,
                        "[SU3_TX_TEST] side=%d dest=0x%02X cmd=\"%s\"",
                        i, target, test_cmd);

                esp_err_t test_err = su3_cli_exec(
                    target,
                    test_cmd,
                    test_rsp,
                    sizeof(test_rsp),
                    1000
                );

                if (test_err == ESP_OK && test_rsp[0] != '\0') {
                    ESP_LOGI(TAG,
                            "[SU3_RX_TEST] success side=%d src=0x%02X rsp=\"%s\"",
                            i, target, test_rsp);
                } else if (test_err == ESP_ERR_TIMEOUT) {
                    ESP_LOGW(TAG,
                            "[SU3_RX_TEST] timeout side=%d dest=0x%02X",
                            i, target);
                } else {
                    ESP_LOGE(TAG,
                            "[SU3_RX_TEST] failed side=%d err=0x%x (%s) rsp=\"%s\"",
                            i,
                            (unsigned)test_err,
                            esp_err_to_name(test_err),
                            test_rsp);
                }
                g_su3_sensor[i].need_setup = false;
                g_su3_sensor[i].setup_done = true;
                s_su3_devic_id_flag = 1;
                devic_id_flag = 1;
                ESP_LOGI(TAG, "setup completed side=%d", i);
            } else {
                g_su3_sensor[i].need_setup = true;
                g_su3_sensor[i].setup_done = false;
                ESP_LOGW(TAG, "setup incomplete side=%d, retrying", i);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

#if SU3_LIST_TEST_ENABLE
static bool su3_list_has_report_zero(const char *rsp)
{
    const char *line = rsp;

    while (line != NULL && *line != '\0') {
        while (*line == '\r' || *line == '\n') {
            line++;
        }
        if (line[0] == '0' && line[1] == ' ') {
            return true;
        }
        line = strchr(line, '\n');
        if (line != NULL) {
            line++;
        }
    }
    return false;
}

/**
 * 主动查询左右传感器的报告目录。
 * 必须运行在独立任务，不能放进 su3_rx_task/on_cli_push：同步等待 list
 * 应答期间仍需要 RX 任务持续收包。
 */
static void su3_list_test_task(void *pv)
{
    static bool _report_zero_sent[SU3_SENSOR_SIDE_MAX];
    (void)pv;

    while (1) {
        bool queried = false;

        for (int i = 0; i < SU3_SENSOR_SIDE_MAX; i++) {
            char rsp[SU3_CLI_RSP_DEFAULT] = {0};
            su3_addr_t target;
            esp_err_t err;

            if (!g_su3_sensor[i].setup_done) {
                continue;
            }

            queried = true;
            target = su3_side_to_addr((su3_side_t)i);
            ESP_LOGI(TAG, "[SU3_LIST_TEST] TX side=%d dest=0x%02X cmd=\"list\"",
                     i, target);

            err = su3_cli_list(target, rsp, sizeof(rsp), SU3_LIST_TIMEOUT_MS);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "[SU3_LIST_TEST] RX side=%d src=0x%02X:\n%s",
                         i, target, rsp[0] != '\0' ? rsp : "<empty>");
#if SU3_REPORT_ZERO_TEST_ENABLE
                if (!_report_zero_sent[i] && su3_list_has_report_zero(rsp)) {
                    snprintf(json_report_name, sizeof(json_report_name),
                             "report-0-side-%d", i);
                    err = su3_cli_fire(target, "report 0");
                    if (err == ESP_OK) {
                        _report_zero_sent[i] = true;
                        ESP_LOGI(TAG, "[SU3_REPORT_TEST] TX side=%d dest=0x%02X "
                                      "cmd=\"report 0\"",
                                 i, target);
                        /* report 0 异步返回 topic 5~12，本轮不再向另一侧发命令。 */
                        vTaskDelay(pdMS_TO_TICKS(SU3_REPORT_RX_SETTLE_MS));
                        break;
                    }
                    ESP_LOGW(TAG, "[SU3_REPORT_TEST] send failed side=%d dest=0x%02X "
                                  "err=0x%x (%s)",
                             i, target, (unsigned)err, esp_err_to_name(err));
                }
#endif
            } else {
                ESP_LOGW(TAG, "[SU3_LIST_TEST] failed side=%d dest=0x%02X "
                              "err=0x%x (%s)",
                         i, target, (unsigned)err, esp_err_to_name(err));
            }

            /* 左右两路严格串行，并给传感器留出少量总线空闲时间。 */
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (queried) {
            vTaskDelay(pdMS_TO_TICKS(SU3_LIST_POLL_MS));
        } else {
            /* 等待 hello -> set addr -> set mode 4 完成。 */
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
#endif

static void su3_app_start(void)
{
    su3_config_t cfg = {
        .uart_port = UART_NUM_1,
        .tx_pin = UART1_TXD,
        .rx_pin = UART1_RXD,
        .baud_rate = 115200,
        .self_addr = SU3_ADDR_ESP32_DEFAULT,
        .addr_left = SU3_ADDR_LEFT_DEFAULT,
        .addr_right = SU3_ADDR_RIGHT_DEFAULT,
    };
    su3_handlers_t h = {
        .on_hello = su3_on_hello,
        .on_cli_push = su3_on_cli_push,
        .on_topic = su3_on_topic,
        .user = NULL,
    };

    if (su3_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "su3_init failed");
        return;
    }
    su3_set_handlers(&h);
    xTaskCreatePinnedToCore(su3_setup_task, "su3_setup", 4096, NULL, 4, NULL, 1);
#if SU3_LIST_TEST_ENABLE
    xTaskCreatePinnedToCore(su3_list_test_task, "su3_list", 4096, NULL, 3, NULL, 1);
#endif
    ESP_LOGI(TAG, "SU3 stack started (legacy uart_data_parser disabled)");
}
#endif /* SU3_USE_NEW_STACK */

void app_control_server(void)
{
    // 初始化MFP队列
    mfp_tx_queue_init();
    
    // 创建MFP解析缓冲区的互斥锁（线程安全保护）
    g_MFP_Parsed_Mutex = xSemaphoreCreateMutex();
    if (g_MFP_Parsed_Mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create MFP parsed mutex!");
    } else {
        ESP_LOGI(TAG, "MFP parsed mutex created successfully");
    }
    
    //ble 数据处理
    xTaskCreatePinnedToCore(ble_data_parser_task, "ble_task", 1024*6, NULL, 6, NULL, 1);   //1024*2//缩减1024*5
    //一键配网
    xTaskCreatePinnedToCore(one_key_config_wifi_task, "one_key_config_wifi_task", 1024*2, NULL, 6, NULL, 1);   //缩减1024*2
    
    su3_app_start();
    ESP_LOGI(TAG, "su3_app_start successfully");

    //utc 获取
    xTaskCreatePinnedToCore(utc_get_task, "utc_get", 1024*6, NULL, 3, NULL, 1);      //缩减4096*2   ok
    
    //系统控制任务（LED + 打鼾检测）- 量产必须保留
    xTaskCreatePinnedToCore(system_control_task, "sys_ctrl", 1024*2, NULL, 2, NULL, 1);
    
    xTaskCreatePinnedToCore(Task_scheduling, "Task_scheduling", 1024*2, NULL, 6, NULL, 1);//缩减2048*7
    // xTaskCreatePinnedToCore(key_task, "key_task", 1024, NULL, 6, NULL, 1);//缩减2048*7

    xTaskCreate(MFP_DataReceive_task, "MFP_DataReceive_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);     // mfp口的接收任务 1024*4

    xTaskCreate(handle_snore_trigger_task, "MFP_DataSend_task ", ECHO_TASK_STACK_SIZE, NULL, 7, NULL); // 打鼾干预任务 1024*4

    xTaskCreatePinnedToCore(mqtt_key_parser_task, "mqtt_key_parser_task", 1024*4, NULL, 5, NULL, 1);

    //调试显示任务 - 通过 ENABLE_DEBUG_DISPLAY 宏控制
#if ENABLE_DEBUG_DISPLAY
    xTaskCreatePinnedToCore(data_display_task, "data_display", 1024*4, NULL, 2, NULL, 1);
#endif
    // 性能监控任务 - 通过 ENABLE_PERFORMANCE_MON 宏控制
#if ENABLE_PERFORMANCE_MON
    xTaskCreatePinnedToCore(vlsit_task, "vlsit_task", 1024*4, NULL, 2, NULL, 1);  // 需要开启 FreeRTOS stats
#endif

    // xTaskCreatePinnedToCore(wifi_check_task, "wifi_check_task", 1024*4, NULL, 5, NULL, 1); 
}

void set_sleep_up_flag(uint8_t data)
{
    sleep_up_flag = data;
}
uint8_t get_sleep_up_flag(void)
{
    return sleep_up_flag;
}
void set_ota_now_flag(uint8_t data)
{
    ota_now_flag = data;
}
uint8_t get_ota_now_flag(void)
{
    return ota_now_flag;
}
//report_cli_data设置指令代码＋数据
void set_report_cli(uint8_t data1,uint8_t data2)
{
    report_cli_data[0] = data1;
    report_cli_data[1] = data2;
}
uint8_t get_devic_id_flag(void)
{
    return devic_id_flag;
}
void sensor_reboot_config(void)
{
    set_rtc_flag = 0;
    devic_id_flag = 0;
}

void set_mode_flag_config(uint8_t data)
{
    set_mode_flag = data;
}

uint8_t get_mode_flag_config(void)
{
    return set_mode_flag;
}

void check_stack_space(void)
{
    TaskHandle_t xTaskHandle = xTaskGetCurrentTaskHandle();
    UBaseType_t remainingStack = uxTaskGetStackHighWaterMark(xTaskHandle);

    printf("Remaining stack space: %u bytes\n", remainingStack);
}
