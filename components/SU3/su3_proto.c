/*
 * @Author: fzh
 * @Date: 2026-07-14 18:19:35
 * @LastEditors: fzh
 * @LastEditTime: 2026-07-14 18:19:35
 */
#include "su3_proto.h"
#include "qs_protobuf.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "su3_proto";

esp_err_t su3_proto_encode_cli(const char *device_id, int32_t timestamp,
                               const char *command,
                               uint8_t *out, size_t out_cap, size_t *out_len)
{
    qs_pb_msg_cli_command msg;
    char pb[160];
    size_t pb_len = sizeof(pb);
    qs_ret_code_t ret;

    if (command == NULL || out == NULL || out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_cap < 2U) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(&msg, 0, sizeof(msg));
    if (device_id != NULL && device_id[0] != '\0') {
        strncpy(msg.device_id, device_id, sizeof(msg.device_id) - 1U);
    } else {
        strncpy(msg.device_id, "SU20000001", sizeof(msg.device_id) - 1U);
    }
    strncpy(msg.command, command, sizeof(msg.command) - 1U);
    msg.timestamp = timestamp;

    ret = qs_pb_cli_command_encode(&msg, pb, &pb_len);
    if (ret != QS_SUCCESS) {
        ESP_LOGE(TAG, "cli encode fail ret=%d", (int)ret);
        return ESP_FAIL;
    }
    if (out_cap < (1U + pb_len)) {
        return ESP_ERR_INVALID_SIZE;
    }

    out[0] = (uint8_t)SU3_TOPIC_CLI;
    memcpy(&out[1], pb, pb_len);
    *out_len = 1U + pb_len;
    return ESP_OK;
}

esp_err_t su3_proto_decode_cli(const uint8_t *pb, size_t pb_len,
                               char *cmd_out, size_t cmd_cap,
                               char *devid_out, size_t devid_cap)
{
    qs_pb_msg_cli_command msg;
    qs_ret_code_t ret;

    if (pb == NULL || pb_len == 0 || cmd_out == NULL || cmd_cap == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&msg, 0, sizeof(msg));
    ret = qs_pb_cli_command_decode((char *)pb, pb_len, &msg);
    if (ret != QS_SUCCESS) {
        ESP_LOGW(TAG, "cli decode fail ret=%d len=%u", (int)ret, (unsigned)pb_len);
        return ESP_FAIL;
    }

    msg.command[sizeof(msg.command) - 1U] = '\0';
    msg.device_id[sizeof(msg.device_id) - 1U] = '\0';
    strncpy(cmd_out, msg.command, cmd_cap - 1U);
    cmd_out[cmd_cap - 1U] = '\0';

    if (devid_out != NULL && devid_cap > 0) {
        strncpy(devid_out, msg.device_id, devid_cap - 1U);
        devid_out[devid_cap - 1U] = '\0';
    }
    return ESP_OK;
}

su3_resp_kind_t su3_proto_classify_cmd(const char *cmd)
{
    if (cmd == NULL) {
        return SU3_RESP_SINGLE;
    }
    if (strcmp(cmd, "list") == 0) {
        return SU3_RESP_MULTIFRAME;
    }
    if (strncmp(cmd, "report", 6) == 0 &&
        (cmd[6] == '\0' || cmd[6] == ' ')) {
        return SU3_RESP_FIRE_FORGET;
    }
    return SU3_RESP_SINGLE;
}
