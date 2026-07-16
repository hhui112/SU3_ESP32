/*
 * @Author: fzh
 * @Date: 2026-07-14 18:19:35
 * @LastEditors: fzh
 * @LastEditTime: 2026-07-14 18:19:35
 */
/**
 * 协议层：CliCommand 及各 topic 的 protobuf 编解码封装。
 * 内部调用 components/protobuf（qs_protobuf）。
 */
#ifndef SU3_PROTO_H_
#define SU3_PROTO_H_

#include "su3_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 编码 CliCommand → 输出完整 PDU（topic 字节 + pb），写入 out */
esp_err_t su3_proto_encode_cli(const char *device_id, int32_t timestamp,
                               const char *command,
                               uint8_t *out, size_t out_cap, size_t *out_len);

/** 解码 topic=4 的 protobuf → command；devid_out 可为 NULL */
esp_err_t su3_proto_decode_cli(const uint8_t *pb, size_t pb_len,
                               char *cmd_out, size_t cmd_cap,
                               char *devid_out, size_t devid_cap);

/** 根据命令字符串判断应答类型（精确匹配，不用 strstr 模糊） */
su3_resp_kind_t su3_proto_classify_cmd(const char *cmd);

#ifdef __cplusplus
}
#endif

#endif /* SU3_PROTO_H_ */
