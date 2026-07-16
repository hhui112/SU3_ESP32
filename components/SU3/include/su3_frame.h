/*
 * @Author: fzh
 * @Date: 2026-07-14 18:19:35
 * @LastEditors: fzh
 * @LastEditTime: 2026-07-14 18:19:35
 */
/**
 * 帧层：AA55 组帧/切帧、CRC、按 src 多帧 PDU 重组。
 * 参考 AiTopper RS485 rs485_parser 的 stream + reasm，去掉温湿度/双 bus 特例。
 */
#ifndef SU3_FRAME_H_
#define SU3_FRAME_H_

#include "su3_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  src_addr;
    uint8_t  dst_addr;
    uint8_t  topic;
    const uint8_t *pb_data; /* 指向 PDU 内 protobuf，不含 topic 字节 */
    size_t   pb_len;
} su3_pdu_info_t;

typedef void (*su3_pdu_cb_t)(const su3_pdu_info_t *pdu, void *user);

typedef struct su3_frame_ctx su3_frame_ctx_t;

/** 创建帧解析上下文（含 ring / 双路上行重组槽） */
su3_frame_ctx_t *su3_frame_create(su3_pdu_cb_t on_pdu, void *user);

void su3_frame_destroy(su3_frame_ctx_t *ctx);

/** 喂入原始 UART 字节；内部同步 AA55、校验 CRC、多帧重组后回调 on_pdu */
void su3_frame_feed(su3_frame_ctx_t *ctx, const uint8_t *data, size_t len);

/**
 * 组一帧写入 out（含 CRC）。
 * pdu = topic(1) + protobuf；flag 默认单帧 0x01。
 * @return 写出字节数，失败返回 0
 */
size_t su3_frame_build(uint8_t src, uint8_t dst, uint8_t flag,
                       const uint8_t *pdu, size_t pdu_len,
                       uint8_t *out, size_t out_cap);

/** 联调统计：成功帧 / CRC 失败 / 流溢出 */
void su3_frame_get_stats(const su3_frame_ctx_t *ctx,
                         uint32_t *frame_ok, uint32_t *crc_fail, uint32_t *overflow);

#ifdef __cplusplus
}
#endif

#endif /* SU3_FRAME_H_ */
