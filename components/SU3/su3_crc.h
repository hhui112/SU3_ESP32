/*
 * @Author: fzh
 * @Date: 2026-07-13 18:19:35
 * @LastEditors: fzh
 * @LastEditTime: 2026-07-13 18:19:35
 */
#ifndef SU3_CRC_H_
#define SU3_CRC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Modbus 风格 CRC16（与传感器帧校验一致） */
uint16_t su3_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* SU3_CRC_H_ */
