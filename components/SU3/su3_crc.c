/*
 * @Author: fzh
 * @Date: 2026-07-14 18:19:35
 * @LastEditors: fzh
 * @LastEditTime: 2026-07-14 18:19:35
 */
#include "su3_crc.h"

/**
 * 与《双床协议》/ ESP_BC crc16_compute / AiTopper rs485_crc16_compute 一致。
 * 注意：不是 Modbus CRC（多项式 0xA001）。
 */
uint16_t su3_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    size_t i;

    if (data == NULL) {
        return 0;
    }

    for (i = 0; i < len; i++) {
        crc = (uint16_t)(((uint16_t)(crc >> 8U)) | (uint16_t)(crc << 8U));
        crc ^= data[i];
        crc ^= (uint16_t)((uint8_t)(crc & 0xFFU) >> 4U);
        crc ^= (uint16_t)((crc << 8U) << 4U);
        crc ^= (uint16_t)(((crc & 0xFFU) << 4U) << 1U);
    }
    return crc;
}
