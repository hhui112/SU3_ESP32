/*
 * @Author: fzh
 * @Date: 2026-07-14 18:19:35
 * @LastEditors: fzh 
 * @LastEditTime: 2026-07-14 18:19:35
 */
#ifndef USE_ADC_H_
#define USE_ADC_H_
#include "driver/gpio.h"

// 对外子程序

extern void CH423_WriteByte(unsigned short cmd); // 写出数据

extern unsigned char CH423_ReadByte(void); // 读取数据

uint8_t get_ch423_oc_high(uint8_t bit, uint8_t out);
uint8_t get_ch423_oc_low(uint8_t bit, uint8_t out);
void pressure_sensor_select(uint16_t pressure_sensor_num);
void ntc_sensor_select(uint16_t pressure_sensor_num);
void valve_out_select(uint16_t valve_out_num);
uint8_t setvalvebit(uint8_t valve_bit_state, uint8_t bit, uint8_t level);
void CH423_I2c_Start(void);
void CH423_I2c_Stop(void);
void CH423_I2c_WrByte(unsigned char dat);
unsigned char CH423_I2c_RdByte(void);
void CH423_Write(unsigned short cmd);
void CH423_WriteByte(unsigned short cmd);
unsigned char CH423_ReadByte();

void initialize_adc(void);

#endif