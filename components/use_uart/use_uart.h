/*
 * @Author: fzh
 * @Date: 2026-07-14 18:19:35
 * @LastEditors: fzh 
 * @LastEditTime: 2026-07-14 18:19:35
 */
#ifndef USE_UART_H_
#define USE_UART_H_

extern union SyncCommunicationData_t g_Sync_TX;
extern union SyncCommunicationData_t g_Sync_RX;

void uart1_config(int txd_pin, int rxd_pin);
void uart2_config(void);
void mfp_gpio_config(void);
uint16_t Modbus_Crc_Compute(const uint8_t *buf, uint16_t bufLen);
uint16_t crc16_check(const uint8_t *buf, uint16_t bufLen);
unsigned char rxCalcCheckSum(void);
unsigned char syncCalcCheckSum(void);
uint8_t syncCalcCheckSum_mfpqueue(const uint8_t *data, uint8_t len);

void mfp_dateSend(void);
void  Debug_printf_buff(uint8_t *buff ,uint16_t len);

void uart_mfp_send(const uint8_t *data, size_t len);


/*
void spi_init(void);
void spi_send_receive_data(uint8_t *tx_data, uint8_t *rx_data, size_t length);
void spi_send_data(uint8_t *data, size_t length);
*/
#endif