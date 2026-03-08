/*
 * m_SerialComm.h
 */

#ifndef INC_M_SERIALCOMM_H_
#define INC_M_SERIALCOMM_H_

#include <stdint.h>
#include "main.h"
#include "AppConfig.h"

extern UART_HandleTypeDef huart3;

#define RX_BUFFER_SIZE 256u
#define TX_BUFFER_SIZE 256u

extern uint8_t tx_buffer[TX_BUFFER_SIZE];
extern uint8_t rx_byte;
extern volatile uint8_t rx_data_ready;
extern volatile uint8_t rx_needs_restart; // YENİ: Kurtarıcı Bayrak

extern void SendSerialData(uint8_t *buffer);

void InitSerialComm(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void ProcessBinaryPacket(void);
uint16_t CalculateCrc16(uint8_t *buffer, uint16_t length);

#endif /* INC_M_SERIALCOMM_H_ */
