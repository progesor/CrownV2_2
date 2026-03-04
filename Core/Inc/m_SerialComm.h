/*
 * m_SerialComm.h
 *
 *  Created on: Jan 27, 2026
 *      Author: Baris
 */

#ifndef INC_M_SERIALCOMM_H_
#define INC_M_SERIALCOMM_H_

#include <stdint.h>
#include "AppConfig.h"



void ReceiveSerialData_Task(uint8_t rx_byte);
void ParseSerialData(uint8_t *line);
uint8_t String2Uint8(uint8_t *buffer);
uint16_t String2Uint16(uint8_t *buffer);
uint32_t String2Uint32(uint8_t *buffer);
float String2Float(uint8_t *buffer);

extern uint8_t serial_comm_ready;
extern uint8_t rx_data_ready;
extern uint8_t tx_data_ready;
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern uint8_t tx_buffer[TX_BUFFER_SIZE];

#endif /* INC_M_SERIALCOMM_H_ */
