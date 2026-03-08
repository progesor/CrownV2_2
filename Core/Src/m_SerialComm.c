#include "m_SerialComm.h"

#include "m_SharedMemory.h"

#include "m_MotorControl.h"

#include "m_IO.h"

#include <string.h>

#include <stdio.h>

uint8_t tx_buffer[TX_BUFFER_SIZE] = {
  0
};

uint8_t rx_byte = 0;
static uint8_t rx_buf[RX_BUFFER_SIZE] = {
  0
};
static uint8_t process_buffer[RX_BUFFER_SIZE] = {
  0
};

static uint16_t rx_idx = 0;
static uint8_t rx_state = 0;
static uint16_t expected_len = 0;

volatile uint8_t rx_data_ready = 0u;
volatile uint16_t rx_data_length = 0u;
volatile uint8_t rx_needs_restart = 0u;

uint16_t Calculate_CRC16(uint8_t * buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t) buffer[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void Init_SerialComm(void) {
  rx_state = 0;
  rx_needs_restart = 0;
  HAL_UART_Receive_IT( & huart3, & rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
  if (huart -> Instance == USART3) {
    switch (rx_state) {
    case 0:
      if (rx_byte == 0xAA) {
        rx_buf[0] = 0xAA;
        rx_idx = 1;
        rx_state = 1;
      }
      break;
    case 1:
      if (rx_byte == 0x55) {
        rx_buf[1] = 0x55;
        rx_idx = 2;
        rx_state = 2;
      } else {
        rx_state = 0;
      }
      break;
    case 2:
      rx_buf[2] = rx_byte;
      expected_len = rx_byte + 6;
      if (expected_len > RX_BUFFER_SIZE) {
        rx_state = 0;
      } else {
        rx_idx = 3;
        rx_state = 3;
      }
      break;
    case 3:
      rx_buf[rx_idx++] = rx_byte;
      if (rx_idx >= expected_len) {
        memcpy(process_buffer, rx_buf, expected_len);
        rx_data_length = expected_len;
        rx_data_ready = 1u;
        rx_state = 0;
      }
      break;
    }
    // Eğer TX o an veri gönderdiği için kilitliyse (HAL_BUSY dönerse),
    // sağır kalmamak için kurtarıcı bayrağı kaldır!
    if (HAL_UART_Receive_IT( & huart3, & rx_byte, 1) != HAL_OK) {
      rx_needs_restart = 1u;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart) {
  if (huart -> Instance == USART3) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    HAL_UART_AbortReceive_IT(huart);
    rx_state = 0;

    if (HAL_UART_Receive_IT( & huart3, & rx_byte, 1) != HAL_OK) {
      rx_needs_restart = 1u;
    }
  }
}

void Process_Binary_Packet(void) {
  if (rx_data_length < 6) return;

  uint8_t payload_length = process_buffer[2];
  uint16_t expected_total_len = 6 + payload_length;

  uint16_t calc_crc = Calculate_CRC16( & process_buffer[2], 2 + payload_length);
  uint16_t rx_crc = process_buffer[expected_total_len - 2] | (process_buffer[expected_total_len - 1] << 8);

  if (calc_crc != rx_crc) return;

  sm.last_valid_comm_time = HAL_GetTick(); // WATCHDOG GÜNCELLEMESİ

  uint8_t cmd = process_buffer[3];
  uint8_t * payload = & process_buffer[4];

  switch (cmd) {
  case 0x01:
	  if (payload_length == 0) {
      	// sm.last_valid_comm_time zaten yukarıda güncellendiği için
     	// burada motor parametrelerine HİÇ DOKUNMUYORUZ!
     	SendSerialData((uint8_t*)"<DBG: HB_OK>\n");
	  }
	  break;
  case 0x10:
    if (payload_length == 4) {
      float target_rpm;
      memcpy( & target_rpm, payload, 4);
      if (sm.act_motor_state != MOT_STATE_VEL_CONTROL) InitControlVel();
      sm.ref_velocity = target_rpm;
      sm.act_motor_state = MOT_STATE_VEL_CONTROL;

      SendSerialData((uint8_t * )
        "<DBG: CMD_RPM_OK>\n");
    }
    break;
  case 0x30: // SET PID (Kp ve Ki)
    if (payload_length == 8) { // 2 adet float = 8 byte
      float new_kp, new_ki;
      memcpy( & new_kp, payload, 4);
      memcpy( & new_ki, payload + 4, 4);

      // Katsayıları canlı olarak Shared Memory'e yaz
      sm.kp_velocity = new_kp;
      sm.ki_velocity = new_ki;

      // Parametreler değiştiği için Anti-Windup vb. resetlensin
      InitControlVel();

      SendSerialData((uint8_t * )
        "<DBG: CMD_PID_OK>\n");
    }
    break;

  case 0x40:
    if (payload_length == 12) { // 3 adet float = 12 byte
      float target_deg, max_rpm, accel_rpm_s;
      memcpy( & target_deg, payload, 4);
      memcpy( & max_rpm, payload + 4, 4);
      memcpy( & accel_rpm_s, payload + 8, 4);

      sm.osc_target_deg = target_deg;
      sm.osc_max_rpm = max_rpm;
      sm.osc_accel_rpm_s = accel_rpm_s;

      // Eğer motor zaten osilasyon modundaysa baştan başlatma (sadece parametreleri canlı güncelle)
      // Eğer farklı bir moddaysa (veya duruyorsa) osilasyonu başlat
      if (sm.act_motor_state != MOT_STATE_POS_CONTROL) {
        sm.act_motor_state = MOT_STATE_POS_CONTROL_INIT;
      }

      SendSerialData((uint8_t * )
        "<DBG: CMD_OSC_OK>\n");
    }
    break;
  case 0x50:
        if (payload_length == 12) {
                  float time_ms, max_rpm, accel_rpm_s;
                  memcpy(&time_ms, payload, 4);
                  memcpy(&max_rpm, payload + 4, 4);
                  memcpy(&accel_rpm_s, payload + 8, 4);

                  sm.osc_time_ms = time_ms;
                  sm.osc_max_rpm = max_rpm;
                  sm.osc_accel_rpm_s = accel_rpm_s;

                  if (sm.act_motor_state != MOT_STATE_TIME_OSC_CONTROL) {
                      sm.act_motor_state = MOT_STATE_TIME_OSC_CONTROL_INIT;
                  }
                  SendSerialData((uint8_t*)"<DBG: CMD_OSC_TIME_OK>\n");
              }
              break;
              // *** YENİ: ARAYÜZE GÜNCEL PARAMETRELERİ OKUYUP GÖNDERME ***
                      case 0x60: // GET PARAMS
                          if (payload_length == 0) {
                              char prm_msg[100];
                              snprintf(prm_msg, sizeof(prm_msg), "<PRM,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f>\n",
                                       sm.kp_velocity, sm.ki_velocity,
                                       sm.osc_time_ms, sm.osc_target_deg,
                                       sm.osc_max_rpm, sm.osc_accel_rpm_s);
                              SendSerialData((uint8_t*)prm_msg);
                          }
                          break;

                      // *** YENİ: KALICI HAFIZAYA (FLASH) KAYDETME ***
                      case 0x70: // SAVE PARAMS TO FLASH
                          if (payload_length == 0) {
                              // Güvenlik: Motor çalışırken Flash'a yazılmaz! (İşlemci kilitlenmesini önler)
                              if (sm.act_motor_state == MOT_STATE_IDLE) {
                                  SaveParamsToFlash();
                                  SendSerialData((uint8_t*)"<DBG: PARAMS_SAVED_FLASH>\n");
                              } else {
                                  SendSerialData((uint8_t*)"<DBG: ERR_CANT_SAVE_WHILE_RUNNING>\n");
                              }
                          }
                          break;
  case 0x20:
    sm.act_motor_state = MOT_STATE_IDLE;
    sm.ref_velocity = 0.0f;
    sm.current_traj_rpm = 0.0f;
    InitControlVel();
    DriveMotor(0.0f, 1u, 1u);

    SendSerialData((uint8_t * )
      "<DBG: CMD_STOP_OK>\n");
    break;
  }
}
