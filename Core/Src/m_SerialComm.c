/*
 * m_SerialComm.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Baris
 */

#include "m_SerialComm.h"
#include "m_SharedMemory.h"
#include "m_MotorControl.h"
#include "m_IO.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// *** Değişkenler m_SerialComm.c
uint8_t serial_comm_ready = 0u;
uint8_t rx_data_ready = 0u;
uint8_t tx_data_ready = 0u;
uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
static uint8_t rx_buffer_temp[RX_BUFFER_SIZE] = {0};
uint8_t tx_buffer[TX_BUFFER_SIZE] = {0};
static volatile uint16_t rx_buffer_temp_index = 0;
static volatile uint16_t rx_data_length = 0;


void ReceiveSerialData_Task(uint8_t rx_byte)
{
    static uint8_t receiving = 0;

    // Paket başlangıcı
    if (rx_byte == '<')
    {
        receiving = 1;
        rx_buffer_temp_index = 0;
        rx_buffer_temp[rx_buffer_temp_index++] = rx_byte;
    }
    else if (receiving)
    {
        // Buffer taşmasını önle
        if (rx_buffer_temp_index < (RX_BUFFER_SIZE - 1))
        {
            rx_buffer_temp[rx_buffer_temp_index++] = rx_byte;

            // Paket başarıyla tamamlandı
            if (rx_byte == '>')
            {
                receiving = 0;

                // Eğer ana döngü müsaitse veriyi aktar
                if (rx_data_ready == 0)
                {
                    rx_buffer_temp[rx_buffer_temp_index] = '\0';
                    strcpy((char*)rx_buffer, (char*)rx_buffer_temp);
                    rx_data_ready = 1u;
                }
            }
        }
        else
        {
            // Buffer doldu ama hala '>' gelmedi. Bu hatalı/gürültülü bir pakettir, iptal et!
            receiving = 0;
            rx_buffer_temp_index = 0;
        }
    }
}

void ParseSerialData(uint8_t *line)
{
    if (line == NULL) return;

    char *start = strchr((char*)line, '<');
    char *end = strchr((char*)line, '>');

    if (start != NULL && end != NULL && end > start)
    {
        *end = '\0';
        char *payload = start + 1;

        char *cmd = strtok(payload, ",");
        char *p1_str = strtok(NULL, ",");
        char *p2_str = strtok(NULL, ",");
        char *p3_str = strtok(NULL, ","); // YENİ: 3. Parametre (İvme vb. için)

        if (cmd != NULL && p1_str != NULL && p2_str != NULL)
        {
            float param1 = atof(p1_str);
            float param2 = atof(p2_str);

            if (strcmp(cmd, "SET_RPM") == 0)
            {
                if (sm.act_motor_state != MOT_STATE_VEL_CONTROL) InitControlVel();
                sm.ref_velocity = param1;
                sm.act_motor_state = MOT_STATE_VEL_CONTROL;
            }
            else if (strcmp(cmd, "OSC") == 0)
            {
                float param3 = (p3_str != NULL) ? atof(p3_str) : 20000.0f; // 3. parametre İVME

                sm.osc_target_deg = param1;
                sm.osc_max_rpm = param2;
                sm.osc_accel_rpm_s = param3; // İvmeyi canlı alıyoruz

                // *** KRİTİK DÜZELTME: CANLI GÜNCELLEME KORUMASI ***
                // Eğer motor zaten osilasyondaysa SADECE hedefleri değiştir, motoru resetleme!
                if (sm.act_motor_state != MOT_STATE_POS_CONTROL)
                {
                    sm.current_traj_rpm = 0.0f;
                    sm.osc_state = 0;
                    InitControlPos();
                    InitControlVel();
                    sm.act_motor_state = MOT_STATE_POS_CONTROL;
                }
            }
            else if (strcmp(cmd, "STOP") == 0)
            {
                sm.act_motor_state = MOT_STATE_IDLE;
                sm.ref_velocity = 0.0f;
                sm.current_traj_rpm = 0.0f;
                InitControlVel();
                DriveMotor(0.0f, 1u, 1u);

                // *** KRİTİK DÜZELTME: POZİSYON ŞİŞMESİNİ (FLOAT OVERFLOW) ÖNLEME ***
                sm.encoder_position_count = 0; // Donanımsal sayacı sıfırla
                sm.act_position = 0.0f;        // Mutlak açıyı sıfırla
                sm.act_position_relative = 0.0f;
            }

            else if (strcmp(cmd, "PID") == 0)
                        {
                            sm.kp_velocity = param1;
                            sm.ki_velocity = param2;
                        }
        }
    }
}




uint8_t String2Uint8(uint8_t *buffer)
{
    char *endptr;

    // Buffer boşsa...
    if (buffer == NULL) return 0;

    unsigned long val = strtoul((char*)buffer, &endptr, 10);

    // Buffer içinde hiç sayı yoksa...
    if (endptr == (char*)buffer) return 0;

    // Sayı dışında karakter varsa (örn "12a" veya "123 ")...
    if (*endptr != '\0') return 0;

    // uint8_t sınırı aşmışsa...
    if (val > 255) val = 255;

    return (uint8_t)val;
}

uint16_t String2Uint16(uint8_t *buffer)
{
    char *endptr;

    // Buffer boşsa...
    if (buffer == NULL) return 0;

    unsigned long val = strtoul((char*)buffer, &endptr, 10);

    // Buffer içinde hiç sayı yoksa...
    if (endptr == (char*)buffer) return 0;

    // Sayı dışında karakter varsa (örn "12a" veya "123 ")...
    if (*endptr != '\0') return 0;

    // uint8_t sınırı aşmışsa...
    if (val > 65535) val = 65535;

    return (uint16_t)val;
}

uint32_t String2Uint32(uint8_t *buffer)
{
    char *endptr;

    // Buffer boşsa...
    if (buffer == NULL) return 0;

    unsigned long val = strtoul((char*)buffer, &endptr, 10);

    // Buffer içinde hiç sayı yoksa...
    if (endptr == (char*)buffer) return 0;

    // Sayı dışında karakter varsa (örn "12a" veya "123 ")...
    if (*endptr != '\0') return 0;

    // uint8_t sınırı aşmışsa...
    if (val > 4294967295) val = 4294967295;

    return (uint32_t)val;
}

float String2Float(uint8_t *buffer)
{
    char *endptr;

    // Buffer boşsa...
    if (buffer == NULL) return 0.0f;

    float val = strtof((char*)buffer, &endptr);

    // Buffer içinde hiç sayı yoksa...
    if (endptr == (char*)buffer) return 0.0f;

    // Sayı dışında karakter varsa (örn "12a" veya "123 ")...
    if (*endptr != '\0') return 0.0f;

    return val;
}
