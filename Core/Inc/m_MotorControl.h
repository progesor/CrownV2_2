/*
 * m_MotorControl.h
 *
 *  Created on: Dec 7, 2025
 *      Author: Baris
 */

#ifndef INC_M_MOTORCONTROL_H_
#define INC_M_MOTORCONTROL_H_

#include <stdint.h>
#include "m_IO.h"

typedef enum
{
    STALL_IDLE = 0,    // Normal akış
    STALL_SUPPRESS,    // İntegratörü iptal et, Kp'yi arttır
    STALL_PROTECT      // Motoru durdur, bekle ve sonra tekrar dene
} StallState_t;

void MotorControl_Task(void);
void InitControlVel(void);
void InitControlPos(void);
float PositionControl(float ref_pos_deg, float act_pos_deg, float uo_vel_limit);
void VelocityControl(float ref_vel_rpm, float act_vel_rpm, float ff_vel_rpm);
float TrajectoryGeneratorVel(float target_rpm, float accel_limit_rpm_s);
//uint8_t TrajectoryGeneratorPosRel(float target_pos_deg, float vel_limit_rpm);
//uint8_t PositionSupervisor(float ref_pos_deg, float act_pos_deg, float pwm_duty);
//uint8_t VelocitySupervisor(float ref_vel_rpm, float act_vel_rpm, float pwm_duty);
uint8_t StallSupervisor(float current, float pwm_duty, float ref_vel, float act_vel);


#endif /* INC_M_MOTORCONTROL_H_ */
