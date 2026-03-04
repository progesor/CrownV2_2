/*
 * m_IO.h
 *
 *  Created on: Dec 7, 2025
 *      Author: Baris
 */

#ifndef INC_M_IO_H_
#define INC_M_IO_H_

#include <stdint.h>
#include "main.h"			// IO lar hakkındaki tanımlamaları alabilmek için
#include "m_Filter.h"

//#define ENCODER_PPR             512u   	            // Enkoder Puls Per Revolution (mekanik tur başına pulse)
//#define ENCODER_CPR             (ENCODER_PPR * 4u)  // Quadrature 4x -> counts per rev
//#define PWM_TIMER_PERIOD        2399u      	        // TIM1->ARR (örneğin 30 kHz için)
//#define AIN_VALUE_MAX			4095.0f		        // Analog inputtan okunacak maks. değer
//#define ADC_CHANNEL_COUNT   	3u                  // ADC kanal sayısı
//
//#define REF_VELOCITY_MAX        35000.0f   	        // ADC tam skalada hedef rpm
//#define REF_VELOCITY_MIN        200.0f   	        // ADC tam skalada hedef rpm
//#define REF_VELOCITY_STEP 		10u				// Hız referans porunun artış skala değeri
//
//#define K_ACT_POSITION			360.0f / (float)ENCODER_CPR
//#define K_RPM_CONVERSION  		(60.0f / (ENCODER_CPR * CONTROL_LOOP_PERIOD))
////#define K_POT_SCALE_MOTOR_RPM	  (REF_VELOCITY_MAX / AIN_VALUE_MAX) // ADC tam skalada hedef motor devir katsayısı (rpm)
//#define K_POT_MOTOR_RPM	        (REF_VELOCITY_MAX / AIN_VALUE_MAX) // ADC tam skalada hedef motor devir katsayısı (rpm)
//#define K_MOTOR_VOLTAGE_SENSE	0.0106f   			// ADC tam skalada hedef motor gerilim katsayısı (V)
////#define K_MOTOR_CURRENT_SENSE	0.0016f   			// ADC tam skalada hedef motor akım katsayısı (A)
//#define K_MOTOR_CURRENT_SENSE	1.6f   				// ADC tam skalada hedef motor akım katsayısı (mA)

void Init_ADC(void);
void Init_Tim1(void);
void Init_Tim2(void);
void Init_Tim4(void);
void UpdateADC_FromDMA_Task(void);
void UpdateEncoder_Task(void);
void ScanIO_Task(void);
void DriveMotor(float pwm_duty, uint8_t direction, uint8_t brake_status);


#endif /* INC_M_IO_H_ */

