/*
 * m_SharedMemory.h
 *
 *  Created on: Jan 24, 2026
 *      Author: Baris
 */

#ifndef INC_M_SHAREDMEMORY_H_
#define INC_M_SHAREDMEMORY_H_

#include <stdint.h>

//*** MotorControl.c > MotorControl_Task() tip tanımları ***
typedef enum {
	MOT_STATE_IDLE,
	MOT_STATE_VEL_CONTROL_INIT,
	MOT_STATE_POS_CONTROL_INIT,
	MOT_STATE_VEL_CONTROL,
	MOT_STATE_POS_CONTROL,
	MOT_STATE_VEL_CONTROL_END,
	MOT_STATE_POS_CONTROL_END,
	MOT_STATE_MANUAL_CONTROL,
	MOT_STATE_TIME_OSC_CONTROL_INIT,
	MOT_STATE_TIME_OSC_CONTROL
} MotorState_t;

typedef struct
{
	//*** BEGIN: IO.c değişkenleri ***
	// UpdateADC_FromDMA_Task()
	float ref_position;							// Referans konum (derece)
	float ref_velocity;       					// Referans hız (rpm)
	float ref_velocity_raw;						// PA0’dan okunacak (rpm)
	float act_pot_value;						// PA0'dan okunan pot değeri (0..4095)
	float act_motor_voltage;					// PA1'den okunan motor gerilim değeri (0..24V)
	float act_motor_current;					// PA2'den okunan motor akım değeri (0..10A)

	// UpdateEncoder_Task()
	volatile int32_t encoder_position_count;	// Toplam konum sayacı (signed 32-bit)
	volatile float act_position;				// Anlık konum (derece)
	volatile float act_velocity;				// Anlık hız (rpm)
	volatile float act_velocity_raw;			// Anlık filtresiz hız (rpm)


	// ScanIO_Task()
	uint8_t mode_button_state;

	// DriveMotor()
	volatile float act_pwm_duty;	 			// DriveMotor() gelen PWM DUTY CYCLE
	//*** END: IO.c değişkenleri ***

	//*** BEGIN: MotorControl.c değişkenleri ***
	// MotorControl_Task()
	volatile MotorState_t act_motor_state;		// Motorun anlık çalışma modu durumu
	volatile float act_position_relative;		// Anlık göreli (relative) konum (derece)
	volatile uint8_t control_mode;				// control_mode = 0: Velocity Mode, control_mode = 1: Position Mode
	volatile float duty_cycle_command;			// Manual Mode sürüşte motora gönderilecek pwm duty cycle değeridir. (0..1f)

	// PositionControl()
	float kp_position;    		              	// Pozisyon kontrol orantı kazancı

	// VelocityControl()
	float kp_velocity;	                 		// Hız kontrol orantı kazancı
	float ki_velocity;  	          			// Hız kontrol integral kazancı *** TI çarpılacak
	//*** END: MotorControl.c değişkenleri ***

	//*** BEGIN: SerialComm.c değişkenleri ***
	// ReceiveSerialData_Task()

	// InterpretSerialData()
	uint32_t last_valid_comm_time;  //
	uint8_t serial_comm_active;		// Seri haberleşmenin aktif olduğunu gösteren değişken
	uint8_t operation_active;		// Saç ekim operasyonunun Aktif/Pasif olduğunu gösteren değişken
	uint8_t operation_start;		// Saç ekim operasyonunu başlatan değişken (Başlat)
	uint8_t operation_stop;			// Saç ekim operasyonunu durduran değişken (Durdur)
	uint8_t pedal_state;			// Pedalın basılı olup olmadığını gösteren değişken
	uint8_t emergency_stop;			// Acil durdur düğmesine basılı olup olmadığını gösteren değişken
	uint8_t pedal_active;			// Pedal kullanımını aktif/Pasif yapan değişken
	uint32_t pedal_counter;			// Pedal sayaç değerini saklayan değişken
	uint16_t time_delay;			// TIME değerini saklayan değişken
	//*** END: SerialComm.c değişkenleri ***

	//*** BEGIN: Oscillation Control değişkenleri ***
	float osc_target_deg;       // Hedef açı
	float osc_max_rpm;          // Osilasyon maksimum hızı
	float osc_accel_rpm_s;      // İvme! Saniyede RPM artışı
	float current_traj_rpm;     // Anlık yörünge hızı
	uint8_t osc_state;          // 0: İleri Git, 1: Geri Dön

	float max_allowed_rpm;

	float osc_time_ms;          // YENİ: Zaman tabanlı mod için hareket süresi
	float osc_timer_ms;         // YENİ: Zaman tabanlı mod sayacı


} SharedMemory_t;

void SharedMemoryInit(void);

extern SharedMemory_t sm;

#endif /* INC_M_SHAREDMEMORY_H_ */
