/*
 * m_SharedMemory.c
 *
 *  Created on: Jan 24, 2026
 *      Author: Baris
 */

#include "m_SharedMemory.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "AppConfig.h"

SharedMemory_t sm;

void SharedMemoryInit(void)
{
	// *** BEGIN: IO.c değişkenleri ***
	// UpdateADC_FromDMA_Task()
	sm.ref_position = 0.0f;					// Referans konum (derece)
	sm.ref_velocity = 0.0f;       			// Referans hız (rpm)
	sm.act_pot_value = 0.0f;				// PA0'dan okunan pot değeri (0..4095)
	sm.act_motor_voltage = 0.0f;			// PA1'den okunan motor gerilim değeri (0..24V)
	sm.act_motor_current = 0.0f;			// PA2'den okunan motor akım değeri (0..10A)

	// UpdateEncoder_Task()
	sm.encoder_position_count = 0u;			// Toplam konum sayacı (signed 32-bit)
	sm.act_position = 0.0f;					// Anlık konum (derece)
	sm.act_velocity = 0.0f;					// Anlık hız (rpm)
	sm.act_velocity_raw = 0.0f;				// Anlık filtresiz hız (rpm)

	// ScanIO_Task()
	sm.mode_button_state = 0u;				// MODE butonunun anlık durumunu (0,1)

	// DriveMotor()
	sm.act_pwm_duty = 0.0f;	 			// DriveMotor() gelen PWM DUTY CYCLE
	// *** END: IO.c değişkenleri ***

	// *** BEGIN: MotorControl.c değişkenleri ***
	// MotorControl_Task()
	sm.act_motor_state = MOT_STATE_IDLE;
	sm.act_position_relative = 0.0f;		// Anlık göreli (relative) konum (derece)
	sm.control_mode = 0u;
	sm.duty_cycle_command = 0.0f;

	// PositionControl()
	sm.kp_position = 6.0f;       //1.5f     // Pozisyon kontrol orantı kazancı. Sadece pos kontrolde kullanılacaksa Kp=1.5 civarı küçük tutmak lazım.
											// Eğer tragen içinde kullanılacaksa Kp=3..5 civarı seçmek lazım. Çünkü tragen içinde hedefe yaklaşıldığında hız
											// kademeli olarak zaltıldığı için hedef konum hatası ölü bölge içine girmiyor.

	// VelocityControl()
	sm.kp_velocity = 1.0f;   //0.8f              		// Hız kontrol orantı kazancı
	sm.ki_velocity = (20.0f * CONTROL_LOOP_PERIOD);     // Hız kontrol integral kazancı *** TI çarpılacak
	// *** END: MotorControl.c değişkenleri ***

	// *** BEGIN: SerialComm.c değişkenleri ***
	// ReceiveSerialData_Task()

	// InterpretSerialData()
	sm.serial_comm_active = 0u;		// Seri haberleşmenin aktif olduğunu gösteren değişken
	sm.operation_active = 0u;		// Saç ekim operasyonunun Aktif/Pasif olduğunu gösteren değişken
	sm.operation_start = 0u;		// Saç ekim operasyonunu başlatan değişken (Başlat)
	sm.operation_stop = 0u;			// Saç ekim operasyonunu durduran değişken (Durdur)
	sm.pedal_state = 0u;			// Pedalın basılı olup olmadığını gösteren değişken
	sm.pedal_active = 0u;			// Pedal kullanımını aktif/Pasif yapan değişken
	sm.pedal_counter = 0u;			// Pedal sayaç değerini saklayan değişken
	sm.time_delay = 0u;				// TIME değerini saklayan değişken
	sm.emergency_stop = 0u;			// Acil durdur düğmesine basılı olup olmadığını gösteren değişken
	// *** END: SerialComm.c değişkenleri ***

	// Osilasyon Başlangıç Değerleri
	sm.osc_target_deg = 180.0f;
	sm.osc_max_rpm = 1500.0f;
	sm.osc_accel_rpm_s = 6000.0f; // Saniyede 20.000 RPM hızlanma
	sm.current_traj_rpm = 0.0f;
	sm.osc_state = 0u;

	sm.max_allowed_rpm = 0.0f;

}

// *** CİHAZ AÇILIRKEN AYARLARI YÜKLEYEN FONKSİYON ***
void LoadParamsFromFlash(void) {
    DeviceParams_t *pFlash = (DeviceParams_t*)FLASH_PARAM_ADDR;

    if (pFlash->magic == PARAM_MAGIC_WORD) {
        // Hafızada ayar var, eski ayarları RAM'e çek!
        sm.kp_velocity = pFlash->kp_vel;
        sm.ki_velocity = pFlash->ki_vel;
        sm.osc_time_ms = pFlash->osc_time_ms;
        sm.osc_target_deg = pFlash->osc_target_deg;
        sm.osc_max_rpm = pFlash->osc_max_rpm;
        sm.osc_accel_rpm_s = pFlash->osc_accel;
    } else {
        // Cihaz fabrikadan yeni çıkmış (veya hafıza silinmiş), Varsayılanları yükle!
        sm.kp_velocity = 1.0f;
        sm.ki_velocity = 5.0f;
        sm.osc_time_ms = 400.0f;
        sm.osc_target_deg = 180.0f;
        sm.osc_max_rpm = 2500.0f;
        sm.osc_accel_rpm_s = 15000.0f;
    }
}

// *** ARAYÜZDEN "KAYDET" DENİLDİĞİNDE ÇALIŞAN FONKSİYON ***
void SaveParamsToFlash(void) {
    HAL_FLASH_Unlock();

    // 1. Önce eski sayfayı silmeliyiz (Flash'ın kuralı)
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_PARAM_ADDR;
    EraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return; // Hata olursa çık
    }

    // 2. Güncel RAM değerlerini yapıya doldur
    DeviceParams_t params;
    params.magic = PARAM_MAGIC_WORD;
    params.kp_vel = sm.kp_velocity;
    params.ki_vel = sm.ki_velocity;
    params.osc_time_ms = sm.osc_time_ms;
    params.osc_target_deg = sm.osc_target_deg;
    params.osc_max_rpm = sm.osc_max_rpm;
    params.osc_accel = sm.osc_accel_rpm_s;

    // 3. Hafızaya Word (4 Byte) adım adım yaz
    uint32_t *pData = (uint32_t*)&params;
    for (int i = 0; i < sizeof(DeviceParams_t)/4; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_PARAM_ADDR + (i*4), pData[i]);
    }

    HAL_FLASH_Lock();
}
