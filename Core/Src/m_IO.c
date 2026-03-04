/*
 * m_IO.c
 *
 *  Created on: Dec 7, 2025
 *      Author: Baris
 */

#include "m_IO.h"
#include "m_SharedMemory.h"
#include "AppConfig.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// Analog ve Dijital IO değişkenleri
static volatile uint16_t adc_dma_buf[ADC_CHANNEL_COUNT] = {0}; 	// DMA'nin dolduracağı buffer
static volatile uint16_t tim4_cnt_last = 0;						// Enkoder okumada kullanılan sayaç değeri


void Init_ADC(void)
{
	// ADC işlemleri
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_CHANNEL_COUNT);		// ADC + DMA sürekli tarama
}

void Init_Tim1(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);			// Motor PWM çıkışı için...
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);  	// Başlangıçta motor durur vaziyette olması için...
}

void Init_Tim2(void)
{
	HAL_TIM_Base_Start_IT(&htim2); // Timer 2'yi kesme modunda başlat
}

void Init_Tim4(void)
{
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);		// Encoder okumak için...
	tim4_cnt_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

void UpdateADC_FromDMA_Task(void)
{
	// 1. Ham verileri DMA buffer'ından al
	uint16_t adc_ch0 = adc_dma_buf[0];   // PA0 - hız referansı
	uint16_t adc_ch1 = adc_dma_buf[1];   // PA1 - motor gerilim feedback
	uint16_t adc_ch2 = adc_dma_buf[2];   // PA2 - motor akım feedback

	// 2. Her kanal için AYRI medyan filtre hafızası (3'er elemanlı diziler)
	// MedianFilter3() yerine MedianFilter5() kullanılacaksa filtre hafızası 5 elemanlıya çıkartılmalıdır.
	static float median_buf_pot[3]	   = {0};
	static float median_buf_voltage[3] = {0};
	static float median_buf_current[5] = {0};

	// 3. Her kanal için AYRI LPF sonuç hafızası
	static float lpf_val_pot 	 = 0.0f;
	static float lpf_val_voltage = 0.0f;
	static float lpf_val_current = 0.0f;

	// --- ADIM 1: MEDYAN FİLTRELEME ---
	// Her kanal sadece kendi dizisini (hafızasını) günceller
	float med_pot  	  = MedianFilter3(median_buf_pot, (float)adc_ch0);
	float med_voltage = MedianFilter3(median_buf_voltage, (float)adc_ch1);
	float med_current = MedianFilter5(median_buf_current, (float)adc_ch2);

	// --- ADIM 2: LOW PASS FİLTRELEME ---
	// Medyan filtreden çıkan temiz veriyi LPF'ye sokuyoruz
	lpf_val_pot  	= LowPassFilter(lpf_val_pot,  med_pot, K_LPF_POT);
	lpf_val_voltage = LowPassFilter(lpf_val_voltage, med_voltage, K_LPF_MOTOR_VOLTAGE);
	lpf_val_current = LowPassFilter(lpf_val_current, med_current, K_LPF_MOTOR_CURRENT);

	// --- ADIM 3: SONUÇLARI SİSTEME YAZ ---
	sm.act_pot_value      = lpf_val_pot;
	sm.act_motor_voltage  = lpf_val_voltage * K_MOTOR_VOLTAGE_SENSE;
	sm.act_motor_current  = lpf_val_current * K_MOTOR_CURRENT_SENSE;

	//	lpf_val_pot 	= (float)adc_ch0;
	//	lpf_val_voltage = (float)adc_ch1;
	//	lpf_val_current = (float)adc_ch2;

	//	lpf_val_pot 	= AdaptiveLowPassFilter(lpf_val_pot, (float)adc_ch0, K_LPF_POT);
	//	lpf_val_voltage = AdaptiveLowPassFilter(lpf_val_voltage, (float)adc_ch1, K_LPF_MOTOR_VOLTAGE);
	//	lpf_val_current = AdaptiveLowPassFilter(lpf_val_current, (float)adc_ch2, K_LPF_MOTOR_CURRENT);

	//	lpf_val_pot 	= AdaptiveLowPassFilter(lpf_val_pot, med_pot, K_LPF_POT);
	//	lpf_val_voltage = AdaptiveLowPassFilter(lpf_val_voltage, med_voltage, K_LPF_MOTOR_VOLTAGE);
	//	lpf_val_current = AdaptiveLowPassFilter(lpf_val_current, med_current, K_LPF_MOTOR_CURRENT);

	//	sm.ref_velocity = (float)ScaleValues(lpf_val_pot, REF_VELOCITY_MAX, REF_VELOCITY_STEP);
	//	sm.ref_velocity = (float)ScaleValues(lpf_val_pot, 4000, REF_VELOCITY_STEP);
	//	sm.ref_position = (float)ScaleValues(lpf_val_pot, 36000, 1000);
}

void UpdateEncoder_Task(void)
{
	// 1. Enkoder sayacını oku
	uint16_t curr_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

	// 2. 16-bit signed fark al (overflow'lu ve yön bilgili)
	int16_t difference = (int16_t)(curr_cnt - tim4_cnt_last);

	// 3. Eski değeri güncelle
	tim4_cnt_last = curr_cnt;

	// 4. 32-bit konum sayacı (toplam count)
	sm.encoder_position_count += (int32_t)difference;

	// 5. Konumu sayacını dereceye çevir (0 = başlangıç referansı)
	sm.act_position = (float)sm.encoder_position_count * K_ACT_POSITION;

	// 6. Hızı hesapla (rpm)
	float rpm = (float)difference * K_RPM_CONVERSION;

	// 7. Hız ölçümünü filtrele

	// Düşük Hız: Hassas Capture verisini kullan
	static float median_buf_rpm[5] = {0};

	// Adım 1: Medyan Filtre (Sıçramaları temizler)
	float rpm_filtered = MedianFilter5(median_buf_rpm, rpm);

	sm.act_velocity = AlphaBetaFilter(rpm_filtered, CONTROL_LOOP_PERIOD);
	sm.act_velocity_raw = rpm;
}

void ScanIO_Task(void)
{
	//sm.pedal_state = HAL_GPIO_ReadPin(DI_Pedal_GPIO_Port, DI_Pedal_Pin);
}


void DriveMotor(float pwm_duty, uint8_t direction, uint8_t brake_status)
{
	if(pwm_duty > 1.0f) pwm_duty = 1.0f;
	else if(pwm_duty < 0.0f) pwm_duty = 0.0f;

	sm.act_pwm_duty = pwm_duty;
	uint16_t pwm_val = (uint16_t)((float)PWM_TIMER_PERIOD * pwm_duty); 				// pwm_duty_cycle: 0..1 aralığında olmalıdır. 0: %0 PWM, 1: %100 PWM
	HAL_GPIO_WritePin(DO_Motor_Brake_GPIO_Port, DO_Motor_Brake_Pin, brake_status);			// brakeStatus = 1: brake OFF, 0:brake ON
	HAL_GPIO_WritePin(DO_Motor_Direction_GPIO_Port, DO_Motor_Direction_Pin, direction);		// direction = 1: CW, 0: CCW
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val);  								// PWM Çıkışı PA8: TIM1 CH1; pwmValue: 0..2399
}
