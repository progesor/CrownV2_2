/*
 * AppConfig.h
 *
 *  Created on: Jan 24, 2026
 *      Author: Baris
 */

#ifndef INC_APPCONFIG_H_
#define INC_APPCONFIG_H_

//*** BEGIN: main.c tanımları ***
#define CONTROL_TIMER_ARR			4999u      									// TIM2->ARR (1999: 500 Hz-2 ms / 4999: 200 Hz-5 ms)
#define CONTROL_LOOP_PERIOD			0.005f 										// 0.002 s
#define CONTROL_LOOP_PERIOD_MS  	(uint16_t)(CONTROL_LOOP_PERIOD * 1000.0f)	// Hız - Konum kontrol ve RPM ölçüm periyodu
#define STATE_LOOP_PERIOD_MS		8u										// Ana programın çalışma periyodu
#define IO_LOOP_PERIOD_MS			100u										// Giriş tarama periyodu
#define LED_LOOP_PERIOD_MS			1000u										// LED yanıp sönme periyodu
#define SERIAL_COMM_LOOP_PERIOD_MS	5u
//*** END: main.c tanımları ***

//*** BEGIN: IO.c tanımları ***
// UpdateADC_FromDMA_Task()
#define AIN_VALUE_MAX			4095.0f		        // Analog inputtan okunacak maks. değer
#define ADC_CHANNEL_COUNT   	3u                  // ADC kanal sayısı
#define REF_VELOCITY_MAX        35000.0f   	        // Referans hız maksimum değeri (rpm)
#define REF_VELOCITY_MIN        50.0f   	        // Referans hız minimum değeri (rpm)
#define REF_VELOCITY_STEP 		10u					// Ölçekli çalışmada referans hız değişimi basamak değeri (rpm)
#define K_POT2MOTOR_RPM	        (REF_VELOCITY_MAX / AIN_VALUE_MAX) 	// Pot değeri > rpm dönüşüm katsayısı
#define K_POT2PWM_VALUE	        (1.0f / AIN_VALUE_MAX) 				// Pot değeri > pwm duty cycle (0..1) dönüşüm katsayısı
#define K_MOTOR_VOLTAGE_SENSE	0.0106f   			// ADC tam skalada hedef motor gerilim katsayısı (V)
#define K_MOTOR_CURRENT_SENSE	1.6f   				// ADC tam skalada hedef motor akım katsayısı (mA)
//#define K_MOTOR_CURRENT_SENSE	0.0016f   			// ADC tam skalada hedef motor akım katsayısı (A)

// UpdateEncoder_Task()
#define ENCODER_PPR             512u   	            // Enkoder Puls Per Revolution (mekanik tur başına pulse)
#define ENCODER_CPR             (ENCODER_PPR * 4u)  // Quadrature 4x -> counts per rev
#define K_ACT_POSITION			360.0f / (float)ENCODER_CPR						// UpdateEncoder_Task() içindeki fark sayaç > anlık konum dönüşüm katsayısı
#define K_RPM_CONVERSION  		(60.0f / (ENCODER_CPR * CONTROL_LOOP_PERIOD))	// UpdateEncoder_Task() içindeki fark sayaç > anlık hız dönüşüm katsayısı

// DriveMotor()
#define PWM_TIMER_PERIOD        2399u      	        // TIM1->ARR (örneğin 30 kHz için)
//*** END: IO.c tanımları ***


//*** BEGIN: MotorControl.c tanımları ***
// MotorControl_Task()
#define POS_ERROR_DEADBAND_DEG	0.5f										// Konum kontrolde hedefe ulaşmadaki ölü bölge tanımıdır. (derece)

// InitControl()


// PositionControl()
#define K_RPM2DEGS				6.0f										// Konum kontrolde RPM cinsinden devir bilgisini DEG/S e çevirir.
#define K_DEGS2RPM				(1.0f / K_RPM2DEGS)								// Konum kontrolde DEG/S cinsinden devir bilgisini RPM e çevirir
#define POS_UO_SAT_LIMIT_MAX	REF_VELOCITY_MAX							// Konum kontrolde kontrolcü çıkışı saturasyon maksimum limiti (rpm)
#define POS_UO_SAT_LIMIT_MIN	(-1.0f * REF_VELOCITY_MAX)					// Konum kontrolde kontrolcü çıkışı saturasyon minimum limiti (rpm)

// VelocityControl()
#define K_RPM_TIMER_PERIOD	    	(PWM_TIMER_PERIOD / REF_VELOCITY_MAX)	// Hız döngüsünde RPM cinsinden hesaplanan kontrol sinyalini TIMER PERIYOT a çevirir
#define K_INV_VELOCITY				(1.0f / REF_VELOCITY_MAX)
#define VEL_FRICTION_FF_LIMIT_RPM	6.0
#define VEL_FRICTION_FF_DUTY		0.049f  								// Hız döngüsünde kuru sürtünmeyi yemek için %3.5 Duty Cycle ekliyoruz.
#define VEL_UI_CLAMP_LIMIT_MAX		1.0f	// 1.0f								// Hız kontrol ui_vel maksimum clamp limiti
#define VEL_UI_CLAMP_LIMIT_MIN		-1.0f	// -1.0f								// Hız kontrol ui_vel minimum clamp limiti
#define VEL_UO_SAT_LIMIT_MAX		1.0f	// 1.0f								// Hız kontrol maksimum hız saturasyon limiti (PWM duty cycle)
#define VEL_UO_SAT_LIMIT_MIN		0.0f									// Hız kontrol minimum hız saturasyon limiti (PWM duty cycle)
#define VEL_ERROR_DEADBAND			3.0f	//1.0f								// Hız kontrolde hedefe ulaşmadaki ölü bölge tanımıdır.

// TrajectoryGeneratorVel()
#define TRAGEN_VEL_ACCEL_RPM_S     	25000.0f			//15000.0f 				// Hız yörünge hesaplayıcı içindeki saniyedeki rpm hızlanma (ivme) (1 saniyede kaç rpm hızlanacak)

// TrajectoryGeneratorPos()
#define TRAGEN_POS_ACCEL_RPM_S				8000.0f 	// 5000.0f			// Trajectory generation içindeki saniyedeki rpm hızlanma (ivme) (1 saniyede kaç rpm hızlanacak)
#define TRAGEN_POS_ACCEL_DEG_S2				(TRAGEN_POS_ACCEL_RPM_S * K_RPM2DEGS)
#define TRAGEN_POS_ERROR_DEADBAND_DEG		3.0f	//0.5					// Konum kontrolde hedefe ulaşmadaki ölü bölge tanımıdır. (deg)
#define TRAGEN_VEL_ERROR_DEADBAND_DEGS		6.0f	//6.0					// Trajectory generation içindeki hız kontrolde hedefe ulaşmadaki ölü bölge tanımıdır. (deg/s)

// VelocitySupervisor()
#define K_VEL_SUPERVISOR_TH					1.2f		// Velocity supervisor içinde stall algılamada kullanılan aşım katsayısı

// StallSupervisor()
#define STALL_CURRENT_LIMIT			200.0f		// Stall durumunu algılamak için gerekli akım limit değeri
#define	STALL_PWM_LIMIT				0.9f		// Duty bu değerin üstündeyse "zorluyor" (stall) say, PWM Duty eşiği (%90)
#define	STALL_POS_ERROR_LIMIT		-50.0f		// Konum hatası bu değerin üstündeyse "zorluyor" (stall) say, Hız hatası eşiği (RPM)
#define	STALL_VEL_ERROR_LIMIT		200.0f		// Hız hatası bu değerin üstündeyse "zorluyor" (stall) say, Hız hatası eşiği (RPM)
#define STALL_SUPPRESS_TIME_MS  	50u      	// Bu süre kadar sıkışırsa PROTECT moduna geç
#define STALL_WAIT_TIME_MS      	500u     	// Koruma modunda bekleme süresi
#define STALL_SUPPRESS_TIME_TICKS   ((uint16_t)(STALL_SUPPRESS_TIME_MS / (CONTROL_LOOP_PERIOD_MS)))
#define STALL_WAIT_TIME_TICKS     	((uint16_t)(STALL_WAIT_TIME_MS / (CONTROL_LOOP_PERIOD_MS)))

//*** END: MotorControl.c tanımları ***


//*** BEGIN: Filter.c tanımları ***
#define K_LPF_RPM_ACTUAL		0.1f	//0.02	// Encoderden okunan hız değeri için Adaptif LPF Katsayısı
#define K_LPF_POT				0.05f   	// ADC okunan değer için Adaptif LPF Katsayısı
#define K_LPF_MOTOR_VOLTAGE		0.1f   	// ADC okunan değer için Adaptif LPF Katsayısı
#define K_LPF_MOTOR_CURRENT		0.2f   	// ADC okunan değer için Adaptif LPF Katsayısı
#define K_FILTER_MEDIAN_WS		5u   		// Median Filter window size

#define K_A_LPF_DIFF_LIMIT_HIGH	20.0f		// Adaptif LPF (şimdiki değer - önceki değer) farkı üst bölge limit değeri
#define K_A_LPF_DIFF_LIMIT_MID	5.0f		// Adaptif LPF (şimdiki değer - önceki değer) farkı orta ve alt bölge limit değeri
#define K_A_LPF_ALPHA_HIGH		0.5f		// Adaptif LPF yüksek hızlı değişim alpha katsayısı (0..1)
#define K_A_LPF_ALPHA_MID		0.1f		// Adaptif LPF orta hızlı değişim alpha katsayısı (0..1)
#define K_A_LPF_ALPHA_LOW		0.02f		// Adaptif LPF düşük hızlı değişim alpha katsayısı (0..1)

#define K_LPF_UO_VEL			0.8f		// VelocityControl() çıkışındaki alçak geçiren filtre katsayısı.

#define K_AB_ALPHA				0.15f	//0.2f		// Alpha-Beta Filtre Alpha katsayısı
#define K_AB_BETA				0.005f//0.012f	//0.022f		// Alpha-Beta Filtre Beta katsayısı
//*** END: Filter.c tanımları ***


//*** BEGIN: SerialComm.c tanımları ***
#define RX_BUFFER_SIZE			256u				// USART üzerinden veri alırken kullanılan buffer dizisi uzunluğu
#define TX_BUFFER_SIZE			256u				// USART üzerinden veri gönderirken kullanılan buffer dizisi uzunluğu
#define RX_TIMEOUT 				500u	// 5u				// USART üzerinden veri alırken timeout değeri (ms)
#define TX_TIMEOUT 				500u	// 200u			// USART üzerinden veri gönderirken timeout değeri (ms)
//*** END: SerialComm.c tanımları ***

#endif /* INC_APPCONFIG_H_ */
