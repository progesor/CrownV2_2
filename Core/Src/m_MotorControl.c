/*
 * m_MotorControl.c
 *
 *  Created on: Dec 7, 2025
 *      Author: Baris
 */

#include "m_MotorControl.h"
#include "m_SharedMemory.h"
#include "AppConfig.h"
#include <math.h>

// MotorControl_Task()
volatile float ref_position_deg = 0.0f;
volatile float act_position_offset = 0.0f;   			// Trajectory generation başındaki mutlak pozisyon
volatile uint8_t stall_exists = 0u;		// *** Debug modunda stall a girip girmediğini algılamak için geçici olarak global ekledim.

// ***VelocitySupervisor detection algoritmasını geliştirirken geçici olarak global tanımladım. Normalde local olacak.
volatile float rv = 0.0f;
volatile float rv1 = 0.0f;

// Position kontrol için (P kontrolör)
float sat_limit_max_position = REF_VELOCITY_MAX;	// Pozisyon kontrol döngüsündeki maks. hız saturasyon limiti (rpm)

// Velocity kontrol için (PI kontrolör)
float k_anti_windup = 0.0f;					// Hız kontrol K_Anti_Windup kazancı *** TI çarpılacak
float ui_vel = 0.0f;						// İntegratör çıkış değeri
float uo_vel_sat0 = 0.0f;

// TrajectoryGeneratorVel()
static float ramped_ref_rpm = 0.0f;



// TrajectoryGeneratorPosRel()
//volatile float traj_pos = 0.0f;      						// Anlık Yörünge Konumu (Sanal Lider)
//volatile float traj_vel = 0.0f;      						// Anlık Yörünge Hızı (Feed Forward için)
//volatile uint8_t target_index = 0;   						// 0: Min konuma git, 1: Max konuma git


// StallSupervisor()
StallState_t act_stall_state = STALL_IDLE;
uint16_t stall_suppress_cnt = 0u;
uint16_t stall_wait_cnt = 0u;
uint8_t stall_detected = 0u;	// *** Debug modunda stall a girip girmediğini algılamak için geçici olarak global ekledim.

void MotorControl_Task(void)
{
	switch(sm.act_motor_state)
	{
		case MOT_STATE_IDLE:
		{
			DriveMotor(0.0f, 1u, 1u);
		} break;

		case MOT_STATE_POS_CONTROL_INIT:
		{
			InitControlVel();
			InitControlPos();

			sm.act_motor_state = MOT_STATE_POS_CONTROL;
		} break;

		case MOT_STATE_POS_CONTROL:
		{
			// Her tick'te relative (göreli) konumu hesapla (Başlangıç noktasını 0 kabul eder)
			sm.act_position_relative = sm.act_position - act_position_offset;

			// Yörünge yöneticisini çalıştır
			RunOscillationTrajectory();

		} break;

		case MOT_STATE_POS_CONTROL_END:
		{
			VelocityControl(0.0f, sm.act_velocity, 0.0f);

			if(fabsf(sm.act_velocity) < VEL_ERROR_DEADBAND)
			{
				DriveMotor(0.0f, 1u, 0u);
				sm.act_motor_state = MOT_STATE_IDLE;
			}

		} break;

		case MOT_STATE_VEL_CONTROL_INIT:
		{
			InitControlVel();
			sm.act_motor_state = MOT_STATE_VEL_CONTROL;
		} break;

		case MOT_STATE_VEL_CONTROL:
		{
			float rv = TrajectoryGeneratorVel(sm.ref_velocity, TRAGEN_VEL_ACCEL_RPM_S);
			VelocityControl(rv, sm.act_velocity, 0.0f);
		} break;

		case MOT_STATE_VEL_CONTROL_END:
		{
			float rv = TrajectoryGeneratorVel(0.0f, TRAGEN_VEL_ACCEL_RPM_S);
			VelocityControl(rv, sm.act_velocity, 0.0f);
			//VelocityControl(0.0f, sm.act_velocity, 0.0f);
			if(fabsf(sm.act_velocity) < VEL_ERROR_DEADBAND)
			{
				DriveMotor(0.0f, 1u, 0u);
				sm.act_motor_state = MOT_STATE_IDLE;
			}
		} break;

		case MOT_STATE_MANUAL_CONTROL:
		{
			float dc = sm.duty_cycle_command;
			//float dc = sm.act_pot_value * K_POT2PWM_VALUE;
			DriveMotor(dc, 1u, 1u);
		} break;

	}
}

// Trapezoidal Osilasyon Fonksiyonu
void RunOscillationTrajectory(void)
{
    float target_pos = (sm.osc_state == 0) ? sm.osc_target_deg : 0.0f;
    float err_pos = target_pos - sm.act_position_relative;
    float abs_err = fabsf(err_pos);

    // --- 1. SIÇRAMA KORUMASI VE DURMA ---
    uint8_t target_reached = 0;
    if (sm.osc_state == 0 && sm.act_position_relative >= sm.osc_target_deg) target_reached = 1;
    if (sm.osc_state == 1 && sm.act_position_relative <= 0.0f) target_reached = 1;

    // Hedefi geçtik mi VEYA tolerans sınırına (1 derece) girdik mi?
    if (target_reached || abs_err < 1.0f)
    {
        sm.current_traj_rpm = 0.0f;
        VelocityControl(0.0f, sm.act_velocity, 0.0f); // Sadece fren yap

        // Motor fiziksel olarak durana kadar bekle
        if (fabsf(sm.act_velocity) < 50.0f) {
            sm.osc_state = (sm.osc_state == 0) ? 1 : 0;
            InitControlVel(); // PID'yi sıfırla ki sıçrama yapmasın
        }
        return;
    }

    // --- 2. HİBRİT KİNEMATİK (Karekök Patlamasını Önleyen Kısım) ---
    float a_deg_s2 = sm.osc_accel_rpm_s * 6.0f;
    if (a_deg_s2 < 1.0f) a_deg_s2 = 1.0f;

    //float max_allowed_rpm = 0.0f;

    if (abs_err > 15.0f)
    {
        // Hedefe 15 dereceden fazla varsa: KAREKÖK (Agresif ve maksimum hızlı yaklaşım)
    	sm.max_allowed_rpm = sqrtf(2.0f * a_deg_s2 * abs_err) / 6.0f;
    }
    else
    {
        // Hedefe 15 dereceden AZ kaldıysa: DOĞRUSAL (İpek gibi süzülme)
        // 15 derecedeki hızı hesaplayıp oradan 0'a doğru çizgi çekiyoruz.
        float v_15_rpm = sqrtf(2.0f * a_deg_s2 * 15.0f) / 6.0f;
        sm.max_allowed_rpm = v_15_rpm * (abs_err / 15.0f);

        // Motorun kilitlenmemesi için minimum bir süzülme hızı (150 RPM) veriyoruz
        if (sm.max_allowed_rpm < 150.0f) sm.max_allowed_rpm = 150.0f;
    }

    if (sm.max_allowed_rpm > sm.osc_max_rpm) {
    	sm.max_allowed_rpm = sm.osc_max_rpm;
    }

    // Yön Ataması
    float target_rpm_with_dir = sm.max_allowed_rpm;
    if (err_pos < 0.0f) target_rpm_with_dir = -target_rpm_with_dir;

    // --- 3. İVME SINIRLAYICI (Slew Rate) ---
    float step_rpm = sm.osc_accel_rpm_s * CONTROL_LOOP_PERIOD; // Eğer loop 1ms ise ivme çok hassas uygulanır

    if (sm.current_traj_rpm < target_rpm_with_dir) {
        sm.current_traj_rpm += step_rpm;
        if (sm.current_traj_rpm > target_rpm_with_dir) sm.current_traj_rpm = target_rpm_with_dir;
    } else if (sm.current_traj_rpm > target_rpm_with_dir) {
        sm.current_traj_rpm -= step_rpm;
        if (sm.current_traj_rpm < target_rpm_with_dir) sm.current_traj_rpm = target_rpm_with_dir;
    }

    // Kontrolcüye Gönder
    VelocityControl(sm.current_traj_rpm, sm.act_velocity, 0.0f);
}

void InitControlVel(void)
{
	k_anti_windup = 1.0f * (sm.ki_velocity / sm.kp_velocity);	// VelocityControl() içindeki Anti-Windup katsayısını günceller.
	ui_vel = 0.0f;				// VelocityControl() içindeki integral kontrolcü çıkışını resetler.
	uo_vel_sat0 = 0.0f;			// VelocityControl() içindeki kontrol sinyali çıkışında yer alan LPF eski çıkış değerini resetler.
	ramped_ref_rpm = 0.0f;		// TrajectoryGeneratorVel() içindeki rampa referans değerini sıfırlar.
}

void InitControlPos(void)
{
	act_position_offset = sm.act_position;	// Başlangıç mutlak konumu al, offset olarak ayarlar. (relative referansın orijini)
	ref_position_deg = sm.ref_position + act_position_offset;	// Realtive uzayda tanımlı ref_position ı mutlak uzaya taşır.
	sm.act_position_relative = 0.0f;		// Relative uzaydaki relative konum hesabında kullanılır.
	stall_suppress_cnt = 0u;
	stall_wait_cnt = 0u;
	stall_detected = 0u;
}

float PositionControl(float ref_pos_deg, float act_pos_deg, float uo_vel_limit)
{
	// 1) Hata: referans derece - ölçülen derece hesaplanır.
	float error = ref_pos_deg - act_pos_deg;

	// 2) Oransal kontrolcü çıkışı hesaplanır ve konum ileri besleme eklenir.
	float up_pos = sm.kp_position * error;

	// 3) P kontrolcü çıkışı HIZ doyum sınırlarına göre değerlendirilir.
//	if(up_pos > POS_UO_SAT_LIMIT_MAX) up_pos = POS_UO_SAT_LIMIT_MAX;
//	else if(up_pos < POS_UO_SAT_LIMIT_MIN) up_pos = POS_UO_SAT_LIMIT_MIN;

	if(up_pos > uo_vel_limit) up_pos = uo_vel_limit;
	else if(up_pos < -uo_vel_limit) up_pos = -uo_vel_limit;

	return up_pos;
}

// m_MotorControl.c içindeki VelocityControl fonksiyonunun güncellenmiş hali
void VelocityControl(float ref_vel_rpm, float act_vel_rpm, float ff_vel_rpm)
{
	uint8_t dir = 1;

	// 1) Yön belirleme
	if(ref_vel_rpm < 0)
	{
		dir = 0;
		ref_vel_rpm *= -1.0f;
		act_vel_rpm *= -1.0f;
	}

	// 2) Normalize etme (Hızları 35000'e bölerek 0.0 ile 1.0 arasına çekiyoruz)
	// (Eğer senin kodunda K_INV_VELOCITY makrosu varsa onu kullanabilirsin, ben garanti olması için böldüm)
	float ref_vel_norm = ref_vel_rpm / 35000.0f;
	float act_vel_norm = act_vel_rpm / 35000.0f;

	// 3) Hata hesaplama
	float error = ref_vel_norm - act_vel_norm;

	// 4) P (Oransal) Kontrol
	float up_vel = sm.kp_velocity * error;

	// 5) I (İntegral) Kontrol (KRİTİK DÜZELTME: Zaman Çarpanı Eklendi!)
	ui_vel += sm.ki_velocity * error * CONTROL_LOOP_PERIOD;

	// İntegratör Kırpma (Windup Koruması) PWM maks %100 olabilir
	if(ui_vel > 1.0f) ui_vel = 1.0f;
	if(ui_vel < -1.0f) ui_vel = -1.0f;

	// 6) Toplam Çıkış (PID Toplamı)
	float uo_vel = up_vel + ui_vel;

	// 7) Çıkış Sınırlandırma (PWM Duty 0.0 ile 1.0 arası olmalı)
	if(uo_vel > 1.0f) uo_vel = 1.0f;
	else if(uo_vel < 0.0f) uo_vel = 0.0f;

	// Motoru sür! Brake = 1 (Fren kapalı, sürüş aktif)
	//DriveMotor(uo_vel, dir, 1u);

	// *** KRİTİK DÜZELTME: MOTOR TİTREŞİMİ VE ISINMAYI ÖNLEYEN FİLTRE ***
	// 1000 Hz'de oluşan anlık matematiksel PID zıplamalarını sönümler, motora ipek gibi PWM basar.
	uo_vel_sat0 = (K_LPF_UO_VEL * uo_vel_sat0) + ((1.0f - K_LPF_UO_VEL) * uo_vel);

	// Motoru sür! Filtrelenmiş uo_vel_sat0 kullanılıyor.
	DriveMotor(uo_vel_sat0, dir, 1u);
}

float TrajectoryGeneratorVel(float target_rpm, float accel_limit_rpm_s)
{
	if (!isfinite(target_rpm))  target_rpm = 0.0f;
    if (!isfinite(accel_limit_rpm_s)) accel_limit_rpm_s = 0.0f;
	accel_limit_rpm_s = fabsf(accel_limit_rpm_s);

	// 1. Bir adımda değişebilecek maksimum hız (RPM)
    float max_delta = accel_limit_rpm_s * CONTROL_LOOP_PERIOD;

    // 2. Hedef ile o anki rampa değeri arasındaki fark
    float err = target_rpm - ramped_ref_rpm;

    // 3. Rampa değerini güncelle (Slew Rate Limiter Mantığı)
    if (err > max_delta)
    {
        ramped_ref_rpm += max_delta; // Pozitif yönde hızlan
    }
    else if (err < -max_delta)
    {
        ramped_ref_rpm -= max_delta; // Negatif yönde hızlan (veya yavaşla)
    }
    else
    {
        ramped_ref_rpm = target_rpm; // Hedefe çok yakınız, eşitle.
    }
    return ramped_ref_rpm;
}

uint8_t StallSupervisor(float current, float pwm_duty, float ref_pos, float act_pos)
{
	//float pos_error = fabsf(ref_pos - act_pos);
	float pos_error_limit = ref_pos * 0.1f;

	// 1. Algılama Mantığı:
	// Akım çok yüksekse VEYA (PWM doyumda VE Hız hatası büyükse)
	//if( (current > STALL_CURRENT_LIMIT) || (pwm_duty > STALL_PWM_LIMIT) || (pos_error < -STALL_POS_ERROR_LIMIT) )
	if( (act_pos > ref_pos + pos_error_limit) || (act_pos < 0 - pos_error_limit) )
	{
		stall_detected = 1u;
	}

	switch(act_stall_state)
	{
		case STALL_IDLE:
		{
			if(stall_detected)
			{
				act_stall_state = STALL_SUPPRESS; // Anında baskılamaya geç
				stall_suppress_cnt = 0u;
			}
		} break;

		case STALL_SUPPRESS:	// Bu modda VelocityControl, Integratörü sıfırlayacak ve Kp'yi artıracak.
		{
			if (stall_detected)
			{
				stall_suppress_cnt++;
				ui_vel = 0.0f;				// integratör birikimini sıfırla

				// Sıkışma çok uzun sürerse motoru korumaya al
				if(stall_suppress_cnt > STALL_SUPPRESS_TIME_TICKS)
				{
					stall_suppress_cnt = 0u;
					stall_wait_cnt = 0u;
					act_stall_state = STALL_PROTECT;
				}
			}
			else
			{
				stall_detected = 0u;
				act_stall_state = STALL_IDLE;
				// Yük kalktıysa normale dön (Histeresis eklenebilir)
			}
		} break;

		case STALL_PROTECT:		// Bu moddaa motor tamamen durdurulur (PWM = 0)
		{
			stall_wait_cnt++;
			ui_vel = 0.0f;				// integratör birikimini sıfırla
			DriveMotor(0.0f, 1u, 0u);	// Motoru durdur (yumuşak: brake off, duty=0)

			if(stall_wait_cnt > STALL_WAIT_TIME_TICKS)
			{
				stall_wait_cnt = 0u;
				stall_detected = 0u;
				act_stall_state = STALL_IDLE;
				// Çıkışta yumuşak başlangıç için hedef konum güncellenebilir
			}
			return 1;	// VelocityControl'ü atla (Motoru durdur)
		} break;
	}

	return 0;	// Normal kontrol çalışsın (SUPPRESS modunda da çalışır)
}


