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

// MotorControlTask()
volatile float act_position_offset = 0.0f;   			// Trajectory generation başındaki mutlak pozisyon

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

/**
 * @brief Executes the motor control state machine.
 *
 * This task is called periodically (e.g., from the TIM2 ISR) to evaluate the current
 * motor state (`sm.act_motor_state`) and execute the appropriate control logic, such as
 * position control, velocity control, or manual control.
 *
 * @return None
 */
void MotorControlTask(void)
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

		case MOT_STATE_TIME_OSC_CONTROL_INIT:
		{
			InitControlVel();
			sm.osc_timer_ms = 0.0f;
			sm.osc_state = 0u;
			sm.current_traj_rpm = 0.0f;
			sm.act_motor_state = MOT_STATE_TIME_OSC_CONTROL;
		} break;

		case MOT_STATE_TIME_OSC_CONTROL:
		{
			RunOscillationTrajectoryTime();
		} break;

	}
}

/**
 * @brief Executes a trapezoidal position-based oscillation trajectory.
 *
 * Calculates the target position and manages the acceleration/deceleration phases
 * to achieve a smooth oscillation movement. It uses a hybrid kinematic approach
 * to prevent square-root explosions and ensures the motor doesn't stall.
 *
 * @return None
 */
void RunOscillationTrajectory(void)
{
    float target_pos = (sm.osc_state == 0) ? sm.osc_target_deg : 0.0f;
    float err_pos = target_pos - sm.act_position_relative;
    float abs_err = fabsf(err_pos);

    // --- 1. OVERSHOOT (AŞIM) VE ACİL FREN MANTIĞI ---
    // Eğer motor hedefi geçtiyse (veya tam üstündeyse) overshot = 1 olur.
    uint8_t overshot = 0;
    if (sm.osc_state == 0 && err_pos <= 0.0f) overshot = 1;
    if (sm.osc_state == 1 && err_pos >= 0.0f) overshot = 1;

    if (overshot)
    {
        // Hedefi geçtik ama motor hala hızlı dönüyor olabilir.
        // Hız 50 RPM'in altına inmeden yön değiştirirsek açı anında kayar!
        if (fabsf(sm.act_velocity) < 50.0f)
        {
            sm.osc_state = (sm.osc_state == 0) ? 1 : 0;
            sm.current_traj_rpm = 0.0f;
            return;
        }
        else
        {
            // Motor hızlıysa: Yönü sakın değiştirme, tam güç fren yapıp durmasını bekle!
            sm.current_traj_rpm = 0.0f;
            VelocityControl(0.0f, sm.act_velocity, 0.0f);
            return;
        }
    }

    // --- 2. ZAMAN-OPTİMAL FRENLEME EĞRİSİ (Karekök) ---
    float a_deg_s2 = sm.osc_accel_rpm_s * 6.0f; // İvmeyi Deg/s^2'ye çevir
    if (a_deg_s2 < 1.0f) a_deg_s2 = 1.0f;

    // Bu mesafede (abs_err) durabilmek için izin verilen MAKSİMUM KİNETİK HIZ
    // Formül: v = sqrt(2 * a * x)
    float max_kinematic_rpm = sqrtf(2.0f * a_deg_s2 * abs_err) / 6.0f;

    // Hedefe çok yaklaşıldığında (< 2 derece) formül titremeye yol açar.
    // 2 derece kala eğriyi iptal edip yumuşak (lineer) bir iniş yapıyoruz.
    if (abs_err < 2.0f)
    {
        float v_2_rpm = sqrtf(2.0f * a_deg_s2 * 2.0f) / 6.0f;
        max_kinematic_rpm = v_2_rpm * (abs_err / 2.0f);
    }

    if (max_kinematic_rpm > sm.osc_max_rpm)
    {
        max_kinematic_rpm = sm.osc_max_rpm;
    }

    // --- 3. ASİMETRİK İVME YÖNETİMİ (Açı Kaçmasını Çözen Kısım) ---
    float step_rpm = sm.osc_accel_rpm_s * CONTROL_LOOP_PERIOD;
    float current_mag = fabsf(sm.current_traj_rpm);

    // Motoru her döngüde ivme limitin kadar hızlandırmaya çalış.
    current_mag += step_rpm;

    // DİKKAT: Eğer hızlanmaya çalıştığın değer, frenleme eğrisinin (kinematik limit)
    // üstünde kalıyorsa, hızı acımasızca eğriye EZİYORUZ.
    // Bu sayede frenleme asla gecikmez ve hedefte milimetrik duruş sağlar.
    if (current_mag > max_kinematic_rpm)
    {
        current_mag = max_kinematic_rpm;
    }

    // Yönü tekrar hesaba kat
    sm.current_traj_rpm = (err_pos > 0.0f) ? current_mag : -current_mag;

    // --- 4. MOTORU SÜR ---
    VelocityControl(sm.current_traj_rpm, sm.act_velocity, 0.0f);
}

/**
 * @brief Executes a foolproof time-based oscillation trajectory.
 *
 * Calculates trajectory speeds based on a time duration instead of position. It calculates
 * the maximum achievable RPM based on acceleration and time to ensure smooth, symmetrical
 * oscillation without exceeding physical limits.
 *
 * @return None
 */
void RunOscillationTrajectoryTime(void)
{
    // --- 1. AKILLI SİMETRİ ASİSTAN (Fiziksel Limit Koruması) ---
    // Verilen ivme ve süre ile motorun "0" noktasından geçerek çizebileceği maksimum tepe hızı hesaplar.
    // Simetrik dönüş için süre: (osc_time_ms / 1000.0)
    // Ulaşılabilecek Maksimum Hız (Üçgen Dalga Tepe Noktası) = (İvme * Süre) / 2.0
    float max_achievable_rpm = (sm.osc_accel_rpm_s * (sm.osc_time_ms / 1000.0f)) / 2.0f;

    float actual_target_rpm = sm.osc_max_rpm;

    // Eğer istenen hedef hız fiziksel olarak imkansızsa, sistemi tek yöne kaydırmamak
    // ve kusursuz osilasyonu korumak için hedefi yapay olarak tıraşla.
    if (actual_target_rpm > max_achievable_rpm) {
        actual_target_rpm = max_achievable_rpm;
    }

    // --- 2. ZAMANLAYICI VE YÖN KONTROLÜ ---
    sm.osc_timer_ms += (CONTROL_LOOP_PERIOD * 1000.0f);

    if (sm.osc_timer_ms >= sm.osc_time_ms) {
        sm.osc_timer_ms -= sm.osc_time_ms; // Taşmayı koru (Ritim bozulmasın)
        sm.osc_state = (sm.osc_state == 0) ? 1 : 0; // Yön değiştir

        // Integrator Windup (Runaway) koruması: Yön değişiminde hatayı çöpe at.
        float safe_traj = sm.current_traj_rpm;
        InitControlVel();
        sm.current_traj_rpm = safe_traj;
    }

    // --- 3. HIZ VE İVME (YÖRÜNGE) HESAPLAMASI ---
    // Akıllı asistanın tıraşladığı "actual_target_rpm" değerini kullanıyoruz.
    float target_rpm_with_dir = (sm.osc_state == 0) ? actual_target_rpm : -actual_target_rpm;
    float step_rpm = sm.osc_accel_rpm_s * CONTROL_LOOP_PERIOD;

    if (sm.current_traj_rpm < target_rpm_with_dir) {
        sm.current_traj_rpm += step_rpm;
        if (sm.current_traj_rpm > target_rpm_with_dir) sm.current_traj_rpm = target_rpm_with_dir;
    } else if (sm.current_traj_rpm > target_rpm_with_dir) {
        sm.current_traj_rpm -= step_rpm;
        if (sm.current_traj_rpm < target_rpm_with_dir) sm.current_traj_rpm = target_rpm_with_dir;
    }

    // --- 4. MUTLAK ÇIKIŞ BARAJI ---
    if (sm.current_traj_rpm > sm.osc_max_rpm) sm.current_traj_rpm = sm.osc_max_rpm;
    if (sm.current_traj_rpm < -sm.osc_max_rpm) sm.current_traj_rpm = -sm.osc_max_rpm;

    // --- 5. MOTORU SÜR ---
    VelocityControl(sm.current_traj_rpm, sm.act_velocity, 0.0f);
}

/**
 * @brief Initializes the velocity control parameters.
 *
 * Resets the integral term, anti-windup gain, LPF memory, and trajectory generator
 * reference to prepare for a new velocity control phase.
 *
 * @return None
 */
void InitControlVel(void)
{
	k_anti_windup = 1.0f * (sm.ki_velocity / sm.kp_velocity);	// VelocityControl() içindeki Anti-Windup katsayısını günceller.
	ui_vel = 0.0f;				// VelocityControl() içindeki integral kontrolcü çıkışını resetler.
	uo_vel_sat0 = 0.0f;			// VelocityControl() içindeki kontrol sinyali çıkışında yer alan LPF eski çıkış değerini resetler."
	ramped_ref_rpm = 0.0f;		// TrajectoryGeneratorVel() içindeki rampa referans değerini sıfırlar.
}

/**
 * @brief Initializes the position control parameters.
 *
 * Sets the current absolute position as the relative origin and resets stall detection counters
 * to prepare for a new position control phase.
 *
 * @return None
 */
void InitControlPos(void)
{
	act_position_offset = sm.act_position;	// Başlangıç mutlak konumu al, offset olarak ayarlar. (relative referansın orijini)
	sm.act_position_relative = 0.0f;		// Relative uzaydaki relative konum hesabında kullanılır.
	stall_suppress_cnt = 0u;
	stall_wait_cnt = 0u;
	stall_detected = 0u;
}

/**
 * @brief Proportional (P) position controller.
 *
 * Calculates a target velocity based on the position error.
 *
 * @param ref_pos_deg The reference (target) position in degrees.
 * @param act_pos_deg The actual (current) position in degrees.
 * @param uo_vel_limit The maximum allowable velocity output limit.
 * @return The calculated velocity output to correct the position error.
 */
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

/**
 * @brief Proportional-Integral (PI) velocity controller.
 *
 * Calculates the PWM duty cycle to achieve the target reference velocity. Includes
 * integral windup protection and low-pass filtering on the output to prevent motor vibration.
 *
 * @param ref_vel_rpm The reference (target) velocity in RPM.
 * @param act_vel_rpm The actual (current) velocity in RPM.
 * @param ff_vel_rpm Feed-forward velocity in RPM (currently unused).
 * @return None
 */
void VelocityControl(float ref_vel_rpm, float act_vel_rpm, float ff_vel_rpm)
{
    uint8_t dir = 1;

    // 1) YÖN BELİRLEME
    if (ref_vel_rpm < 0.0f)
    {
        dir = 0;
        ref_vel_rpm = -ref_vel_rpm;
        act_vel_rpm = -act_vel_rpm;
    }

    // 2) NORMALIZE ETME (0.0f - 1.0f aralığı)
    float ref_vel_norm = ref_vel_rpm / REF_VELOCITY_MAX;
    float act_vel_norm = act_vel_rpm / REF_VELOCITY_MAX;

    // --- 3) İLERİ BESLEME (FEED-FORWARD) ---
    // Osilasyon modundaki ani hızlanmalara sıfır gecikme ile yanıt verir.
    // PID'nin hata oluşturmasını beklemeden gereken tahmini taban gücü anında motora iletir.
    float u_ff = ref_vel_norm;

    // 4) HATA HESAPLAMA (Artık sadece dış bozucu etkenleri, yani senin elini hissedecek)
    float error = ref_vel_norm - act_vel_norm;

    // 5) P (Oransal) KONTROL
    // Titremeyi önlemek için arayüzden Kp değerini düşük tutmalısın (Örn: 0.5 ile 2.0 arası)
    float up_vel = sm.kp_velocity * error;

    // 6) I (İntegral) KONTROL (TORK CANAVARI)
    // Elinle tuttuğunda gücü üretecek olan kısım burası. Arayüzden Ki'yi yüksek tut (Örn: 15.0 - 40.0)
    float ui_delta = sm.ki_velocity * error * CONTROL_LOOP_PERIOD;
    float ui_raw = ui_vel + ui_delta;

    // 7) HAM ÇIKIŞ TOPLAMI (FF + P + I)
    float uo_raw = u_ff + up_vel + ui_raw;

    // 8) ÇIKIŞ SINIRLANDIRMA (Saturation)
    float uo_sat = uo_raw;
    if (uo_sat > VEL_UO_SAT_LIMIT_MAX) uo_sat = VEL_UO_SAT_LIMIT_MAX;
    else if (uo_sat < VEL_UO_SAT_LIMIT_MIN) uo_sat = VEL_UO_SAT_LIMIT_MIN; // 0.0f

    // 9) GERÇEK ANTI-WINDUP (BCAW)
    float k_aw = 1.0f;
    ui_vel = ui_raw - (k_aw * (uo_raw - uo_sat));

    // 10) ÇIKIŞ FİLTRESİ
    uo_vel_sat0 = (0.2f * uo_vel_sat0) + (0.8f * uo_sat);

    // 11) MOTORU SÜRME
    if (ref_vel_rpm < VEL_ERROR_DEADBAND && act_vel_rpm < VEL_ERROR_DEADBAND)
    {
        ui_vel = 0.0f; // Tam duruş anında integratörü sıfırla ki bir sonraki kalkışta zıplama yapmasın
        DriveMotor(0.0f, 1u, 0u);
    }
    else
    {
        DriveMotor(uo_vel_sat0, dir, 1u);
    }
}

/**
 * @brief Generates a velocity trajectory with a slew rate limit.
 *
 * Smoothly ramps the reference velocity towards the target velocity, limiting the
 * maximum change per control loop based on the specified acceleration limit.
 *
 * @param target_rpm The final target velocity in RPM.
 * @param accel_limit_rpm_s The maximum allowable acceleration in RPM per second.
 * @return The ramped reference velocity for the current control step.
 */
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

/**
 * @brief Supervises the motor to detect and handle stall conditions.
 *
 * Monitors position error (and potentially current/PWM) to determine if the motor is
 * mechanically stalled. If a stall is detected, it handles suppression (e.g., clearing
 * integrators) and eventually protection (stopping the motor).
 *
 * @param current The actual motor current (currently unused in logic).
 * @param pwm_duty The current PWM duty cycle (currently unused in logic).
 * @param ref_pos The reference (target) position.
 * @param act_pos The actual (current) position.
 * @return 1 if the motor is in a protected (stopped) state due to a stall, 0 otherwise.
 */
uint8_t StallSupervisor(float current, float pwm_duty, float ref_pos, float act_pos)
{
	// Hız Modu ve Yeni Zamanlı Osilasyon Modunda Pozisyon Sıkışmasını Yoksay!
	if (sm.act_motor_state == MOT_STATE_VEL_CONTROL || sm.act_motor_state == MOT_STATE_TIME_OSC_CONTROL) {
	      stall_detected = 0u;
	      act_stall_state = STALL_IDLE;
	   return 0;
	}

	//float pos_error = fabsf(ref_pos - act_pos);
	float pos_error_limit = ref_pos * 0.1f;

	// 1. Algılama Mantığı:
	// Akım çok yüksekse VEYA (PWM doyumda VE Hız hatası büyükse)
	//if( (current > STALL_CURRENT_LIMIT) || (pwm_duty > STALL_PWM_LIMIT) || (pos_error < -STALL_POS_ERROR_LIMIT) )
	if( (act_pos > ref_pos + pos_error_limit) || (act_pos < 0.0f - pos_error_limit) )
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


