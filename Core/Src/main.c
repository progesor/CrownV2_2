/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/** 	BCD Teknik ARGE ve Robotik Otomasyon San. Tic. Ltd. Şti.
 	 	İstanbul, 2025

 	 	Bu kod ERTIP firmasının Crown V1.3 model kartı için üretilmiştir.
 	 	STM32F103C8T6 model MCU kullanılmıştır.

 	 	Tim4 Ch1 ve Ch2 combined modda enkoder okumak için kulanılmıştır.
 	 	Enkoder dönüş hızı (rpm), dönüş sayısı (distance_pulse) ve dönüş açısı (derece)
 	 	hesaplanmaktadır.

 	 	- Analog girişler DMA + SCAN MODE ile okunmuştur. Okunan değerlerin manipülasyonu (filtreleme vb.)
 	 	Callback dışında ayrı bir AdcUpdate() fonksiyonu içinde ve farklı interval de yapılmıştır.
 	 	- Callback çalışma frekansı yüksek olduğu için, data manipülasyonu Callback içinde yapıldığında,
 	 	işlemci tüm gücünü DMA Update e harcıyor ve uwTich dahil diğer fonksiyonlar çalışmıyor. Bu nedenle
 	 	hem ADC ranklarının Sampling Time değeri en büyük değer olan 239,5 cycle a çekildi, hem Clock Configuration
 	 	bölümünden ADC1 prescalar değeri en büyük değer olan /8 e ayarlandı (böylece DMA ölçüm frekansı düşürüldü),
 	 	hem de NVIC'ten DMA Interrupt priority si düşürüldü (değer olarak 10 a yükseltildi).
		- TIM1_CH1'den üretilen motor PWM değeri fonksiyonu ters çalışıyordu. Diğer bir ifadeyle __HAL_TIM_SET_COMPARE()
		fonksiyonuna girilen pwmValue değeri yükseldikçe motor devri düşüyordu. Bu sorunu pwmValue değerini max_pwm_periyot
		değerinden çıkararak çözülmüştü. Onun yerine CubeMX > Timers > TIM1 > Configuration > Channel1 içindeki CH polarity
		değerini low yaparak çözdük. Böylelikle her döngüde çıkarma işlemi yapmaktan kurtulduk.
		- main.c dosyası m_IO.c, m_MotorControl.c ve m_Filter.c olarak 3 modüle ayrıldı.
		-- m_IO.c de MCU giriş çıkışları ile ilgili temel fonksiyonlar yazıldı. Bunlar genel olarak DMA ile okunan ham ADC
		verilerinin (pot, motor voltage, motor current) işlenmesi, dijital girişlerin okunması, gijital çıkışların yazılması
		ve PWM üretilmesi olarak sınıflandırılabilir.
		-- m_MotorControl.c de genel olarak komum ve hız kontrol fonksiyonları yazıldı.
		-- m_Filter.c de okunan analog verilerin filtrelenmesi ve ölçeklendirilmesi ile ilgili fonksiyonlar yazıldı. 
		- Hız referansı üreten PositionControl() fonksiyonu oluşturuldu. Konum kontrolünde standart Oransal (P) kontrol
		algoritması kullanıldı. Fonksiyon çıkışı hız doyum değerlerine göre (posSatLimitMax) sınırlandırıldı. Fonksiyona
		konum ileri besleme (ffPos) parametresi eklendi.
		- Motorun sürülmesi için gerekli PWM periyodu değerini üreten VelocityControl() fonksiyonu oluşturuldu. Hız kontrolünde
		standart Oransal-İntegral (PI) kontrol algoritması kullanıldı. İntegratör hesabına döngü peryotu (dTs) eklenerek
		Ki katsayısının döngü periyodu değişiminden etkilenmesi önlendi. Aynı zamanda integratöre geri-hesaplamalı antiwindup
		(Back Calculation AntiWindup - BCAW) algoritması eklenerek integral uçması sorunu önlendi. Fonksiyon çıkışı PWM duty cycle
		doyum değerlerine göre (velSatLimitMin, velSatLimitMax) sınırlandırıldı. Burada bahsedilen duty cycle limitleri fonksiyonun
		global olarak diğer uygulamalarla da uyumlu olabilmesi için 0.0f..1.0f aralığında olacak şekilde belirlendi.
		Böylelikle % duty cycle %0.0f..%100.0f aralığına ayarlanmış oldu. Bu düzenleme Kp ve Ki değerlerinin belirlenmesinde de etkili olacak.
		- BCAW interal uçması önleme algoritmasının etkiniğini düzenleyen kAntiWindup katsayısı döngü periyodu (dTs) ile ilişkilendirilerek
		katsayının döngü periyodu değişiminden etkilenmesi önlendi. kAntiWindup katsayısı belirlenirken genellikle ya Ki değerine ya da 1/dTs
		periyodu değerine eşitlenir.
		- Versiyon 1.0.5 ile birlikte değişken isimlendirmesi camel type'tan alt çizgili tipe dönüştürülmüştür.
		- VelocityControl() fonksiyonu içinde normalize edilmiş değerler kullanılmıştır. Böylelikle hız kontrolcü üniversal uyumlu hale getirilmiştir.
		- Versiyon 1.0.5.1 de kontrol döngüsü Timer 2 interrupt içinde yürütülmektedir. Bu yapıyı uygularken bazı sorunlar çıktı. En belirgin olanı referans
		potundan okunan değerlerde negatif anlamsız pikler oluşmaya başladı. Bunun interrupt çakışmasından kaynaklanabileceği belirtildi. Bu sorun ile uğraşmayı
		şimdilik bıraktım fakat ileride bakacağım.
		- Versiyon 1.0.6.1 de temel olarak seri haberleşmeyi de sisteme ekledim. VelocityControl() döngüsünü friction compansation ve deadband kontrolleri ile daha stabil
		hale getirdim. Belirli hız referansının altında motor hareket etmezken durduğu yerde ısınmasını önlemek için friction compansation etkisini koşulu hale getirdim.
		Böylece vel_ref belirli bir değerin altındaysa friction compansation kanıyor. PID ayarlarını düzelttim.PID kontrol kısmını normalize ettim. Önceleri Kp=15..20 civarı
		gezinirken şimdi Kp=1 civarı. Benzer şekilde Ki=20..30 civarı gezinirken şimdi Ki=0.1 civarında. TrajectoryGenerator() fonksiyonunu 2 farklı versiyon olarak
		yeniden düzenledim. Mutlak (absolute) hareket için TrajectoryGeneratorAbs() ve göreli (relative) hareket için TrajectoryGeneratorRel()oluşturdum.
		Versiyon 1.6.0 absolute olarak kaldı, artık relative'den devam ediyorum. Trajectory generator içerisine POS_DEADBAND ve VEL_DEADBAND parametreleri ekleyerek
		düşük hızlardki kararsız durumların önüne geçtim.
		- Versiyon 1.0.7'de temel olarak shared memory benzeri bir yapıya geçtim. Modüller arasında veri transferi için m_SharedMemory.c isminde yeni bir modül oluşturdum.
		Ayrıca Appconfig.h oluşturarak tüm #define ları içerisine attım. Böylelikl yazılımla ilgili tüm parametreler bir yerde toplanmış oldu. Fakat hız kontrol döngüsünde
		sorun yaşadım sorunun kaynağını bulamadım. Bu nedenle Velocitycontrol() fonksiyonunun stabil çalıştığı 1.5.1'e geri döndüm.
		- Versiyon 1.0.7.1'de 1.0.5.1'i temel aldım. Enkoder verisini doğru okuyabilen en güncel versiyonun bu olduğunu fark ettim.
		V1.0.7.1'i de yedek olarak tutarak V1.0.7.2'ye yükselttim. Bu versiyonda dikkatli olarak sırasıyla SharedMemory ve AppConfig
		yaklaşımlarını entegre edeceğim. Sonrasında hız ve konum kontrol döngülerindeki değişiklikleri aktaracağım. En son da seri haberleşme fonksiyonlarını ekleyeceğim.
		Sonrasında da VelTraGen ve PosTraGen fonksiyonlarının ayarlarını yapacağım.
		- Versiyon 1.0.8'e seri haberleşme eklendi. ERTIP_Crown_V1.3_Haberleşme_Protokolü_V1.1'e göre cevap mesajları gönderilir oldu. TrajectoryGeneratorVel() içindeki
		static float ramped_ref_rpm değişkeni globale taşınarak hız referansı değişimindeki sıfırlama hatası düzeltildi. Oscillation Mode da seri iletişimde donma oluyor.
		Ayrıca motor elle tutulduğunda aşırı salınıma giriyor.
		- Versiyon 1.0.8.1 e clamp fonksiyonu oluştur ve VelocityControl() içindeki antiwindup çıkışına da clamp ekle. Hız ve konum TraGen içindeki ivmelenme değerlerini
		seri haberleşme ile değiştirilebilir yap.


 	 	Dr. Barış DOĞAN
 */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "m_SharedMemory.h"
#include "m_IO.h"
#include "m_MotorControl.h"
#include "m_Filter.h"
#include "m_SerialComm.h"
#include "AppConfig.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern TIM_HandleTypeDef htim1;		// PWM timer (TIM1)
extern TIM_HandleTypeDef htim2;		// Control Loop timer (TIM2)
extern TIM_HandleTypeDef htim4;		// Encoder Handle timer (TIM4)


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void SendSerialData(uint8_t *buffer);

//uint8_t rx_byte = 0u;		// Seri haberleşmeden gelen byte
ProgramState_t operation_state;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
 * @brief  Period elapsed callback in non-blocking mode.
 *
 * Called when the specified timer reaches its period and generates an interrupt.
 * For TIM2, this executes the hard real-time control loop tasks, including
 * reading ADC values, updating the encoder, and running the motor control task.
 *
 * @param  htim Pointer to a TIM_HandleTypeDef structure that contains
 *              the configuration information for the specified TIM module.
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        HAL_GPIO_WritePin(DO_Led_GPIO_Port, DO_Led_Pin, 1u);
        UpdateAdcFromDmaTask();
        UpdateEncoderTask();
        MotorControlTask();
        HAL_GPIO_WritePin(DO_Led_GPIO_Port, DO_Led_Pin, 0u);
    }
}

/**
 * @brief Safely sends a buffer of data over UART3 without blocking forever.
 *
 * Implements a non-blocking or timeout-limited method to transmit data over
 * the serial connection. This ensures the main loop isn't stalled if the
 * communication hardware fails.
 *
 * @param buffer Pointer to the null-terminated string/data to be sent.
 * @retval None
 */
void SendSerialData(uint8_t *buffer)
{
    if (buffer == NULL) return;
    uint16_t len = strlen((char*)buffer);
    // 10ms timeout sayesinde cihaz donanımsal arıza olsa bile ASLA çökmez/kilitlenmez!
    HAL_UART_Transmit(&huart3, buffer, len, 10);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // *** TEST: SysTick'i doğrudan configure et ***

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // ADC ve DMA başlatma işlemleri
  InitAdc();

  // TIM1: PWM CH1 başlat (PA8)
  InitTim1();

  // TIM2: Control Loop
  InitTim2();

  // TIM4: Enkoder sayacı timer'ını başlat
  InitTim4();

  // *** UART'I TIM2'DEN DAHA ÖNCELİKLİ HALE GETİR (Overrun Çözümü) ***
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0); // En yüksek öncelik
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);   // Daha düşük öncelik

  InitSerialComm();

  SharedMemoryInit();
  LoadParamsFromFlash(); // YENİ: Açılışta kalıcı hafızadan PID ve Osilasyon ayarlarını çeker

  InitControlVel();

  uint32_t serial_comm_loop_ms = HAL_GetTick();
  uint32_t state_loop_ms = HAL_GetTick();
  uint32_t io_loop_ms = HAL_GetTick();
  uint32_t led_loop_ms = HAL_GetTick();

  operation_state = OP_STATE_IDLE;
  uint32_t time_counter = 0u;

  char boot_msg[] = "\n[STM32] SISTEM BASARIYLA BASLATILDI!\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)boot_msg, strlen(boot_msg), 100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  uint32_t now_ms = HAL_GetTick();

	  	// --- YENİ: OTONOM DONANIMSAL PEDAL KONTROLÜ (ACTIVE-LOW) ---
	  	      static uint8_t last_pedal_state = 1; // Başlangıçta pedal çekili (1) varsayılır
	  	      uint8_t current_pedal_state = HAL_GPIO_ReadPin(DI_Pedal_GPIO_Port, DI_Pedal_Pin);

	  	      // 1. DURUM: Pedal BASILDI (1'den 0'a Düştü - Düşen Kenar)
	  	      if (current_pedal_state == 0 && last_pedal_state == 1) {
	  	          if (sm.act_motor_state == MOT_STATE_IDLE) {
	  	              // Son kullanılan moda göre motoru şahlandır!
	  	              if (sm.last_active_mode == 1) {
	  	                  sm.act_motor_state = MOT_STATE_VEL_CONTROL_INIT;
	  	              } else if (sm.last_active_mode == 2) {
	  	                  sm.act_motor_state = MOT_STATE_POS_CONTROL_INIT;
	  	              } else if (sm.last_active_mode == 3) {
	  	                  sm.act_motor_state = MOT_STATE_TIME_OSC_CONTROL_INIT;
	  	              }
	  	              // Node.js backend'ine bilgi ver
	  	              SendSerialData((uint8_t*)"<EVT:PEDAL,1>\n");
	  	          }
	  	      }
	  	    // 2. DURUM: Pedal ÇEKİLDİ (0'dan 1'e Çıktı - Yükselen Kenar)
	  	          else if (current_pedal_state == 1 && last_pedal_state == 0) {
	  	              // Motoru acil olarak güvenli durdurma moduna al
	  	              sm.act_motor_state = MOT_STATE_IDLE;

	  	              // DİKKAT: sm.ref_velocity = 0.0f; SATIRINI SİLDİK!
	  	              // (Tekrar basıldığında eski hızı hatırlaması için)

	  	              sm.current_traj_rpm = 0.0f;
	  	              InitControlVel();
	  	              DriveMotor(0.0f, 1u, 1u);

	  	              SendSerialData((uint8_t*)"<EVT:PEDAL,0>\n");
	  	          }

	  	      last_pedal_state = current_pedal_state; // Durumu güncelle

	        // *** YENİ: KİLİTLENEN UART'I KURTARICI ***
	        // Eğer kesme içinde başlatılamayan bir dinleme varsa, ana döngüde yeniden dene!
	        if (rx_needs_restart)
	        {
	            if (HAL_UART_Receive_IT(&huart3, &rx_byte, 1) == HAL_OK) {
	                rx_needs_restart = 0u; // Başarıyla kurtarıldı!
	            }
	        }

	        // *** KRİTİK BLOK KORUMASI (Data Race Önlemi) ***
	        if(rx_data_ready)
	        {
	            __disable_irq();             // Tüm kesmeleri (TIM2 dahil) kısa süreliğine DURDUR
	            ProcessBinaryPacket();     // Verileri güvenle ve bölünmeden güncelle
	            __enable_irq();              // Kesmeleri tekrar AÇ

     	      rx_data_ready = 0u;
	        }

	  	// *** MEDİKAL GÜVENLİK KİLİDİ (WATCHDOG) ***
	  	   // Eğer motor boşta değilse VE son geçerli komuttan bu yana 1000ms (1 saniye) geçtiyse
	  	      if (sm.act_motor_state != MOT_STATE_IDLE)
	  	      {
	  	          // DÜZELTME: now_ms yerine doğrudan o anki HAL_GetTick() değerini kullanıyoruz (Underflow Önlemi)
	  	          if ((HAL_GetTick() - sm.last_valid_comm_time) > 1000u)
	  	          {
	  	              // İletişim koptu! Motoru derhal durdur.
	  	              sm.act_motor_state = MOT_STATE_IDLE;
	  	              sm.ref_velocity = 0.0f;
	  	              sm.current_traj_rpm = 0.0f;
	  	              InitControlVel();
	  	              DriveMotor(0.0f, 1u, 1u);

	  	              SendSerialData((uint8_t*)"<DBG: WATCHDOG_EMERGENCY_STOP>\n");
	  	          }
	  	      }

	  if ((now_ms - serial_comm_loop_ms) >= 50u)
	        {
	            // ESKİ HALİ: serial_comm_loop_ms += 50u;
	            // YENİ HALİ: İşlemci gecikse bile SPAM yapmasını önler!
	            serial_comm_loop_ms = now_ms;

	            snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "<TEL,%.0f,%.1f,%.2f,%.2f>\n",
	                     sm.act_velocity, sm.act_position, sm.act_pwm_duty, sm.act_motor_current);
	            SendSerialData(tx_buffer);
	        }


	  // *** BEGIN: State Loop
	  if ((now_ms - state_loop_ms) >= STATE_LOOP_PERIOD_MS)
	  {
		  state_loop_ms += STATE_LOOP_PERIOD_MS;

		  //HAL_GPIO_WritePin(DO_Led_GPIO_Port, DO_Led_Pin, 1);

		  switch(operation_state)
		  {
		  	  case OP_STATE_IDLE:	// idle
		  	  {
		  		if((sm.pedal_active && !sm.pedal_state) || sm.operation_start)
		  		{
		  			sm.operation_start = 0u;
		  			if(sm.control_mode == 0)
		  			{
		  				operation_state = OP_STATE_CONT_MODE_INIT;				// Trigger CONTINUOUS mode
		  			}
		  			else if(sm.control_mode == 1)
		  			{
		  				operation_state = OP_STATE_OSC_MODE_VEL_CONTROL_INIT;	// Trigger OSCILLATION mode
		  			}
		  			else
		  			{
		  				operation_state = OP_STATE_MAN_MODE;					// Trigger MANUAL mode
		  			}
		  		}
		  	  } break;

		  	  case OP_STATE_CONT_MODE_INIT:	// CONTINUOUS Mode 1) Trigger Velocity Control
		  	  {
		  		  sm.act_motor_state = MOT_STATE_VEL_CONTROL_INIT;
		  		  operation_state = OP_STATE_CONT_MODE;
		  	  } break;

		  	  case OP_STATE_CONT_MODE:	// CONTINUOUS Mode 2) Velocity Control
		  	  {
		  		  if(sm.act_motor_state == MOT_STATE_VEL_CONTROL)		// Hız kontrol döngüsü başlamışsa...
		  		  {
		  			  if((sm.pedal_active && sm.pedal_state) || sm.operation_stop)	// Pedal kullanımı aktifse ve pedal bırakılmışsa veya Durdur butonuna basılmışsa...
		  			  {
		  				  sm.operation_stop = 0u;	// Durdur işlemini resetle.
		  				  sm.act_motor_state = MOT_STATE_VEL_CONTROL_END;
		  				  operation_state = OP_STATE_CONT_MODE_END;		// Motor durdur.
		  			  }
		  		  }
		  	  } break;

		  	  case OP_STATE_CONT_MODE_END:	// CONTINUOUS Mode 3) End Velocity Control
		  	  {
  				  if(sm.act_motor_state == MOT_STATE_IDLE)
  				  {
  					  operation_state = OP_STATE_IDLE;
  				  }
		  	  } break;

		  	  case OP_STATE_OSC_MODE_VEL_CONTROL_INIT:	// OSCILLATION Mode 1) Trigger Velocity Control
		  	  {
		  		  time_counter = 0u;									// TIME gecikmesi için sayaç sıfırlanır.
		  		  sm.act_motor_state = MOT_STATE_VEL_CONTROL_INIT;		// Hız kontrol döngüsü başlatılır.
		  		  operation_state = OP_STATE_OSC_MODE_VEL_CONTROL;
		  	  } break;

		  	  case OP_STATE_OSC_MODE_VEL_CONTROL:	// OSCILLATION Mode 2) Velocity Control
		  	  {
		  		  if(sm.act_motor_state == MOT_STATE_VEL_CONTROL)		// Hız kontrol döngüsü başlamışsa...
		  		  {
		  			  time_counter += STATE_LOOP_PERIOD_MS;			// Oscillation Mode başında hız döngüsünü çalışma süresini belirleyen sayaç
		  			  if(time_counter >= sm.time_delay)				// Sayaç değeri arayüzden belirlenen TIME değerine ulaşmışsa...
		  			  {
		  				  time_counter = 0u;								// Sayaç sıfırlanır.
				  		  sm.act_motor_state = MOT_STATE_VEL_CONTROL_END;
		  				  operation_state = OP_STATE_OSC_MODE_VEL_CONTROL_END;
		  			  }
		  		  }
		  	  } break;

		  	  case OP_STATE_OSC_MODE_VEL_CONTROL_END:	// OSCILLATION Mode 3) End Velocity Control
		  	  {
  				  if(sm.act_motor_state == MOT_STATE_IDLE)
  				  {
  					  operation_state = OP_STATE_OSC_MODE_POS_CONTROL_INIT;
  				  }
		  	  } break;


		  	  case OP_STATE_OSC_MODE_POS_CONTROL_INIT:	// OSCILLATION Mode 4) Trigger Position Control
		  	  {
		  		  sm.act_motor_state = MOT_STATE_POS_CONTROL_INIT;		// Konum kontrol döngüsü başlatılır.
		  		  operation_state = OP_STATE_OSC_MODE_POS_CONTROL;
		  	  } break;

		  	  case OP_STATE_OSC_MODE_POS_CONTROL:	// OSCILLATION Mode 5) Position Control
		  	  {
		  		  if(sm.act_motor_state == MOT_STATE_POS_CONTROL)		// Konum kontrol döngüsü başlamış mı kontrol edilir.
		  		  {
		  			  if((sm.pedal_active && sm.pedal_state) || sm.operation_stop)	// Pedal kullanımı aktifse ve pedal bırakılmışsa veya Durdur butonuna basılmışsa...
		  			  {
		  				  sm.operation_stop = 0u;	// Durdur işlemini resetle.
				  		  sm.act_motor_state = MOT_STATE_POS_CONTROL_END;
		  				  operation_state = OP_STATE_OSC_MODE_POS_CONTROL_END;		// Motor durdur.
		  			  }
		  		  }
		  	  } break;

		  	  case OP_STATE_OSC_MODE_POS_CONTROL_END:	// OSCILLATION Mode 6) End Position Control
		  	  {
  				  if(sm.act_motor_state == MOT_STATE_IDLE)
  				  {
  					  operation_state = OP_STATE_IDLE;
  				  }
		  	  } break;

		  	  case OP_STATE_MAN_MODE:	// MANUAL Mode
		  	  {
		  		  sm.act_motor_state = MOT_STATE_MANUAL_CONTROL;
	  			  if(sm.operation_stop)			// Durdur butonuna basılmışsa...
	  			  {
	  				  sm.operation_stop = 0u;	// Durdur işlemini resetle.
	  				  sm.act_motor_state = MOT_STATE_IDLE;
	  				  operation_state = OP_STATE_IDLE;
	  			  }
		  	  } break;

		  }

		  //HAL_GPIO_WritePin(DO_Led_GPIO_Port, DO_Led_Pin, 0);

	  }
	  // *** END: State Loop

	  // *** BEGIN: Led Blink Loop
	  if ((now_ms - led_loop_ms) >= LED_LOOP_PERIOD_MS)
	  {
		  led_loop_ms += LED_LOOP_PERIOD_MS;
		  //HAL_GPIO_TogglePin(DO_Led_GPIO_Port, DO_Led_Pin);
	  }
	  // *** END: Led Blink Loop

	  // *** BEGIN: Scan IO Loop
	  if ((now_ms - io_loop_ms) >= IO_LOOP_PERIOD_MS)
	  {
		  io_loop_ms += IO_LOOP_PERIOD_MS;
		  //ScanIO_Task();
		  sm.pedal_state = HAL_GPIO_ReadPin(DI_Pedal_GPIO_Port, DI_Pedal_Pin);
		  //sm.pedal_active = HAL_GPIO_ReadPin(DI_Pedal_Enable_GPIO_Port, DI_Pedal_Enable_Pin);

	  }
	  // *** END: Scan IO Loop

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_Led_GPIO_Port, DO_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DO_E2prom_WC_Pin|DO_E2prom_CE_Pin|DO_Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO_Motor_Direction_Pin|DO_Motor_Brake_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DO_Led_Pin */
  GPIO_InitStruct.Pin = DO_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_Pedal_Pin DI_Pedal_Enable_Pin */
  GPIO_InitStruct.Pin = DI_Pedal_Pin|DI_Pedal_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_E2prom_WC_Pin DO_E2prom_CE_Pin DO_Buzzer_Pin */
  GPIO_InitStruct.Pin = DO_E2prom_WC_Pin|DO_E2prom_CE_Pin|DO_Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_HMI_Tus2_Pin DI_HMI_Tus1_Pin DI_HMI_Tus8_Pin DI_HMI_Tus6_Pin
                           DI_HMI_Tus5_Pin DI_HMI_Tus4_Pin */
  GPIO_InitStruct.Pin = DI_HMI_Tus2_Pin|DI_HMI_Tus1_Pin|DI_HMI_Tus8_Pin|DI_HMI_Tus6_Pin
                          |DI_HMI_Tus5_Pin|DI_HMI_Tus4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_HMI_Tus3_Pin DI_HMI_Tus7_Pin */
  GPIO_InitStruct.Pin = DI_HMI_Tus3_Pin|DI_HMI_Tus7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_Motor_Direction_Pin DO_Motor_Brake_Pin */
  GPIO_InitStruct.Pin = DO_Motor_Direction_Pin|DO_Motor_Brake_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C_SCL_E2prom_Pin I2C_SDA_E2prom_Pin */
  GPIO_InitStruct.Pin = I2C_SCL_E2prom_Pin|I2C_SDA_E2prom_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_I2C1_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // PA0, PA1, PA2 -> ADC1_IN0, IN1, IN2
  GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  *
  * Disables interrupts and loops infinitely. Used as a hard fault or HAL
  * error fallback.
  *
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
