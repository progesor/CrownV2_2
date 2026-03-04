/*
 * m_Filter.c
 *
 *  Created on: Dec 7, 2025
 *      Author: Baris
 */

#include "m_Filter.h"
#include "AppConfig.h"

float future_value = 0.0f;
float rate_velocity = 0.0f;

int ScaleValues(uint16_t value_in, int limit, int step)
{
    // Güvenlik: ADC sınırla
    if(value_in > 4095) value_in = 4095;
    else if(value_in < 0) value_in = 0;

    // Limit ve step uyumlu olsun: limit % step == 0 olmalı
    int levels = (2 * limit) / step + 1;  // örn: 600,100 → 13 seviye

    // index = round( adc * (levels - 1) / 4095 )
    // round için +4095/2 ekliyoruz (integer aritmetiği)
    int index = (value_in * (levels - 1) + 4095 / 2) / 4095;

    // -limit den başlayarak step step ilerle
    int output = -limit + index * step;

    return output;
}

float LowPassFilter(float old_value, float new_value, float k_filter)
{
	float filtered_value = k_filter * new_value + (1 - k_filter) * old_value;
	return filtered_value;
}

float AdaptiveLowPassFilter(float old_value, float new_value, float k_filter)
{
    // The larger the difference, the smaller alpha (faster response)
	// Calculate the adaptive smoothing factor (alpha)
	float difference = fabs(new_value - old_value);
	float alpha = 0.0f;
	if(difference > K_A_LPF_DIFF_LIMIT_HIGH) alpha = K_A_LPF_ALPHA_HIGH;
	else if(difference > K_A_LPF_DIFF_LIMIT_MID) alpha = K_A_LPF_ALPHA_MID;
	else alpha =  K_A_LPF_ALPHA_LOW;

    return LowPassFilter(old_value, new_value, alpha);
}

float AlphaBetaFilter(float sensor_value, float dt)
{
    // 1. ADIM: TAHMİN
    // Önceki hız ve ivmeye bakarak "yeni hız şu olmalı" diyoruz.
    future_value = future_value + (rate_velocity * dt);

    // 2. ADIM: HATA HESABI
    // Sensörden gelen gerçek veri ile bizim tahminimiz arasındaki farkı buluyoruz.
    float error = sensor_value - future_value;

    // 3. ADIM: GÜNCELLEME (Düzeltme)
    // Bulduğumuz hata payını kullanarak tahminimzi ve değişim oranını iyileştiriyoruz.
    future_value = future_value + (K_AB_ALPHA * error);
    rate_velocity = rate_velocity + (K_AB_BETA / dt) * error;

    return future_value;
}

// Sensörden her yeni veri geldiğinde bu fonksiyonu çağırın
float UpdateMedianFilter(float new_value, uint8_t window_size)
{
	static float sensor_buffer[K_FILTER_MEDIAN_WS];
	static uint8_t buffer_index = 0;
	static uint8_t buffer_filled = 0; // Buffer tamamen dolana kadar hatalı sonuç almamak için

	// 1. Yeni veriyi dairesel tampona ekle
    sensor_buffer[buffer_index] = new_value;
    buffer_index = (buffer_index + 1) % window_size;

    // Buffer doluluğunu takip et (isteğe bağlı)
    if (!buffer_filled && buffer_index == 0) buffer_filled = 1;

    // 2. Orijinal veriyi bozmamak için kopyasını oluştur
    float temp_buffer[window_size];
    for(uint8_t i = 0; i < window_size; i++)
    {
    	temp_buffer[i] = sensor_buffer[i];
    }

    // 3. Medyanı hesapla ve döndür
    // Not: Buffer tam dolmadan hesaplama yapmak istersen n parametresini ona göre ayarlamalısın
    return MedianFilter(temp_buffer, window_size);
}


float MedianFilter(float *input_buffer, uint8_t window_size)
{
	// 1. Bubble Sort (Küçük n için optimize)
	for (uint8_t i = 0; i < window_size - 1; i++)
	{
		for (uint8_t j = 0; j < window_size - 1 - i; j++)
		{
			if (input_buffer[j] > input_buffer[j + 1])
			{
				float swap = input_buffer[j];
				input_buffer[j] = input_buffer[j + 1];
				input_buffer[j + 1] = swap;
			}
		}
	}

	// 2. Medyanı döndür
	return input_buffer[window_size / 2]; // n hep tek (3, 5, 7) ise doğrudan orta eleman
}

float MedianFilter3(float buf[], float new_data)
{
    buf[0] = buf[1];
    buf[1] = buf[2];
    buf[2] = new_data;

    // 3 elemanı sıralayıp ortadakini döndürür
    float a = buf[0], b = buf[1], c = buf[2];
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

float MedianFilter5(float buf[], float new_data)
{
    // 1. Verileri kaydır (Hafızayı güncelle)
    buf[0] = buf[1];
    buf[1] = buf[2];
    buf[2] = buf[3];
    buf[3] = buf[4];
    buf[4] = new_data;

    // 2. Sıralama yapmak için geçici bir diziye kopyala
    // (Orijinal buf dizisinin sırasını bozmamalıyız çünkü o zaman kaydırma yanlış olur)
    float temp[5];
    for(int i = 0; i < 5; i++) {
        temp[i] = buf[i];
    }

    // 3. Basit Sıralama (Bubble Sort) - 5 eleman için çok hızlıdır
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4 - i; j++) {
            if (temp[j] > temp[j + 1]) {
                float t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }

    // 4. Sıralanmış dizinin tam ortasındaki (3. eleman, indeks 2) değeri döndür
    return temp[2];
}
