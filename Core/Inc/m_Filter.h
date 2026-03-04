/*
 * m_Filter.h
 *
 *  Created on: Dec 7, 2025
 *      Author: Baris
 */

#ifndef INC_M_FILTER_H_
#define INC_M_FILTER_H_

#include <stdint.h>
#include <math.h>

int ScaleValues(uint16_t value_in, int limit, int step);
float LowPassFilter(float old_value, float new_value, float k_filter);
float AdaptiveLowPassFilter(float old_value, float new_value, float k_filter);
float AlphaBetaFilter(float sensor_value, float dt);
float UpdateMedianFilter(float new_value, uint8_t window_size);
float MedianFilter(float *input_buffer, uint8_t window_size);
float MedianFilter3(float buf[], float new_data);
float MedianFilter5(float buf[], float new_data);

#endif /* INC_M_FILTER_H_ */
