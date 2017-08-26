#ifdef  _ANALYSIS_H_
#define _ANALYSIS_H_

#include "FreeRTOS.h"

uint32_t heart_rate_analysis(uint16_t x, uint32_t time);
uint32_t breath_rate_analysis(uint16_t x, uint32_t time);
uint32_t turning_analysis(uint16_t x, uint32_t time);

void sleep_time_analysis(uint16_t x, uint32_t time_idx);

#endif