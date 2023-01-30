#pragma once
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
// Initialize the LPS25HB sensor
// Wake up and automatic measurements
// return - HAL_OK or HAL_ERROR

uint8_t lps25hb_init(void);

// Read temperature
// return - result in degrees C

float lps25hb_read_temp(void);

// Read the pressure
// reuturn - result in hPa
float lps25hb_read_pressure(void);
