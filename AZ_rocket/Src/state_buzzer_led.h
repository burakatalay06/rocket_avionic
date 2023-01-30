/*
 * state_buzzer_led.h
 *
 *  Created on: Apr 30, 2022
 *      Author: burak
 */

#ifndef SRC_STATE_BUZZER_LED_H_
#define SRC_STATE_BUZZER_LED_H_

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


#define buzzer_Pin GPIO_PIN_1
#define buzzer_GPIO_Port GPIOB
#define led_blue_Pin GPIO_PIN_13
#define led_blue_GPIO_Port GPIOB
#define led_red_Pin GPIO_PIN_15
#define led_red_GPIO_Port GPIOB




void state_buzzer_led_init();

void buzzer_sound_ready_to_flight();

void buzzer_sound_battery_error();

void buzzer_sound_bmp_error();

void buzzer_sound_eeprom_error();

void buzzer_sound_mission_complate();

void led_blue_gps_error();

void led_red_bmp_error();

void led_blue_red_low_battery();

void led_blue_eeprom_problem();

#endif /* SRC_STATE_BUZZER_LED_H_ */
