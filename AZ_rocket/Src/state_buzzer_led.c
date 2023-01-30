/*
 * state_buzzer_led.c
 *
 *  Created on: Apr 30, 2022
 *      Author: burak
 */

#ifndef SRC_STATE_BUZZER_LED_C_
#define SRC_STATE_BUZZER_LED_C_

#include "state_buzzer_led.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

uint16_t timer_val;
uint16_t timer_val_2;


//--------------durum ledleri -----------------//
//1-)bmp280 hatası  ---->led_red yanar  OKE
//2-)gps hatası  ----> led_blue yanıp söner 	OKE
//3-)pil az hatası ----> led_blue ve led_red yanıp söner	OKE
//4-)eeprom yazma hatası ----> led_blue yanar	OKE
//red_led ve led_blue
//--------------durum ledleri -----------------//

void state_buzzer_led_init() {  //int main() içerisinde çalıştır

	HAL_TIM_Base_Start(&htim1); //start timer     BUZZER İÇİN TİMER
	timer_val = __HAL_TIM_GET_COUNTER(&htim1); //get current timer

	HAL_TIM_Base_Start(&htim2); //start timer	   	LEDLER İÇİN TİMER
	timer_val_2 = __HAL_TIM_GET_COUNTER(&htim2); //get current timer

}

void buzzer_sound_ready_to_flight() {
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 1000) { //her 1 saniyede bir girer
		HAL_GPIO_TogglePin(GPIOB, buzzer_Pin);
		timer_val = __HAL_TIM_GET_COUNTER(&htim1);
	}
}

void buzzer_sound_battery_error() {
	HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
}

void buzzer_sound_bmp_error() {
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 15000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 30000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 32000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 37000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
		timer_val = __HAL_TIM_GET_COUNTER(&htim1);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 39000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 60000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
		timer_val = __HAL_TIM_GET_COUNTER(&htim1);
	}
}

void buzzer_sound_eeprom_error() {
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 15000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 30000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 35000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 40000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 45000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 50000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
		timer_val = __HAL_TIM_GET_COUNTER(&htim1);
	}
}

void buzzer_sound_mission_complate() {
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 15000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 20000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 25000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 40000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 45000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 50000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 52000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 57000) {
		HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);
		timer_val = __HAL_TIM_GET_COUNTER(&htim1);
	}

}



void led_red_bmp_error() {
	HAL_GPIO_WritePin(GPIOB, led_red_Pin, GPIO_PIN_SET);
}



void led_blue_gps_error(){
	if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val_2>= 15000) {
		HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val_2 >= 30000) {
		HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_RESET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val_2>= 45000) {
		HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val_2 >= 60000) {
		HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_RESET);
		timer_val_2 = __HAL_TIM_GET_COUNTER(&htim2);
	}
}

void led_blue_red_low_battery(){
	if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val_2>= 15000) {
		HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, led_red_Pin, GPIO_PIN_SET);
	}
	if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val_2 >= 30000) {
		HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, led_red_Pin, GPIO_PIN_RESET);

	}
	if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val_2>= 45000) {
		HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, led_red_Pin, GPIO_PIN_SET);

	}
	if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val_2 >= 60000) {
		HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, led_red_Pin, GPIO_PIN_RESET);
		timer_val_2 = __HAL_TIM_GET_COUNTER(&htim2);
	}
}

void led_blue_eeprom_problem(){
	HAL_GPIO_WritePin(GPIOB, led_blue_Pin, GPIO_PIN_SET);
}

#endif /* SRC_STATE_BUZZER_LED_C_ */
