/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bmp280.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "lwgps/lwgps.h"
#include "state_buzzer_led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//--------------GPS İÇİN -----------------//
lwgps_t gps;

uint8_t rx_buffer[128];
uint8_t rx_index = 0;
uint8_t rx_data = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);

		if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_data;
		} else {
			lwgps_process(&gps, rx_buffer, rx_index + 1);
			rx_index = 0;
			rx_data = 0;
		}
	}

}

//--------------GPS İÇİN -----------------//

BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;
float first_pressure, altitude, first_altitude;

uint16_t size;
uint8_t Data[256];
uint8_t Rx_data[150];

uint16_t battery_raw;
uint16_t battery_raw_median_1, battery_raw_median_2, battery_raw_median_3,
		battery_raw_median_4;
uint16_t battery_voltage;

uint16_t timer_value;
int deployments_launch_counter = 1;

uint8_t gps_saat, gps_dakika, gps_saniye;

//########################################################################
//------PAKETLİK STRİNG VERİLER-----//
char paket_ayrilma_durumu[2];
char paket_batarya[4];
char paket_altitude[10];
char paket_lng[10];
char paket_lat[10];
char paket_saat[10];

char paket[58]; // lora ile gönderilecek olan veri paketi

//########################################################################
//------STATE'LER-----//
//########################################################################

//------state management variables-----//
bool state_of_bmp280;
bool state_of_eeprom;
bool state_of_battery;
bool state_of_gps;
bool state_of_deployment;
//------state management variables-----//
//########################################################################
//------EEPROM VERİLER-----//
uint8_t eeprom_paket[62]; // lora ile gönderilecek olan veri paketi

int eeprom_paket_sayac = 1;
uint8_t eeprom_offset;

uint8_t dataRead[150]; //eepromdan veri okumak için
uint8_t dataWrite[100]; //eeproma veri yazmak için

uint8_t eeprom_read_for_transmit[62];
uint8_t eeprom_page = 1;
uint8_t eeprom_page_preview;
//########################################################################

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

//--------------------------EEEPROMUM 1. SAYFASINA SON YAZILAN VERİNİN SAYFA NUMARASI YAZILIYOR--------------------------//

	EEPROM_Read(1, 2, eeprom_page_preview, 5);

	if (eeprom_page_preview > 0) {
		eeprom_page = eeprom_page_preview;
	} else
		eeprom_page = 1;
//--------------------------EEEPROMUM 1. SAYFASINA SON YAZILAN VERİNİN SAYFA NUMARASI YAZILIYOR--------------------------//

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	HAL_Delay(500);

	state_buzzer_led_init(); //timerları başlatır

	timer_value = __HAL_TIM_GET_COUNTER(&htim1); //get current timer

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		state_of_bmp280 = false;
		size = sprintf((char*) Data, "BMP280 initialization failed\n");
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	size = sprintf((char*) Data, "BMP280: found %s\n",
			bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart1, Data, size, 1000);

	while (!bmp280_read_float(&bmp280, &temperature, &first_pressure, &humidity)) {
		size = sprintf((char*) Data, "Altitude reading failed\n");
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		HAL_Delay(2000);
	}

	bool apogee_detecting(float *first_altitude, float *second_altitude) {
		float fark;
		fark = *second_altitude - *first_altitude;
		if (*second_altitude >= 60.0) {
			if (fark <= -1.0) {
				return true;
			} else {
				return false;
			}
		} else {
			return false;
		}
	}

	bool battery_voltage_read_4_times() {

		if (__HAL_TIM_GET_COUNTER(&htim1) - timer_value >= 15000) {
			battery_raw_median_1 = battery_raw;
		}
		if (__HAL_TIM_GET_COUNTER(&htim1) - timer_value >= 16000) {
			battery_raw_median_2 = battery_raw;
		}
		if (__HAL_TIM_GET_COUNTER(&htim1) - timer_value >= 17000) {
			battery_raw_median_3 = battery_raw;
		}
		if (__HAL_TIM_GET_COUNTER(&htim1) - timer_value >= 18000) {
			battery_raw_median_4 = battery_raw;

			timer_value = __HAL_TIM_GET_COUNTER(&htim1);
			return true;
		}
		return false;
	}

	uint16_t median_filtersh(uint16_t data_1, uint16_t data_2, uint16_t data_3,
			uint16_t data_4) {
		uint16_t total[4], voltage, min, max, toplam;

		total[1] = data_1;
		total[2] = data_2;
		total[3] = data_3;
		total[4] = data_4;

		for (int j = 0; j < 4; j++) {
			for (int i = 0; i < 4; i++) {
				if (total[i] < total[i + 1]) {
					voltage = total[i];
					total[i] = total[i + 1];
					total[i + 1] = voltage;
				}
			}
		}
		min = total[1];
		max = total[4];
		toplam = min + max;
		toplam = toplam / 2;
		return toplam;
	}

	HAL_GPIO_WritePin(GPIOA, deployment_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, deployment_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, eeprom_erase_ping_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, eeprom_read_ping_Pin, GPIO_PIN_RESET);

	//--------------GPS İÇİN -----------------//
	lwgps_init(&gps);
	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	//--------------GPS İÇİN -----------------//

	void state_management_with_leds_and_buzzer() {
		//1-)bmp280 hatası
		//2-)gps hatası
		//3-)pil az hatası
		//4-)eeprom yazma hatası  BUNU YAP!!!!!1
		if (state_of_bmp280) {
			led_red_bmp_error();
			buzzer_sound_bmp_error();
		}
		if (state_of_battery) {
			led_blue_red_low_battery();
			buzzer_sound_battery_error();
		}
		if (state_of_gps) {
			led_blue_gps_error();
			buzzer_sound_eeprom_error();		//TODO:BURASI DE�?İ�?EBİLİR
		}

		if (state_of_deployment) {
			//TODO:BURAYA AYRILMA OLDUKTAN SONRA OLMASINI İSTEDİ�?İN �?EYLERİ YAZ!!!!!!
		}
	}

	//TODO:EEPROM MODLARINI AKTİVE ET

	while (HAL_GPIO_ReadPin(GPIOB, eeprom_read_Pin) != GPIO_PIN_RESET) { //PA11 İLE PA12 KISA DEVRE İKEN EEPROM VERİLERİNİ YAZMA MODUNA GİRER

		//BURADA EEPROM VERİLERİNİ TRANSMİT EDİLİYOR

		for (int i = 0; i < 4096; i++) {

			EEPROM_Read(i, 2, eeprom_read_for_transmit, 62);
			HAL_Delay(10);
			HAL_UART_Transmit(&huart1, eeprom_read_for_transmit,
					strlen(eeprom_read_for_transmit), 50);

			HAL_Delay(10);
			HAL_GPIO_TogglePin(GPIOB, led_blue_Pin);

		}

	}

	while (HAL_GPIO_ReadPin(GPIOA, eeprom_erase_Pin) != GPIO_PIN_RESET) { // KISA DEVRE İKEN EEPROM VERİLERİNİ SİLER MODUNA GİRER

		for (int i = 0; i < 4096; i++)  //tüm sayfaları silinir
				{
			EEPROM_PageErase(i);
			HAL_Delay(10);
			HAL_GPIO_TogglePin(GPIOB, led_red_Pin);

		}
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		buzzer_sound_mission_complate();

		gps_saat = gps.hours == 55 ? 0 : gps.hours;
		gps_dakika = gps.minutes == 87 ? 0 : gps.minutes;
		gps_saniye = gps.seconds == 247 ? 0 : gps.seconds;

		sprintf(paket_lat, "%f", gps.latitude);
		sprintf(paket_lng, "%f", gps.longitude);
		sprintf(paket_saat, "%d:%d:%d", gps_saat, gps_dakika, gps_saniye);

		state_of_gps = gps.fix_mode == '1' ? false : true; //gps'in bulduğu uydu sayısına göre gps durumunu günceller

		HAL_ADC_Start(&hadc1); //starting to ADC
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // 0V iken 0 3.3V iken 4095 değeri verir
		battery_raw = HAL_ADC_GetValue(&hadc1) - 3770; //gerilim bölücü değerlerini değiştir
		battery_raw = battery_raw / 4.8;
		if (battery_voltage_read_4_times()) {
			battery_voltage = median_filtersh(battery_raw_median_1,
					battery_raw_median_2, battery_raw_median_3,
					battery_raw_median_4);
		}

		state_of_battery = battery_voltage <= 30 ? false : true; //batarya durumuna göre batarya durumunu günceller

		//--------- LİPO MAX 8.4 min 7.4 (https://blog.ampow.com/lipo-voltage-chart/)

		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			size = sprintf((char*) Data,
					"Temperature/pressure reading failed\n");
			HAL_UART_Transmit(&huart1, Data, size, 1000);
			HAL_Delay(2000);
		}
		first_altitude = altitude;
		HAL_Delay(10);
		bmp280_read_altitude(&bmp280, &first_pressure, &pressure, &altitude);
		sprintf(paket_altitude, "%.2f", altitude);
		HAL_Delay(50);

		if (apogee_detecting(&first_altitude, &altitude)
				&& deployments_launch_counter < 20) {
			sprintf((char*) paket_ayrilma_durumu, "+");
			HAL_GPIO_WritePin(GPIOA, deployment_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, deployment_2_Pin, GPIO_PIN_SET);
			deployments_launch_counter++;
			state_of_deployment = true;

		} else {
			//sprintf((char*) paket_ayrilma_durumu, "-");
			state_of_deployment = false;
			HAL_GPIO_WritePin(GPIOA, deployment_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, deployment_2_Pin, GPIO_PIN_RESET);
		}

		sprintf(paket_batarya, "%d", battery_voltage);

		size = sprintf(paket, "%s,%s,%s,%s m,%s,%s\n", paket_saat, paket_lat,
				paket_lng, paket_altitude, paket_ayrilma_durumu, paket_batarya);

		HAL_Delay(10);
		HAL_UART_Transmit(&huart1, (uint8_t*) paket, size, HAL_MAX_DELAY);

		size = sprintf((uint8_t*) eeprom_paket, "%s/", paket);
		HAL_Delay(10);

		if (eeprom_paket_sayac == 1) { // bu blok eeproma veri yazdırılmadan ÖNCE çalıştırılacak
									   //offset  1. veride 2 ofsetli 2.veride 65 offsetli 3. veride 128 offsetli 4. veride 191
			eeprom_offset = 2;
		} else if (eeprom_paket_sayac == 2) {
			eeprom_offset = 65;

		} else if (eeprom_paket_sayac == 3) {
			eeprom_offset = 128;
		} else if (eeprom_paket_sayac == 4) {
			eeprom_offset = 191;
		}

		if (eeprom_page < 4095) {
			EEPROM_Write(eeprom_page, eeprom_offset, (uint8_t*) eeprom_paket,
					62);
			HAL_Delay(10); //page ve offset ile oyna
			EEPROM_Write(1, 2, (uint8_t*) eeprom_page, 4);
			HAL_Delay(10);
			if (eeprom_paket_sayac > 3) { //bu blok eeproma veri yazdıktan SONRA çalıştırılacak
				eeprom_page++;
				eeprom_paket_sayac = 1;
			} else {
				eeprom_paket_sayac++;
			}
			state_of_eeprom = true;
		} else
			state_of_eeprom = false;

		HAL_Delay(100);

		//TODO:EEPROM STATE AL

		//---------------------------------------EEPROM---------------------------------------//
//		EEPROM_Write(eeprom_page, 10, dataWrite, strlen(dataWrite));
//		HAL_Delay(10);
//		EEPROM_Write_NUM(1, 10, eeprom_page); // eeproma yazılan son verinin sayfa numarasını, eepromun ilk sayfasına yazar!!!
//
//		HAL_Delay(10);
//		EEPROM_Read(page, offset, data, size);
//
//		EEPROM_Read(4095, 0, dataRead, 150);
//		size = sprintf((char*) Data, "%s\n", dataRead);
//		HAL_UART_Transmit(&huart1, Data, size, 1000);
//
//		//HAL_UART_Transmit(&huart1, dataRead[], 1, 1000);
//		HAL_Delay(500);
		//---------------------------------------EEPROM---------------------------------------//

		state_management_with_leds_and_buzzer();

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 3599;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 3599;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			deployment_2_Pin | buzzer_Pin | led_blue_Pin | led_red_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(eeprom_erase_ping_GPIO_Port, eeprom_erase_ping_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(deployment_1_GPIO_Port, deployment_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(eeprom_read_ping_GPIO_Port, eeprom_read_ping_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin : deployment_2_Pin */
	GPIO_InitStruct.Pin = deployment_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(deployment_2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : buzzer_Pin led_blue_Pin led_red_Pin eeprom_read_ping_Pin */
	GPIO_InitStruct.Pin = buzzer_Pin | led_blue_Pin | led_red_Pin
			| eeprom_read_ping_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : eeprom_erase_ping_Pin */
	GPIO_InitStruct.Pin = eeprom_erase_ping_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(eeprom_erase_ping_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : eeprom_erase_Pin */
	GPIO_InitStruct.Pin = eeprom_erase_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(eeprom_erase_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : deployment_1_Pin */
	GPIO_InitStruct.Pin = deployment_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(deployment_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : eeprom_read_Pin */
	GPIO_InitStruct.Pin = eeprom_read_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(eeprom_read_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
