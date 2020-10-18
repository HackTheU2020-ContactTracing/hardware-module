/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 2020 HackTheU Contact Tracing
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

// Method declarations
void initializeUART3(void);
void transmit_bt_char(char c);
void transmit_bt(char* chars);
void store_data(char* chars);
void transmit_lora_char(char c);
void transmit_lora(char* chars);
void initializeUART2(void);
void SystemClock_Config(void);

// Global variables
int bytes_received = 0;
int upload_mode = 0;
int mac_counter = 0;
char mac_addresses[256];

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  HAL_Init();

	initializeUART3();
	initializeUART2();

  SystemClock_Config();
	

	transmit_bt("AT+INIT\r\n"); // Initialize the SPP profile lib
	HAL_Delay(1000); // Wait 1 second
	transmit_bt("AT+INQ?\r\n"); // Inquire bluetooth device
	GPIOC->ODR |= 1 << 6;
		
  while (1)
  {
  }

}

void USART2_IRQHandler(void) {
	char characters[256];
	uint32_t character_iter = 0;
	while((USART2->ISR >> 5) & 1)
	{
		characters[character_iter] = USART2->RDR;
		character_iter++;
	}
	characters[character_iter] = 0;
	transmit_bt(characters);
	store_data(characters);
}

void USART3_4_IRQHandler(void) {
	char characters[256];
	uint32_t character_iter = 0;
	while((USART3->ISR >> 5) & 1)
	{
		characters[character_iter] = USART3->RDR;
		character_iter++;
		if((USART3->ISR >> 3) & 1)
			transmit_bt("overrun occured");
	}
	characters[character_iter] = 0;
	transmit_lora(characters); 
	
}

void initializeUART3(void) {
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // enable system clock in RCC peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= ((1<<23) | (1<<21));
	
	USART3->BRR = HAL_RCC_GetHCLKFreq()/38400; // set Baud rate to be 38400 bits/second
	
	USART3->CR1 |= ((1<<3) | (1<<2)); // enable tx/rx hardware
	USART3->CR1 |= (1<<5); // enable rx reg not empty interrupt
	USART3->CR1 |= 1;			 // enable USART3
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 2);

  // set pins pc10 and pc11 into alternate function mode for UART3_TX/RX
	GPIOC->AFR[1] |= (1<<8); 
	GPIOC->AFR[1] |= (1<<12);
}

void initializeUART2(void) {
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable system clock in RCC peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  GPIOA->MODER |= 0xa0000000; //set PA14 & PA15 to be alternate function mode. 
	
	GPIOA->AFR[1] &= ~(0xFF000000); // make sure all other bits are zeroes
	GPIOA->AFR[1] |= (1 << 24) | (1 << 28); //set PA14 & PA15 to use AF1
		
	USART2->BRR = HAL_RCC_GetHCLKFreq()/115200; //set the baud rate to 115200
	
	USART2->CR1 |= (1 << 3) | (1 << 2) | (1 << 5); //enable the transmit and receive
	
	USART2->CR2 |= (1<<12);

  NVIC_EnableIRQ(USART2_IRQn); //enable the USART2 interrupt
  NVIC_SetPriority(USART2_IRQn,1); //set the USART2 priority
	
	USART2->CR1 |= 1; //enable the USART2 peripheral
}

void transmit_bt_char(char c)
{
	while(!(USART3->ISR & (1 << 7))) //check for the transmission to be clear for the next byte
	{
	}
	USART3->TDR = c; //set the character to be transmitted
}

void transmit_bt(char* chars)
{
	//loop through all the characters checking for null and an invalid pointer
	uint32_t counter = 0;
	while(chars != 0 && chars[counter] != 0)
	{
		transmit_bt_char(chars[counter]);
		counter++;
	}
}

void transmit_lora_char(char c)
{
	while(!(USART2->ISR & (1 << 7))) //check for the transmission to be clear for the next byte
	{
	}
	USART2->TDR = c; //set the character to be transmitted
}

void transmit_lora(char* chars)
{
	//loop through all the characters checking for null and an invalid pointer
	uint32_t counter = 0;
	while(chars != 0 && chars[counter] != 0)
	{
		transmit_lora_char(chars[counter]);
		counter++;
	}
}

void store_data(char* chars) {
	int rssi_value;
	int distance;
	int transmit_power = 4;
	
	// Check that chars is an INQ response
	if (chars[0] == '+' && chars[1] == 'I' && chars[2] == 'N' && chars[3] == 'Q') {
		// Convert RSSI hex string to an int
		for (int i = strlen(chars) - 4; i < 4; i++) {
			if (chars[i] >= '0' && chars[i] <= '9')
					rssi_value = rssi_value + chars[i] - '0';
			if (chars[i] >= 'A' && chars[i] <= 'F')
					rssi_value = rssi_value + chars[i] - 'A' + 10;
		}
		// Convert RSSI to distance
		distance = 10 ^ ((transmit_power - rssi_value)/(20));
		// Check that the Signal strength is strong enough
		if (distance < 2) {
			// Save the MAC address
			mac_addresses[mac_counter++] = chars[5];
			mac_addresses[mac_counter++] = chars[6];
			mac_addresses[mac_counter++] = chars[7];
			mac_addresses[mac_counter++] = chars[8];
			mac_addresses[mac_counter++] = chars[10]; // Skip over colon in INQ
			mac_addresses[mac_counter++] = chars[11];
			mac_addresses[mac_counter++] = chars[13]; // Skip over colon in INQ
			mac_addresses[mac_counter++] = chars[14];
			mac_addresses[mac_counter++] = chars[15];
			mac_addresses[mac_counter++] = chars[16];
			mac_addresses[mac_counter++] = chars[17];
			mac_addresses[mac_counter++] = chars[18];
		}
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
