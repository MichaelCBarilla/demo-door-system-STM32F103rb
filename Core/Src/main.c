/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include "retarget.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
struct Pin_Layout {
	 GPIO_TypeDef *GPIOx;
   unsigned short int  GPIO_Pin;
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Door 1 Pin Layouts. */
const struct Pin_Layout DOOR_1_RED_LED = {GPIOC, GPIO_PIN_12};
const struct Pin_Layout DOOR_1_GREEN_LED = {GPIOC, GPIO_PIN_11};
const struct Pin_Layout DOOR_1_OUTSIDE_SWITCH = {GPIOC, GPIO_PIN_10};
const struct Pin_Layout DOOR_1_INSIDE_SWITCH = {GPIOD, GPIO_PIN_2};

/* Door 2 Pin Layouts. */
const struct Pin_Layout DOOR_2_RED_LED = {GPIOA, GPIO_PIN_9};
const struct Pin_Layout DOOR_2_GREEN_LED = {GPIOC, GPIO_PIN_0};
const struct Pin_Layout DOOR_2_OUTSIDE_SWITCH = {GPIOC, GPIO_PIN_3};
const struct Pin_Layout DOOR_2_INSIDE_SWITCH = {GPIOC, GPIO_PIN_1};

/* Misc. Switches */
const struct Pin_Layout PUSH_TO_LOCK = {GPIOA, GPIO_PIN_12};
const struct Pin_Layout EMERGENCY = {GPIOC, GPIO_PIN_8};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void is_pushed_to_lock_pressed(bool *door_1_unlocked, bool *door_2_unlocked, bool *waiting_on_ptl_release_ptr, bool *waiting_on_door_close_ptr);
void is_emergency_pressed(bool *door_1_unlocked, bool *door_2_unlocked, bool *waiting_on_emergency_release_ptr);
void update_door_1_leds(bool *door_1_unlocked_ptr);
void check_door_1_status(bool *door_1_unlocked_ptr, bool *door_2_unlocked_ptr, bool *waiting_on_door_close_ptr);
void update_door_2_leds(bool *door_2_unlocked_ptr);
void check_door_2_status(bool *door_1_unlocked_ptr, bool *door_2_unlocked_ptr, bool *waiting_on_door_close_ptr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	RetargetInit(&huart2);
	bool door_1_unlocked = true;
	bool door_2_unlocked = true;
	bool waiting_on_ptl_release = false;
	bool waiting_on_emergency_release = false;
	bool waiting_on_door_close = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	is_pushed_to_lock_pressed(&door_1_unlocked, &door_2_unlocked, &waiting_on_ptl_release, &waiting_on_door_close);
  	is_emergency_pressed(&door_1_unlocked, &door_2_unlocked, &waiting_on_emergency_release);
  	check_door_1_status(&door_1_unlocked, &door_2_unlocked, &waiting_on_door_close);
		update_door_1_leds(&door_1_unlocked);
		check_door_2_status(&door_1_unlocked, &door_2_unlocked, &waiting_on_door_close);
		update_door_2_leds(&door_2_unlocked);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA6 PA7
                           PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void is_pushed_to_lock_pressed(bool *door_1_unlocked_ptr, bool *door_2_unlocked_ptr, bool *waiting_on_ptl_release_ptr, bool *waiting_on_door_close_ptr)
{
	bool pressed_down = HAL_GPIO_ReadPin(PUSH_TO_LOCK.GPIOx, PUSH_TO_LOCK.GPIO_Pin) == 0;

	if (!pressed_down && *waiting_on_ptl_release_ptr && !*waiting_on_door_close_ptr) {
		*door_1_unlocked_ptr = false;
		*door_2_unlocked_ptr = false;
		*waiting_on_ptl_release_ptr = false;
	} else if (pressed_down) {
		*waiting_on_ptl_release_ptr = true;
	}
}

void is_emergency_pressed(bool *door_1_unlocked_ptr, bool *door_2_unlocked_ptr, bool *waiting_on_emergency_release_ptr)
{
	bool pressed_down = HAL_GPIO_ReadPin(EMERGENCY.GPIOx, EMERGENCY.GPIO_Pin) == 0;

	if (!pressed_down && *waiting_on_emergency_release_ptr) {
		*door_1_unlocked_ptr = true;
		*door_2_unlocked_ptr = true;
		*waiting_on_emergency_release_ptr = false;
	} else if (pressed_down) {
		*waiting_on_emergency_release_ptr = true;
	}
}

void check_door_1_status(bool *door_1_unlocked_ptr, bool *door_2_unlocked_ptr, bool *waiting_on_door_close_ptr)
{
	bool opening_door_1_from_inside = HAL_GPIO_ReadPin(DOOR_1_INSIDE_SWITCH.GPIOx, DOOR_1_INSIDE_SWITCH.GPIO_Pin) == 0;

	/* Check if opening door 1 from inside */
	if (opening_door_1_from_inside) {

	}

	/* Check if door 1 is unlocked. */
	if (*door_1_unlocked_ptr) {

		bool door_1_opened = HAL_GPIO_ReadPin(DOOR_1_OUTSIDE_SWITCH.GPIOx, DOOR_1_OUTSIDE_SWITCH.GPIO_Pin) == 0;

		/* Set boolean to tell push to lock to wait for door to close before locking */
		*waiting_on_door_close_ptr = door_1_opened;

		/* Check if door 1 is opened. If so, lock door 2. */
		if (door_1_opened) {
			*door_2_unlocked_ptr = false;
		} else {
			*door_2_unlocked_ptr = true;
		}
	}

	return;
}

void update_door_1_leds(bool *door_1_unlocked_ptr)
{
	if (*door_1_unlocked_ptr) {
		HAL_GPIO_WritePin(DOOR_1_RED_LED.GPIOx, DOOR_1_RED_LED.GPIO_Pin, 0);
		HAL_GPIO_WritePin(DOOR_1_GREEN_LED.GPIOx, DOOR_1_GREEN_LED.GPIO_Pin, 1);
	} else {
		HAL_GPIO_WritePin(DOOR_1_RED_LED.GPIOx, DOOR_1_RED_LED.GPIO_Pin, 1);
		HAL_GPIO_WritePin(DOOR_1_GREEN_LED.GPIOx, DOOR_1_GREEN_LED.GPIO_Pin, 0);
	}
	return;
}

void check_door_2_status(bool *door_1_unlocked_ptr, bool *door_2_unlocked_ptr, bool *waiting_on_door_close_ptr)
{
	/* Check if door 2 is unlocked. */
	if (*door_2_unlocked_ptr) {

		bool door_2_opened = HAL_GPIO_ReadPin(DOOR_2_OUTSIDE_SWITCH.GPIOx, DOOR_2_OUTSIDE_SWITCH.GPIO_Pin) == 0;

		/* Set boolean to tell push to lock to wait for door to close before locking */
		*waiting_on_door_close_ptr = door_2_opened;

		/* Check if door 2 is opened. If so, lock door 1. */
		if (door_2_opened) {
			*door_1_unlocked_ptr = false;
		} else {
			*door_1_unlocked_ptr = true;
		}
	}
	return;
}

void update_door_2_leds(bool *door_2_unlocked_ptr)
{
	if (*door_2_unlocked_ptr) {
		HAL_GPIO_WritePin(DOOR_2_RED_LED.GPIOx, DOOR_2_RED_LED.GPIO_Pin, 0);
		HAL_GPIO_WritePin(DOOR_2_GREEN_LED.GPIOx, DOOR_2_GREEN_LED.GPIO_Pin, 1);
	} else {
		HAL_GPIO_WritePin(DOOR_2_RED_LED.GPIOx, DOOR_2_RED_LED.GPIO_Pin, 1);
		HAL_GPIO_WritePin(DOOR_2_GREEN_LED.GPIOx, DOOR_2_GREEN_LED.GPIO_Pin, 0);
	}
	return;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
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
