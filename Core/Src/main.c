/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
uint32_t volatile mymillis = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void my_delay_ms(uint32_t ms)
{
  // read from TIM2 counter and increment a variable
  // clear the status register and repeat until the millis_val

  int counter = 0;
  while (counter < ms)
  {
    if ((TIM2->SR >> 0) & 1) // check if 1ms has passed
    {
      counter++;
      TIM2->SR &= ~(1 << 0); // reset the update interrupt flag
    }
  }
}

void TIM2_IRQHandler(void)
{
  mymillis++;
  TIM2->SR &= ~(1 << 0);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  // Enable PortA clock (for led)
  RCC->IOPENR |= (1 << 0);
  // Enable PortC clock (for button)
  RCC->IOPENR |= (1 << 2);

  // Set PA5 as output (LED)
  GPIOA->MODER |= (1 << 10);
  GPIOA->MODER &= ~(1 << 11);

  // Set PC13 as input (button)
  GPIOC->MODER &= ~(1 << 26);
  GPIOC->MODER &= ~(1 << 27);

  // hoping this would make the button be a pull down and led would be off by default, instead I have to modify the IF statement in the while loop
  GPIOC->PUPDR &= ~(1 << 26);
  GPIOC->PUPDR &= ~(1 << 27);
  GPIOC->PUPDR |= (1 << 27);

  // TIM2 setup
  RCC->APBENR1 |= (1 << 0); // enable the clock for the timer
  //RCC->APBENR1 |= (1 << 30);
  TIM2->CR1 &= ~(1 << 4); // set TIM2 as count up
  TIM2->PSC = 15;
  TIM2->ARR = 999;
  TIM2->DIER |= (1<<0);
  TIM2->CR1 |= (1 << 0); // enable the timer

  // interrupt setup
  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_EnableIRQ(TIM2_IRQn);
  TIM2->SR &= ~(1<<0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* Questions for LAB:
      - how to check if a bit is 0 in a register (without using '!')
    */

//    if (!(GPIOC->IDR >> 13) & 1) // ! shouldnt be needed - checks if button is pushed
//    {
//      GPIOA->ODR ^= (1 << 5); // turn on the led
//      my_delay_ms(500);
//    }
//    else
//    {
//      GPIOA->ODR &= ~(1 << 5); // turn off LED if button not pushed
//    }

	  if (mymillis >= 500){
		  GPIOA->ODR ^= (1 << 5); // turn on the led
		  mymillis = 0;
	  }

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

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
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
