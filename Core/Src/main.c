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
uint16_t volatile ledMillis = 0; // used to check millis intervals for internal led blinking

uint16_t volatile adcValue = 0; // ADC value

uint16_t volatile potenInterval = 0; // millis blink interval based off potentiometer

uint16_t volatile extLedMillis = 0; // used to check millis interval since last blink for external led

int enableBlink = 1; // should the LEDs blink
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TIM2_IRQHandler(void) // interrupt called when a millisecond passes
{
  ledMillis++; // increases millisecond count for internal led blink interval
  extLedMillis++;
  TIM2->SR &= ~(1 << 0);
}

void EXTI4_15_IRQHandler(void) // interrupt for when button is pushed
{
  enableBlink = !enableBlink;

  // Reset the pending register
  EXTI->RPR1 = 1 << 13;
}

void ADC1_COMP_IRQHandler(void)
{
  adcValue = ADC1->DR;
  ADC1->ISR &= ~(1 << 2);
  ADC1->CR |= (1 << 2);
}

// https://github.com/arduino/ArduinoCore-avr/blob/eabd762a1edcf076877b7bec28b7f99099141473/cores/arduino/WMath.cpp#L52
int myMap(int x, int in_min, int in_max, int out_min, int out_max) // this function definition is taken from the Arduino source code
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

  // enable PortB clock (for ext led2)
  RCC->IOPENR |= (1 << 1);

  // Enable PortC clock (for button)
  RCC->IOPENR |= (1 << 2);

  // Set PA5 as output (LED)
  GPIOA->MODER |= (1 << 10);
  GPIOA->MODER &= ~(1 << 11);

  // Set PB0 as output (LED)
  GPIOB->MODER |= (1 << 0);
  GPIOB->MODER &= ~(1 << 1);

  // Set PC13 as input (button)
  GPIOC->MODER &= ~(1 << 26);
  GPIOC->MODER &= ~(1 << 27);

  // hoping this would make the button be a pull down and led would be off by default, instead I have to modify the IF statement in the while loop
  GPIOC->PUPDR &= ~(1 << 26);
  GPIOC->PUPDR &= ~(1 << 27);
  GPIOC->PUPDR |= (1 << 27);

  // TIM2 setup
  RCC->APBENR1 |= (1 << 0); // enable the clock for the timer
  // RCC->APBENR1 |= (1 << 30);
  TIM2->CR1 &= ~(1 << 4); // set TIM2 as count up
  TIM2->PSC = 15;         // set the prescaler value
  TIM2->ARR = 999;
  TIM2->DIER |= (1 << 0); // enable the TIM2 to create an interrupt event
  TIM2->CR1 |= (1 << 0);  // enable the timer

  // TIM2 clock interrupt setup
  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_EnableIRQ(TIM2_IRQn);
  TIM2->SR &= ~(1 << 0);

  // External Periphal (button) interrupt setup
  EXTI->RTSR1 |= (1 << 13);
  EXTI->EXTICR[3] |= (0x2 << 8); // Set PC-13 as GPIO pin for interrupt
  EXTI->IMR1 |= (1 << 13);
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);

  // set PA0 as input (potentiometer)
  // keeping the register mode at its default state (analog mode - reset state)
  GPIOA->MODER &= ~(1 << 0);
  GPIOA->MODER &= ~(1 << 1);

  RCC->APBENR2 |= (1 << 20); // enable ADC
  ADC1->CFGR1 &= ~(1 << 3); // set ADC to asynchronous clock mode
  ADC1->CFGR1 &= ~(1 << 4);

  ADC1->CFGR2 &= ~(1 << 30); // set ADC to asynchronous clock mode
  ADC1->CFGR2 &= ~(1 << 31);

  ADC->CCR &= ~(1 << 18);
  ADC->CCR &= ~(1 << 19);
  ADC->CCR &= ~(1 << 20);
  ADC->CCR &= ~(1 << 21);

  ADC1->CHSELR |= (1 << 0); // enable pin 0 for ADC

  ADC1->SMPR |= (1 << 4);   // set the clock cycle to 12.5
  ADC1->SMPR |= (1 << 5);
  ADC1->SMPR &= ~(1 << 6);

  ADC1->IER |= (1 << 2); // enable the ADC interrupt
  ADC1->CR |= (1 << 0);  // enable the ADC

//  ADC1->ISR |= (1 << 0);
  NVIC_SetPriority(ADC1_COMP_IRQn, 2);
  NVIC_EnableIRQ(ADC1_COMP_IRQn);
  ADC1->CR |= (1 << 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* Questions for LAB:
      - how to check if a bit is 0 in a register (without using '!')
    */

    if (ledMillis >= 500 && enableBlink)
    {
      GPIOA->ODR ^= (1 << 5); // toggle the led
      ledMillis = 0;
    }

    potenInterval = myMap(adcValue, 0, 4095, 100, 500); // get the interval for 2nd led controlled by the potentiometer

    if (extLedMillis >= potenInterval && enableBlink)
    {
      GPIOB->ODR ^= (1 << 0); // toggle the led
      extLedMillis = 0;
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
