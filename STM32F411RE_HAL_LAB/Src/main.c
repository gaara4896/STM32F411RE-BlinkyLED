/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void TurnOnLed(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void TurnOffLed(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void ToggleLed(void){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

int UserButtonIsPressed(void){
	return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0;
}

void TurnOn(GPIO_TypeDef *GPIOx, uint32_t Pin){
	HAL_GPIO_WritePin(GPIOx, Pin, GPIO_PIN_RESET);
}

void TurnOff(GPIO_TypeDef *GPIOx, uint32_t Pin){
	HAL_GPIO_WritePin(GPIOx, Pin, GPIO_PIN_SET);
}

void TurnOnAll(void){
	TurnOn(GPIOC, GPIO_PIN_0);
	TurnOn(GPIOC, GPIO_PIN_1);
	TurnOn(GPIOC, GPIO_PIN_2);
	TurnOn(GPIOC, GPIO_PIN_3);
	TurnOn(GPIOA, GPIO_PIN_6);
	TurnOn(GPIOA, GPIO_PIN_7);
	TurnOn(GPIOA, GPIO_PIN_11);
	TurnOn(GPIOB, GPIO_PIN_12);
}

void TurnOffAll(void){
	TurnOff(GPIOC, GPIO_PIN_0);
	TurnOff(GPIOC, GPIO_PIN_1);
	TurnOff(GPIOC, GPIO_PIN_2);
	TurnOff(GPIOC, GPIO_PIN_3);
	TurnOff(GPIOA, GPIO_PIN_6);
	TurnOff(GPIOA, GPIO_PIN_7);
	TurnOff(GPIOA, GPIO_PIN_11);
	TurnOff(GPIOB, GPIO_PIN_12);
}

void TurnOffAndOn(GPIO_TypeDef *GPIOx, uint32_t Pin){
	TurnOffAll();
	TurnOn(GPIOx, Pin);
}

void InitOdPdPin(GPIO_TypeDef *GPIOx, uint32_t Pin){
	GPIO_InitTypeDef GPIO_InitStruct_ODPD;
	GPIO_InitStruct_ODPD.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct_ODPD.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct_ODPD.Pin = Pin;
	GPIO_InitStruct_ODPD.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct_ODPD);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  int delay = 5;
  int counter = 1;
  int pressed = 0;

  GPIO_InitTypeDef GPIO_InitStruct_PP;
  GPIO_InitStruct_PP.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_PP.Pin = GPIO_PIN_5;
  GPIO_InitStruct_PP.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_PP);

  GPIO_InitStruct_PP.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_PP);

  GPIO_InitTypeDef GPIO_InitStruct_Button;
  GPIO_InitStruct_Button.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct_Button.Pin = GPIO_PIN_13;
  GPIO_InitStruct_Button.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_Button);

  InitOdPdPin(GPIOC, GPIO_PIN_0);
  InitOdPdPin(GPIOC, GPIO_PIN_1);
  InitOdPdPin(GPIOC, GPIO_PIN_2);
  InitOdPdPin(GPIOC, GPIO_PIN_3);
  InitOdPdPin(GPIOA, GPIO_PIN_6);
  InitOdPdPin(GPIOA, GPIO_PIN_7);
  InitOdPdPin(GPIOA, GPIO_PIN_11);
  InitOdPdPin(GPIOB, GPIO_PIN_12);

  TurnOffAll();
  //TurnOnLed();
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if(UserButtonIsPressed()){
		  if(!pressed){
			  delay = 5 * ((counter % 5) + 1);
			  counter += 1;
			  pressed = 1;
		  }
	  }else{
		  if(pressed){
			  pressed = 0;
		  }
	  }
	  TurnOffAndOn(GPIOA, GPIO_PIN_6);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOA, GPIO_PIN_7);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOA, GPIO_PIN_11);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOB, GPIO_PIN_12);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOC, GPIO_PIN_0);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOC, GPIO_PIN_1);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOC, GPIO_PIN_2);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOC, GPIO_PIN_3);
	  HAL_Delay(delay);

	  TurnOffAndOn(GPIOC, GPIO_PIN_2);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOC, GPIO_PIN_1);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOC, GPIO_PIN_0);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOB, GPIO_PIN_12);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOA, GPIO_PIN_11);
	  HAL_Delay(delay);
	  TurnOffAndOn(GPIOA, GPIO_PIN_7);
	  HAL_Delay(delay);

	  TurnOnLed();
	  HAL_Delay(delay);
	  TurnOffLed();

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
