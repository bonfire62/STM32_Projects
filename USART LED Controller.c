/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
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
#include "stm32f0xx_hal.h"
#include "string.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void Char_Transmit(char);
void Char_Receive(void);
void input_test(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
				int redflag = 0;
				int blueflag = 0;
				int orangeflag = 0;
				int greenflag = 0;
				
				int zero_flag = 0;
				int one_flag = 0;
				int two_flag = 0;
				char s[] = "";
				char t[] = "";
				int stringcomplete = 0;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	// enables the clock for the GPIO A set (usart 3)
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_USART3_CLK_ENABLE();
	
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

	GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);	
HAL_GPIO_Init(GPIOC, &initStr);
	
  /* USER CODE BEGIN 2 */
	// Sets the GPIOC pins to the alternate function 
GPIOC->MODER |= GPIO_MODER_MODER4_1;
GPIOC->MODER |= GPIO_MODER_MODER5_1;
	
GPIOC->AFR[0] |= (1 << 16);
GPIOC->AFR[0] |= (1 << 20);

	// Set the Baud rate
	int hclk = HAL_RCC_GetHCLKFreq();
	int baud = hclk / 115200;
	//enable baud rate
	USART3->BRR |= baud;
	USART3->CR1 |= 0xC;
	USART3->CR1 |= 0x1;
	
	//enabling the register not empty interrupt
	//USART3->CR1 |= USART_CR1_RXNEIE;
	//NVIC_EnableIRQ(USART3_4_IRQn);
	//NVIC_SetPriority(USART3_4_IRQn, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		char test[] = "test of the usart system ";
//		int i = 0;
//		while(test[i] != '\0')
//	{
//		Char_Transmit(test[i]);
//		i++;
//	}
		char input;
		Char_Receive();
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
  }
  /* USER CODE END 3 */

}

void Char_Transmit(char a)
{
	while(!(USART3->ISR & USART_ISR_TXE))
	{}
	USART3->TDR = a;
}


//char Char_Recieve()
//{
//	static int i;
//	char letter;
//	while(!(USART3->ISR & USART_ISR_RXNE_Msk))
//			{}
//			letter = (char)USART3->RDR;
//			return letter;
//}
void Char_Receive(void){
		while(!(USART3->ISR & USART_ISR_RXNE_Msk));
		static int str_cnt;
		
		char letter = (char)USART3->RDR;
	
	if(letter == '\r')
	{
		s[str_cnt] = '\0';
		input_test();	
		str_cnt = 0;
	}
	else{
		s[str_cnt] = letter;
		str_cnt++;
			//Char_Transmit(letter);
	}	
}


void input_test(void)
{

	// checks for the "no flag set" string case, e.g. red \r
	if(redflag == 0 && blueflag == 0 && greenflag == 0 && orangeflag == 0)
	{
		if(strcmp(s, "red") == 0)
		{
			redflag = 1;
			blueflag = greenflag = orangeflag = 0;
			strcpy(s, "");
			Char_Transmit('r');
		}
		else if(strcmp(s, "blue") == 0)
		{
			blueflag = 1;
			redflag = greenflag = orangeflag = 0;
			strcpy(s, "");
			Char_Transmit('b');
		}
		else if(strcmp(s, "green") == 0)
		{
			greenflag = 1;
			blueflag = redflag = orangeflag = 0;
			strcpy(s, "");
			Char_Transmit('g');
		}
		else if(strcmp(s, "orange") == 0)
		{
			orangeflag = 1;
			blueflag = redflag = greenflag = 0;
			strcpy(s, "");
			Char_Transmit('o');
		}
	}
	else if((strcmp(s, "on") == 0))
	{
		if(redflag)
		{
		GPIOC->ODR |= GPIO_ODR_6;
		GPIOC->ODR &= ~GPIO_ODR_7;
		GPIOC->ODR &= ~GPIO_ODR_8;
		GPIOC->ODR &= ~GPIO_ODR_9;
		strcpy(s, "");
		}
		else if(blueflag)
		{
		GPIOC->ODR &= ~GPIO_ODR_6;
		GPIOC->ODR |= GPIO_ODR_7;
		GPIOC->ODR &= ~GPIO_ODR_8;
		GPIOC->ODR &= ~GPIO_ODR_9;
			strcpy(s, "");
		}
		else if(greenflag)
		{
		GPIOC->ODR &= ~GPIO_ODR_6;
		GPIOC->ODR &= ~GPIO_ODR_7;
		GPIOC->ODR |= GPIO_ODR_8;
		GPIOC->ODR &= ~GPIO_ODR_9;
			strcpy(s, "");
		}
		else if(orangeflag)
		{
		GPIOC->ODR &= ~GPIO_ODR_6;
		GPIOC->ODR &= ~GPIO_ODR_7;
		GPIOC->ODR &= ~GPIO_ODR_8;
		GPIOC->ODR |= GPIO_ODR_9;
			strcpy(s, "");
		
		}
	}
};

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
