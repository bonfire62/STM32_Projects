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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

void SystemClock_Config(void);
void Error_Handler(void);
void I2CWriteAdd(int address, int data, int bytes);
int16_t x,y;

int main(void)
{

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	//led enable
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//I2C pin enable	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
	//I2C clock enable
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; 
	
	//Initialize LED's
	GPIO_InitTypeDef ledInitStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
																	GPIO_MODE_OUTPUT_PP,
																	GPIO_SPEED_FREQ_LOW,
																	GPIO_NOPULL};
																	
	HAL_GPIO_Init(GPIOC, &ledInitStr);

	
	//Set PB11 and PB13 to alternate function mode, 
	//open-drain output type, and 
	//select I2C2_SDA as its alternate function (AF1) & (AF5) respectively
	//AF Mode
  GPIOB->MODER |= (1 << 23) | (1 << 27); 
	//Push pull
	GPIOB->PUPDR |= (1 << 22) | ( 1 << 26); 
	//Open drain output
	GPIOB->OTYPER |= (1 << 11) | (1 << 13);
// set alternate function parameters	
	GPIOB->AFR[1] |= (1 << 12) | (0x5 << 20);


	//Set PB14 and PC0 to output mode, push-pull output type, 
	//and initialize/set the pin high.
	GPIOB->MODER |= (1 << 28);
	GPIOC->MODER |= (1 << 0);
	GPIOC->BSRR = 1;
	GPIOB->BSRR = (1 << 14);

	//Set TIMINGR reg to use 100 kHZ standard mode
	I2C2->TIMINGR = (1 << 28) | 0x13 | (0xF << 8) | (0x2 << 16) | (0x4 << 20); 				
	
	//Enable I2C peripheral 
	I2C2->CR1 |= I2C_CR1_PE;//0x1;
	
	//Setting the transaction parameters
	I2CWriteAdd(0x0F, NULL, 1);
	
	//Reload the CR2 register
	I2C2->CR2 = (0x6B << 1) | (1 << 16) | I2C_CR2_START | I2C_CR2_RD_WRN; 
	
	//Wait RXNE (Receive Register Not Empty) or 
	//NACKF (Slave Not-Acknowledge) flags are set.
	while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
	
	// Continue if the RXNE flag is set.
	if(I2C2->ISR & I2C_ISR_NACKF){
		while(1){
			GPIOC->ODR ^= GPIO_ODR_8; //error
			HAL_Delay(1000);
		}
	}

	//Check the contents of the RXNE register to see if it matches the 
	//known value of the “WHO_AM_I” register.
	if(I2C2->RXDR == 0xD4) {
		//GPIOC->ODR |= GPIO_ODR_6; //WORKS
	}
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(!(I2C2->ISR & I2C_ISR_TC));
	
	//Set the STOP bit in the CR2 register to release the I2C bus.
	I2C2->CR2 |= (1 << 14);
	
	//Initializing the Gyroscope

	I2CWriteAdd(0x20, 0xB, 2);

	I2C2->CR2 |= (1 << 14);
	
	
	while(1){		
		//Initialize the sensor to read the X 
		//Set the transaction parameters in the CR2 register.
		I2CWriteAdd(0xA8, NULL, 1);
		
		//Reload the CR2 register
		I2C2->CR2 = (0x6B << 1) | (2 << 16) | I2C_CR2_START | I2C_CR2_RD_WRN; //Set the L3GD20 slave address = 0x6B
		
		//Wait until either of the RXNE (Receive Register Not Empty) or 
		//NACKF (Slave Not-Acknowledge) flags are set.
		while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
		
		// Continue if the RXNE flag is set.
		if(I2C2->ISR & I2C_ISR_NACKF){
			while(1){
				GPIOC->ODR ^= GPIO_ODR_9; //error
				HAL_Delay(1000);
			}
		}

		//Check the contents of the RXNE
		x = I2C2->RXDR;
		
		//Wait until either of the RXNE
		while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
		
		// Continue if the RXNE flag is set.
		if(I2C2->ISR & I2C_ISR_NACKF){
			while(1){
				GPIOC->ODR ^= GPIO_ODR_8; //error
				HAL_Delay(1000);
			}
		}
		
		//add high bits
		x |= (I2C2->RXDR << 8);
		
		//Wait until the TC (Transfer Complete) flag is set. //DIFFERENT HERE
		while(!(I2C2->ISR & I2C_ISR_TC));
		
		//Initialize the sensor to read the X 
		//Set the transaction parameters in the CR2 register.
		I2CWriteAdd(0xAA, NULL, 1);
		
		//Reload the CR2 register
		I2C2->CR2 = (0x6B << 1) | (2 << 16) | I2C_CR2_START | I2C_CR2_RD_WRN; //Set the L3GD20 slave address = 0x6B
		
		//Wait until either of the RXNE (Receive Register Not Empty) or 
		//NACKF (Slave Not-Acknowledge) flags are set.
		while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
		
		// Continue if the RXNE flag is set.
		if(I2C2->ISR & I2C_ISR_NACKF){
			while(1){
				GPIOC->ODR ^= GPIO_ODR_8; //error
				HAL_Delay(1000);
			}
		}

		//Check the contents of the RXNE
		y = I2C2->RXDR;
		
		//Wait until either of the RXNE (Receive Register Not Empty) or 
		//NACKF (Slave Not-Acknowledge) flags are set.
		while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
		
		// Continue if the RXNE flag is set.
		if(I2C2->ISR & I2C_ISR_NACKF){
			while(1){
				GPIOC->ODR ^= GPIO_ODR_8; //error
				HAL_Delay(1000);
			}
		}
		
		y |= (I2C2->RXDR << 8);
		
		//Wait until the TC (Transfer Complete) flag is set. //DIFFERENT HERE
		while(!(I2C2->ISR & I2C_ISR_TC));
		
		//Use the four LEDs to indicate whether each measured 
		//axis is positive or negative
		if(x > 10000){
			GPIOC->ODR |= GPIO_ODR_6;
			GPIOC->ODR &= ~GPIO_ODR_8;
		}
		else if (x < -10000){
			GPIOC->ODR |= GPIO_ODR_8;
			GPIOC->ODR &= ~GPIO_ODR_6;
		}
		
		if(y > 10000){
			GPIOC->ODR |= GPIO_ODR_9;
			GPIOC->ODR &= ~GPIO_ODR_7;
		}
		else if (y < -10000){
			GPIOC->ODR |= GPIO_ODR_7;
			GPIOC->ODR &= ~GPIO_ODR_9;
		}
		HAL_Delay(100);
	}
}

/*Set slave address*/
void I2CWriteAdd(int address, int data, int bytes){
	//Set the transaction parameters in the CR2 register.
	I2C2->CR2 = (0x6B << 1) | (bytes << 16) | I2C_CR2_START ; //Set the L3GD20 slave address = 0x6B
	
	//Wait until either of the TXIS (Transmit Register Empty/Ready) 
	//or NACKF (Slave Not-Acknowledge)flags are set.
	while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));
	
	//If the NACKF flag is set, the slave did not respond to the address frame. 
	//Continue if the TXIS flag is set.
	if(I2C2->ISR & I2C_ISR_NACKF){
		while(1){
			GPIOC->ODR ^= GPIO_ODR_8; //error
			HAL_Delay(300);
		}
	}

	//Write the address of the CNTRL_REG_1 register into the I2C transmit register. (TXDR)
	I2C2->TXDR = address;
	
	if(data != NULL){
	
		//Wait until either of the TXIS (Transmit Register Empty/Ready) 
		//or NACKF (Slave Not-Acknowledge)flags are set.
		while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));
	
		//If the NACKF flag is set, the slave did not respond to the address frame. 
		//Continue if the TXIS flag is set.
		if(I2C2->ISR & I2C_ISR_NACKF){
			while(1){
				GPIOC->ODR ^= GPIO_ODR_8; 
				HAL_Delay(300);
			}
		}

		//Write data to the register
		I2C2->TXDR = data;
	}
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(!(I2C2->ISR & I2C_ISR_TC));
}

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
