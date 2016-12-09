/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-April-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_ADDRESS        4

/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 50 MHz */
/* This example use TIMING to 0x40912732 to reach 100 kHz speed (Rise time = 700 ns, Fall time = 100 ns) */
#define I2C_TIMING      0x40912732

#define RXBUFFSIZE 20
#define AUXBUFFSIZE 805
#define  PERIOD_VALUE       (uint32_t)(1332 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE/4)        /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*12.5/100) /* Capture Compare 4 Value  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t rxbuff[RXBUFFSIZE];
volatile uint8_t auxbuff[AUXBUFFSIZE];
volatile uint8_t str_index = 0;
volatile uint8_t flag = 0;
uint8_t motor_status = MOTOR_OFF;

/* Buffer used for transmission */
uint8_t lorem[] = "In the great green room \nThere was a telephone \nAnd a red balloon \nAnd a picture of \nThe cow jumping over the moon \nAnd there were three little bears sitting on chairs \nAnd two little kittens \nAnd a pair of mittens \nAnd a little toy house \nAnd a young mouse \nAnd a comb and a brush and a bowl full of mush \nAnd a quiet old lady who was whispering hush \n\nGoodnight room \nGoodnight moon \nGoodnight cow jumping over the moon \nGoodnight light \nAnd the red balloon \n\nGoodnight bears \nGoodnight chairs \nGoodnight kittens \nAnd goodnight mittens \n\nGoodnight clocks \nAnd goodnight socks \nGoodnight little house \nAnd goodnight mouse \n\nGoodnight comb \nAnd goodnight brush \nGoodnight nobody \nGoodnight mush \n\nAnd goodnight to the old lady whispering hush \n\nGoodnight stars \nGoodnight air \nGood night noises everywhere";



UART_HandleTypeDef UartHandle;
UART_HandleTypeDef UartAux;

/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
static void SystemClock_Config(void);
static void Error_Handler(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  
  /* Configure the MPU attributes as Write Through */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
   */



  HAL_Init();

  /* Configure the system clock to 216 MHz */
  SystemClock_Config();


  /* Compute the prescaler value to have TIM3 counter clock equal to 18000000 Hz */
    uhPrescalerValue = (uint32_t)(78);


  /* Add your application code here */

  TimHandle.Instance = TIM3;

  TimHandle.Init.Prescaler         = uhPrescalerValue;
  TimHandle.Init.Period            = PERIOD_VALUE;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* Set the pulse value for channel 1 */
  sConfig.Pulse = PULSE1_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }


  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  UartAux.Instance        = USARTaux;

  UartAux.Init.BaudRate   = 38400;
  UartAux.Init.WordLength = UART_WORDLENGTH_8B;
  UartAux.Init.StopBits   = UART_STOPBITS_1;
  UartAux.Init.Parity     = UART_PARITY_NONE;
  UartAux.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartAux.Init.Mode       = UART_MODE_TX_RX;
  UartAux.Init.OverSampling = UART_OVERSAMPLING_16;
  UartAux.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }

  if (HAL_UART_Init(&UartAux) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }


  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)rxbuff, RXBUFFSIZE) != HAL_OK) {
	  Error_Handler();
  }

  while(1);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            PLL_R                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;  
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/* This function overrides the compiler's version of putchar to reroute those characters
 * to UartHandle, which sends to the computer terminal.
 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/*
 * This function updates the pwm signal for pwm channel 1 with a new duty cycle
 */
void update_pwm(int* dc) {
	if (motor_status == MOTOR_OFF) {
		return;
	} else if (motor_status == MOTOR_ON) {
		if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
		{
			/* PWM Generation Error */
			Error_Handler();
		}

		sConfig.Pulse = (uint32_t)(PERIOD_VALUE * (float)*dc/100.0f);

		if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
		{
			/* Configuration Error */
			Error_Handler();
		}
		if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
		{
			/* PWM Generation Error */
			Error_Handler();
		}

	}
}
/*
 * This function prints a char array to a given Uart of the F767
 */
void print(UART_HandleTypeDef *huart, const uint8_t buff[], uint16_t n) {
	int i;
	for (i=0; i<n; i++) {
		if (buff[i] == 0x00) break;
		HAL_UART_Transmit(huart, (uint8_t *)&buff[i], 1, 0xFFFF);
	}
}

/*
 * Clears the rxbuff string
 */
void clearstring() {
	int i;
	for (i=0; i<RXBUFFSIZE; i++) {
		rxbuff[i] = 0x00;
	}
}

/*
 * Sends the rxbuff string to the terminal and clears it. Not super necessary to have anymore
 * but it is still and use and I haven't gotten around to replacing it.
 */
void sendstring() {
	print(&UartHandle, rxbuff, RXBUFFSIZE);
	clearstring();
}

/*
 * Returns the length of the string in the char array of length n
 */
uint8_t lenstr(uint8_t *str, uint8_t n) {
	uint16_t i=0;
	while (i < n) {
		if (str[i] == 0x00) break;
		i++;
	}
	return i;
}

/*
 * Compares the first 'index' values of the char arrays a and b and returns the number of
 * inequalities found.
 */
uint16_t str_comp(const char *a, const char *b, uint16_t index) {
	uint16_t i=0;
	uint16_t nerr = 0;
	while(i<index) {
		if (a[i] != b[i]) nerr++;
		i++;
	}
	return nerr;
}

/*
 * Copies the contents of a into b up to the RXBUFFSIZE, only meant to be used
 * for arrays of length RXBUFFSIZE. Again, this is am artifact of my first pass
 * through where I just needed stuff to work and didn't worry about modularity/robustness.
 */
void strcopy(uint8_t *a, uint8_t *b) {
	uint16_t i=0;
	while(i<RXBUFFSIZE){
		b[i] = a[i];
		i++;
	}
}

/*
 * Takes a char array pointer and its length and puts the value of the decimal number
 * the string represents into n, if it is a number. Otherwise returns 0.
 */
uint8_t str2int(volatile uint8_t *a, uint8_t str_len, int* n) {
	uint8_t j = str_len-1;
	uint8_t i;
	*n = 0;
	for (i=0; i<str_len; i++){
		if (a[i] < 0x30 || a[i] > 0x39) return 0;
		uint8_t k = j;
		int temp = a[i] - 0x30;
		while (k > 0){
			temp = temp*10;
			k--;
		}
		*n += temp;
		j--;
	}
	return 1;
}

/*
 * The character match flag callback function.
 * This function is called whenever the character that the terminal ends all its strings
 * with (0x3b, ';') is received. This means a string of less than RXBUFFSIZE characters has
 * been received and we should look at what it is.
 */
void cmf_UART_callback() {

	uint8_t str_index = lenstr(rxbuff, RXBUFFSIZE);
	if (str_index == 0) return;
	printf("\n%d ", str_index);

	// If the command UARTXXXX, where XXXX is some integer number of tests to be performed,
	// then perform those tests (send the lorem string to the Arduino and compare the received
	// string to the one sent. We use a power drill to simulate EM noise on the channel and record errors.
	if (!str_comp(rxbuff, "UART", 4)) {
		printf("\nUART STRESS TESTING\n");
		uint16_t ntests = 0;
		printf("ndigits: %d\n", str_index - 4);
		str2int(rxbuff+4, str_index-4, &ntests);
		printf("ntests: %d\n", ntests);
		uint16_t nerr = 0;
		uint16_t i=0;
		while (i < ntests) {
			print(&UartAux, lorem, AUXBUFFSIZE);
			if(HAL_UART_Receive(&UartAux, (uint8_t *)auxbuff, AUXBUFFSIZE, 0xF) != HAL_OK) {
				  Error_Handler();
			}
			nerr += str_comp(auxbuff, lorem, AUXBUFFSIZE);
			i++;
		}
		printf("num error bytes: %d\n", nerr);
	}

	int dc; // used in case (2)

//	Check for control strings
	switch (str_index) {
	case (8): //motor
			if (!str_comp(rxbuff, "motor-on", str_index)) {
				if (motor_status == MOTOR_OFF) {
					if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
					{
						/* PWM Generation Error */
						Error_Handler();
					}
					motor_status = MOTOR_ON;
				}
				printf("\nreceived motor-on\n");
			}
			break;
	case (9): //motor
			if (!str_comp(rxbuff, "motor-off", str_index)) {
				if (motor_status == MOTOR_ON) {
					if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
					{
						Error_Handler();
					}
					motor_status = MOTOR_OFF;
				}
				printf("\nreceived motor-off\n");
			}
			break;
	case (2)://PWM duty cycle value
			if(str2int(rxbuff, str_index, &dc)) {
				update_pwm(&dc);
				printf("Received int: %d\n", dc);
			}
			else printf("Wasn't an integer\n");
	}

	printf(rxbuff);
	printf("\n");
	clearstring();

}

/*
 * This is the callback used when the receive buffer rxbuff is full, meaning
 * the terminal sent at least 20 characters.
 */
void main_UART_callback(UART_HandleTypeDef *huart) {
	sendstring();
}

// I no longer want to use the interrupts for the auxillary UART, commented out

//void aux_UART_callback(UART_HandleTypeDef *huart) {
//	printf("aux_UART_callback: ");
//	printf(auxbuff);
//	printf("\n");
//	int i;
//
//	__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
//
//	uint8_t errmsg[] = "error\n";
//	uint8_t noerrmsg[] = "no error\n";
//	uint16_t e = str_comp(auxbuff, lorem, AUXBUFFSIZE);
//	if (e) {
//		HAL_UART_Transmit(&UartHandle, (uint8_t *)&errmsg, 6, 0xFFFF);
//		printf("%d\n", e);
//	}
//	else {
//		HAL_UART_Transmit(&UartHandle, (uint8_t *)&noerrmsg, 9, 0xFFFF);
//
//	}
//
//	for (i=0; i<AUXBUFFSIZE; i++) {
//			auxbuff[i] = 0x00;
//	}
//
//}

/*
 * If an error flag caused an interrupt, this callback is called. For debugging purposes only.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	printf("error callback\n");
	if (huart->Instance == USARTx) {
		if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)rxbuff, RXBUFFSIZE) != HAL_OK) {
		  Error_Handler();
		}
	}
}

/*
 * This is the callback called by the interrupt service routine. From here, depending
 * on which Uart caused the interrupt and what type of interrupt it was, we call the
 * approproate callback function.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance->ISR & USART_ISR_CMF) {
		// Character Match Flag interrupt
		__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
		SET_BIT(huart->Instance->ICR, USART_ICR_CMCF);
		if (huart->Instance != USARTaux) cmf_UART_callback();
	}
	else {
		if (huart->Instance == USARTx) main_UART_callback(huart);
	}

	if (huart->Instance == USARTaux) {
		//As of 7 Dec, no longer use interrupts for UartAux
	} else {
		if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)rxbuff, RXBUFFSIZE) != HAL_OK) {
				  Error_Handler();
		}
	}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
