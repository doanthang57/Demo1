#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//#########BH1750##########
		int8_t datalux[2];
    uint8_t *pBuffer;
    uint8_t Nbyte = 2;
    uint16_t lux;
	  float flevel = 0;
		int level = 0;
//#########UART############		

		char UART3_Data;
		char UART_Buffer[32];
		char i = 0;
		uint8_t cplt = 0;
//#########Flash############		
		uint8_t data;
		uint32_t FlashAddress= 0x080E0000;
		char seri[4];
		char readseri;
		uint8_t id = 0;
//#################################//


//* bien du lieu UART sent*//
	uint8_t a = 0;
	char* data3 = "@L1KXx";
	char* data4 = "@L1KYx";
	
	uint8_t b = 0;
	char* data1 = "@L2KXx";
	char* data2 = "@L2KYx";
	
	uint8_t c = 0;
	char* data5 = "@L3KXx";
	char* data6 = "@L3KYx";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);

//#####Chuong trinh con dang code########//
uint32_t Read_Flash( uint32_t adress);
void Write_Flash(uint32_t add, uint8_t data);
void ReadADCControl(void);
void ReadGWControl(void);
void Button (void);
void ReadLux (void);
void save (void);



int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  HAL_UART_Receive_IT(&huart3, (uint8_t*) &UART3_Data, 1);
 
	
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);


  while (1)
   {
		 save();
		 if(id == 1)
		 {
			 Write_Flash(0x080E0000, readseri);
			 id = 0;
		 }
		 data = Read_Flash(0x080E0000);
		 ReadLux();
		 if(cplt == 0)
		 {
			//Button();
			 HAL_Delay(100);
		 }
		 else{
			 if(data == '1'){
				 ReadGWControl();
				 HAL_Delay(1000);
				 cplt = 0;
			 }
		 }
}
	}
/** System Button Configuration*/
void Button (void)
{		
	///******** read input PA3 set on ***********/
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==GPIO_PIN_SET && a == 0)
		{
			HAL_Delay(50);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==GPIO_PIN_SET)
				{
					a = 1;
					HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==GPIO_PIN_SET);
					HAL_UART_Transmit_IT(&huart3, data1, 6 );
					HAL_Delay(200);
				}
		}
	///************ read input PA3 set off **************/	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==GPIO_PIN_SET && a == 1)
		{
			HAL_Delay(50);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==GPIO_PIN_SET)
				{
					a = 0;
					HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==GPIO_PIN_SET);
					HAL_UART_Transmit_IT(&huart3, data2, 6 );
					HAL_Delay(200);
			}
		}
		///******** read input PA5 set on ***********/	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_SET && b == 0)
		{
			HAL_Delay(50);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_SET)
				{
					b = 1;
					HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_SET);
					HAL_UART_Transmit_IT(&huart3, data3, 6);
					HAL_Delay(200);
				}
		}
	///************ read input PA5 set off **************/	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_SET && b == 1)
		{
			HAL_Delay(50);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_SET)
				{
					b = 0;
					HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_SET);
					HAL_UART_Transmit_IT(&huart3, data4, 6 );
					HAL_Delay(200);
			}
		}
	///******** read input PA3 set on ***********/
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET && c == 0)
		{
			HAL_Delay(50);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET)
				{
					c = 1;
					HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET);
					HAL_UART_Transmit_IT(&huart3, data5, 6 );
					HAL_Delay(200);
				}
		}
	///************ read input PA3 set off **************/	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET && c == 1)
		{
			HAL_Delay(50);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET)
				{
					c = 0;
					HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET);
					HAL_UART_Transmit_IT(&huart3, data6, 6);
					HAL_Delay(200);
			}
		}
	} // dong if cua button
void ReadLux (void)
{
	 datalux[0] = BH1750_POWER_ON;
	 datalux[1] = RESET;
	 HAL_I2C_Master_Transmit(&hi2c1, BH1750_I2CADDR , datalux, 2, 100);
	 datalux[0] = BH1750_CONTINUOUS_HIGH_RES_MODE;
	 HAL_I2C_Master_Transmit(&hi2c1, BH1750_I2CADDR , datalux, 1, 100);
	 HAL_Delay(180);
	 HAL_I2C_Master_Receive(&hi2c1, BH1750_I2CADDR , datalux, 2, 100);

    lux = (datalux[0] << 8) | datalux[1];
    flevel = (float)lux/1.2; // convert to real lux
    level = (float)lux/1.2; // convert to real lux
}
void ReadGWControl(void)
{
	//@V4BXS600x
	  if( (UART_Buffer[0] == '@') &&(UART_Buffer[1] == 'L') && (UART_Buffer[2] == '1')&& (UART_Buffer[3] == 'K'))
			{
				if(UART_Buffer[4]== 'X')
				{
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,1);
					b = 1;
					HAL_Delay(200);
					
				}
				if(UART_Buffer[4]== 'Y')
				{
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,0);
					b =  0;
					HAL_Delay(200);
					
				}
			}
		if( (UART_Buffer[0] == '@') &&(UART_Buffer[5] == 'L') && (UART_Buffer[6] == '2')&& (UART_Buffer[7] == 'K'))
			{
				if(UART_Buffer[8]== 'X')
				{
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1);
					a = 1;
					HAL_Delay(200);
					
				}
				if(UART_Buffer[8]== 'Y')
				{
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0);
					a = 0;
					HAL_Delay(200);
					
				}
			}
	  if( (UART_Buffer[0] == '@') &&(UART_Buffer[9] == 'L') && (UART_Buffer[10] == '3')&& (UART_Buffer[11] == 'K'))
			{
				if(UART_Buffer[12]== 'X')
				{
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,1);
					c = 1;
					HAL_Delay(200);
					
				}
				if(UART_Buffer[12]== 'Y')
				{
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,0);
					c = 0;
					HAL_Delay(200);
					
				}
			}
}			
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART3){
	 UART_Buffer[i] = UART3_Data;
  if(UART3_Data != '\n'){
		if(UART_Buffer[0] == '@'){
		 UART_Buffer[i] = UART3_Data;
		 i++;
			}
		}
  if(UART3_Data == 'x'){
   i = 0;
   }
   cplt = 1;
   HAL_UART_Receive_IT(&huart3, (uint8_t*) &UART3_Data, 1);
  }
}
/** System Clock Configuration
*/
void save(void)
{
	if(UART_Buffer[1] == 'S')
	{
		seri[0] = '@';
		seri[1] = 'S';
		seri[2] = UART_Buffer[2];
		seri[3] = UART_Buffer[3];
		if(seri[3] == '1')
		{
			readseri = '1';
			id = 1;
		}
		else{
			readseri = '0';
			id = 1;
		}
	}
}

void Write_Flash(uint32_t add, uint8_t data)
{
     HAL_FLASH_Unlock();
     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
     FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);
     HAL_FLASH_Program(TYPEPROGRAM_WORD, FlashAddress, data);
     HAL_FLASH_Lock();
}

uint32_t Read_Flash( uint32_t adress)
{
	uint32_t data_flash;
	data_flash = *(uint32_t*)adress;
	
return data_flash;

}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_OUT_Pin|LED_OUTD13_Pin|LED_OUTD14_Pin|LED_OUTD15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_Pin ButtonA6_Pin ButtonA7_Pin */
  GPIO_InitStruct.Pin = Button_Pin|ButtonA6_Pin|ButtonA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_OUT_Pin LED_OUTD13_Pin LED_OUTD14_Pin LED_OUTD15_Pin */
  GPIO_InitStruct.Pin = LED_OUT_Pin|LED_OUTD13_Pin|LED_OUTD14_Pin|LED_OUTD15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
