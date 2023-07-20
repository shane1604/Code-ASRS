/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* QUY TAC */
/* Quy dinh 1 la chieu duong cua tat ca cac truc
	 1 là step
	 1 la DIR thuan
	 Ngat b12 X
	 Ngat B13 Y (cam bien khe)
	 Ngat B14 Z 
	 
	 DUNG VI BUOC 4
	 20 STEP = 1mm
	 TRUC Y nhân 2 he so. => 10 STEP = 1mm
	 
	 */
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cstdio> 
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DIRx_PIN GPIO_PIN_1
#define DIRx_PORT GPIOA
#define STEPx_PIN GPIO_PIN_2
#define STEPx_PORT GPIOA

#define DIRy_PIN GPIO_PIN_3
#define DIRy_PORT GPIOA
#define STEPy_PIN GPIO_PIN_4
#define STEPy_PORT GPIOA

#define DIRz_PIN GPIO_PIN_5
#define DIRz_PORT GPIOA
#define STEPz_PIN GPIO_PIN_6
#define STEPz_PORT GPIOA

#define xungStart htim2.Instance -> CCR2 =500;
#define xungStop htim2.Instance -> CCR2 = 0; 

#define STEP 2600  //khoang cách giua các ô hàng
#define INIT_Z 200 //khoang cách tu vi tri dau tien den ke hang tang 1
#define INIT_X 2600 //vi tri dau tien den o hang thu nhat

	int stepperDirection = 0;
	int delayFlag = 0;

	int at_x = 0; 
	int at_y = 0;
	int at_z = 0;

	int stepDelay = 1000; // 1000us more delay means less speed
	int delayFlagX = 0;
	int delayFlagY = 0;
	int delayFlagZ = 0;
	
	int slots[2][2] = {0};
	int currentX, currentZ;
	int targetX, targetZ;
	
	//int isFull = 1;
	
	void reset_xyz()
{
	at_x = 0;
	at_y = 0;
	at_z =0;
	
	delayFlagX = 0;
	delayFlagY = 0;
	delayFlagZ = 0;
}

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
	
}
volatile int cnt1 =0;
volatile int cnt2 =0;
	int cnt3 =0;
	int cnt4 =0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	cnt1++;
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0 && at_x ==0)
	{ 
		cnt2++;
		at_x = 1;
		stepperDirection = 0;
    delayFlagX = 1;
		/*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); */
	//	for(int x =500000; x>0; x--);
                // Device header
	//	HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)  == 1 &&( at_y == 0))
	{  
		cnt3++;
		at_y = 1;
		stepperDirection = 1;
    delayFlagY = 1;
			//for(int x =500000; x>0; x--);
		/*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); */
	//	HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
		
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)  == 0 && at_z == 0)
	{
		cnt4++;
		at_z = 1;
		stepperDirection = 1; //up
    delayFlagZ = 1;
	}
}
//STEP
void step (int steps, uint8_t direction, uint16_t delay, GPIO_TypeDef* DIR_x_Port, uint16_t DIR_x_Pin, GPIO_TypeDef* STEP_x_Port, uint16_t STEP_x_Pin )
{
  int x;
  if (direction == 0)
    HAL_GPIO_WritePin(DIR_x_Port, DIR_x_Pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(DIR_x_Port, DIR_x_Pin, GPIO_PIN_SET);
  for(x=0; x<steps; x=x+1)
  {
    HAL_GPIO_WritePin(STEP_x_Port, STEP_x_Pin, GPIO_PIN_SET);
    microDelay(delay);
    HAL_GPIO_WritePin(STEP_x_Port, STEP_x_Pin, GPIO_PIN_RESET);
    microDelay(delay);
  }
}
void setHomeX()
{
	 HAL_GPIO_WritePin(DIRx_PORT, DIRx_PIN, GPIO_PIN_RESET);// 0 -
	while (delayFlagX == 0)
    {
     HAL_GPIO_WritePin(STEPx_PORT, STEPx_PIN, GPIO_PIN_SET);
     microDelay(1000);
     HAL_GPIO_WritePin(STEPx_PORT, STEPx_PIN, GPIO_PIN_RESET);
     microDelay(1000);
    }
}
void setHomeY()
	{
		HAL_GPIO_WritePin(DIRy_PORT, DIRy_PIN, GPIO_PIN_RESET);// 0 - 
		while (delayFlagY == 0)
    {
      HAL_GPIO_WritePin(STEPy_PORT, STEPy_PIN, GPIO_PIN_SET);
      microDelay(1000);
      HAL_GPIO_WritePin(STEPy_PORT, STEPy_PIN, GPIO_PIN_RESET);
      microDelay(1000);
    }
	}
void setHomeZ() //DIR SET -> UP
	{
		HAL_GPIO_WritePin(DIRz_PORT, DIRz_PIN, GPIO_PIN_RESET); //0 -
		while (delayFlagZ == 0)
    {
      HAL_GPIO_WritePin(STEPz_PORT, STEPz_PIN, GPIO_PIN_SET);
     microDelay(1000);
      HAL_GPIO_WritePin(STEPz_PORT, STEPz_PIN, GPIO_PIN_RESET);
     microDelay(1000);
    }
	}

	int checkSensor()
	{
		if(HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) == 0)
		{
			return 1;
		}
		return 0;
	}
 
int checkAllSlots(int* x, int* z)
{
    int isFull = 1;
    int min_distance = 10000;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            if (slots[i][j] == 0)
            {
                int distance = i + j;
                if (distance < min_distance)
                {
                    *x = j;
                    *z = i;
                    min_distance = distance;
                    isFull = 0;
                }
            }
    return isFull;
}

void getPackage()
{
		step(1300, 1, 1000, DIRz_PORT, DIRz_PIN, STEPz_PORT, STEPz_PIN); //DIRz_PIN = 1 -> set -> UP
		HAL_Delay(2000);
	
		step(1200, 1, 1000, DIRy_PORT, DIRy_PIN, STEPy_PORT, STEPy_PIN);  //DIRy = 1 -> SET -> GIVE
		HAL_Delay(2000);
	
		step(400, 1, 1000, DIRz_PORT, DIRz_PIN, STEPz_PORT, STEPz_PIN); //DIRz_PIN = 1 -> set -> UP
		HAL_Delay(2000);
	
		step(1200, 0, 1000, DIRy_PORT, DIRy_PIN, STEPy_PORT, STEPy_PIN); //DIRy = 0 -> RESET -> back
		HAL_Delay(2000);
	
		step(1700, 0, 1000, DIRz_PORT, DIRz_PIN, STEPz_PORT, STEPz_PIN); 
		HAL_Delay(2000);
		
}

void moveTo(int x, int z)
{
    x = x * STEP; //so buoc tu ke nay sang ke khac 
		x = x + INIT_X; //he so khoi tao ban dau tu in toi vi tri dau tien
    step(x, 1, 1000, DIRx_PORT, DIRx_PIN, STEPx_PORT, STEPx_PIN);; //buoc tu ke 1->2
		
	
    z = z * (STEP - 1100);
    z = z + INIT_Z;
	  step(z, 1, 1000, DIRz_PORT, DIRz_PIN, STEPz_PORT, STEPz_PIN);; 
		
		HAL_Delay(2000);

    /*
    ...
    */
}
/* USER CODE END 0 */
void storagePackage()
{
		step(1200, 1, 1000, DIRy_PORT, DIRy_PIN, STEPy_PORT, STEPy_PIN);  //DIRy = 1 -> SET -> give
		HAL_Delay(2000);
	
		step(200, 0, 1000, DIRz_PORT, DIRz_PIN, STEPz_PORT, STEPz_PIN); 
		HAL_Delay(2000);
	
		step(1200, 0, 1000, DIRy_PORT, DIRy_PIN, STEPy_PORT, STEPy_PIN); //DIRy = 1 -> RESET -> back
		HAL_Delay(2000);
}
void setSlot(int x, int z)
{
	slots[z][x] = 1;
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
	
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

		setHomeX();
		setHomeY();
		setHomeZ();
		HAL_Delay(2000);
		reset_xyz();
		
		
		HAL_Delay(3000);
		while(1)
		{
			if(checkSensor())
			{
				HAL_Delay(2000);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
				HAL_Delay(1000);
				
					if(checkAllSlots(&targetX, &targetZ) == 0)
					{
						getPackage();
						moveTo(targetX, targetZ);

						storagePackage();
						setSlot(targetX, targetZ);
			
						HAL_Delay(2000);
						reset_xyz();
						setHomeZ();
						setHomeX();
						setHomeY();
						HAL_Delay(2000);
						reset_xyz();
						
						HAL_Delay(3000);
						
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
						HAL_Delay(3000);
					}
					else 
					{ 
						if(checkAllSlots(&targetX, &targetZ) == 1)
							{
								for(int i = 0; i< 200; i++)
								{
								HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
								HAL_Delay(500);
								}
							}	
					}			
			}
			else 
			{ 
				if(checkAllSlots(&targetX, &targetZ) == 1)
					{
						for(int i = 0; i< 200; i++)
						{
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
						HAL_Delay(500);
						}
					}	
			}
/*
	//THIS FOR TEST	
	while(1)
		{
			if(checkSensor())
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
				HAL_Delay(1000);
			}
		}
	return 0;
	*/
	}
return 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM2_Init(void)



{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); //LED
	
  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PB11 */ 												//LED
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SENSOR_Pin */
  GPIO_InitStruct.Pin = SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SENSOR_GPIO_Port, &GPIO_InitStruct);
	
  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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




