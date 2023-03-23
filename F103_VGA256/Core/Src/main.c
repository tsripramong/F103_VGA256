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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vga256.h>
#include "color100.h"
#include "flower100.h"
#include "rgb100.h"
#include "tetris.h"
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
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_up;

/* USER CODE BEGIN PV */
//uint16_t VOFFSET=3,preVoffset=3500;
uint16_t VOFFSET=0,preVoffset=95;
uint16_t vga_stop=0;
int16_t line=0;
uint16_t firstTrig=1;

GPIO_TypeDef *keyPort = GPIOA;
uint16_t keyRowPin[4] = {GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_12};
uint16_t keyColPin[3] = {GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7};
int keyMap[4][3] = {{1,2,3},{4,5,6},{7,8,9},{11,10,12}};
uint16_t vga_voff[16];
uint32_t GPIOB_ODR;
//NOTE: We can not use GPIOB for other GPIOs because we sent data directly to GPIOB
char msg[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void VGA_update(){
	vga_voff[0]=VOFFSET;
	for(int i=1;i<16;i++){
		vga_voff[i]=vga_voff[i-1]+VGA_LBUFFERSIZE;
	}
}

extern void tetrisDelay(int ms){
	HAL_Delay(ms);
}

extern uint8_t getch(uint8_t *ch){
	for(int i=0;i<4;i++){
		//Prime row output for ready to bring col to ground
		HAL_GPIO_WritePin(keyPort,keyRowPin[i],0);
	}
	//Detect column pressed
	int i,j;
	for(i=0;i<3;i++){
		if(HAL_GPIO_ReadPin(keyPort,keyColPin[i])==0){
			//Testing for row
			for(j=0;j<4;j++){
				HAL_GPIO_WritePin(keyPort,keyRowPin[j],1);
				if(HAL_GPIO_ReadPin(keyPort,keyColPin[i]))
					break;
			}
	    }
		if(j<4)break;
	}
    if(i<3){  //a key is pressed
    	*ch = keyMap[j][i];
    	return 1;
    }
    return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
     if(htim==&htim2){
     //V-sync detected
		line=-4;
		if(firstTrig){
			for(int i=0;i<preVoffset;i++);
			if(
            HAL_DMA_Start_IT(&hdma_tim4_up,(uint32_t)VGA_obuffer,GPIOB_ODR,VGA_FULL)
			!= HAL_OK){
				while(1){
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					HAL_Delay(1000);
				}
			}
			firstTrig=0;
		}
	}
}

static void DMA_HalfCpltCallback(DMA_HandleTypeDef *hdma){
     //fill in line1
	if((line<0)||(line>=VGA_VBUFFER)){
		memset((uint8_t *)VGA_obuffer,0,VGA_FULL);
	}else{
		for(int i=0;i<8;i++){
			memcpy((uint8_t *)VGA_obuffer + vga_voff[i],VGA_buffer[line],VGA_LBUFFER);
		}
	}
	line++;
	if(vga_stop){
		HAL_DMA_Abort_IT(&hdma_tim4_up);
		firstTrig=1;
		vga_stop=0;
		VGA_update();
	}
	if(line>=VGA_VBUFFER && !firstTrig){
		HAL_DMA_Abort_IT(&hdma_tim4_up);
		firstTrig=1;
	}
}

static void DMA_CpltCallback(DMA_HandleTypeDef *hdma){
    //fill in line2 (later half)
	if((line<0)||(line>=VGA_VBUFFER)){
		memset((uint8_t *)VGA_obuffer,0,VGA_FULL);
	}else{
		for(int i=8;i<16;i++){
			memcpy((uint8_t *)VGA_obuffer + vga_voff[i],VGA_buffer[line],VGA_LBUFFER);
		}
	}
	line++;
	if(vga_stop){
		HAL_DMA_Abort_IT(&hdma_tim4_up);
		firstTrig=1;
		vga_stop=0;
		VGA_update();
	}
	if(line>=VGA_VBUFFER && !firstTrig){
		HAL_DMA_Abort_IT(&hdma_tim4_up);
		firstTrig=1;
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  VGA_update();
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_DMA_Abort(&hdma_tim4_up);
    if(
    	  HAL_DMA_RegisterCallback(&hdma_tim4_up,HAL_DMA_XFER_HALFCPLT_CB_ID,DMA_HalfCpltCallback)
    	  !=HAL_OK){
    	  while(1){
    		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    		  HAL_Delay(500);
    	  }
    }
    if(
          HAL_DMA_RegisterCallback(&hdma_tim4_up,HAL_DMA_XFER_CPLT_CB_ID,DMA_CpltCallback)
          !=HAL_OK){
    	  while(1){
    		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    		  HAL_Delay(250);
    	  }
    }
    GPIOB_ODR = (uint32_t)&(GPIOB->ODR)+1;

    //Start GPIO-DMA output
    __HAL_DMA_ENABLE_IT(&hdma_tim4_up,DMA_IT_TC);
    __HAL_DMA_ENABLE_IT(&hdma_tim4_up,DMA_IT_HT);
    __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_UPDATE);
    HAL_TIM_Base_Start(&htim4);

    //Start H-sync
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);

    //Start V-sync
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);



    HAL_Delay(100);
    vga_stop=1;

    ClearScreen(VGA_BLACK);
    DrawRectangle(0,0,VGA_WIDTH-1,VGA_HEIGHT-1,VGA_WHITE);
    char msg[32]="Testing";
    SetCursor(3,3);
    WriteString(msg,Font_7x10,VGA_GREEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int r,x,y,w,h,z=0,c;
  while (1)
  {
// If you want to only try Tetris, uncomment here
//	  tetris();
//	  continue;

	  r = rand()%25;
	  x = rand()%VGA_WIDTH;
	  y = rand()%VGA_HEIGHT;
	  c = rand()%256;
	  DrawCircle(x,y,r,c);
	  r = rand()%25;
	  x = rand()%VGA_WIDTH;
	  y = rand()%VGA_HEIGHT;
	  c = rand()%256;
	  FillCircle(x,y,r,c);
	  x = rand()%VGA_WIDTH;
	  y = rand()%VGA_HEIGHT;
	  w = rand()%30;
	  h = rand()%30;
	  c = rand()%256;
	  DrawRectangle(x,y,x+w,y+h,c);
	  x = rand()%VGA_WIDTH;
	  y = rand()%VGA_HEIGHT;
	  w = rand()%30;
	  h = rand()%30;
	  c = rand()%256;
	  FillRectangle(x,y,w,h,c);

	  /*
	  {
		  uint8_t ch;
	      if(getch(&ch)){
	    	  sprintf(msg,"%d",ch);
	    	  SetCursor(1,1);
	    	  WriteString(msg,Font_7x10,VGA_WHITE);
	      }
	  }
	  HAL_Delay(100);
	  continue;
	  */
	  z=z+1;
	  if(z>=100){
		  /////////
		  ClearScreen(VGA_BLACK);
		  ShowImage((uint8_t *)colors,100,75,0,0);
		  HAL_Delay(5000);
		  ShowImage((uint8_t *)flower,100,63,0,0);
		  HAL_Delay(5000);
		  ShowImage((uint8_t *)rgb,100,75,0,0);
		  HAL_Delay(5000);

		  tetris();
		  //////////
		  z=0;
		  ClearScreen(VGA_BLACK);
		  HAL_Delay(100);
		  vga_stop=1;
		  sprintf(msg,"Testing");
		  DrawRectangle(0,0,VGA_WIDTH-1,VGA_HEIGHT-1,VGA_WHITE);
		  SetCursor(3,3);
		  WriteString(msg,Font_7x10,VGA_GREEN);
		  sprintf(msg,"%d ",VOFFSET);
		  SetCursor(3,50);
		  WriteString(msg,Font_7x10,VGA_WHITE);
	  }
	  HAL_Delay(100);
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1024-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 72-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 625-2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, G0_Pin|G1_Pin|G2_Pin|R0_Pin
                          |R1_Pin|R2_Pin|B0_Pin|B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : COL0_Pin COL1_Pin COL2_Pin */
  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : G0_Pin G1_Pin G2_Pin R0_Pin
                           R1_Pin R2_Pin B0_Pin B1_Pin */
  GPIO_InitStruct.Pin = G0_Pin|G1_Pin|G2_Pin|R0_Pin
                          |R1_Pin|R2_Pin|B0_Pin|B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW0_Pin ROW1_Pin ROW2_Pin ROW3_Pin */
  GPIO_InitStruct.Pin = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
