/*#include "main.h"
#include "lcd1602_i2c.h"
#include "mk_dht11.h"
#include "queue.h"


EventQueue event;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

dht11_t dht11;
char temp[16];
char humid[16];
enum SystemEvent currentEvent=EVENT01;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void readSensor();
void displayLCD();
void transmitData();
void initEventQueue();
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();


  EventQueue_Init(&event);
  initEventQueue();
  lcd1602_i2c_init(&hi2c1);
  init_dht11(&dht11, &htim1, DHT11_GPIO_Port, DHT11_Pin);
  void (*execute_task[])(void) = {readSensor,displayLCD,transmitData};
  HAL_TIM_Base_Start_IT(&htim2);
  while (1)
  {
	  if(EventQueue_isEmpty(&event)){
		  HAL_SuspendTick();
		  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		  HAL_ResumeTick();
	  }
	  else{
		  (*execute_task[currentEvent])();
		  currentEvent=EventQueue_Get(&event);
		  }
  }
}

void readSensor(){
	EventQueue_Put(&event, EVENT02);
	readDHT11(&dht11);
	HAL_Delay(100);

}

void displayLCD(){
	EventQueue_Put(&event, EVENT03);
	sprintf(temp,"TEMP=%.1f",(float)dht11.temperature);
	sprintf(humid,"HUMID=%.1f",(float)dht11.humidty);
	lcd1602_i2c_clear();
	lcd1602_i2c_1stLine();
	lcd1602_i2c_printf(temp);
	lcd1602_i2c_2ndLine();
	lcd1602_i2c_printf(humid);

}
void transmitData(){
	sprintf(temp,"TEMP=%.1f",(float)dht11.temperature);
	sprintf(humid,"HUMID=%.1f",(float)dht11.humidty);
	HAL_UART_Transmit(&huart1,(uint8_t*)temp, sizeof(temp), 1000);
	HAL_UART_Transmit(&huart1,(uint8_t*)"\n", 2, 100);
	HAL_UART_Transmit(&huart1,(uint8_t*)humid, sizeof(humid), 1000);
	HAL_UART_Transmit(&huart1,(uint8_t*)"\n", 2, 100);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == htim2.Instance)
 {

		 EventQueue_Put(&event, EVENT01);
 }
}

void initEventQueue(){
	event.event[0]=EVENT01;
	event.event[1]=EVENT02;
	event.event[2]=EVENT03;
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

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
    Error_Handler();
  }


}

static void MX_TIM1_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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
}
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }


}

static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }


}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);


  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);


}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif

*/
/*
 * A0 Temp Sensor
 * A1 Humid Sensor
 * I2C1 B6-SCL B7-SDA
 * UART1 A9-TX A10-RX
 */






#include "main.h"
#include "lcd1602_i2c.h"
#include "queue.h"
#include "mk_dht11.h"

EventQueue event;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
//ADC_HandleTypeDef hadc1;
char temp[16];
char humid[16];
uint32_t adcValueCN1;
uint32_t adcValueCN2;
float temp1;
float humid1;
dht11_t dht11;
uint32_t timeSystick;
uint32_t cycle=500;
enum SystemEvent currentEvent=EVENT01;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
//static void MX_ADC1_Init(void);
void readSensor();
void displayLCD();
void transmitData();
void initEventQueue();
//void ADC_Select_CH0 (void);
//void ADC_Select_CH1 (void);
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  MX_USART1_UART_Init();
  MX_TIM2_Init();
  //MX_ADC1_Init();
  EventQueue_Init(&event);
  initEventQueue();
  lcd1602_i2c_init(&hi2c1);
  void (*execute_task[])(void) = {readSensor,displayLCD,transmitData};
  //HAL_TIM_Base_Start_IT(&htim2);
  HAL_SYSTICK_Config(500);
  init_dht11(&dht11, &htim2, DHT11_GPIO_Port, DHT11_Pin);

  while (1)
  {
	  timeSystick= HAL_GetTick();
	  if(timeSystick%cycle){
		  EventQueue_Put(&event, EVENT01);
	  }
	  if(EventQueue_isEmpty(&event)){
		  HAL_SuspendTick();
		  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		  HAL_ResumeTick();
	  }
	  else{
		  (*execute_task[currentEvent])();
		  currentEvent=EventQueue_Get(&event);
		  }
  }
}
void readSensor(){
	EventQueue_Put(&event, EVENT02);
	readDHT11(&dht11);
//	ADC_Select_CH0();
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 1000);
//	uint32_t a=0;
//	for(int i=0;i<3;i++){
//		a+= HAL_ADC_GetValue(&hadc1);
//	}
//	adcValueCN1=a/3;
//	HAL_ADC_Stop(&hadc1);
//	temp1= adcValueCN1*22/273.0;
//	HAL_Delay(100);
//	ADC_Select_CH1();
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 1000);
//	uint32_t k=0;
//	for(int i=0;i<3;i++){
//		k+= HAL_ADC_GetValue(&hadc1);
//	}
//	adcValueCN2=k/3;
//	humid1=(float)adcValueCN2*100.0/4095;
//	HAL_ADC_Stop(&hadc1);

}

void displayLCD(){
	EventQueue_Put(&event, EVENT03);
	sprintf(temp,"TEMP=%.1f",(float)dht11.temperature);
	sprintf(humid,"humid=%.1f",(float)dht11.humidty);
//	sprintf(temp,"TEMP=%.1f",temp1);
//	sprintf(humid,"humid=%.1f",humid1);
	lcd1602_i2c_clear();
	lcd1602_i2c_1stLine();
	lcd1602_i2c_printf(temp);
	lcd1602_i2c_2ndLine();
	lcd1602_i2c_printf(humid);

}
void transmitData(){
//	sprintf(temp,"TEMP=%.1f",temp1);
//	sprintf(humid,"humid=%.1f",humid1);
	sprintf(temp,"TEMP=%.1f",(float)dht11.temperature);
	sprintf(humid,"humid=%.1f",(float)dht11.humidty);
	HAL_UART_Transmit(&huart1,(uint8_t*)temp, sizeof(temp), 1000);
	HAL_UART_Transmit(&huart1,(uint8_t*)"\n", 2, 100);
	HAL_UART_Transmit(&huart1,(uint8_t*)humid, sizeof(humid), 1000);
	HAL_UART_Transmit(&huart1,(uint8_t*)"\n", 2, 100);
}

void initEventQueue(){
	event.event[0]=EVENT01;
	event.event[1]=EVENT02;
	event.event[2]=EVENT03;
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
// if(htim->Instance == htim2.Instance)
// {
//		 EventQueue_Put(&event, EVENT01);
// }
//}

/*
void ADC_Select_CH0 (void)
{
 ADC_ChannelConfTypeDef sConfig = {0};
 sConfig.Channel = ADC_CHANNEL_0;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
}
void ADC_Select_CH1 (void)
{
 ADC_ChannelConfTypeDef sConfig = {0};
 sConfig.Channel = ADC_CHANNEL_1;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
}
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
/*
static void MX_ADC1_Init(void)
{


  ADC_ChannelConfTypeDef sConfig = {0};


  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }


  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
 */
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

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
