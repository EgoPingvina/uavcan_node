#include "main.hpp"
#include "uavcan_node.hpp"
#include "NumericConvertions.hpp"

const uint32_t START_DELAY_MS = 5000;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

void ErrorHandler(char * file, int line)
{
	// ToDo make a notification about error
	while (true)
	{

	}
}

/// <summary>
/// System Clock Configuration
/// </summary>
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	
	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType		= RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState				= RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue	= 16; // ToDo разобраться, почему не RCC_HSICALIBRATION_DEFAULT
	RCC_OscInitStruct.PLL.PLLState			= RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource			= RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL			= RCC_PLL_MUL16;
	
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();
	
	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType			= RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource		= RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider		= RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider	= RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider	= RCC_HCLK_DIV1;
	
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		Error_Handler();
}

/// <summary>
/// CAN Initialization Function
/// </summary>
static void MX_CAN_Init(void)
{
	hcan.Instance					= CAN1;
	hcan.Init.Prescaler				= 2;
	hcan.Init.Mode					= CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth			= CAN_SJW_1TQ;
	hcan.Init.TimeSeg1				= CAN_BS1_13TQ;
	hcan.Init.TimeSeg2				= CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode		= DISABLE;
	hcan.Init.AutoBusOff			= DISABLE;
	hcan.Init.AutoWakeUp			= DISABLE;
	hcan.Init.AutoRetransmission	= DISABLE;
	hcan.Init.ReceiveFifoLocked		= DISABLE;
	hcan.Init.TransmitFifoPriority	= DISABLE;
	  
	if (HAL_CAN_Init(&hcan) != HAL_OK)
		Error_Handler();
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if (htim->Instance == TIM3)
	{

		/* TIM3 GPIO Configuration
		 * PA6     ------> TIM3_CH1
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* TIM3 GPIO Configuration
		 * PA7     ------> TIM3_CH2
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}

/// <summary>
/// TIM3 Initialization Function
/// </summary>
static void MX_TIM3_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	
	htim3.Instance						= TIM3;
	htim3.Init.Prescaler				= 63;
	htim3.Init.CounterMode				= TIM_COUNTERMODE_UP;
	htim3.Init.Period					= 2500;
	htim3.Init.ClockDivision			= TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload		= TIM_AUTORELOAD_PRELOAD_DISABLE;
	  
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
		Error_Handler();
	  
	sMasterConfig.MasterOutputTrigger	= TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode		= TIM_MASTERSLAVEMODE_DISABLE;
	  
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
		Error_Handler();
	  
	sConfigOC.OCMode					= TIM_OCMODE_PWM1;
	sConfigOC.Pulse						= 0;
	sConfigOC.OCPolarity				= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode				= TIM_OCFAST_DISABLE;
	  
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		Error_Handler();
	    
	HAL_TIM_MspPostInit(&htim3);
}

/// <summary>
///  USART2 Initialization Function
/// </summary>
static void MX_USART2_UART_Init(void)
{
	huart2.Instance				= USART2;
	huart2.Init.BaudRate		= 230400;
	huart2.Init.WordLength		= UART_WORDLENGTH_8B;
	huart2.Init.StopBits		= UART_STOPBITS_1;
	huart2.Init.Parity			= UART_PARITY_NONE;
	huart2.Init.Mode			= UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl		= UART_HWCONTROL_NONE;
	huart2.Init.OverSampling	= UART_OVERSAMPLING_16;
		
	if (HAL_UART_Init(&huart2) != HAL_OK)
		Error_Handler();
}

/// <summary>
/// GPIO Initialization Function
/// </summary>
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin		= GPIO_PIN_4;
	GPIO_InitStruct.Mode	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull	= GPIO_NOPULL;
	GPIO_InitStruct.Speed	= GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	 /* USER CODE END 6 */
}
#endif

int main(void)
{
	 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	
	/* Configure the system clock */
	SystemClock_Config();
	
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();

	uavcan_node::configureNode();
	uavcan_node::NodeStartSub();
	uavcan_node::NodeStartPub();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
	/* Infinite loop */
	while (1)
	{
		uavcan_node::NodeSpin();

		if (HAL_GetTick() < START_DELAY_MS)
			continue;

		int value = 0;
		if (uavcan_node::getIntRaw(&value))
		{
			TIM3->CCR2 = TIM3->CCR1 =
				value < 1
				? 900
				: NumericConvertions::RangeTransform<1, 8191, 1075, 1950>(value);
		}
	}
}