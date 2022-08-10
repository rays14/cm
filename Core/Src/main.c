/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum cm_state_t {
      INIT,            // Init system.
      CANID,           // Read PWM and identify myself.
      CANVALVECMD,     // Read CAN bus valve commands and act upon then.
      CANPRESSUREREAD, // Read pressure.
      CANHEIGHTREAD,   // Read height.
      TEMPREAD,        // Read temperature.
      READADC,         // READ ADC
      CANSTATUSREPORT  // Report status.
} cm_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
cm_state_t cmFsm = INIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Counter
#define MAX_COUNT    1000
#define SECS(x)      x / 0.01
#define AVG_DATA_LEN 10
#define CAN_ADDRESS(designator, value_6_bit) (((uint16_t)designator << 6) | (uint16_t)value_6_bit)

uint16_t canAddresses[] = {
    CAN_ADDRESS(0xA, 0),
    CAN_ADDRESS(0xA, 1),
    CAN_ADDRESS(0xA, 2),
    CAN_ADDRESS(0xA, 3),
    CAN_ADDRESS(0xA, 4),
    CAN_ADDRESS(0xA, 5),
    CAN_ADDRESS(0xA, 6),
    CAN_ADDRESS(0xA, 7)
};
uint16_t myCANAddress = CAN_ADDRESS(0xA, 0);
uint32_t counter = MAX_COUNT;
uint32_t toggle  = 1;
uint32_t pwmOnTime = 0;
uint32_t canIdCounter = 0;

#define TIMCLOCK   90000000
#define PRESCALAR  90

uint32_t icVal1 = 0;     // Input capture value 1 (rising edge)
uint32_t icVal2 = 0;     // Input capture value 2 (falling edge)
uint32_t difference = 0; // Difference of the two above values
int isFirstCaptured = 0; // State of edge capture

float    frequency          = 0;
uint32_t usWidth            = 0;
uint32_t data[AVG_DATA_LEN] = {0,}; // Running average array
float    avgWidth           = 0.0f; // Average pulse width

uint32_t adcVal[3];
float adcVoltage[3];

#if 0
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        if (Is_First_Captured==0) // if the first rising edge is not captured
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
            Is_First_Captured = 1;  // set the first captured as true
        }

        else   // If the first rising edge is captured, now we will capture the second edge
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value

            if (IC_Val2 > IC_Val1)
            {
                Difference = IC_Val2-IC_Val1;
            }

            else if (IC_Val1 > IC_Val2)
            {
                Difference = (0xffffffff - IC_Val1) + IC_Val2;
            }

            float refClock = TIMCLOCK/(PRESCALAR);

            frequency = refClock/Difference;

            __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
            Is_First_Captured = 0; // set it back to false
        }
    }
}
#endif

float avg(uint32_t *data, uint32_t len, uint32_t newValue) {
    float    avg = 0;
    float    sum = 0;
    uint32_t i   = 0;

    // Shift all values
    sum = sum - (float)data[0];
    for (i = 1; i < len; i++) {
        data[i - 1] = data[i];
    }
    data[i - 1] = newValue;
    sum = sum + (float)newValue;
    avg = sum / len;

    return avg;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  {                // if the interrupt source is channel1
        if (isFirstCaptured == 0) {                                  // if the first value is not captured
            icVal1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
            isFirstCaptured = 1;                                     // set the first captured as true
        } else {                                                     // if the first is already captured
            icVal2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value

            if (icVal2 > icVal1) {
                difference = icVal2 - icVal1;
            } else if (icVal1 > icVal2) {
                difference = (0xffffffff - icVal1) + icVal2;
            }

            float refClock = TIMCLOCK / (PRESCALAR);
            float mFactor  = 1000000 / refClock;

            usWidth  = difference * mFactor;
            if (usWidth < htim->Init.Period) {                        // Check to make sure we are not getting overflow
                avgWidth = avg(data, AVG_DATA_LEN, usWidth);
            }

            // ------------------------------------------------------------------
            // *** SidRay - DO NOT RESET THE COUNTER. ***
            // Resetting the counter messes up the the base counter and the pwm
            //__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
            // ------------------------------------------------------------------

            isFirstCaptured = 0; // set it back to false
        }
    }
}

void delay(uint32_t maxDelayAmount) {
    volatile uint32_t amount = maxDelayAmount;
    while (amount > 0) {
        amount--;
    }
}

void channelSelect(int channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    if (channel == 1) {
        sConfig.Channel = ADC_CHANNEL_4;
    } else if (channel == 2) {
        sConfig.Channel = ADC_CHANNEL_13;
    } else if (channel == 3) {
        sConfig.Channel = ADC_CHANNEL_14;
    }
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
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
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  pwmOnTime = 2000;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // Decrement toggle counter.
      counter--;
      if (counter == 0) {
          counter = MAX_COUNT;
          toggle = 1 - toggle;
      }

      // Timer2 channel 3 is pwm generation.
#if TEST
      TIM2->CCR3 = pwmOnTime;
      if (toggle) {
          pwmOnTime++;
      } else {
          pwmOnTime--;
      }
#endif

      printf("usWidth = %d", (int)usWidth);

      // Toggle PB8 - CN1 pin 10.
      if (toggle) {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
      } else {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
      }
      delay(1000);

      switch (cmFsm) {
      case INIT:            // Initialize
          canIdCounter = 0;
          cmFsm = CANID;
          break;
      case CANID:           // Read PWM and identify myself.
          canIdCounter++;
          if (canIdCounter > SECS(4)) {
              cmFsm = CANVALVECMD;

              float duty = avgWidth / htim2.Init.Period;

              if (duty >= 89.0f && duty < 100.0f) {
                  myCANAddress = canAddresses[0];
                  TIM2->CCR3 = (uint32_t)(0.8f * (float)htim2.Init.Period);
              } else if (duty > 79.0f && duty < 89.0f) {
                  myCANAddress = canAddresses[1];
                  TIM2->CCR3 = (uint32_t)(0.7f * (float)htim2.Init.Period);
              } else if (duty > 69.0f && duty < 79.0f) {
                  myCANAddress = canAddresses[2];
                  TIM2->CCR3 = (uint32_t)(0.6f * (float)htim2.Init.Period);
              } else if (duty > 59.0f && duty < 69.0f) {
                  myCANAddress = canAddresses[3];
                  TIM2->CCR3 = (uint32_t)(0.5f * (float)htim2.Init.Period);
              } else if (duty > 49.0f && duty < 59.0f) {
                  myCANAddress = canAddresses[4];
                  TIM2->CCR3 = (uint32_t)(0.4f * (float)htim2.Init.Period);
              } else if (duty > 39.0f && duty < 49.0f) {
                  myCANAddress = canAddresses[5];
                  TIM2->CCR3 = (uint32_t)(0.3f * (float)htim2.Init.Period);
              } else if (duty > 29.0f && duty < 39.0f) {
                  myCANAddress = canAddresses[6];
                  TIM2->CCR3 = (uint32_t)(0.2f * (float)htim2.Init.Period);
              } else if (duty > 19.0f && duty < 29.0f) {
                  myCANAddress = canAddresses[7];
                  TIM2->CCR3 = (uint32_t)(0.1f * (float)htim2.Init.Period);
              } else {
                  cmFsm = INIT;
                  TIM2->CCR3 = (uint32_t)(0.95f * (float)htim2.Init.Period);
              }
          }

          // IMPORTANT : Jump state for testing only
          cmFsm = CANVALVECMD;
          break;
      case CANVALVECMD:     // Read CAN bus valve commands and act upon then.
          cmFsm = CANPRESSUREREAD;
          break;
      case CANPRESSUREREAD: // Read pressure.
          cmFsm = CANHEIGHTREAD;
          break;
      case CANHEIGHTREAD:   // Read height.
          cmFsm = TEMPREAD;
          break;
      case TEMPREAD:        // Read temperature.
          cmFsm = READADC;
          break;
      case READADC:         // Read ADC.
          cmFsm = CANSTATUSREPORT;
          break;
      case CANSTATUSREPORT:  // Report status.
          channelSelect(1);
          HAL_ADC_Start(&hadc1);
          HAL_ADC_PollForConversion(&hadc1, 1000);
          adcVal[0] = HAL_ADC_GetValue(&hadc1);
          HAL_ADC_Stop(&hadc1);

          channelSelect(2);
          HAL_ADC_Start(&hadc1);
          HAL_ADC_PollForConversion(&hadc1, 1000);
          adcVal[1] = HAL_ADC_GetValue(&hadc1);
          HAL_ADC_Stop(&hadc1);

          channelSelect(3);
          HAL_ADC_Start(&hadc1);
          HAL_ADC_PollForConversion(&hadc1, 1000);
          adcVal[2] = HAL_ADC_GetValue(&hadc1);
          HAL_ADC_Stop(&hadc1);

          adcVoltage[0] = 3.3f * ((float)adcVal[0] / 4095.0f);
          adcVoltage[1] = 3.3f * ((float)adcVal[1] / 4095.0f);
          adcVoltage[2] = 3.3f * ((float)adcVal[2] / 4095.0f);

          cmFsm = CANVALVECMD;
          break;
      default:
          break;
      }



      //HAL_Delay(10);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  // Manually copy this from the IOC generated code!!

  // Common config
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }



#if 0
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
#endif
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 40;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_3TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
