/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Điều khiển xe qua UART/ESP32, kèm chức năng HC-SR04 Input Capture
  * và Dừng Khẩn Cấp (Override). Đã tích hợp MSP và IRQ Handler.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    UART_STATE_IDLE,
    UART_STATE_RECEIVING_SPEED
} UART_State_t;

typedef enum {
    US_IDLE,
    US_RISING_EDGE,
    US_FALLING_EDGE
} US_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_CODE_LENGTH 4     // 3 ký tự số + 1 null terminator
#define PWM_MAX 255             // PWM Resolution (0-255)

// Chân Direction (Motor L298N)
#define M1_IN1_PORT GPIOB
#define M1_IN1_PIN  GPIO_PIN_0
#define M1_IN2_PORT GPIOB
#define M1_IN2_PIN  GPIO_PIN_1
#define M2_IN3_PORT GPIOA
#define M2_IN3_PIN  GPIO_PIN_11
#define M2_IN4_PORT GPIOA
#define M2_IN4_PIN  GPIO_PIN_12

// Chân HC-SR04
#define US_TRIG_PORT GPIOA
#define US_TRIG_PIN  GPIO_PIN_0
#define US_ECHO_CHANNEL TIM_CHANNEL_2
#define COLLISION_THRESHOLD 15 // Ngưỡng dừng khẩn cấp (cm)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t receivedData = 0;
uint8_t uartRxBuffer[SPEED_CODE_LENGTH];
uint8_t rxIndex = 0;
UART_State_t uartState = UART_STATE_IDLE;

volatile uint8_t currentSpeed = 0;
volatile char currentDirection = 'S';
volatile bool isOverridden = false; // Cờ ghi đè

volatile US_State_t usState = US_IDLE;
volatile uint32_t captureTime1 = 0;
volatile uint32_t captureTime2 = 0;
volatile uint32_t pulseDuration = 0;
volatile float distance_cm = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);

// Bổ sung IRQ Handler và MSP vào file main.c
void TIM2_IRQHandler(void);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic);

/* USER CODE BEGIN PFP */
void setMotorSpeed(uint8_t speed);
void setCarMotion(char direction, uint8_t speed);
void ProcessUartData(uint8_t data);
void HCSR04_StartMeasurement(void);
void CheckCollisionAndOverride(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setMotorSpeed(uint8_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    __HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, speed); // Motor 1
    __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_1, speed); // Motor 2
    currentSpeed = speed;
}

void setCarMotion(char direction, uint8_t speed)
{
    if (isOverridden && direction != 'S') {
        return;
    }

    currentDirection = direction;
    setMotorSpeed(speed);

    if (direction == 'S') {
        HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, RESET);
        HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, RESET);
        HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, RESET);
        HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, RESET);

        if (isOverridden) isOverridden = false;
        return;
    }

    // Reset Direction pins
    HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, RESET);
    HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, RESET);
    HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, RESET);
    HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, RESET);

    // LOGIC DIRECTION GỐC CỦA BẠN
    switch (direction) {
        case 'F': // Tiến
            HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, SET);
            HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN4_PIN, SET);
            break;

        case 'B': // Lùi
            HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN1_PIN, SET);
            HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN3_PIN, SET);
            break;

        case 'L': // Rẽ Trái (Phải Tiến, Trái Lùi)
            HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, SET);
            HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, SET);
            break;

        case 'R': // Rẽ Phải (Trái Tiến, Phải Lùi)
            HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, SET);
            HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, SET);
            break;

        default:
            setMotorSpeed(0);
            break;
    }
}

void ProcessUartData(uint8_t data)
{
    if (uartState == UART_STATE_IDLE)
    {
        if (data == 'F' || data == 'B' || data == 'L' || data == 'R' || data == 'S') {
            setCarMotion(data, currentSpeed);
        } else if (data == 's') {
            uartState = UART_STATE_RECEIVING_SPEED;
            rxIndex = 0;
        }
    }
    else if (uartState == UART_STATE_RECEIVING_SPEED)
    {
        if (rxIndex < SPEED_CODE_LENGTH - 1)
        {
            uartRxBuffer[rxIndex++] = data;
        }

        if (rxIndex == SPEED_CODE_LENGTH - 1)
        {
            uartRxBuffer[rxIndex] = '\0';
            int newSpeed = atoi((char*)uartRxBuffer);
            setCarMotion(currentDirection, (uint8_t)newSpeed);
            uartState = UART_STATE_IDLE;
            rxIndex = 0;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        ProcessUartData(receivedData);
        HAL_UART_Receive_IT(&huart1, &receivedData, 1);
    }
}

// ======================= HC-SR04 LOGIC =======================

void HCSR04_StartMeasurement(void)
{
    // Gửi xung 10us TRIG
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset Timer

    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET);
    for(int i = 0; i < 720; i++) { __NOP(); } // Delay ~10us
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);

    usState = US_IDLE;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (usState == US_IDLE)
        {
            // Cạnh lên (Rising Edge)
            captureTime1 = HAL_TIM_ReadCapturedValue(htim, US_ECHO_CHANNEL);
            usState = US_RISING_EDGE;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, US_ECHO_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (usState == US_RISING_EDGE)
        {
            // Cạnh xuống (Falling Edge)
            captureTime2 = HAL_TIM_ReadCapturedValue(htim, US_ECHO_CHANNEL);

            if (captureTime2 > captureTime1)
            {
                pulseDuration = captureTime2 - captureTime1;
            }
            else // Timer Overflow
            {
                pulseDuration = (0xFFFF - captureTime1) + captureTime2;
            }

            // distance (cm) = duration (us) / 58
            distance_cm = (float)pulseDuration / 58.0f;

            usState = US_FALLING_EDGE;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, US_ECHO_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

void CheckCollisionAndOverride(void)
{
    if (currentDirection != 'F') {
        isOverridden = false;
        return;
    }

    if (distance_cm <= COLLISION_THRESHOLD && distance_cm > 0.0f) {
        if (!isOverridden) {
            setCarMotion('S', 0);
            isOverridden = true;
        }
    }
    else if (isOverridden) {
        isOverridden = false;
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
  unsigned long lastUSMeasurement = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1 , TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_1);

  HAL_TIM_IC_Start_IT(&htim2, US_ECHO_CHANNEL); // Bắt đầu Input Capture

  HAL_UART_Receive_IT(&huart1, &receivedData, 1);

  setCarMotion('S', 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Đo HC-SR04 định kỳ (200ms)
    if (HAL_GetTick() - lastUSMeasurement > 200)
    {
        lastUSMeasurement = HAL_GetTick();
        HCSR04_StartMeasurement();
    }

    // Kiểm tra và thực hiện dừng khẩn cấp
    CheckCollisionAndOverride();

    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (GIỮ NGUYÊN)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  * @brief TIM1 Initialization Function (GIỮ NGUYÊN)
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  // Loại bỏ HAL_TIM_MspPostInit(&htim1); để tránh gọi hàm không định nghĩa
}

/**
  * @brief TIM2 Initialization Function (ĐÃ SỬA ĐỔI CHO HC-SR04 INPUT CAPTURE)
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71; // 1us tick (72MHz / 72)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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

  // Cấu hình Input Capture cho Kênh 2 (PA1/ECHO)
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function (GIỮ NGUYÊN)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // Loại bỏ HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function (GIỮ NGUYÊN)
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function (Cấu hình Chân Direction và PA0/TRIG)
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, US_TRIG_PIN|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 (Motor Direction)*/
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Cấu hình PA0 (TRIG)
  GPIO_InitStruct.Pin = US_TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 (Motor Direction)*/
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// ==========================================================
// TỰ ĐỊNH NGHĨA MSP VÀ IRQ HANDLER CHO HC-SR04 (TIM2)
// ==========================================================

/**
  * @brief  Hàm MSP cho Input Capture (Thay thế stm32f1xx_hal_msp.c)
  * @param  htim_ic: Con trỏ đến cấu trúc TIM_HandleTypeDef
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim_ic->Instance==TIM2)
    {
        /* Bật clock cho Timer 2 và GPIOA */
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* Cấu hình PA1 (ECHO) là Alternate Function Input (TIM2_CH2) */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Cấu hình Ngắt NVIC cho Timer 2 */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

/**
  * @brief This function handles TIM2 global interrupt. (Thay thế stm32f1xx_it.c)
  */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

// ==========================================================

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
