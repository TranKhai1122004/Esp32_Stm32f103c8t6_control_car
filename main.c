/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Mã điều khiển hai động cơ DC qua L298N, nhận lệnh từ ESP32 qua UART1.
  * @note           : Sử dụng PA8 (TIM1_CH1) và PB4 (TIM3_CH1) cho PWM.
  * Sử dụng PB0, PB1, PA11, PA12 cho chân Direction.
  * Sử dụng PA10 (RX) cho UART1 để nhận lệnh.
  * Cần #include <stdlib.h> cho hàm atoi().
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h> // BỔ SUNG: Cần cho hàm atoi() để chuyển chuỗi tốc độ sang số
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Định nghĩa trạng thái nhận UART
typedef enum {
    UART_STATE_IDLE,
    UART_STATE_RECEIVING_SPEED
} UART_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_CODE_LENGTH 4     // 3 ký tự số (ví dụ: '255') + 1 null terminator
#define PWM_MAX 255             // PWM Resolution (0-255)

// Định nghĩa Chân Direction
// Motor 1 - Bên Phải (Giả sử)
#define M1_IN1_PORT GPIOB
#define M1_IN1_PIN  GPIO_PIN_0
#define M1_IN2_PORT GPIOB
#define M1_IN2_PIN  GPIO_PIN_1

// Motor 2 - Bên Trái (Giả sử)
#define M2_IN3_PORT GPIOA
#define M2_IN3_PIN  GPIO_PIN_11
#define M2_IN4_PORT GPIOA
#define M2_IN4_PIN  GPIO_PIN_12
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
// Buffer chứa 3 ký tự số + 1 null terminator
uint8_t uartRxBuffer[SPEED_CODE_LENGTH];
uint8_t rxIndex = 0;
UART_State_t uartState = UART_STATE_IDLE;

// Biến điều khiển động cơ
volatile uint8_t currentSpeed = 0;    // Tốc độ hiện tại (0-255)
volatile char currentDirection = 'S'; // 'F', 'B', 'L', 'R', 'S'
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void setMotorSpeed(uint8_t speed);
void setCarMotion(char direction, uint8_t speed);
void ProcessUartData(uint8_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Thiết lập tốc độ PWM cho cả hai động cơ.
  */
void setMotorSpeed(uint8_t speed)
{
    // Đảm bảo tốc độ không vượt quá giới hạn PWM_MAX
    if (speed > PWM_MAX) speed = PWM_MAX;

    // Motor 1 (PA8 / TIM1_CH1)
    __HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, speed);
    // Motor 2 (PB4 / TIM3_CH1)
    __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_1, speed);

    currentSpeed = speed;
}

/**
  * @brief  Thiết lập hướng di chuyển cho xe.
  */
void setCarMotion(char direction, uint8_t speed)
{
    // Cập nhật biến trạng thái hiện tại
    currentDirection = direction;

    // Thiết lập PWM trước (hoặc sau)
    setMotorSpeed(speed);

    // Nếu lệnh là dừng, không cần đặt chân direction
    if (direction == 'S') {
        // setMotorSpeed(0) đã được gọi ở trên.
        // Motor dừng.
        HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, RESET);
        HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, RESET);
        HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, RESET);
        HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, RESET);
        return;
    }

    // Đặt lại chân Direction về LOW trước khi thiết lập
    HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, RESET);
    HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, RESET);
    HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, RESET);
    HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, RESET);

    switch (direction) {
        case 'F': // Tiến (Forward) - Cả hai motor quay tiến
            HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, SET); // M1 (Phải) Tiến
            HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN4_PIN, SET); // M2 (Trái) Tiến
            break;

        case 'B': // Lùi (Backward) - Cả hai motor quay lùi
            HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN1_PIN, SET); // M1 (Phải) Lùi
            HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN3_PIN, SET); // M2 (Trái) Lùi
            break;

        case 'L': // Rẽ Trái (Turn Left) - Quay tại chỗ: Phải Tiến, Trái Lùi
            HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, SET); // M1 (Phải) Tiến
            HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, SET); // M2 (Trái) Lùi
            break;

        case 'R': // Rẽ Phải (Turn Right) - Quay tại chỗ: Trái Tiến, Phải Lùi
            HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, SET); // M1 (Phải) Lùi
            HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, SET); // M2 (Trái) Tiến
            break;

        default:
            // Lệnh không hợp lệ, đảm bảo motor dừng
            setMotorSpeed(0);
            break;
    }
}

/**
  * @brief  Xử lý dữ liệu nhận được từ UART.
  * Lệnh: 'F', 'B', 'L', 'R', 'S' (Direction/Stop) hoặc 's' + 'xxx' (Speed)
  */
void ProcessUartData(uint8_t data)
{
    if (uartState == UART_STATE_IDLE)
    {
        if (data == 'F' || data == 'B' || data == 'L' || data == 'R' || data == 'S') {
            // Lệnh Hướng/Dừng: Giữ tốc độ hiện tại (currentSpeed)
            setCarMotion(data, currentSpeed);
        } else if (data == 's') { // Nhận 's' nhỏ để báo hiệu lệnh Speed sắp đến
            uartState = UART_STATE_RECEIVING_SPEED;
            rxIndex = 0;
        }
    }
    else if (uartState == UART_STATE_RECEIVING_SPEED)
    {
        // Nhận 3 ký tự số
        if (rxIndex < SPEED_CODE_LENGTH - 1)
        {
            uartRxBuffer[rxIndex++] = data;
        }

        // Sau khi nhận đủ 3 ký tự số (rxIndex = 3)
        if (rxIndex == SPEED_CODE_LENGTH - 1)
        {
            // Thêm null terminator vào cuối buffer
            uartRxBuffer[rxIndex] = '\0';

            // Chuyển chuỗi số sang int
            int newSpeed = atoi((char*)uartRxBuffer);

            // Áp dụng tốc độ và giữ nguyên hướng hiện tại
            // Lệnh tốc độ chỉ thay đổi currentSpeed, không thay đổi direction
            setCarMotion(currentDirection, (uint8_t)newSpeed);

            // Reset trạng thái
            uartState = UART_STATE_IDLE;
            rxIndex = 0;
        }
    }
}

// Xử lý ngắt nhận hoàn tất (Interrupt Callback)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        ProcessUartData(receivedData);
        // Kích hoạt lại chế độ nhận ngắt cho lần sau
        HAL_UART_Receive_IT(&huart1, &receivedData, 1);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Khởi động PWM
  HAL_TIM_PWM_Start(&htim1 , TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_1);

  // Khởi động nhận ngắt UART
  HAL_UART_Receive_IT(&huart1, &receivedData, 1);

  // Dừng động cơ ở trạng thái khởi động
  setCarMotion('S', 0); // Ban đầu currentSpeed = 0, nên motor dừng

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Đảm bảo không có thêm logic trong vòng lặp vô tận, chỉ chờ ngắt
    HAL_Delay(10);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

