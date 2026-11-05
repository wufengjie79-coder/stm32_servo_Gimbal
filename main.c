/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "ssd1306.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0xBB

#define FRAME_LEN     10
#define MAX_FRAME_LEN 20

/* ==================== 舵机 & PID ==================== */

/* SG90 脉宽 -> CCR 换算（100kHz 计数，1 计数 = 10us）
 * 0.5ms = 50    1.5ms = 150    2.5ms = 250   */
#define SERVO_CCR_MIN    50
#define SERVO_CCR_MAX    250

/* PID 固定运行周期。20ms = 50Hz，和舵机的 PWM 周期同步最顺。
 * 舵机转速 = SERVO_MAX_DELTA / SERVO_PERIOD_MS
 * 6° / 20ms = 300°/s */
#define SERVO_PERIOD_MS  20

/* 增量限幅：单次最多转几度 */
#define SERVO_MAX_DELTA  6.0f

/* ==================== 云台方向配置 ====================
 * ==================================================== */
#define PAN_SIGN          (-1.0f)   /* 水平 */
#define TILT_SIGN         (-1.0f)   /* 垂直 */

/* 竖直舵机机械装反时置 1；置 0 表示不反向 */
#define TILT_MOUNT_INVERT 1

/* ==================== 上电初始视角 ====================
 * 水平居中 90。
 * 垂直居中 120。
 * ==================================================== */
#define PAN_HOME_DEG      90.0f
#define TILT_HOME_DEG     120.0f

typedef struct
{
    float kp;          /* 比例 */
    float ki;          /* 积分 */
    float kd;          /* 微分 */

    float deadzone;    /* 死区：误差小于这个值 PID 输出0 */

    float e1;          /* 上一次误差 */
    float e2;          /* 上上次误差 */

    float out;         /* 当前角度（累加结果） */
    float out_min;     /* 角度下限 */
    float out_max;     /* 角度上限 */
} PID_t;

QueueHandle_t servoQueue;   /* vUartTask -> vServoTask */

PID_t pid_pan;              /* 水平 */
PID_t pid_tilt;             /* 垂直 */


QueueHandle_t ledQueue;
QueueHandle_t coordQueue;


static uint8_t  uart_rx_buf[MAX_FRAME_LEN];
static uint16_t uart_rx_len = 0;
static uint8_t  rx_ch;


typedef struct
{
    uint8_t  data[MAX_FRAME_LEN];
    uint16_t len;
} UartFrame_t;

// 解析出来的结构体x,y
typedef struct
{
    float x;
    float y;
}Data_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// 串口回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart != &huart1) return;

    // 空闲时只等帧头
    if(uart_rx_len == 0)
    {
        if(rx_ch == FRAME_HEAD)
        {
            uart_rx_buf[0] = rx_ch;
            uart_rx_len = 1;
        }
        HAL_UART_Receive_IT(&huart1, &rx_ch, 1);
        return;
    }

    // 收到帧头后，固定收满 10 字节
    uart_rx_buf[uart_rx_len++] = rx_ch;

    if(uart_rx_len >= FRAME_LEN)
    {
        if(uart_rx_buf[FRAME_LEN - 1] == FRAME_TAIL)   // 只检查第 10 字节
        {
            UartFrame_t frame;
            BaseType_t woken = pdFALSE;
            memcpy(frame.data, uart_rx_buf, FRAME_LEN);
            frame.len = FRAME_LEN;
            xQueueSendFromISR(ledQueue, &frame, &woken);
            portYIELD_FROM_ISR(woken);
        }
        uart_rx_len = 0;
    }

    HAL_UART_Receive_IT(&huart1, &rx_ch, 1);
}

// 串口出错后自动重启
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        uart_rx_len = 0;
        HAL_UART_Receive_IT(&huart1, &rx_ch, 1);
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 串口任务
void vUartTask(void *pvParameters)
{
    UartFrame_t frame;
    Data_t coord;

    (void)pvParameters;

    while(1)
    {

        if(xQueueReceive(ledQueue, &frame, portMAX_DELAY) == pdPASS)
        {
            // 10字节，开头AA，结尾BB
            if((frame.len == FRAME_LEN) &&
               (frame.data[0] == FRAME_HEAD) &&
               (frame.data[FRAME_LEN - 1] == FRAME_TAIL))
            {
                memcpy(&coord.x, &frame.data[1], 4);   // 第1~4字节
                memcpy(&coord.y, &frame.data[5], 4);   // 第5~8字节


                // 覆盖式写入，只留最新
                xQueueOverwrite(coordQueue, &coord);
                // 给舵机任务
                xQueueOverwrite(servoQueue, &coord);  
            }
        }
    }
}




// LED任务
void vLedTask(void *pvParameters)
{
    Data_t dat;
    char line[24];

    (void)pvParameters;

    ssd1306_Init(&hi2c1);
    ssd1306_Fill(Black);

    // 标签只写一次
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("X:", Font_11x18, White);

    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString("Y:", Font_11x18, White);

    // 只刷用到的 5 页
    ssd1306_UpdatePages(&hi2c1, 0, 4);

    while(1)
    {
        if(xQueueReceive(coordQueue, &dat, portMAX_DELAY) == pdPASS)
        {
            snprintf(line, sizeof(line), "%8.2f", dat.x);
            ssd1306_SetCursor(22, 0);
            ssd1306_WriteString(line, Font_11x18, White);

            snprintf(line, sizeof(line), "%8.2f", dat.y);
            ssd1306_SetCursor(22, 20);
            ssd1306_WriteString(line, Font_11x18, White);

            ssd1306_UpdatePages(&hi2c1, 0, 4);
        }
    }
}

/* ---------- 角度 -> PWM 比较值 ---------- */
static void servo_set_angle(TIM_HandleTypeDef *htim, uint32_t ch, float angle)
{
    uint32_t ccr;

    /* 限幅到机械范围，防止撞限位 */
    if(angle < 0.0f)   angle = 0.0f;
    if(angle > 180.0f) angle = 180.0f;

    /* 0度 -> 50，180度 -> 250，线性插值 */
    ccr = (uint32_t)(SERVO_CCR_MIN + angle * ((SERVO_CCR_MAX - SERVO_CCR_MIN) / 180.0f) + 0.5f);

    __HAL_TIM_SET_COMPARE(htim, ch, ccr);
}

/* ---------- 增量式 PID ---------- */
static float pid_update(PID_t *p, float err)
{
    float delta;

    /* 1. 死区：误差很小就当成 0
     *    放在最前面，是为了不让噪声进入积分项 */
    if((err > -p->deadzone) && (err < p->deadzone))
    {
        err = 0.0f;
    }

    /* 2. 增量式 PID 三项
     *    P: 误差的变化量   -> 脸移动越快，推得越猛
     *    I: 误差的绝对量   -> 把静止偏差慢慢磨掉
     *    D: 误差变化的加速度 -> 抑制冲过头 */
    delta = p->kp * (err - p->e1)
          + p->ki * err
          + p->kd * (err - 2.0f * p->e1 + p->e2);

    /* 3. 单次增量限幅，避免突然猛甩 */
    if(delta >  SERVO_MAX_DELTA) delta =  SERVO_MAX_DELTA;
    if(delta < -SERVO_MAX_DELTA) delta = -SERVO_MAX_DELTA;

    /* 4. 滚动保存历史误差 */
    p->e2 = p->e1;
    p->e1 = err;

    /* 5. 累加到输出角度 */
    p->out += delta;

    /* 6. 输出限幅
     *    同时天然抑制积分饱和：被限住后 out 不再增长 */
    if(p->out > p->out_max) p->out = p->out_max;
    if(p->out < p->out_min) p->out = p->out_min;

    return p->out;
}

/* ---------- 舵机任务 ---------- */
void vServoTask(void *pvParameters)
{
    Data_t d = { 0.0f, 0.0f };
    float  pan;
    float  tilt;
    TickType_t last_wake;

    (void)pvParameters;

    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* PA8  水平 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* PA0  垂直 */

    /* 上电初始视角：水平居中，垂直抬头。
     * 注意垂直这里也要走和循环里同一套反向映射，
     * 否则上电位置和运行时会差一个镜像。 */
    pid_pan.out  = PAN_HOME_DEG;
    pid_tilt.out = TILT_HOME_DEG;

    servo_set_angle(&htim1, TIM_CHANNEL_1, pid_pan.out);

    if(TILT_MOUNT_INVERT)
    {
        servo_set_angle(&htim2, TIM_CHANNEL_1, 180.0f - pid_tilt.out);
    }
    else
    {
        servo_set_angle(&htim2, TIM_CHANNEL_1, pid_tilt.out);
    }

    last_wake = xTaskGetTickCount();

    for(;;)
    {
        /* 只取最新目标，取不到就沿用上一次的值。
         * 关键是：PID 不再由串口触发，而是固定 20ms 跑一次，
         * 所以舵机速度与上位机帧率无关。 */
        (void)xQueuePeek(servoQueue, &d, 0);

        /* 误差乘方向系数，保证是负反馈 */
        pan  = pid_update(&pid_pan,  PAN_SIGN  * d.x);
        tilt = pid_update(&pid_tilt, TILT_SIGN * d.y);

        servo_set_angle(&htim1, TIM_CHANNEL_1, pan);

        /* 竖直舵机的机械安装方向补偿 */
        if(TILT_MOUNT_INVERT)
        {
            servo_set_angle(&htim2, TIM_CHANNEL_1, 180.0f - tilt);
        }
        else
        {
            servo_set_angle(&htim2, TIM_CHANNEL_1, tilt);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SERVO_PERIOD_MS));
    }
}


/* ============================================================
 * 业务代码写在这里。
 *
 * 任务函数原型：
 *     void vXxxTask(void *pvParameters);
 *
 * 任务必须在 main() 的 USER CODE BEGIN 2 里用 xTaskCreate() 创建，
 * 创建后由 vTaskStartScheduler() 启动调度。
 * ============================================================ */
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* 在这里创建任务*/
// 创建一个接收串口数据的任务
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);

  // 队列
  ledQueue   = xQueueCreate(3, sizeof(UartFrame_t));

  // 第二、三个队列：只能放 1 个
  coordQueue = xQueueCreate(1, sizeof(Data_t));   // 给显示任务
  servoQueue = xQueueCreate(1, sizeof(Data_t));   // 给舵机任务

  if((ledQueue == NULL) || (coordQueue == NULL) || (servoQueue == NULL))
  {
      Error_Handler();
  }

    /* ---------- PID 参数初始化 ---------- */

  /* 水平舵机
   * 固定 20ms 周期下，err=100 时 delta 约等于限幅 6°，即跑满 300°/s。
   * 增益再往上加不会更快，只会变成全速开关式抖动。 */
  pid_pan.kp       = 1.0f;
  pid_pan.ki       = 0.90f;
  pid_pan.kd       = 0.5f;
  pid_pan.deadzone = 0.05f;
  pid_pan.e1       = 0.0f;
  pid_pan.e2       = 0.0f;
  pid_pan.out      = PAN_HOME_DEG;
  pid_pan.out_min  = 10.0f;
  pid_pan.out_max  = 170.0f;

  /* 垂直舵机：增益小一点，且机械范围更窄（防止云台撞底座） */
  pid_tilt.kp       = 1.2f;
  pid_tilt.ki       = 1.0f;
  pid_tilt.kd       = 0.5f;
  pid_tilt.deadzone = 0.05f;
  pid_tilt.e1       = 0.0f;
  pid_tilt.e2       = 0.0f;
  pid_tilt.out      = TILT_HOME_DEG;
  pid_tilt.out_min  = 30.0f;
  pid_tilt.out_max  = 150.0f;

  // 串口任务优先级 3（高）
  if(xTaskCreate(vUartTask, "UART", 256, NULL, 3, NULL) != pdPASS)
  {
      Error_Handler();
  }

  /* 显示任务优先级降到 1：它刷 OLED 是阻塞 I2C，一次约 16ms。
   * 和舵机任务同优先级会让舵机等它写完，表现为反应迟钝。 */
  if(xTaskCreate(vLedTask, "LED", 384, NULL, 1, NULL) != pdPASS)
  {
      Error_Handler();
  }

    // 舵机任务：优先级 2，和显示任务同级
  if(xTaskCreate(vServoTask, "SERVO", 256, NULL, 2, NULL) != pdPASS)
  {
      Error_Handler();
  }


  HAL_UART_Receive_IT(&huart1, &rx_ch, 1);
  

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim1.Init.Prescaler = 720 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000 - 1;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72- 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500 - 1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
