/**
 ******************************************************************************
 * @file    servo.c
 * @brief   舵机 PWM 驱动实现
 *
 *  硬件映射:
 *    TIM4_CH1 → PD12 (AF2) → SERVO_1
 *    TIM4_CH2 → PD13 (AF2) → SERVO_2
 *    TIM3_CH1 → PC6  (AF2) → SERVO_3
 *    TIM3_CH2 → PC7  (AF2) → SERVO_4
 *    TIM3_CH3 → PC8  (AF2) → SERVO_5
 *
 *  时钟参数 (STM32F407, APB1=42MHz, TIMCLK=84MHz):
 *    PSC = 84-1   → 定时器计数频率 1MHz (1us/tick)
 *    ARR = 20000-1 → PWM 周期 20ms (50Hz)
 *    CCR 范围: 500~2500 → 脉宽 0.5ms~2.5ms (0°~180°)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "servo.h"
#include "main.h"
/* 定时器句柄 ----------------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* 每个舵机的通道配置缓存 (用于快速更新CCR) --------------------------------*/
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} ServoChannel_t;

static const ServoChannel_t servo_channels[SERVO_COUNT] = {
    [SERVO_1] = { &htim4, TIM_CHANNEL_1 },   /* PD12 */
    [SERVO_2] = { &htim4, TIM_CHANNEL_2 },   /* PD13 */
    [SERVO_3] = { &htim3, TIM_CHANNEL_1 },   /* PC6  */
    [SERVO_4] = { &htim3, TIM_CHANNEL_2 },   /* PC7  */
    [SERVO_5] = { &htim3, TIM_CHANNEL_3 },   /* PC8  */
};

/* 私有函数声明 --------------------------------------------------------------*/
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* ========================================================================== */
/*                         TIM3 PWM MSP 初始化                                */
/* ========================================================================== */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        /* 使能 TIM3 时钟 */
        __HAL_RCC_TIM3_CLK_ENABLE();

        /* 配置 PC6/PC7/PC8 为 AF2 (TIM3_CH1/CH2/CH3) */
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;

        /* PC6 — TIM3_CH1 */
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* PC7 — TIM3_CH2 */
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* PC8 — TIM3_CH3 */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
    else if (htim->Instance == TIM4)
    {
        /* 使能 TIM4 时钟 */
        __HAL_RCC_TIM4_CLK_ENABLE();

        /* 配置 PD12/PD13 为 AF2 (TIM4_CH1/CH2) */
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;

        /* PD12 — TIM4_CH1 */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* PD13 — TIM4_CH2 */
        GPIO_InitStruct.Pin = GPIO_PIN_13;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

/* ========================================================================== */
/*                       TIM3 初始化 (CH1/CH2/CH3)                            */
/* ========================================================================== */
static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 84 - 1;           /* 84MHz / 84 = 1MHz     */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 20000 - 1;         /* 1MHz / 20000 = 50Hz   */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置 PWM 通道 (3个通道统一配置) */
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 1500;                      /* 初始脉宽 1.5ms (90°)  */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    /* CH1: PC6 */
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /* CH2: PC7 */
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    /* CH3: PC8 */
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================== */
/*                       TIM4 初始化 (CH1/CH2)                                */
/* ========================================================================== */
static void MX_TIM4_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 84 - 1;           /* 84MHz / 84 = 1MHz     */
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 20000 - 1;         /* 1MHz / 20000 = 50Hz   */
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置 PWM 通道 (2个通道统一配置) */
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 1500;                      /* 初始脉宽 1.5ms (90°)  */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

    /* CH1: PD12 */
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /* CH2: PD13 */
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================== */
/*                          公开 API                                          */
/* ========================================================================== */

/**
 * @brief  初始化全部 5 路舵机 PWM 输出
 * @note   调用后所有舵机输出 1.5ms 脉宽 (90° 中立位置)
 *         需在 MX_GPIO_Init() 之后调用
 */
void Servo_Init(void)
{
    MX_TIM3_Init();   /* PC6/PC7/PC8 */
    MX_TIM4_Init();   /* PD12/PD13   */

    /* 启动全部 5 路 PWM */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   /* PD12 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);   /* PD13 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* PC6  */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* PC7  */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   /* PC8  */
}

/**
 * @brief  设置舵机角度
 * @param  id        舵机编号 (SERVO_1 ~ SERVO_5)
 * @param  angle_deg 目标角度 (0.0° ~ 180.0°, 自动限幅)
 *
 *  脉宽映射: 0.5ms=0°  1.5ms=90°  2.5ms=180°
 *            pulse = 500 + angle × (2000/180)
 */
void Servo_SetAngle(ServoID id, float angle_deg)
{
    uint16_t pulse_us;

    if (id >= SERVO_COUNT) return;

    /* 限幅 */
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;

    /* 角度 → 脉宽: 0°=500us, 180°=2500us */
    pulse_us = (uint16_t)(SERVO_PULSE_MIN_US +
                (angle_deg / 180.0f) * 2000.0f);

    Servo_SetPulseUs(id, pulse_us);
}

/**
 * @brief  直接设置 PWM 脉宽
 * @param  id       舵机编号 (SERVO_1 ~ SERVO_5)
 * @param  pulse_us 脉宽 (us), 自动限幅到 500~2500
 */
void Servo_SetPulseUs(ServoID id, uint16_t pulse_us)
{
    if (id >= SERVO_COUNT) return;

    /* 限幅 */
    if (pulse_us < SERVO_PULSE_MIN_US)  pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > SERVO_PULSE_MAX_US)  pulse_us = SERVO_PULSE_MAX_US;

    /* 脉宽(us) 即 CCR 值 (因为定时器时钟 = 1MHz = 1us/tick) */
    __HAL_TIM_SET_COMPARE(servo_channels[id].htim,
                          servo_channels[id].channel,
                          pulse_us);
}

/**
 * @brief  停止单路舵机 PWM 输出
 * @param  id  舵机编号 (SERVO_1 ~ SERVO_5)
 */
void Servo_Stop(ServoID id)
{
    if (id >= SERVO_COUNT) return;

    HAL_TIM_PWM_Stop(servo_channels[id].htim,
                     servo_channels[id].channel);
}

/**
 * @brief  停止全部 5 路舵机 PWM 输出
 */
void Servo_StopAll(void)
{
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}
