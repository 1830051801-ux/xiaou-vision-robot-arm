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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rtthread.h>
#include "app_threads.h"
#include "trajectory_planner.h"
#include "arm_kinematics.h"
#include <math.h>
#include "led.h"
#include "telemetry.h"

extern void rt_hw_board_init(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <rtdef.h>

uint8_t flag = 0;   /* 璋冭瘯鐘舵€佹爣璁?*/
extern int8_t READ_FLAG;        /* DrEmpower_can.c */
extern void send_command(uint8_t id_num, char cmd, unsigned char *data, uint8_t rt);
extern struct format_data_struct {
    unsigned char byte_data[40];
    float value_data[10];
} data_list;

/* ======================================================================== */
/*   CAN 鍙戦€佹祴璇曠嚎绋?鈥?鑷鍒濆鍖?CAN 骞跺懆鏈熸€у彂閫?                           */
/* ======================================================================== */
static rt_uint8_t  can_test_stack[1024];
static struct rt_thread can_test_tcb;

static int can_self_init(void)
{
    uint32_t timeout;

    /* ====== 鍏ㄧ▼鎵嬪姩鍒濆鍖?CAN1, 缁曡繃 HAL ====== */

    /* 1. 澶嶄綅 CAN1 澶栬 */
    __HAL_RCC_CAN1_FORCE_RESET();
    for (volatile int d = 0; d < 100; d++) __NOP();
    __HAL_RCC_CAN1_RELEASE_RESET();
    for (volatile int d = 0; d < 100; d++) __NOP();

    /* 2. 浣胯兘 CAN1 鏃堕挓 */
    __HAL_RCC_CAN1_CLK_ENABLE();

    /* 3. 閰嶇疆 GPIO: PA11=RX, PA12=TX (涓?123 椤圭洰 F103 涓€鑷? */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_11;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;           /* 涓嶄笂鎷? 鍖归厤 F103 鎴愬姛浠ｇ爜 */
        g.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOA, &g);

        g.Pin = GPIO_PIN_12;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOA, &g);
    }

    /* 3.5 璇婃柇: CAN 鏃堕挓 */
    rt_kprintf("[CAN] HSE=%s APB1ENR=0x%08lX\n",
               (RCC->CR & RCC_CR_HSERDY) ? "OK" : "OFF",
               RCC->APB1ENR);

    /* 4. 杩涘叆 init 妯″紡, 鍚屾椂璁?NART=1 (鍗曟鍙戦€? 涓嶉噸璇?涓嶇瓑ACK) */
    CAN1->MCR = CAN_MCR_INRQ             /* 璇锋眰杩涘叆 init 妯″紡 */
              | CAN_MCR_NART;            /* 鍗曟鍙戦€佹ā寮? 鏃犻渶ACK */
    __DSB();
    timeout = 0;
    while (((CAN1->MSR & CAN_MSR_INAK) == 0) && (timeout < 10000000U)) timeout++;
    if ((CAN1->MSR & CAN_MSR_INAK) == 0)
    {
        rt_kprintf("[CAN] Cannot enter init! MSR=0x%08lX MCR=0x%08lX\n", CAN1->MSR, CAN1->MCR);
        return -1;
    }
    rt_kprintf("[CAN] Enter init OK, MSR=0x%08lX MCR=0x%08lX\n", CAN1->MSR, CAN1->MCR);

    /* 5. 閰嶇疆鏃跺簭: 42MHz/(7脳6)=1Mbps */
    CAN1->BTR = (6U << 0)                /* BRP=6 鈫?prescaler=7 */
              | (3U << 16)               /* TS1=3 鈫?BS1=4TQ */
              | (0U << 20)               /* TS2=0 鈫?BS2=1TQ */
              | (0U << 24);              /* SJW=0 鈫?1TQ, 鎬?TQ=1渭s */
    rt_kprintf("[CAN] BTR=0x%08lX (1Mbps)\n", CAN1->BTR);

    /* 5.5 纭 PA11 瀹為檯鐢靛钩 */
    rt_kprintf("[CAN] PA11=%lu (0=dom/1=rec)\n",
               (GPIOA->IDR & GPIO_PIN_11) ? 1UL : 0UL);

    /* 6. 閫€鍑?init 鈥?娓?INRQ, 淇濈暀 NART */
    CAN1->MCR = CAN_MCR_NART;            /* INRQ=0 閫€鍑篿nit, NART=1 鍗曟鍙戦€?*/
    __DSB();
    timeout = 0;
    while (((CAN1->MSR & CAN_MSR_INAK) != 0) && (timeout < 100000000U)) timeout++;
    if ((CAN1->MSR & CAN_MSR_INAK) != 0)
    {
        rt_kprintf("[CAN] Cannot exit init! MSR=0x%08lX MCR=0x%08lX ESR=0x%08lX\n",
                   CAN1->MSR, CAN1->MCR, CAN1->ESR);
        if (CAN1->ESR & (1U << 7))
            rt_kprintf("[CAN] -> ESR shows BUS-OFF!\n");
        if (CAN1->ESR & (1U << 4))
            rt_kprintf("[CAN] -> ESR shows LEC error: 0x%02lX\n",
                       (CAN1->ESR >> 4) & 0x7U);
        return -1;
    }
    rt_kprintf("[CAN] Exit init OK, MSR=0x%08lX\n", CAN1->MSR);

    /* 7. 閰嶇疆婊ゆ尝鍣? 鎺ユ敹鍏ㄩ儴 */
    CAN1->FMR |= CAN_FMR_FINIT;          /* 婊ゆ尝鍣ㄥ垵濮嬪寲妯″紡 */
    CAN1->FA1R &= ~(1U << 0);            /* Filter 0 绂佺敤 */
    CAN1->FS1R |= (1U << 0);             /* Filter 0 = 32-bit scale */
    CAN1->FM1R &= ~(1U << 0);            /* Filter 0 = mask mode */
    CAN1->FFA1R &= ~(1U << 0);           /* Filter 0 鈫?FIFO0 */
    CAN1->sFilterRegister[0].FR1 = 0;    /* ID + Mask = 0 (鍖归厤鎵€鏈? */
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FA1R |= (1U << 0);             /* Filter 0 鍚敤 */
    CAN1->FMR &= ~CAN_FMR_FINIT;         /* 閫€鍑烘护娉㈠櫒鍒濆鍖?*/
    rt_kprintf("[CAN] Filter configured\n");

    /* 8. 浣胯兘 CAN RX 涓柇 (NVIC + 澶栬) */
    SET_BIT(CAN1->IER, CAN_IER_FMPIE0);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    /* 9. 鍚屾 HAL 鍙ユ焺鐘舵€?(Can_Send_Msg 渚濊禆) */
    hcan1.Instance = CAN1;
    hcan1.State = HAL_CAN_STATE_LISTENING;

    rt_kprintf("[CAN] Init OK, MCR=0x%08lX TSR=0x%08lX ESR=0x%08lX\n",
               CAN1->MCR, CAN1->TSR, CAN1->ESR);
    return 0;
}

static void can_test_thread(void *parameter)
{
    uint8_t test_id = 1;          /* 娴嬭瘯鍏宠妭 ID */
    uint8_t step   = 0;

    rt_thread_mdelay(500);
    rt_kprintf("\n===== CAN Single Servo Test =====\n");

    /* 鍒濆鍖?CAN */
    flag = 2;
    if (can_self_init() != 0)
    {
        flag = 4;
        rt_kprintf("[CAN] Init failed\n");
        while (1) { rt_thread_mdelay(1000); }
    }
    flag = 3;

    /* 鍏堟€ュ仠 */
    estop(0);
    rt_thread_mdelay(200);
    rt_kprintf("[CAN] Ready, test_id=%d\n", test_id); 

    /* 绌洪棽, 绛夊緟鍗忚绾跨▼椹卞姩 */
    while (1)
    {
        rt_thread_mdelay(1000);
    }
}

/* ======================================================================== */
/*   鑸垫満娴嬭瘯绾跨▼ (PC6=SERVO_3, 鏁板瓧鑸垫満 20ms鍛ㄦ湡 0-180掳)                     */
/* ======================================================================== */
static rt_uint8_t  servo_test_stack[512];
static struct rt_thread servo_test_tcb;

static void servo_test_thread(void *parameter)
{
    rt_thread_mdelay(1000);
    rt_kprintf("[SERVO] Init all...\n");
    Servo_Init();                              /* 5璺叏閮ㄥ垵濮嬪寲 */
    rt_kprintf("[SERVO] GPIOC_MODER=0x%08lX (PC6=%lu, 2=AF)\n",
               GPIOC->MODER, (GPIOC->MODER >> 12) & 3UL);
    /* 鍙繚鐣橮C6, 褰诲簳鍏虫帀鍏朵粬4璺?*/
    Servo_Stop(SERVO_1);
    Servo_Stop(SERVO_2);
    Servo_Stop(SERVO_4);
    Servo_Stop(SERVO_5);
    TIM4->CR1  &= ~TIM_CR1_CEN;              /* TIM4 璁℃暟鍣ㄥ仠鎺?*/
    TIM3->CCR2  = 0;                         /* CH2 姣旇緝鍊煎綊闆?*/
    TIM3->CCR3  = 0;                         /* CH3 姣旇緝鍊煎綊闆?*/
    /* PC7/PC8 鏀瑰洖妯℃嫙妯″紡, 闃叉娴姩寮曡剼鑰﹀悎鍣０ */
    {   GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_7 | GPIO_PIN_8;
        g.Mode = GPIO_MODE_ANALOG;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &g); }
    /* PD12/PD13 涔熸敼鍥炴ā鎷?*/
    {   GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_12 | GPIO_PIN_13;
        g.Mode = GPIO_MODE_ANALOG;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &g); }
    rt_kprintf("[SERVO] Unused pins -> ANALOG\n");

    /* 璇婃柇: TIM3 瀵勫瓨鍣?(CEN=bit0, CC1E=bit0) */
    rt_kprintf("[SERVO] TIM3_CR1=0x%04lX CEN=%lu  CCER=0x%04lX CC1E=%lu  CCR1=%lu\n",
               TIM3->CR1, (TIM3->CR1 & 1UL),
               TIM3->CCER, (TIM3->CCER & 1UL),
               __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1));

    /* 澶圭埅娴嬭瘯: 寮犫啋鍚堚啋鍗婂紑 */
   // rt_kprintf("[SERVO] 寮犲紑(180deg)...\n"); Servo_SetAngle(SERVO_3, 120.0f); rt_thread_mdelay(2000);
   // rt_kprintf("[SERVO] 闂悎(0deg)...\n");   Servo_SetAngle(SERVO_3, 60.0f);   rt_thread_mdelay(2000);
    rt_kprintf("[SERVO] 鍗婂紑(90deg)...\n");  Servo_SetAngle(SERVO_3, 30.0f);rt_thread_mdelay(2000);
 //   rt_kprintf("[SERVO] 鍗婂紑(90deg)...\n");  Servo_SetAngle(SERVO_3, 00.0f);rt_thread_mdelay(2000);

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}

/* ======================================================================== */
/*   鍗忚娴嬭瘯绾跨▼ (USART2 RX 鈫?甯цВ鏋?鈫?妯℃嫙鍔ㄤ綔鏁版嵁鎵撳嵃)                       */
/*   PC绔彂 HEX 甯? AA CMD LEN SEQ PAYLOAD... CRCLO CRCHI 55               */
/*   CRC 濉?0xFF 0xFF 鍙烦杩囨牎楠?                                            */
/* ======================================================================== */
static rt_uint8_t  proto_test_stack[2048];
static struct rt_thread proto_test_tcb;
static struct proto_parser proto_parser;

static const char *cmd_name(uint8_t cmd)
{
    switch (cmd) {
    case CMD_PING:          return "PING";
    case CMD_ESTOP:         return "ESTOP";
    case CMD_MOVE_JOINT:    return "MOVE_JOINT";
    case CMD_MOVE_JOINTS:   return "MOVE_JOINTS";
    case CMD_MOVE_CART:     return "MOVE_CART";
    case CMD_GRASP_MOVE:    return "GRASP_MOVE";
    case CMD_STOP:          return "STOP";
    case CMD_GET_STATE:     return "GET_STATE";
    case CMD_SET_ZERO:      return "SET_ZERO";
    default:                return "UNKNOWN";
    }
}

#define GRIP1   GRIP_MODE1_CLOSE
#define REL1    GRIP_MODE1_RELEASE
#define GRIP2   GRIP_MODE2_CLOSE
#define REL2    GRIP_MODE2_RELEASE

static uint8_t toggle1 = 0, toggle2 = 0, raw_mon = 0, sys_print_en = 0;
static volatile uint8_t abort_flag = 0;

/* 鍙腑鏂欢鏃? 姣?0ms妫€鏌SART2, 閬?4 00鍒欑疆abort_flag */
static void delay_ms_abort(uint32_t ms)
{
    for (uint32_t t = 0; t < ms && !abort_flag; t += 50) {
        uint32_t d = (ms - t) < 50 ? (ms - t) : 50;
        rt_thread_mdelay(d);
        while (Dbg_Uart_Available()) {
            uint8_t c = (uint8_t)Dbg_Uart_GetChar();
            if (c == 0x13 || c == 0x14) { abort_flag = 1; estop(0); return; }
        }
    }
}

/* ======================================================================== */
/*   璋冭瘯鍛戒护瑙ｆ瀽: [CMD(1B) SIZE(1B) PAYLOAD(SIZE bytes)]                      */
/*   鐢?USART2 鎺ユ敹, 鐩存帴鍦?proto 绾跨▼澶勭悊                                      */
/* ======================================================================== */
/* 鍥哄畾闄愪綅 (闆剁偣鏍囧畾鍚庣殑瀹夊叏鑼冨洿) */
static const float jlim_min[7] = {0, -20,-30,-40,-180, 0,0};
static const float jlim_max[7] = {0,  20, 60, 10, 180, 0,0};
static float dlim_min[7] = {0,-20,-30,-40,-180,0,0};  /* 鍔ㄦ€侀檺浣?鍒濆=瀹夊叏鍊?*/
static float dlim_max[7] = {0, 20, 60, 10, 180,0,0};
static uint8_t dlim_lock = 1;           /* 1=閿?瀹夊叏) 0=寮€(鍔ㄦ€? */

static int check_limit(uint8_t id, int8_t ang)
{
    if (id < 1 || id > 4) return 0;
    float lo = dlim_lock ? jlim_min[id] : dlim_min[id];
    float hi = dlim_lock ? jlim_max[id] : dlim_max[id];
    if ((float)ang < lo || (float)ang > hi) {
        rt_kprintf("[LIMIT] J%d=%d out of [%d,%d]%s skipped\r\n",
                   id, ang, (int)lo, (int)hi, dlim_lock?"":"(dyn)");
        return 0;
    }
    return 1;
}

/* S鏇茬嚎闃诲鎵ц: cur鈫抰arget, 鎻愬墠overlap_ms鍒囦笅涓€娈典繚璇佹祦鐣?*/
static void s_curve_move(float cur[4], const float target[4])
{
    static struct traj_planner tp;
    traj_init(&tp);
    traj_set_joint_ptp(&tp, target, 30.0f, 60.0f, 300.0f);
    traj_start(&tp, 0, cur);
    uint32_t el = 0;
    while (!abort_flag) {
        float out[4];
        if (traj_update(&tp, el, out) != TRAJ_RUNNING) break;
        if (el % 50 == 0)
            for (int i = 0; i < 4; i++) set_angle(i+1, out[i], 40.0f, 10.0f, 0);
        rt_thread_mdelay(TRAJ_INTERPOLATION_MS);
        el += TRAJ_INTERPOLATION_MS;
    }
    if (!abort_flag) {
        rt_thread_mdelay(200);
        for (int i = 0; i < 4; i++) {
            set_angle(i+1, target[i], 20.0f, 5.0f, 0);
            cur[i] = target[i];
        }
    }
    for (int i = 0; i < 4; i++) cur[i] = target[i]; /* 杩借釜鐩爣 */
}

static void grip_profile_angles(uint8_t profile, float *close_deg, float *open_deg)
{
    switch (profile) {
    case 2:
        *close_deg = GRIP_MODE2_CLOSE;
        *open_deg = GRIP_MODE2_RELEASE;
        break;
    case 3:
        *close_deg = GRIP_MODE3_CLOSE;
        *open_deg = GRIP_MODE3_RELEASE;
        break;
    case 4:
        *close_deg = GRIP_MODE4_CLOSE;
        *open_deg = GRIP_MODE4_RELEASE;
        break;
    default:
        *close_deg = GRIP_MODE1_CLOSE;
        *open_deg = GRIP_MODE1_RELEASE;
        break;
    }
}

static int execute_coordinate_grasp(float px, float py, float dx, float dy, uint8_t grip_profile)
{
    if (dx == 0.0f && dy == 0.0f) {
        dx = px;
        dy = py;
    }

    float pick_r = sqrtf(px * px + py * py);
    float drop_r = sqrtf(dx * dx + dy * dy);
    float j1_pick = atan2f(px, py) * 57.29578f - 90.0f;
    float j1_drop = atan2f(dx, dy) * 57.29578f - 90.0f;
    float close_deg = 0.0f;
    float open_deg = 0.0f;
    grip_profile_angles(grip_profile, &close_deg, &open_deg);

    if (pick_r < 250.0f || pick_r > 430.0f ||
        drop_r < 250.0f || drop_r > 430.0f ||
        j1_pick < -170.0f || j1_pick > 170.0f ||
        j1_drop < -170.0f || j1_drop > 170.0f) {
        rt_kprintf("[COORD] safety reject pick_r=%d drop_r=%d J1=%d/%d\r\n",
                   (int)pick_r, (int)drop_r, (int)j1_pick, (int)j1_drop);
        return -1;
    }

    rt_kprintf("[COORD] pick(%d,%d) drop(%d,%d) J1=%d/%d profile=%d\r\n",
               (int)px, (int)py, (int)dx, (int)dy,
               (int)j1_pick, (int)j1_drop, grip_profile);

    set_angle(1, j1_pick, 30, 10, 1);
    set_angle(2, 45, 30, 10, 1);
    set_angle(3, 45, 30, 10, 1);
    set_angle(4, 0, 30, 10, 1);
    delay_ms_abort(2200);
    if (abort_flag) return -2;

    set_angle(2, -11, 30, 10, 1);
    set_angle(3, -12, 30, 10, 1);
    delay_ms_abort(1500);
    if (abort_flag) return -2;

    Servo_SetAngle(SERVO_3, close_deg);
    delay_ms_abort(500);
    if (abort_flag) return -2;

    set_angle(2, 45, 30, 10, 1);
    set_angle(3, 45, 30, 10, 1);
    delay_ms_abort(1800);
    if (abort_flag) return -2;

    set_angle(1, j1_drop, 30, 10, 1);
    delay_ms_abort(1200);
    if (abort_flag) return -2;

    set_angle(2, -11, 30, 10, 1);
    set_angle(3, -12, 30, 10, 1);
    delay_ms_abort(1500);
    if (abort_flag) return -2;

    Servo_SetAngle(SERVO_3, open_deg);
    delay_ms_abort(500);
    if (abort_flag) return -2;

    set_angle(1, 0, 30, 10, 1);
    set_angle(2, 45, 30, 10, 1);
    set_angle(3, 45, 30, 10, 1);
    set_angle(4, 0, 30, 10, 1);
    delay_ms_abort(1800);
    return abort_flag ? -2 : 0;
}

static void debug_cmd_exec(uint8_t cmd, uint8_t sz, const uint8_t *pl)
{
    switch (cmd) {

    case 0x04: /* 04 08 ID1 ID2 ID3 ID4 ANG1 ANG2 ANG3 ANG4, ID=0璺宠繃 */
        if (sz >= 8) {
            rt_kprintf("[DBG] Batch: ");
            for (int i = 0; i < 4; i++) {
                uint8_t jid = pl[i];
                int8_t  ang = (int8_t)pl[i + 4];
                if (jid >= 1 && jid <= 4 && check_limit(jid, ang)) {
                    set_angle(jid, (float)ang, 30.0f, 10.0f, 1);
                    rt_kprintf("J%d=%d ", jid, ang);
                }
            }
            rt_kprintf("\r\n");
        }
        break;

    case 0x11:
    case 0x15:
    case 0x18:
    case 0x1A:
    case 0x1B:
    case 0x1C:
    case 0x1D:
    case 0x1E:
        rt_kprintf("[DBG] legacy fixed demo command disabled; use 0x14 coordinate payload.\r\n");
        break;

    case 0x13:
        estop(0);
        rt_kprintf("[ESTOP] All joints stopped\r\n");
        break;

    case 0x16: /* 16 00 寮€鍏冲姩鎬侀檺浣嶉攣 */
        dlim_lock = !dlim_lock;
        rt_kprintf("[LIMIT] Dynamic limit: %s\r\n", dlim_lock?"LOCKED(safe)":"UNLOCKED(dyn)");
        break;

    case 0x17: /* 17 03 J MIN MAX 鍔ㄦ€佽闄愪綅 */
        if (sz >= 3 && pl[0]>=1 && pl[0]<=4) {
            int8_t lo=(int8_t)pl[1], hi=(int8_t)pl[2];
            dlim_min[pl[0]]=lo; dlim_max[pl[0]]=hi;
            rt_kprintf("[LIMIT] J%d dyn=[%d,%d]\r\n", pl[0], lo, hi);
        }
        break;

    case 0x14: /* 20-byte coordinate grasp payload, or empty payload for reset */
        if (sz >= 20) {
            float px=(int16_t)(pl[0]|(pl[1]<<8))/10.0f;
            float py=(int16_t)(pl[2]|(pl[3]<<8))/10.0f;
            float dx=(int16_t)(pl[8]|(pl[9]<<8))/10.0f;
            float dy=(int16_t)(pl[10]|(pl[11]<<8))/10.0f;
            uint8_t grip_profile=pl[19];
            abort_flag = 0;
            if (execute_coordinate_grasp(px, py, dx, dy, grip_profile) == 0) {
                rt_kprintf("[COORD] debug grasp done\r\n");
            }
        } else {
            rt_kprintf("[ZERO] ESTOP+Reset\r\n");
            abort_flag = 1;
            estop(0); rt_thread_mdelay(200);
            set_angle(1,0,30,10,1); set_angle(2,0,30,10,1);
            set_angle(3,0,30,10,1); set_angle(4,0,30,10,1);
            abort_flag = 0;
        }
        break;

    case 0x12: /* 闆剁偣璁剧疆: 12 01 [id] */
        if (sz >= 1) {
            set_zero_position(pl[0]);
            rt_kprintf("[DBG] Joint %d set zero (permanent)\r\n", pl[0]);
        }
        break;

    case 0x05: /* gripper test: 05 02 [profile] [action] */
        if (sz >= 2) {
            uint8_t profile = pl[0], act = pl[1];
            float close_deg = 0.0f;
            float open_deg = 0.0f;
            grip_profile_angles(profile, &close_deg, &open_deg);
            if (act == 1) {
                Servo_SetAngle(SERVO_3, close_deg);
                rt_kprintf("[DBG] Grip profile%d CLOSE %d deg\n", profile, (int)close_deg);
            } else {
                Servo_SetAngle(SERVO_3, open_deg);
                rt_kprintf("[DBG] Grip profile%d OPEN  %d deg\n", profile, (int)open_deg);
            }
        }
        break;

    case 0x06: /* 鍗曞叧鑺傝搴? 06 02 [id] [angle] (angle=int8) */
        if (sz >= 2) {
            uint8_t id  = pl[0];
            int8_t  ang = (int8_t)pl[1];
            if (id >= 1 && id <= 7 && check_limit(id, ang)) {
                set_angle(id, (float)ang, 30.0f, 10.0f, 1);     
                rt_kprintf("[DBG] Joint%d -> %d deg\n", id, ang);
            }
        }
        break;

    case 0x07: /* 鍥炶瑙掑害: 07 01 [id] */
        if (sz >= 1) {
            uint8_t id = pl[0];
            float a = get_angle(id);
            rt_kprintf("[DBG] Joint%d angle = %d deg\n", id, (int)a);
        }
        break;

    case 0x08: /* 鍥炶鐘舵€? 08 01 [id] */
        if (sz >= 1) {
            uint8_t id = pl[0];
            struct servo_state st = get_state(id);
            rt_kprintf("[DBG] Joint%d angle=%d speed=%d\n", id, (int)st.angle, (int)st.speed);
        }
        break;

    case 0x09: /* 绯荤粺鐘舵€? 09 00 */
        sys_dump_status();
        break;

    case 0x0A: /* 鍒囨崲鍘熷鐩戞祴: 0A 00 */
        raw_mon = !raw_mon;
        rt_kprintf("[DBG] Raw monitor: %s\n", raw_mon ? "ON" : "OFF");
        break;

    case 0x0B: /* 鎬ュ仠: 0B 01 [id] */
        if (sz >= 1) estop(pl[0]);
        rt_kprintf("[DBG] ESTOP id=%d\n", pl[0]);
        break;

    case 0x0C: /* 寮€鍚懆鏈熺姸鎬佹墦鍗? 0C 00 */
        sys_print_en = 1;
        rt_kprintf("[DBG] Periodic status: ON\n");
        break;

    case 0x0D: /* 鍏抽棴鍛ㄦ湡鐘舵€佹墦鍗? 0D 00 */
        sys_print_en = 0;

		rt_kprintf("[DBG] Periodic status: OFF\n");
        break;

    case 0x0E: /* 鎵撳嵃涓€娆＄郴缁熶俊鎭? 0E 00 */
        sys_dump_status();
        break;

    case 0x0F: /* 0F 00 甯姪 */
        rt_kprintf("\r\n===== Debug Commands =====\r\n");
        rt_kprintf(" 04 08 IDs Angs        batch joint angles, ID=0 skips\r\n");
        rt_kprintf(" 05 02 P A             gripper profile P, action A=0 open / 1 close\r\n");
        rt_kprintf(" 12 01 J               set joint zero\r\n");
        rt_kprintf(" 13 00                 emergency stop all joints\r\n");
        rt_kprintf(" 14 14 payload         coordinate grasp payload, same as CMD_GRASP_MOVE\r\n");
        rt_kprintf(" 14 00                 reset four joints to zero\r\n");
        rt_kprintf(" 16 00                 toggle dynamic limits\r\n");
        rt_kprintf(" 17 03 J L H           set dynamic joint limit, L/H=int8\r\n");
        rt_kprintf(" 06 02 J D             move joint J to D degrees\r\n");
        rt_kprintf(" 07 01 J               read joint angle\r\n");
        rt_kprintf(" 08 01 J               read joint state\r\n");
        rt_kprintf(" 09 00                 print system status\r\n");
        rt_kprintf(" 0A 00                 toggle raw byte monitor\r\n");
        rt_kprintf(" 0B 01 J               emergency stop one joint\r\n");
        rt_kprintf(" 0C/0D                 periodic status on/off\r\n");
        rt_kprintf(" 0E 00                 print one system snapshot\r\n");
        rt_kprintf(" 0F 00                 help\r\n");
        rt_kprintf(" 11/15/18/1A-1E        disabled legacy demo slots\r\n");
        rt_kprintf(" 10 XX                 set motor ID\r\n");
        rt_kprintf(" 20                    read motor ID\r\n");
        rt_kprintf("====================\r\n");
        break;

    default:
        rt_kprintf("[DBG] Unknown cmd 0x%02X\n", cmd);
        break;
    }
}

static void proto_test_thread(void *parameter)
{
    struct proto_frame frame;
    uint8_t ch;

    rt_thread_mdelay(500);
    Dbg_Uart_StartRx();
    proto_init(&proto_parser);
    Servo_SetAngle(SERVO_3, REL1);
    rt_kprintf("\r\n===== Proto: 01/02=Grip 05~0F=Debug USART1=Pi =====\r\n");

    while (1)
    {
        /* ---- USART2 (璋冭瘯涓插彛): 澶圭埅/ID/璋冭瘯鍛戒护 ---- */
        while (Dbg_Uart_Available()) {
            ch = (uint8_t)Dbg_Uart_GetChar();

            /* 璋冭瘯鍛戒护: [CMD >= 0x04 && <= 0x18] */
            if (ch >= 0x04 && ch <= 0x1E) {
                uint8_t dbg_cmd = ch;
                /* 璇?SIZE */
                while (!Dbg_Uart_Available()) { rt_thread_mdelay(5); }
                uint8_t dbg_sz = (uint8_t)Dbg_Uart_GetChar();
                /* 璇?PAYLOAD */
                uint8_t dbg_pl[16];
                for (int i = 0; i < dbg_sz && i < 16; i++) {
                    while (!Dbg_Uart_Available()) { rt_thread_mdelay(5); }
                    dbg_pl[i] = (uint8_t)Dbg_Uart_GetChar();
                }
                debug_cmd_exec(dbg_cmd, dbg_sz, dbg_pl);
            } else if (ch == 0x01) {
                if (toggle1 == 0) {
                    Servo_SetAngle(SERVO_3, GRIP1);
                    rt_kprintf("[G1] GRIP  %d deg\n", (int)GRIP1);
                    toggle1 = 1;
                } else {
                    Servo_SetAngle(SERVO_3, REL1);
                    rt_kprintf("[G1] REL   %d deg\n", (int)REL1);
                    toggle1 = 0;
                }
            } else if (ch == 0x02) {
                if (toggle2 == 0) {
                    Servo_SetAngle(SERVO_3, GRIP2);
                    rt_kprintf("[G2] GRIP  %d deg\n", (int)GRIP2);
                    toggle2 = 1;
                } else {
                    Servo_SetAngle(SERVO_3, REL2);
                    rt_kprintf("[G2] REL   %d deg\n", (int)REL2);
                    toggle2 = 0;
                }
            } else if (ch == 0x20) {
                /* 鎵嬪姩鍙?read-property 鏌ヨ, 杞纭欢FIFO */
                float vd[3] = {31001, 3, 0};  int td[3] = {1,1,3};
                format_data(vd, td, 3, "encode");
                rt_kprintf("[ID] query bytes: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                           data_list.byte_data[0],data_list.byte_data[1],data_list.byte_data[2],data_list.byte_data[3],
                           data_list.byte_data[4],data_list.byte_data[5],data_list.byte_data[6],data_list.byte_data[7]);
                uint8_t tx_ret = Can_Send_Msg((0 << 5) + 0x1E, 8, data_list.byte_data);
                rt_kprintf("[ID] TX ret=%d\n", tx_ret);
                rt_thread_mdelay(20);
                uint32_t fmp = CAN1->RF0R & 3UL;
                rt_kprintf("[ID] After TX: RF0R FMP=%lu ESR=0x%08lX\n",
                           fmp, CAN1->ESR);
                if (fmp > 0) {
                    CAN_RxHeaderTypeDef rh;
                    uint8_t buf[8];
                    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rh, buf);
                    rt_kprintf("[ID] Got frame ID=0x%03X DLC=%d data=%02X%02X%02X%02X%02X%02X%02X%02X\n",
                               rh.StdId, rh.DLC, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
                } else {
                    /* 鍐嶈瘯 id=1 */
                    tx_ret = Can_Send_Msg((1 << 5) + 0x1E, 8, data_list.byte_data);
                    rt_thread_mdelay(20);
                    fmp = CAN1->RF0R & 3UL;
                    rt_kprintf("[ID] id=1 TX ret=%d FMP=%lu\n", tx_ret, fmp);
                }
            } else if (ch == 0x10) {
                while (!Dbg_Uart_Available()) { rt_thread_mdelay(5); }
                uint8_t new_id = (uint8_t)Dbg_Uart_GetChar();
                if (new_id >= 1 && new_id <= 254) {
                    rt_kprintf("[ID] Setting ID -> %d ...\n", new_id);
                    set_id(0, new_id);
                    rt_thread_mdelay(500);
                    uint8_t verify = get_id(0);
                    rt_kprintf("[ID] Verify: %d  %s\n", verify,
                               (verify == new_id) ? "OK!" : "FAIL!");
                }
            } else if (ch > 180) {
                proto_rx_byte(&proto_parser, ch);
            }
        }

        /* ---- USART1 (鏍戣帗娲?: 鍘熷瀛楄妭鎵撳嵃 + 鍠傚崗璁В鏋愬櫒 ---- */
        while (Rpi_Uart_Available()) {
            ch = (uint8_t)Rpi_Uart_GetChar();
            if (raw_mon) rt_kprintf("[Pi RAW] 0x%02X\n", ch);
            proto_rx_byte(&proto_parser, ch);
        }

        /* ---- 鍗忚甯цВ鏋?---- */
        if (proto_try_parse(&proto_parser, &frame) && frame.valid) {
            rt_kprintf("[Pi->MCU] CMD=0x%02X(%s)", frame.cmd, cmd_name(frame.cmd));
            if (frame.cmd == CMD_MOVE_CART && frame.len >= 8) {
                int16_t x = (int16_t)(frame.payload[0] | (frame.payload[1] << 8));
                int16_t y = (int16_t)(frame.payload[2] | (frame.payload[3] << 8));
                int16_t z = (int16_t)(frame.payload[4] | (frame.payload[5] << 8));
                int16_t yaw = (int16_t)(frame.payload[6] | (frame.payload[7] << 8));
                rt_kprintf(" -> X=%d.%d Y=%d.%d Z=%d.%d Yaw=%d.%d",
                           x / 10, (x % 10 + 10) % 10,
                           y / 10, (y % 10 + 10) % 10,
                           z / 10, (z % 10 + 10) % 10,
                           yaw / 10, (yaw % 10 + 10) % 10);
            }
            if (frame.cmd == CMD_GRASP_MOVE && frame.len >= 20) {
                float px = (int16_t)(frame.payload[0]  | (frame.payload[1]  << 8)) / 10.0f;
                float py = (int16_t)(frame.payload[2]  | (frame.payload[3]  << 8)) / 10.0f;
                float dx = (int16_t)(frame.payload[8]  | (frame.payload[9]  << 8)) / 10.0f;
                float dy = (int16_t)(frame.payload[10] | (frame.payload[11] << 8)) / 10.0f;
                uint8_t grip_profile = frame.payload[19];

                rt_kprintf(" -> coordinate grasp");
                abort_flag = 0;
                g_sys.state = SYS_STATE_RUNNING;
                int grasp_ret = execute_coordinate_grasp(px, py, dx, dy, grip_profile);
                if (grasp_ret == 0) {
                    rt_kprintf("[Pi->MCU] coordinate grasp done\r\n");
                } else if (grasp_ret == -2) {
                    rt_kprintf("[Pi->MCU] coordinate grasp aborted\r\n");
                } else {
                    rt_kprintf("[Pi->MCU] coordinate grasp rejected\r\n");
                }
                g_sys.state = SYS_STATE_READY;
            }
            rt_kprintf("\n");
            sys_cmd_inc();
        }

        /* ---- 鍔ㄤ綔瀹屾垚璺熻釜 (姣忕妫€鏌ヤ竴娆? ---- */
        {   static uint32_t last_check = 0;
            static rt_uint8_t last_state = SYS_STATE_READY;
            if (rt_tick_get() - last_check > 1000) {
                last_check = rt_tick_get();
                if (last_state >= SYS_STATE_RUNNING && g_sys.state == SYS_STATE_READY) {
                    rt_kprintf("[Pi->MCU] === Motion DONE ===\n");
                }
                if (g_sys.state == SYS_STATE_ERROR) {
                    rt_kprintf("[Pi->MCU] === Motion ERROR code=%d ===\n",
                               g_sys.error_code);
                }
                last_state = g_sys.state;
            }
        }

        rt_thread_mdelay(10);
    }
}

/* ======================================================================== */
/*   CAN + 澶圭埅 纭欢鍒濆鍖?(鐢?main 璋冪敤涓€娆?                                  */
/* ======================================================================== */
static int can_hw_init(void) {
    int ret = can_self_init();
    if (ret == 0) sys_mark_ready(SYS_MOD_CAN);
    return ret;
}
static void gripper_hw_init(void) {
    Servo_Init();
    Servo_Stop(SERVO_1); Servo_Stop(SERVO_2);
    Servo_Stop(SERVO_4); Servo_Stop(SERVO_5);
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM3->CCR2 = 0; TIM3->CCR3 = 0;
    { GPIO_InitTypeDef g = {0};
      g.Pin = GPIO_PIN_7|GPIO_PIN_8; g.Mode = GPIO_MODE_ANALOG;
      g.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOC, &g); }
    { GPIO_InitTypeDef g = {0};
      g.Pin = GPIO_PIN_12|GPIO_PIN_13; g.Mode = GPIO_MODE_ANALOG;
      g.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOD, &g); }
    Servo_SetAngle(SERVO_3, GRIP_MODE1_RELEASE);
    sys_mark_ready(SYS_MOD_SERVO);
}

/* ======================================================================== */
/*   绯荤粺鐩戞帶绾跨▼ 鈥?LED蹇冭烦 + 鍛ㄦ湡鐘舵€佹墦鍗?                                    */
/* ======================================================================== */
static rt_uint8_t  sysmon_stack[512];
static struct rt_thread sysmon_tcb;

static void sysmon_thread(void *parameter)
{
    LED_Init();
    sys_mark_ready(SYS_MOD_USART2);
    sys_mark_ready(SYS_MOD_KINEMATICS);
    sys_mark_ready(SYS_MOD_TRAJECTORY);
    g_sys.state = SYS_STATE_READY;

    rt_kprintf("\r\n[SYSMON] threads: sysmon proto\r\n");
    rt_kprintf("[SYSMON] CAN=%s GRIP=%s USART1=%s\r\n",
               (g_sys.modules_ready & SYS_MOD_CAN)   ? "OK" : "--",
               (g_sys.modules_ready & SYS_MOD_SERVO) ? "OK" : "--",
               (g_sys.modules_ready & SYS_MOD_USART1)? "OK" : "--");
    rt_kprintf("[SYSMON] Ready, period print: OFF\r\n");

    while (1)
    {
        /* LED: 姝ｅ父=鎱㈤棯, 寮傚父=蹇棯 */
        uint32_t period = (g_sys.state >= SYS_STATE_ESTOP) ? 100 : 250;
        LED1_On();  LED2_Off();  rt_thread_mdelay(period);
        LED1_Off(); LED2_On();   rt_thread_mdelay(period);

        /* 姣?5 绉掓眹鎬昏緭鍑?*/
        g_sys.uptime_ms += 500;
        static uint8_t tick = 0;
        if (++tick >= 10) {
            tick = 0;
            if (sys_print_en) sys_dump_status();
        }
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

  /* Configure the system clock (HSI 鈫?PLL 168MHz) */
  SystemClock_Config();

  /* RT-Thread kernel init (SysTick, heap, board components) */
  rt_hw_board_init();

  /* PendSV 浼樺厛绾у繀椤昏鍒版渶浣庯紝鍚﹀垯浼氭姠鍗?SysTick 瀵艰嚧 tick 涓㈠け */
  NVIC_SetPriority(PendSV_IRQn, 0x0F);

  /* ---- 璋冭瘯涓插彛蹇呴』鏃╀簬 rt_show_version() 鍒濆鍖?---- */
  MX_USART2_UART_Init();                   /* USART2: printf 杈撳嚭 (PA2/PA3) */
  Dbg_Printf_Init();                       /* 鍒濆鍖栬皟璇?printf */

  rt_show_version();
  rt_system_timer_init();
  rt_system_scheduler_init();
  rt_system_timer_thread_init();
  rt_thread_idle_init();

  //Can_Config(); /* 宸插睆钄?鈥?瑁告満娴嬭瘯 */
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();                          /* GPIO 鏃堕挓 + 寮曡剼榛樿鐘舵€?*/
  can_hw_init();                            /* CAN 鎬荤嚎 (PA11/PA12, 1Mbps) */
  sem_can_rx_done = rt_sem_create("can_rx", 0, RT_IPC_FLAG_FIFO); /* CAN璇诲洖淇″彿閲?*/
  gripper_hw_init();                        /* 澶圭埅 PWM (PC6)             */
  //MX_DMA_Init();                         /* 宸插睆钄?*/
  //MX_ADC1_Init();                        /* 宸插睆钄?*/
  //MX_IWDG_Init();                        /* 宸插睆钄?*/
  MX_USART1_UART_Init();                   /* USART1: 鏍戣帗娲鹃€氳 (PA9/PA10) */
  Rpi_Uart_StartRx();                      /* 鍚姩鏍戣帗娲句腑鏂帴鏀?*/
  //Telem_Init();                          /* 閬ユ祴 鈥?CAN娴嬭瘯鏈熼棿绂佺敤 */
  /* USER CODE BEGIN 2 */

  rt_kprintf("\n===== System Boot =====\n");
  rt_kprintf("[Main] System Boot\n");
  flag = 1;

  /* ---- 鍒涘缓 绯荤粺鐩戞帶绾跨▼ (LED + 鐘舵€? ---- */
  rt_err_t result = rt_thread_init(&sysmon_tcb, "sysmon", sysmon_thread,
                    RT_NULL, sysmon_stack, sizeof(sysmon_stack), 22, 5);
  if (result == RT_EOK) {
      rt_thread_startup(&sysmon_tcb);
  } else {
      rt_kprintf("[Main] sysmon init FAILED!\n");
  }

  /* ---- 鍒涘缓 鍗忚娴嬭瘯绾跨▼ (USART2 RX, 瑙ｆ瀽鏍戣帗娲惧抚) ---- */
  result = rt_thread_init(&proto_test_tcb,
                          "proto",
                          proto_test_thread,
                          RT_NULL,
                          proto_test_stack,
                          sizeof(proto_test_stack),
                          19,     /* priority 19 */
                          5);
  if (result == RT_EOK)
  {
      rt_thread_startup(&proto_test_tcb);
  }
  else
  {
      rt_kprintf("[Main] Protocol test thread init FAILED!\n");
  }

  /* USER CODE END 2 */

  /* Start RT-Thread scheduler (never returns) */
  rt_system_scheduler_start();

  /* Infinite loop (scheduler start failed fallback) */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
