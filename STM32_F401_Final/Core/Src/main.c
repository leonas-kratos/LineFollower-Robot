/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Line Follower PID - TB6612 + TCRT5000 x5
 *                   + Encoder + HC-05 Bluetooth
 *
 * ============================================================
 * PINOUT
 * ============================================================
 * Motor A / Left:
 *   PB0  = PWMA = TIM3_CH3
 *   PC13 = AIN1
 *   PB3  = AIN2
 *
 * Motor B / Right:
 *   PA7  = PWMB = TIM3_CH2
 *   PB15 = BIN1
 *   PB10 = BIN2
 *
 * TB6612:
 *   PC9  = STBY
 *
 * Encoder:
 *   PA0 PA1 = Left Encoder  TIM2
 *   PB6 PB7 = Right Encoder TIM4
 *
 * IR Sensor (TCRT5000 - line den nen trang):
 *   Nen trang  -> ADC thap  (phan xa nhieu)
 *   Line den   -> ADC cao   (hap thu anh sang)
 *   => Phat hien line khi ADC > threshold
 *
 *   PB1 = L2  (ADC_CH9)
 *   PC4 = L1  (ADC_CH14)
 *   PC5 = C   (ADC_CH15)
 *   PA6 = R1  (ADC_CH6)
 *   PC3 = R2  (ADC_CH13)
 *
 * UART:
 *   PA9  PA10 = USART1 HC-05
 *   PA2  PA3  = USART2 Debug
 *
 * LED:
 *   PA5 = LD2
 *
 * NOTE:
 *   PC13 is now AIN1, so physical button B1 is disabled.
 *   Use Bluetooth command S to Start/Stop.
 *
 * ============================================================
 * THAY DOI SO VOI PHIEN BAN CU
 * ============================================================
 *  1. Sua polarity sensor: line den = ADC > threshold (dung voi TCRT5000)
 *  2. Weighted error 5 muc rieng biet (-4,-2,0,+2,+4) thay vi 2 muc
 *  3. Turn memory: nho huong re cuoi -> LOST_LINE re dung huong
 *  4. Soft-start: tang PWM tuyen tinh trong SOFTSTART_MS khi khoi dong
 *  5. Adaptive base speed smooth hon (dung factor thay vi penalty tuyen tinh)
 *  6. Integral anti-windup don gian hoa
 *  7. [MOI] ADC moving average N=8, cap nhat moi ADC_UPDATE_MS (2ms)
 *     thay vi chi doc 1 lan moi 10ms -> giam noise khi toc do cao
 *  8. [MOI] Logic count >= 4: tinh huong re tu cam bien TRANG (minority)
 *     thay vi reset error ve 0 -> xu ly dung line rong / ngã tu
 *
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim3;
TIM_HandleTypeDef  htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;

/* ============================================================
 * DEFAULT PARAMETERS
 * ============================================================ */
#define DEFAULT_KP                5.0f
#define DEFAULT_KI                0.0f
#define DEFAULT_KD                3.5f
#define DEFAULT_BASE_SPEED        35
#define DEFAULT_TURN_LOST         20
#define DEFAULT_MANUAL_SPEED      30
#define DEFAULT_MANUAL_TIMEOUT    200
#define DEFAULT_W_L2              2.0f
#define DEFAULT_W_L1              1.0f
#define DEFAULT_W_R1              1.0f
#define DEFAULT_W_R2              2.0f
#define DEFAULT_BIAS_A            2.5f
#define DEFAULT_BIAS_B            0.0f
#define DEFAULT_CL_ENABLED        1
#define DEFAULT_CL_KP             1.5f
#define DEFAULT_CL_KD             0.8f
#define DEFAULT_IR_THRESHOLD      900

/* Soft-start */
#define SOFTSTART_MS              350

/* Turn memory decay */
#define TURN_MEMORY_DECAY         0.90f

/* Encoder */
#define ENCODER_PPR               11
#define GEAR_RATIO                30
#define ENCODER_COUNTS_PER_REV    (ENCODER_PPR * 4 * GEAR_RATIO)
#define SPEED_CALC_MS             20

/* ============================================================
 * [MOI] ADC MOVING AVERAGE
 * ADC_AVG_N phai la luy thua 2 de dung shift bit thay chia
 *   N=8  -> >> 3   (khuyen nghi: track binh thuong)
 *   N=16 -> >> 4   (track nhieu nhieu, doi kem nhanh hon mot chut)
 * ADC_UPDATE_MS: chu ky lay mau vao ring buffer (ms)
 *   2ms = 500Hz sampling rate (gap 5x so voi main loop 10ms)
 * ============================================================ */
#define ADC_AVG_N                 1
#define ADC_AVG_SHIFT             1       /* log2(ADC_AVG_N) */
#define ADC_UPDATE_MS             1

/* ============================================================
 * TUNABLE PARAMETERS
 * ============================================================ */
static float    g_kp             = DEFAULT_KP;
static float    g_ki             = DEFAULT_KI;
static float    g_kd             = DEFAULT_KD;
static int8_t   g_base_speed     = DEFAULT_BASE_SPEED;
static int8_t   g_turn_lost      = DEFAULT_TURN_LOST;
static int8_t   g_manual_speed   = DEFAULT_MANUAL_SPEED;
static uint32_t g_manual_timeout = DEFAULT_MANUAL_TIMEOUT;
static float    g_w_l2           = DEFAULT_W_L2;
static float    g_w_l1           = DEFAULT_W_L1;
static float    g_w_r1           = DEFAULT_W_R1;
static float    g_w_r2           = DEFAULT_W_R2;
static float    g_bias_a         = DEFAULT_BIAS_A;
static float    g_bias_b         = DEFAULT_BIAS_B;
static uint8_t  g_cl_enabled     = DEFAULT_CL_ENABLED;
static float    g_cl_kp          = DEFAULT_CL_KP;
static float    g_cl_kd          = DEFAULT_CL_KD;
static uint16_t g_ir_threshold   = DEFAULT_IR_THRESHOLD;

/* ============================================================
 * PID STATE
 * ============================================================ */
static float    g_error         = 0.0f;
static float    g_last_error    = 0.0f;
static float    g_integral      = 0.0f;
static uint32_t g_last_pid_time = 0;

/* Turn memory */
static float    g_turn_memory   = 0.0f;

/* ============================================================
 * ADC / IR
 * DMA order: CH9(L2) CH14(L1) CH15(C) CH6(R1) CH13(R2)
 * ============================================================ */
#define ADC_CHANNELS 5
static volatile uint16_t g_adc_buf[ADC_CHANNELS];

/* [MOI] Ring buffer moving average */
static uint16_t  g_adc_ring[ADC_CHANNELS][ADC_AVG_N];
static uint8_t   g_adc_ring_idx  = 0;
static uint32_t  g_adc_last_tick = 0;

/* Gia tri da loc - dung trong LineFollow_PID */
static uint16_t g_ir_L2, g_ir_L1, g_ir_C, g_ir_R1, g_ir_R2;

/* ============================================================
 * ENCODER / SPEED
 * ============================================================ */
static int32_t  g_enc_left_prev   = 0;
static int32_t  g_enc_right_prev  = 0;
static float    g_rpm_left        = 0.0f;
static float    g_rpm_right       = 0.0f;
static uint32_t g_speed_last_tick = 0;

/* ============================================================
 * CLOSED-LOOP SPEED BALANCE
 * ============================================================ */
static float    g_cl_error_prev  = 0.0f;
static float    g_cl_correction  = 0.0f;

/* ============================================================
 * SOFT-START
 * ============================================================ */
static uint32_t g_softstart_tick  = 0;
static uint8_t  g_softstart_done  = 0;
static float    g_softstart_scale = 0.0f;

/* ============================================================
 * UART / BLUETOOTH
 * ============================================================ */
#define UART_MS      200
#define BT_BUF_SIZE  16

typedef enum
{
    MODE_AUTO   = 0,
    MODE_MANUAL = 1
} DriveMode_t;

static uint8_t  g_bt_rx_byte = 0;
static char     g_bt_cmd_buf[BT_BUF_SIZE];
static uint8_t  g_bt_cmd_len = 0;

volatile uint8_t     g_running          = 0;
volatile DriveMode_t g_mode             = MODE_AUTO;
volatile uint32_t    g_bt_last_cmd_tick = 0;
volatile uint8_t     g_manual_active    = 0;
volatile uint8_t     g_tuning           = 0;

static uint8_t g_resume_run = 0;
static int8_t  g_spd_A = 0;
static int8_t  g_spd_B = 0;
static char    g_state[20] = "STOP";

/* ============================================================
 * FUNCTION PROTOTYPES
 * ============================================================ */
void SystemClock_Config(void);
void Error_Handler(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

void TB6612_Enable(void);
void TB6612_Disable(void);
void MotorA_SetSpeed(int speed);
void MotorB_SetSpeed(int speed);
void Motors_Stop(void);
void Motors_Brake(void);
void ToggleRunning(void);
void SetMode(DriveMode_t mode);
void ManualDrive(uint8_t cmd);
void ManualTimeoutCheck(void);
void ADC_UpdateIR(void);
void Encoder_UpdateSpeed(void);
void LineFollow_PID(void);
void UART_PrintStatus(void);
void BT_Send(const char *str);
void BT_SendStatus(void);
void BT_SendTuningMenu(void);
void BT_ProcessCommand(const char *cmd);
void Tuning_ResetDefaults(void);
void EnterTuningMode(void);
void ExitTuningMode(void);

/* ============================================================
 * DEBUG UART (printf -> USART2)
 * ============================================================ */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ============================================================
 * BLUETOOTH SEND
 * ============================================================ */
void BT_Send(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/* ============================================================
 * TB6612 ENABLE / DISABLE
 * ============================================================ */
void TB6612_Enable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void TB6612_Disable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

/* ============================================================
 * MOTOR A (Left)
 *   PWMA = TIM3_CH3 (PB0)
 *   AIN1 = PC13
 *   AIN2 = PB3
 * ============================================================ */
void MotorA_SetSpeed(int speed)
{
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;

    g_spd_A = speed;

    uint32_t pwm_raw = (uint32_t)(abs(speed) * 65535 / 100);

    float factor = g_cl_enabled
                 ? (1.0f + g_cl_correction / 100.0f)
                 : (1.0f + g_bias_a / 100.0f);

    float biased = (float)pwm_raw * factor;
    if (biased < 0.0f)     biased = 0.0f;
    if (biased > 65535.0f) biased = 65535.0f;

    if (speed > 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)biased);
}

/* ============================================================
 * MOTOR B (Right)
 *   PWMB = TIM3_CH2 (PA7)
 *   BIN1 = PB15
 *   BIN2 = PB10
 * ============================================================ */
void MotorB_SetSpeed(int speed)
{
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;

    g_spd_B = speed;

    uint32_t pwm_raw = (uint32_t)(abs(speed) * 65535 / 100);

    float biased = g_cl_enabled
                 ? (float)pwm_raw
                 : (float)pwm_raw * (1.0f + g_bias_b / 100.0f);

    if (biased < 0.0f)     biased = 0.0f;
    if (biased > 65535.0f) biased = 65535.0f;

    if (speed > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)biased);
}

/* ============================================================
 * MOTORS STOP / BRAKE
 * ============================================================ */
void Motors_Stop(void)
{
    g_spd_A = 0;
    g_spd_B = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}

void Motors_Brake(void)
{
    g_spd_A = 0;
    g_spd_B = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}

/* ============================================================
 * [MOI] ADC UPDATE - MOVING AVERAGE
 *
 * Thay vi chi copy raw DMA buffer 1 lan moi 10ms, ham nay:
 *   1. Chay moi ADC_UPDATE_MS (2ms) - tan so cao hon 5x
 *   2. Luu mau vao ring buffer g_adc_ring[5][8]
 *   3. Tinh trung binh 8 mau bang shift bit (nhanh hon chia)
 *
 * Ket qua: g_ir_L2..R2 la gia tri da lam muot, giam noise
 * rat nhieu khi xe chay nhanh tren track rung/doi truong.
 *
 * Ring buffer dung ky thuat AND mask thay MOD:
 *   idx = (idx + 1) & (N-1)   <=>   idx = (idx+1) % N
 *   Chi dung duoc khi N la luy thua 2 (4, 8, 16, ...)
 * ============================================================ */
void ADC_UpdateIR(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - g_adc_last_tick) < ADC_UPDATE_MS)
        return;
    g_adc_last_tick = now;

    /* Doc atomic tu DMA buffer */
    uint16_t raw[ADC_CHANNELS];
    __disable_irq();
    raw[0] = g_adc_buf[0];   /* L2 */
    raw[1] = g_adc_buf[1];   /* L1 */
    raw[2] = g_adc_buf[2];   /* C  */
    raw[3] = g_adc_buf[3];   /* R1 */
    raw[4] = g_adc_buf[4];   /* R2 */
    __enable_irq();

    /* Ghi mau moi vao ring buffer */
    g_adc_ring_idx = (g_adc_ring_idx + 1) & (ADC_AVG_N - 1);
    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++)
        g_adc_ring[ch][g_adc_ring_idx] = raw[ch];

    /* Tinh trung binh: sum / 8 = sum >> 3 */
    uint32_t sum[ADC_CHANNELS] = {0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < ADC_AVG_N; i++)
        for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++)
            sum[ch] += g_adc_ring[ch][i];

    g_ir_L2 = (uint16_t)(sum[0] >> ADC_AVG_SHIFT);
    g_ir_L1 = (uint16_t)(sum[1] >> ADC_AVG_SHIFT);
    g_ir_C  = (uint16_t)(sum[2] >> ADC_AVG_SHIFT);
    g_ir_R1 = (uint16_t)(sum[3] >> ADC_AVG_SHIFT);
    g_ir_R2 = (uint16_t)(sum[4] >> ADC_AVG_SHIFT);
}

/* ============================================================
 * ENCODER SPEED + CLOSED-LOOP BALANCE
 * ============================================================ */
void Encoder_UpdateSpeed(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - g_speed_last_tick) < SPEED_CALC_MS)
        return;

    float dt_s = (now - g_speed_last_tick) / 1000.0f;
    g_speed_last_tick = now;

    int32_t cnt_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t cnt_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    int32_t delta_left  = cnt_left  - g_enc_left_prev;
    int32_t delta_right = cnt_right - g_enc_right_prev;

    /* Xu ly overflow bo dem 16-bit */
    if (delta_left  >  32767) delta_left  -= 65536;
    if (delta_left  < -32768) delta_left  += 65536;
    if (delta_right >  32767) delta_right -= 65536;
    if (delta_right < -32768) delta_right += 65536;

    g_enc_left_prev  = cnt_left;
    g_enc_right_prev = cnt_right;

    g_rpm_left  = ((float)delta_left  / ENCODER_COUNTS_PER_REV) / dt_s * 60.0f;
    g_rpm_right = ((float)delta_right / ENCODER_COUNTS_PER_REV) / dt_s * 60.0f;

    if (g_cl_enabled && g_running)
    {
        float cl_error = g_rpm_left - g_rpm_right;
        float cl_deriv = (cl_error - g_cl_error_prev) / dt_s;

        g_cl_correction -= (g_cl_kp * cl_error + g_cl_kd * cl_deriv);

        if (g_cl_correction >  15.0f) g_cl_correction =  15.0f;
        if (g_cl_correction < -15.0f) g_cl_correction = -15.0f;

        g_cl_error_prev = cl_error;
    }
    else
    {
        g_cl_correction = 0.0f;
        g_cl_error_prev = 0.0f;
    }
}

/* ============================================================
 * TOGGLE RUNNING (AUTO mode)
 * ============================================================ */
void ToggleRunning(void)
{
    if (g_mode != MODE_AUTO) return;
    if (g_tuning) return;

    g_running ^= 1;

    if (g_running)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

        g_integral        = 0.0f;
        g_last_error      = 0.0f;
        g_cl_correction   = 0.0f;
        g_cl_error_prev   = 0.0f;
        g_turn_memory     = 0.0f;
        g_last_pid_time   = HAL_GetTick();

        g_softstart_done  = 0;
        g_softstart_scale = 0.0f;
        g_softstart_tick  = HAL_GetTick();

        strncpy(g_state, "SOFTSTART      ", sizeof(g_state));

        printf("\r\n=== AUTO: START ===\r\n");
        BT_Send("$CMD,START\r\n");
    }
    else
    {
        Motors_Brake();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

        g_softstart_done  = 0;
        g_softstart_scale = 0.0f;

        strncpy(g_state, "STOP           ", sizeof(g_state));

        printf("\r\n=== AUTO: STOP ===\r\n");
        BT_Send("$CMD,STOP\r\n");
    }
}

/* ============================================================
 * LINE FOLLOW PID
 *
 * Sensor polarity (TCRT5000, line den nen trang):
 *   Nen trang  -> ADC thap  -> KHONG tren line
 *   Line den   -> ADC cao   -> TREN line
 *   => sX = 1 khi g_ir_X > g_ir_threshold
 *
 * Error position (duong = lech trai, am = lech phai):
 *   L2 = +4, L1 = +2, C = 0, R1 = -2, R2 = -4
 *
 * [MOI] Logic count >= 4 (4-5 cam bien den):
 *   - count == 5 (tat ca den): nga tu thuc su -> error = 0
 *   - count == 4 (1 cam bien trang): xe lech tren line rong
 *     -> Tinh error tu cam bien TRANG (minority) de dinh huong ve giua
 *     -> Scale error * 0.6 de tranh phan ung qua manh
 *
 * Vi sao dung cam bien trang?
 *   Khi L2 TRANG giua 4 cam bien den = xe dang lech sang phai
 *   (L2 bi truot ra ngoai line) -> phai quay trai (error am)
 *   => error = -(trong_so_L2) = re phai dua ve giua ✓
 *
 * Soft-start:
 *   Trong SOFTSTART_MS dau tien, scale toan bo output PWM
 *   tu 0 len 1.0 tuyen tinh
 *
 * Turn memory:
 *   Low-pass filter cua g_error, dung khi LOST_LINE
 * ============================================================ */
void LineFollow_PID(void)
{
    /* ---- 1. Doc trang thai sensor ---- */
    uint8_t sL2 = (g_ir_L2 > g_ir_threshold) ? 1 : 0;
    uint8_t sL1 = (g_ir_L1 > g_ir_threshold) ? 1 : 0;
    uint8_t sC  = (g_ir_C  > g_ir_threshold) ? 1 : 0;
    uint8_t sR1 = (g_ir_R1 > g_ir_threshold) ? 1 : 0;
    uint8_t sR2 = (g_ir_R2 > g_ir_threshold) ? 1 : 0;

    int count = sL2 + sL1 + sC + sR1 + sR2;

    /* ---- 2. Soft-start scale ---- */
    if (!g_softstart_done)
    {
        uint32_t elapsed = HAL_GetTick() - g_softstart_tick;

        if (elapsed >= SOFTSTART_MS)
        {
            g_softstart_done  = 1;
            g_softstart_scale = 1.0f;
        }
        else
        {
            g_softstart_scale = (float)elapsed / (float)SOFTSTART_MS;
        }
    }

    /* ---- 3. Xu ly LOST LINE ---- */
    if (count == 0)
    {
        strncpy(g_state, "LOST_LINE      ", sizeof(g_state));

        int lost_spd = (int)((float)g_turn_lost * g_softstart_scale);
        if (lost_spd < 15) lost_spd = 15;

        if (g_turn_memory > 0.5f)
        {
            MotorA_SetSpeed(-lost_spd);
            MotorB_SetSpeed( lost_spd);
        }
        else if (g_turn_memory < -0.5f)
        {
            MotorA_SetSpeed( lost_spd);
            MotorB_SetSpeed(-lost_spd);
        }
        else
        {
            MotorA_SetSpeed(lost_spd);
            MotorB_SetSpeed(lost_spd);
        }

        return;
    }

    /* ---- 4. [MOI] Xu ly count >= 4 ---- */
    if (count >= 4)
    {
        /*
         * Tinh cam bien TRANG (minority trong truong hop 4 den / 1 trang)
         *
         * Nguyen tac:
         *   Cam bien nao TRANG = cam bien do dang ra ngoai line
         *   => Can re VE PHIA cam bien trang do de xe ve giua line
         *
         * Dau error tu cam bien trang (NGUOC voi error den):
         *   L2 trang (xe lech phai) -> error am  -> motor PID re trai -> ve giua ✓
         *   L1 trang (xe lech phai nhe) -> error am nhe
         *   R1 trang (xe lech trai nhe) -> error duong nhe
         *   R2 trang (xe lech trai) -> error duong -> motor PID re phai -> ve giua ✓
         *
         * Scale factor 0.6: giam phan ung khi tren line rong
         * de tranh xe dao dong. Tang len 0.8 neu xe con truot ra.
         */

        uint8_t wL2 = 1 - sL2;  /* 1 neu L2 trang */
        uint8_t wL1 = 1 - sL1;
        uint8_t wC  = 1 - sC;
        uint8_t wR1 = 1 - sR1;
        uint8_t wR2 = 1 - sR2;

        int white_count = wL2 + wL1 + wC + wR1 + wR2;

        if (white_count == 0)
        {
            /* Tat ca 5 cam bien den = nga tu that su */
            strncpy(g_state, "CROSS_ALL      ", sizeof(g_state));
            g_error       = 0.0f;
            g_turn_memory = 0.0f;
        }
        else
        {
            /* 1-2 cam bien trang: xe lech tren line rong */
            strncpy(g_state, "CROSS_LEAN     ", sizeof(g_state));

            float white_sum   = 0.0f;
            float white_total = 0.0f;

            if (wL2) { white_sum -= g_w_l2; white_total += g_w_l2; }
            if (wL1) { white_sum -= g_w_l1; white_total += g_w_l1; }
            if (wC)  { white_sum += 0.0f;   white_total += 1.0f;   }
            if (wR1) { white_sum += g_w_r1; white_total += g_w_r1; }
            if (wR2) { white_sum += g_w_r2; white_total += g_w_r2; }

            g_error = (white_total > 0.0f)
                    ? (white_sum / white_total) * 0.6f
                    : 0.0f;
        }
    }
    else
    {
        /* ---- 5. Normal PID tracking (count 1-3) ---- */
        strncpy(g_state, "PID_TRACKING   ", sizeof(g_state));

        float weighted_sum = 0.0f;
        float weight_total = 0.0f;

        if (sL2) { weighted_sum +=  g_w_l2; weight_total += g_w_l2; }
        if (sL1) { weighted_sum +=  g_w_l1; weight_total += g_w_l1; }
        if (sC)  { weighted_sum +=  0.0f;   weight_total += 1.0f;   }
        if (sR1) { weighted_sum += -g_w_r1; weight_total += g_w_r1; }
        if (sR2) { weighted_sum += -g_w_r2; weight_total += g_w_r2; }

        g_error = (weight_total > 0.0f) ? (weighted_sum / weight_total) : 0.0f;
    }

    /* ---- 6. Cap nhat turn memory ---- */
    g_turn_memory = g_turn_memory * TURN_MEMORY_DECAY
                  + g_error * (1.0f - TURN_MEMORY_DECAY);

    /* ---- 7. Tinh PID ---- */
    uint32_t now = HAL_GetTick();
    float dt = (now - g_last_pid_time) / 1000.0f;
    if (dt <= 0.001f) dt = 0.01f;
    if (dt > 0.1f)    dt = 0.1f;
    g_last_pid_time = now;

    float P = g_kp * g_error;

    g_integral += g_error * dt;

    float max_integral = 50.0f;
    if (g_integral >  max_integral) g_integral =  max_integral;
    if (g_integral < -max_integral) g_integral = -max_integral;

    float I = g_ki * g_integral;
    float D = g_kd * (g_error - g_last_error) / dt;

    float output = P + I + D;

    g_last_error = g_error;

    /* ---- 8. Adaptive base speed ---- */
    float abs_output     = fabsf(output);
    float reduction_rate = 0.04f;
    float speed_factor   = 1.0f / (1.0f + abs_output * reduction_rate);

    int min_base = 20;
    int adaptive_base = (int)((float)g_base_speed * speed_factor);
    if (adaptive_base < min_base) adaptive_base = min_base;

    /* ---- 9. Tinh speed tung banh ---- */
    int speedA = adaptive_base + (int)output;
    int speedB = adaptive_base - (int)output;

    if (speedA >  100) speedA =  100;
    if (speedA < -100) speedA = -100;
    if (speedB >  100) speedB =  100;
    if (speedB < -100) speedB = -100;

    /* ---- 10. Ap dung soft-start scale ---- */
    speedA = (int)((float)speedA * g_softstart_scale);
    speedB = (int)((float)speedB * g_softstart_scale);

    if (!g_softstart_done && g_softstart_scale > 0.05f)
        strncpy(g_state, "SOFTSTART      ", sizeof(g_state));

    MotorA_SetSpeed(speedA);
    MotorB_SetSpeed(speedB);
}

/* ============================================================
 * TUNING DEFAULTS
 * ============================================================ */
void Tuning_ResetDefaults(void)
{
    g_kp             = DEFAULT_KP;
    g_ki             = DEFAULT_KI;
    g_kd             = DEFAULT_KD;
    g_base_speed     = DEFAULT_BASE_SPEED;
    g_turn_lost      = DEFAULT_TURN_LOST;
    g_manual_speed   = DEFAULT_MANUAL_SPEED;
    g_manual_timeout = DEFAULT_MANUAL_TIMEOUT;
    g_w_l2           = DEFAULT_W_L2;
    g_w_l1           = DEFAULT_W_L1;
    g_w_r1           = DEFAULT_W_R1;
    g_w_r2           = DEFAULT_W_R2;
    g_bias_a         = DEFAULT_BIAS_A;
    g_bias_b         = DEFAULT_BIAS_B;
    g_cl_enabled     = DEFAULT_CL_ENABLED;
    g_cl_kp          = DEFAULT_CL_KP;
    g_cl_kd          = DEFAULT_CL_KD;
    g_ir_threshold   = DEFAULT_IR_THRESHOLD;

    g_integral       = 0.0f;
    g_last_error     = 0.0f;
    g_cl_correction  = 0.0f;
    g_cl_error_prev  = 0.0f;
    g_turn_memory    = 0.0f;

    printf("[TUNE] Reset defaults\r\n");
    BT_Send("$TUNE,RESET_OK\r\n");
    BT_SendTuningMenu();
}

/* ============================================================
 * TUNING MENU
 * ============================================================ */
void BT_SendTuningMenu(void)
{
    char buf[160];

    BT_Send("$TUNE_MENU_START\r\n");

    snprintf(buf, sizeof(buf), "$TUNE,Kp=%.2f Ki=%.3f Kd=%.2f\r\n",
             g_kp, g_ki, g_kd);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,W_L2=%.2f W_L1=%.2f W_R1=%.2f W_R2=%.2f THR=%d\r\n",
             g_w_l2, g_w_l1, g_w_r1, g_w_r2, g_ir_threshold);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,BASE=%d LOST=%d MAN=%d TIMEOUT=%lu\r\n",
             g_base_speed, g_turn_lost, g_manual_speed, (unsigned long)g_manual_timeout);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,BIAS_A=%+.2f BIAS_B=%+.2f CL=%s\r\n",
             g_bias_a, g_bias_b, g_cl_enabled ? "ON" : "OFF");
    BT_Send(buf);

    BT_Send("$CMD: S Start/Stop | A Auto | M Manual | I/K/J/L move\r\n");
    BT_Send("$CMD: T Tuning | Q Exit | R Reset\r\n");
    BT_Send("$TUNE: P/I/D Kp Ki Kd | B base | O lost | V man | X timeout\r\n");
    BT_Send("$TUNE: W wL2 | N wL1 | U wR1 | Y wR2 | Z threshold\r\n");
    BT_Send("$TUNE: G biasA | H biasB | C clEn | E clKp | F clKd\r\n");
    BT_Send("$TUNE_MENU_END\r\n");
}

/* ============================================================
 * ENTER / EXIT TUNING MODE
 * ============================================================ */
void EnterTuningMode(void)
{
    if (g_tuning)
    {
        BT_SendTuningMenu();
        return;
    }

    g_resume_run    = g_running;
    g_running       = 0;
    g_manual_active = 0;

    Motors_Brake();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    strncpy(g_state, "TUNING         ", sizeof(g_state));

    g_tuning = 1;

    printf("\r\n=== TUNING MODE ===\r\n");
    BT_Send("$TUNING_START\r\n");
    BT_SendTuningMenu();
}

void ExitTuningMode(void)
{
    if (!g_tuning) return;

    g_tuning = 0;

    g_integral      = 0.0f;
    g_last_error    = 0.0f;
    g_cl_correction = 0.0f;
    g_turn_memory   = 0.0f;

    if (g_resume_run && g_mode == MODE_AUTO)
    {
        g_softstart_done  = 0;
        g_softstart_scale = 0.0f;
        g_softstart_tick  = HAL_GetTick();

        g_running = 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        strncpy(g_state, "SOFTSTART      ", sizeof(g_state));
        BT_Send("$TUNING_EXIT,RESUME\r\n");
    }
    else
    {
        g_running = 0;
        Motors_Brake();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        strncpy(g_state, "STOP           ", sizeof(g_state));
        BT_Send("$TUNING_EXIT,STOP\r\n");
    }

    g_resume_run = 0;
}

/* ============================================================
 * BLUETOOTH COMMAND PROCESSOR
 * ============================================================ */
void BT_ProcessCommand(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') return;

    uint8_t c    = (uint8_t)cmd[0];
    float   fval = (cmd[1] != '\0') ? atof(&cmd[1]) : -999.0f;
    int     val  = (int)fval;

    if (c == 'T' || c == 't')
    {
        EnterTuningMode();
        return;
    }

    if (g_tuning)
    {
        char ack[80];

        if (c == 'Q' || c == 'q')
        {
            ExitTuningMode();
            return;
        }

        if (c == 'R' || c == 'r')
        {
            Tuning_ResetDefaults();
            return;
        }

        if (cmd[1] == '\0')
        {
            BT_Send("$ERR,Need value\r\n");
            return;
        }

        switch (c)
        {
            case 'P': case 'p':
                g_kp = fval;
                snprintf(ack, sizeof(ack), "$TUNE,Kp=%.2f\r\n", g_kp);
                break;
            case 'I': case 'i':
                g_ki = fval;
                snprintf(ack, sizeof(ack), "$TUNE,Ki=%.3f\r\n", g_ki);
                break;
            case 'D': case 'd':
                g_kd = fval;
                snprintf(ack, sizeof(ack), "$TUNE,Kd=%.2f\r\n", g_kd);
                break;
            case 'B': case 'b':
                g_base_speed = (int8_t)val;
                snprintf(ack, sizeof(ack), "$TUNE,BASE=%d\r\n", g_base_speed);
                break;
            case 'O': case 'o':
                g_turn_lost = (int8_t)val;
                snprintf(ack, sizeof(ack), "$TUNE,LOST=%d\r\n", g_turn_lost);
                break;
            case 'V': case 'v':
                g_manual_speed = (int8_t)val;
                snprintf(ack, sizeof(ack), "$TUNE,MAN=%d\r\n", g_manual_speed);
                break;
            case 'X': case 'x':
                g_manual_timeout = (uint32_t)val;
                snprintf(ack, sizeof(ack), "$TUNE,TIMEOUT=%lu\r\n",
                         (unsigned long)g_manual_timeout);
                break;
            case 'W': case 'w':
                g_w_l2 = fval;
                snprintf(ack, sizeof(ack), "$TUNE,W_L2=%.2f\r\n", g_w_l2);
                break;
            case 'N': case 'n':
                g_w_l1 = fval;
                snprintf(ack, sizeof(ack), "$TUNE,W_L1=%.2f\r\n", g_w_l1);
                break;
            case 'U': case 'u':
                g_w_r1 = fval;
                snprintf(ack, sizeof(ack), "$TUNE,W_R1=%.2f\r\n", g_w_r1);
                break;
            case 'Y': case 'y':
                g_w_r2 = fval;
                snprintf(ack, sizeof(ack), "$TUNE,W_R2=%.2f\r\n", g_w_r2);
                break;
            case 'Z': case 'z':
                g_ir_threshold = (uint16_t)val;
                snprintf(ack, sizeof(ack), "$TUNE,THR=%d\r\n", g_ir_threshold);
                break;
            case 'G': case 'g':
                g_bias_a = fval;
                snprintf(ack, sizeof(ack), "$TUNE,BIAS_A=%.2f\r\n", g_bias_a);
                break;
            case 'H': case 'h':
                g_bias_b = fval;
                snprintf(ack, sizeof(ack), "$TUNE,BIAS_B=%.2f\r\n", g_bias_b);
                break;
            case 'C': case 'c':
                g_cl_enabled    = val ? 1 : 0;
                g_cl_correction = 0.0f;
                snprintf(ack, sizeof(ack), "$TUNE,CL=%s\r\n",
                         g_cl_enabled ? "ON" : "OFF");
                break;
            case 'E': case 'e':
                g_cl_kp = fval;
                snprintf(ack, sizeof(ack), "$TUNE,CL_Kp=%.2f\r\n", g_cl_kp);
                break;
            case 'F': case 'f':
                g_cl_kd = fval;
                snprintf(ack, sizeof(ack), "$TUNE,CL_Kd=%.2f\r\n", g_cl_kd);
                break;
            default:
                BT_Send("$ERR,Unknown tuning cmd\r\n");
                return;
        }

        BT_Send(ack);
        BT_SendTuningMenu();
        return;
    }

    /* Non-tuning commands */
    if (c == 'A' || c == 'a') { SetMode(MODE_AUTO);   return; }
    if (c == 'M' || c == 'm') { SetMode(MODE_MANUAL);  return; }

    if (c == 'S' || c == 's')
    {
        if (g_mode == MODE_AUTO)
            ToggleRunning();
        return;
    }

    if (g_mode == MODE_MANUAL)
        ManualDrive(c);
}

/* ============================================================
 * SET MODE
 * ============================================================ */
void SetMode(DriveMode_t mode)
{
    g_tuning          = 0;
    g_resume_run      = 0;
    g_mode            = mode;
    g_running         = 0;
    g_manual_active   = 0;
    g_cl_correction   = 0.0f;
    g_softstart_done  = 0;
    g_softstart_scale = 0.0f;
    g_turn_memory     = 0.0f;

    Motors_Brake();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    strncpy(g_state, "STOP           ", sizeof(g_state));

    if (mode == MODE_AUTO)
    {
        printf("\r\n>>> MODE AUTO\r\n");
        BT_Send("$MODE,AUTO\r\n");
    }
    else
    {
        printf("\r\n>>> MODE MANUAL\r\n");
        BT_Send("$MODE,MANUAL\r\n");
    }
}

/* ============================================================
 * MANUAL DRIVE
 * ============================================================ */
void ManualDrive(uint8_t cmd)
{
    switch (cmd)
    {
        case 'I': case 'i':
            strncpy(g_state, "MAN-FORWARD    ", sizeof(g_state));
            MotorA_SetSpeed(g_manual_speed);
            MotorB_SetSpeed(g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick();
            g_manual_active    = 1;
            break;
        case 'K': case 'k':
            strncpy(g_state, "MAN-BACKWARD   ", sizeof(g_state));
            MotorA_SetSpeed(-g_manual_speed);
            MotorB_SetSpeed(-g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick();
            g_manual_active    = 1;
            break;
        case 'J': case 'j':
            strncpy(g_state, "MAN-LEFT       ", sizeof(g_state));
            MotorA_SetSpeed(-g_manual_speed);
            MotorB_SetSpeed( g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick();
            g_manual_active    = 1;
            break;
        case 'L': case 'l':
            strncpy(g_state, "MAN-RIGHT      ", sizeof(g_state));
            MotorA_SetSpeed( g_manual_speed);
            MotorB_SetSpeed(-g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick();
            g_manual_active    = 1;
            break;
        default:
            strncpy(g_state, "MAN-STOP       ", sizeof(g_state));
            g_manual_active = 0;
            Motors_Brake();
            break;
    }
}

/* ============================================================
 * MANUAL TIMEOUT CHECK
 * ============================================================ */
void ManualTimeoutCheck(void)
{
    if (g_mode != MODE_MANUAL || !g_manual_active)
        return;

    if ((HAL_GetTick() - g_bt_last_cmd_tick) >= g_manual_timeout)
    {
        g_manual_active = 0;
        strncpy(g_state, "MAN-STOP       ", sizeof(g_state));
        Motors_Brake();
    }
}

/* ============================================================
 * UART STATUS PRINT
 * ============================================================ */
void UART_PrintStatus(void)
{
    printf("[%s][%s][%s] IR:%4d %4d %4d %4d %4d | %-15s | Err:%5.2f Mem:%5.2f | A:%+4d B:%+4d | RPM L:%+6.1f R:%+6.1f | CL:%s %+.1f%% | SS:%.2f\r\n",
           g_mode == MODE_AUTO ? "AUTO  " : "MANUAL",
           g_running ? "RUN" : "STP",
           g_tuning  ? "TUNE" : "    ",
           g_ir_L2, g_ir_L1, g_ir_C, g_ir_R1, g_ir_R2,
           g_state,
           g_error,
           g_turn_memory,
           (int)g_spd_A,
           (int)g_spd_B,
           g_rpm_left,
           g_rpm_right,
           g_cl_enabled ? "ON " : "OFF",
           g_cl_correction,
           g_softstart_scale);
}

/* ============================================================
 * BLUETOOTH STATUS
 * ============================================================ */
void BT_SendStatus(void)
{
    if (g_tuning) return;

    char buf[160];
    snprintf(buf, sizeof(buf),
             "$%s,%s,%d,%d,%d,%d,%d,%s,%.2f,%+d,%+d,RPM:%.1f/%.1f,CL:%s\r\n",
             g_mode == MODE_AUTO ? "AUTO" : "MAN",
             g_running ? "RUN" : "STP",
             g_ir_L2, g_ir_L1, g_ir_C, g_ir_R1, g_ir_R2,
             g_state,
             g_error,
             (int)g_spd_A,
             (int)g_spd_B,
             g_rpm_left,
             g_rpm_right,
             g_cl_enabled ? "ON" : "OFF");

    BT_Send(buf);
}

/* ============================================================
 * INTERRUPT HANDLERS
 * ============================================================ */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    uint8_t c = g_bt_rx_byte;

    if (c == '\r' || c == '\n' || c == ';' || c == ' ')
    {
        if (g_bt_cmd_len > 0)
        {
            g_bt_cmd_buf[g_bt_cmd_len] = '\0';
            BT_ProcessCommand(g_bt_cmd_buf);
            g_bt_cmd_len = 0;
        }
    }
    else if (g_bt_cmd_len < BT_BUF_SIZE - 1)
    {
        g_bt_cmd_buf[g_bt_cmd_len++] = (char)c;

        if (g_bt_cmd_len == 1)
        {
            if (!g_tuning)
            {
                if (c == 'A' || c == 'a' || c == 'M' || c == 'm' ||
                    c == 'T' || c == 't' || c == 'S' || c == 's' ||
                    c == 'I' || c == 'i' || c == 'K' || c == 'k' ||
                    c == 'J' || c == 'j' || c == 'L' || c == 'l')
                {
                    g_bt_cmd_buf[g_bt_cmd_len] = '\0';
                    BT_ProcessCommand(g_bt_cmd_buf);
                    g_bt_cmd_len = 0;
                }
            }
            else
            {
                if (c == 'Q' || c == 'q' || c == 'R' || c == 'r')
                {
                    g_bt_cmd_buf[g_bt_cmd_len] = '\0';
                    BT_ProcessCommand(g_bt_cmd_buf);
                    g_bt_cmd_len = 0;
                }
            }
        }
    }
    else
    {
        g_bt_cmd_buf[g_bt_cmd_len] = '\0';
        BT_ProcessCommand(g_bt_cmd_buf);
        g_bt_cmd_len = 0;
    }

    HAL_UART_Receive_IT(&huart1, &g_bt_rx_byte, 1);
}

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_buf, ADC_CHANNELS);

    TB6612_Enable();
    Motors_Stop();

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    SetMode(MODE_AUTO);
    HAL_UART_Receive_IT(&huart1, &g_bt_rx_byte, 1);

    printf("\r\n========================================\r\n");
    printf(" Line Follower PID STM32F401RE\r\n");
    printf(" ADC: moving average N=%d, update %dms\r\n", ADC_AVG_N, ADC_UPDATE_MS);
    printf(" Sensor: line den = ADC cao (> threshold)\r\n");
    printf(" MotorA: PB0 PWM, PC13 AIN1, PB3 AIN2\r\n");
    printf(" MotorB: PA7 PWM, PB15 BIN1, PB10 BIN2\r\n");
    printf(" Start/Stop: Bluetooth command S\r\n");
    printf("========================================\r\n\r\n");

    BT_Send("$BOOT,LineFollower_F401\r\n");
    BT_SendTuningMenu();

    uint32_t last_print = 0;
    uint32_t loop_tick  = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /*
         * [MOI] ADC_UpdateIR() goi O DAY - ben ngoai khoi 10ms
         * Ham tu kiem tra thoi gian (ADC_UPDATE_MS = 2ms)
         * -> cap nhat ring buffer 5x nhanh hon main loop
         * -> g_ir_L2..R2 luon fresh khi LineFollow_PID() doc
         */
        ADC_UpdateIR();

        /* Main loop 10ms */
        if ((now - loop_tick) >= 10)
        {
            loop_tick = now;

            /* ADC_UpdateIR() da chuyen ra ngoai, khong goi o day nua */
            Encoder_UpdateSpeed();

            if (g_mode == MODE_AUTO && g_running && !g_tuning)
            {
                LineFollow_PID();
            }
            else if (g_mode == MODE_AUTO && !g_running && !g_tuning)
            {
                Motors_Brake();
            }

            if (g_mode == MODE_MANUAL && !g_tuning)
            {
                ManualTimeoutCheck();
            }
        }

        /* Status print 200ms */
        if ((now - last_print) >= UART_MS)
        {
            last_print = now;
            UART_PrintStatus();
            BT_SendStatus();
        }
    }
}

/* ============================================================
 * SYSTEM CLOCK CONFIG
 * HSI -> PLL -> 84 MHz
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/* ============================================================
 * DMA INIT
 * ============================================================ */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_adc1.Instance                 = DMA2_Stream0;
    hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
        Error_Handler();

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* ============================================================
 * ADC1 INIT
 * Scan 5 kenh lien tuc qua DMA
 * Rank1: CH9  = PB1 = L2
 * Rank2: CH14 = PC4 = L1
 * Rank3: CH15 = PC5 = C
 * Rank4: CH6  = PA6 = R1
 * Rank5: CH13 = PC3 = R2
 * ============================================================ */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig   = {0};
    GPIO_InitTypeDef       GPIO_Init = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_Init.Mode = GPIO_MODE_ANALOG;
    GPIO_Init.Pull = GPIO_NOPULL;

    GPIO_Init.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    GPIO_Init.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);

    GPIO_Init.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = ADC_CHANNELS;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = EOC_SEQ_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    sConfig.Channel = ADC_CHANNEL_9;  sConfig.Rank = 1; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CHANNEL_14; sConfig.Rank = 2; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CHANNEL_15; sConfig.Rank = 3; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CHANNEL_6;  sConfig.Rank = 4; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CHANNEL_13; sConfig.Rank = 5; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/* ============================================================
 * TIM2 - Left Encoder (PA0 PA1)
 * ============================================================ */
static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig   = {0};
    GPIO_InitTypeDef        GPIO_Init = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_Init.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_Init.Mode      = GPIO_MODE_AF_PP;
    GPIO_Init.Pull      = GPIO_PULLUP;
    GPIO_Init.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_Init.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 65535;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 10;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 10;

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
        Error_Handler();
}

/* ============================================================
 * TIM3 - PWM Motor A (PB0 = CH3) + Motor B (PA7 = CH2)
 * ============================================================ */
static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef   GPIO_Init = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_Init.Pin       = GPIO_PIN_7;
    GPIO_Init.Mode      = GPIO_MODE_AF_PP;
    GPIO_Init.Pull      = GPIO_NOPULL;
    GPIO_Init.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_Init.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    GPIO_Init.Pin       = GPIO_PIN_0;
    GPIO_Init.Mode      = GPIO_MODE_AF_PP;
    GPIO_Init.Pull      = GPIO_NOPULL;
    GPIO_Init.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_Init.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();
}

/* ============================================================
 * TIM4 - Right Encoder (PB6 PB7)
 * ============================================================ */
static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig   = {0};
    GPIO_InitTypeDef        GPIO_Init = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_Init.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_Init.Mode      = GPIO_MODE_AF_PP;
    GPIO_Init.Pull      = GPIO_PULLUP;
    GPIO_Init.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_Init.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 10;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 10;

    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
        Error_Handler();
}

/* ============================================================
 * USART1 - HC-05 Bluetooth (PA9 TX, PA10 RX) 38400 baud
 * ============================================================ */
static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_Init = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_Init.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_Init.Mode      = GPIO_MODE_AF_PP;
    GPIO_Init.Pull      = GPIO_NOPULL;
    GPIO_Init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Init.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 38400;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

/* ============================================================
 * USART2 - Debug (PA2 TX, PA3 RX) 115200 baud
 * ============================================================ */
static void MX_USART2_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_Init = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_Init.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_Init.Mode      = GPIO_MODE_AF_PP;
    GPIO_Init.Pull      = GPIO_NOPULL;
    GPIO_Init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Init.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

/* ============================================================
 * GPIO INIT
 * ============================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_Init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 | GPIO_PIN_13, GPIO_PIN_RESET);

    GPIO_Init.Pin   = GPIO_PIN_5;
    GPIO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull  = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    GPIO_Init.Pin   = GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_15;
    GPIO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull  = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);

    GPIO_Init.Pin   = GPIO_PIN_9 | GPIO_PIN_13;
    GPIO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull  = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);
}

/* ============================================================
 * ERROR HANDLER
 * ============================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
