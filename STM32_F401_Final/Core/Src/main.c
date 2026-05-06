/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Line Follower PID - TB6612 + TCRT5000 x5 (ADC DMA)
 *                   + Encoder GA12-N20 TIM2/TIM4 + Closed-loop balance + HC-05
 *
 * ============================================================
 * PINOUT
 * ============================================================
 * PWM Motor (TIM3):
 *   PB0 (TIM3_CH3) = PWMA - Motor A (Left)
 *   PA7 (TIM3_CH2) = PWMB - Motor B (Right)
 *
 * TB6612 Direction GPIO:
 *   PB13 = AIN1,  PB14 = AIN2
 *   PB15 = BIN1,  PB10 = BIN2
 *   PC9  = STBY
 *
 * Encoder GA12-N20 (Quadrature):
 *   PA0 (TIM2_CH1) = Left  Encoder A
 *   PA1 (TIM2_CH2) = Left  Encoder B
 *   PB6 (TIM4_CH1) = Right Encoder A
 *   PB7 (TIM4_CH2) = Right Encoder B
 *
 * IR Sensor TCRT5000 - ADC1 DMA (0-4095, line = low):
 *   PB1  ADC1_CH9  = L2 (Far Left)
 *   PC4  ADC1_CH14 = L1 (Left)
 *   PC5  ADC1_CH15 = C  (Center)
 *   PA6  ADC1_CH6  = R1 (Right)
 *   PC3  ADC1_CH13 = R2 (Far Right)
 *
 * UART:
 *   PA9  TX1 / PA10 RX1 = HC-05 Bluetooth  @ 38400
 *   PA2  TX2 / PA3  RX2 = ST-Link Debug    @ 115200
 *
 * Other:
 *   PC13 = Button B1 (Toggle Start/Stop)
 *   PA5  = LED LD2 (run status)
 *
 * ============================================================
 * BLUETOOTH COMMANDS
 * ============================================================
 * Enter Tuning Mode: T    Exit: Q
 *
 * Line PID:
 *   P2.5   Kp        I0.01  Ki       D1.2   Kd
 *   B40    BASE_SPEED         O30    LOST_SPEED
 *   W9.5   W_OUTER            N2.0   W_INNER
 *   Z1500  IR threshold (0-4095, default 2000)
 *
 * Motor balance:
 *   G3.5   BIAS_A Motor A +3.5%  (manual, only when C0)
 *   H-2.0  BIAS_B Motor B -2.0%  (manual, only when C0)
 *   C1/C0  Enable/disable closed-loop encoder balance
 *   E1.5   Closed-loop Kp
 *   F0.8   Closed-loop Kd
 *
 * Manual drive:
 *   V40    MANUAL_SPEED    X250  MANUAL_TIMEOUT
 *
 * Other:
 *   R   Reset defaults    Q   Exit tuning
 *   S   Toggle Start/Stop (AUTO mode only)
 *   A/M Switch AUTO/MANUAL mode
 *   I/K/J/L  Manual drive (MANUAL mode)
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* =========================================================================
 * Handles
 * =========================================================================*/
TIM_HandleTypeDef  htim2;    /* Left  Encoder */
TIM_HandleTypeDef  htim3;    /* PWM Motor     */
TIM_HandleTypeDef  htim4;    /* Right Encoder */
UART_HandleTypeDef huart1;   /* HC-05         */
UART_HandleTypeDef huart2;   /* Debug         */
ADC_HandleTypeDef  hadc1;    /* IR x5         */
DMA_HandleTypeDef  hdma_adc1;

/* =========================================================================
 * Default values (used on Reset command R)
 * =========================================================================*/
#define DEFAULT_KP                12.0f
#define DEFAULT_KI                0.0f
#define DEFAULT_KD                3.5f
#define DEFAULT_BASE_SPEED        35
#define DEFAULT_TURN_LOST         40
#define DEFAULT_MANUAL_SPEED      30
#define DEFAULT_MANUAL_TIMEOUT    200
#define DEFAULT_W_OUTER           8.0f
#define DEFAULT_W_INNER           1.5f
#define DEFAULT_BIAS_A            0.0f
#define DEFAULT_BIAS_B            0.0f
#define DEFAULT_CL_ENABLED        0
#define DEFAULT_CL_KP             1.5f
#define DEFAULT_CL_KD             0.8f
#define DEFAULT_IR_THRESHOLD      2000

/*
 * Encoder GA12-N20:
 *   Change ENCODER_PPR and GEAR_RATIO to match your motor label.
 *   Common values: 7, 11, 12, 20 PPR | gear ratio: 30, 50, 100, 150
 *   counts/wheel rev = PPR * 4 (quadrature x4) * GEAR_RATIO
 */
#define ENCODER_PPR               11
#define GEAR_RATIO                30
#define ENCODER_COUNTS_PER_REV    (ENCODER_PPR * 4 * GEAR_RATIO)
#define SPEED_CALC_MS             20

/* =========================================================================
 * Runtime parameters
 * =========================================================================*/
static float    g_kp             = DEFAULT_KP;
static float    g_ki             = DEFAULT_KI;
static float    g_kd             = DEFAULT_KD;
static int8_t   g_base_speed     = DEFAULT_BASE_SPEED;
static int8_t   g_turn_lost      = DEFAULT_TURN_LOST;
static int8_t   g_manual_speed   = DEFAULT_MANUAL_SPEED;
static uint32_t g_manual_timeout = DEFAULT_MANUAL_TIMEOUT;
static float    g_w_outer        = DEFAULT_W_OUTER;
static float    g_w_inner        = DEFAULT_W_INNER;
static float    g_bias_a         = DEFAULT_BIAS_A;
static float    g_bias_b         = DEFAULT_BIAS_B;
static uint8_t  g_cl_enabled     = DEFAULT_CL_ENABLED;
static float    g_cl_kp          = DEFAULT_CL_KP;
static float    g_cl_kd          = DEFAULT_CL_KD;
static uint16_t g_ir_threshold   = DEFAULT_IR_THRESHOLD;

/* Line PID */
static float    g_error         = 0.0f;
static float    g_last_error    = 0.0f;
static float    g_integral      = 0.0f;
static uint32_t g_last_pid_time = 0;

/* =========================================================================
 * ADC DMA buffer - 5 channels in rank order
 * [0]=L2(CH9) [1]=L1(CH14) [2]=C(CH15) [3]=R1(CH6) [4]=R2(CH13)
 * =========================================================================*/
#define ADC_CHANNELS  5
static volatile uint16_t g_adc_buf[ADC_CHANNELS];
static uint16_t g_ir_L2, g_ir_L1, g_ir_C, g_ir_R1, g_ir_R2;

/* =========================================================================
 * Encoder & Speed
 * =========================================================================*/
static int32_t  g_enc_left_prev   = 0;
static int32_t  g_enc_right_prev  = 0;
static float    g_rpm_left        = 0.0f;
static float    g_rpm_right       = 0.0f;
static uint32_t g_speed_last_tick = 0;

/* Closed-loop balance */
static float    g_cl_error_prev  = 0.0f;
static float    g_cl_correction  = 0.0f;

/* =========================================================================
 * Misc constants
 * =========================================================================*/
#define DEBOUNCE_MS  200
#define UART_MS      200

/* =========================================================================
 * Types & state
 * =========================================================================*/
typedef enum { MODE_AUTO = 0, MODE_MANUAL = 1 } DriveMode_t;

#define BT_BUF_SIZE  16
static uint8_t  g_bt_rx_byte     = 0;
static char     g_bt_cmd_buf[BT_BUF_SIZE];
static uint8_t  g_bt_cmd_len     = 0;

volatile uint8_t     g_running          = 0;
volatile uint32_t    g_btn_last_tick    = 0;
volatile DriveMode_t g_mode             = MODE_AUTO;
volatile uint32_t    g_bt_last_cmd_tick = 0;
volatile uint8_t     g_manual_active    = 0;
volatile uint8_t     g_tuning           = 0;
static   uint8_t     g_resume_run       = 0;

static int8_t  g_spd_A = 0;
static int8_t  g_spd_B = 0;
static char    g_state[20] = "STOP";

/* =========================================================================
 * Prototypes
 * =========================================================================*/
void SystemClock_Config(void);
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

/* =========================================================================
 * Redirect printf to UART2
 * =========================================================================*/
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void BT_Send(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/* =========================================================================
 * TB6612 STBY
 * =========================================================================*/
void TB6612_Enable(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   }
void TB6612_Disable(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); }

/* =========================================================================
 * MotorA_SetSpeed  (Left wheel)
 *   Closed-loop ON  -> apply g_cl_correction from encoder
 *   Closed-loop OFF -> apply g_bias_a (manual)
 * =========================================================================*/
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

    if (speed > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)biased);
}

/* =========================================================================
 * MotorB_SetSpeed  (Right wheel)
 *   Closed-loop: Motor B unchanged (only A is corrected)
 *   Manual: apply g_bias_b
 * =========================================================================*/
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

    if (speed > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)biased);
}

void Motors_Stop(void)
{
    g_spd_A = 0; g_spd_B = 0;
    HAL_GPIO_WritePin(GPIOB,
        GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}

void Motors_Brake(void)
{
    g_spd_A = 0; g_spd_B = 0;
    HAL_GPIO_WritePin(GPIOB,
        GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}

/* =========================================================================
 * ADC_UpdateIR - copy DMA buffer to readable variables
 * =========================================================================*/
void ADC_UpdateIR(void)
{
    g_ir_L2 = g_adc_buf[0];
    g_ir_L1 = g_adc_buf[1];
    g_ir_C  = g_adc_buf[2];
    g_ir_R1 = g_adc_buf[3];
    g_ir_R2 = g_adc_buf[4];
}

/* =========================================================================
 * Encoder_UpdateSpeed
 * Called every loop iteration, internally throttled to SPEED_CALC_MS.
 *
 * RPM = (delta_counts / COUNTS_PER_REV) / dt_s * 60  (wheel shaft RPM)
 *
 * Closed-loop PD balance:
 *   error = rpm_left - rpm_right
 *   > 0 -> left faster  -> reduce Motor A (negative correction)
 *   < 0 -> right faster -> boost  Motor A (positive correction)
 * =========================================================================*/
void Encoder_UpdateSpeed(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - g_speed_last_tick) < SPEED_CALC_MS) return;

    float dt_s = (now - g_speed_last_tick) / 1000.0f;
    g_speed_last_tick = now;

    int32_t cnt_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t cnt_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    int32_t delta_left  = cnt_left  - g_enc_left_prev;
    int32_t delta_right = cnt_right - g_enc_right_prev;

    /* 16-bit wrap-around handling */
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

/* =========================================================================
 * ToggleRunning - shared between physical button and BT command S
 * =========================================================================*/
void ToggleRunning(void)
{
    if (g_mode != MODE_AUTO) return;
    if (g_tuning) return;

    g_running ^= 1;

    if (g_running) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        g_integral      = 0;
        g_last_error    = 0;
        g_cl_correction = 0;
        g_cl_error_prev = 0;
        strncpy(g_state, "PID_RUN        ", sizeof(g_state));
        printf("\r\n=== AUTO: START ===\r\n");
        BT_Send("$CMD,START\r\n");
    } else {
        Motors_Brake();
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        strncpy(g_state, "STOP           ", sizeof(g_state));
        printf("\r\n=== AUTO: STOP (BRAKE) ===\r\n");
        BT_Send("$CMD,STOP\r\n");
    }
}

/* =========================================================================
 * Tuning_ResetDefaults
 * =========================================================================*/
void Tuning_ResetDefaults(void)
{
    g_kp             = DEFAULT_KP;
    g_ki             = DEFAULT_KI;
    g_kd             = DEFAULT_KD;
    g_base_speed     = DEFAULT_BASE_SPEED;
    g_turn_lost      = DEFAULT_TURN_LOST;
    g_manual_speed   = DEFAULT_MANUAL_SPEED;
    g_manual_timeout = DEFAULT_MANUAL_TIMEOUT;
    g_w_outer        = DEFAULT_W_OUTER;
    g_w_inner        = DEFAULT_W_INNER;
    g_bias_a         = DEFAULT_BIAS_A;
    g_bias_b         = DEFAULT_BIAS_B;
    g_cl_enabled     = DEFAULT_CL_ENABLED;
    g_cl_kp          = DEFAULT_CL_KP;
    g_cl_kd          = DEFAULT_CL_KD;
    g_ir_threshold   = DEFAULT_IR_THRESHOLD;
    g_integral       = 0;
    g_last_error     = 0;
    g_cl_correction  = 0;
    g_cl_error_prev  = 0;

    printf("[TUNE] Reset to defaults\r\n");
    BT_Send("$TUNE,RESET_OK\r\n");
    BT_SendTuningMenu();
}

/* =========================================================================
 * BT_SendTuningMenu
 * =========================================================================*/
void BT_SendTuningMenu(void)
{
    char buf[160];

    BT_Send("$TUNE_MENU_START\r\n");
    BT_Send("$TUNE,---- STOPPED - TUNING PID ----\r\n");

    snprintf(buf, sizeof(buf), "$TUNE,Kp=%.2f Ki=%.3f Kd=%.2f\r\n",
             g_kp, g_ki, g_kd);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,W_OUT=%.2f W_IN=%.2f THR=%d\r\n",
             g_w_outer, g_w_inner, g_ir_threshold);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,BASE=%d LOST=%d\r\n",
             g_base_speed, g_turn_lost);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,MAN_SPD=%d TIMEOUT=%lu\r\n",
             g_manual_speed, (unsigned long)g_manual_timeout);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,BIAS_A=%+.2f%% BIAS_B=%+.2f%% (manual, C0 only)\r\n",
             g_bias_a, g_bias_b);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,CLOSED_LOOP=%s CL_Kp=%.2f CL_Kd=%.2f\r\n",
             g_cl_enabled ? "ON " : "OFF", g_cl_kp, g_cl_kd);
    BT_Send(buf);

    snprintf(buf, sizeof(buf), "$TUNE,RPM_L=%+.1f RPM_R=%+.1f CORR=%+.1f%%\r\n",
             g_rpm_left, g_rpm_right, g_cl_correction);
    BT_Send(buf);

    BT_Send("$TUNE,----------------------------------\r\n");
    BT_Send("$TUNE_CMD: P=Kp I=Ki D=Kd\r\n");
    BT_Send("$TUNE_CMD: W=W_Outer N=W_Inner Z=IRthresh\r\n");
    BT_Send("$TUNE_CMD: G=BiasA(%) H=BiasB(%)\r\n");
    BT_Send("$TUNE_CMD: C1/C0=ClosedLoop E=CL_Kp F=CL_Kd\r\n");
    BT_Send("$TUNE_CMD: B=BaseSpd O=LostSpd V=ManSpd X=Timeout\r\n");
    BT_Send("$TUNE_CMD: R=Reset  Q=Exit+Resume\r\n");
    BT_Send("$TUNE_MENU_END\r\n");
}

/* =========================================================================
 * EnterTuningMode / ExitTuningMode
 * =========================================================================*/
void EnterTuningMode(void)
{
    if (g_tuning) { BT_SendTuningMenu(); return; }
    g_resume_run    = (uint8_t)g_running;
    g_running       = 0;
    g_manual_active = 0;
    Motors_Brake();
    strncpy(g_state, "TUNING         ", sizeof(g_state));
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    g_tuning = 1;
    printf("\r\n=== TUNING MODE ===\r\n");
    BT_Send("$TUNING_START\r\n");
    BT_SendTuningMenu();
}

void ExitTuningMode(void)
{
    if (!g_tuning) return;
    g_tuning        = 0;
    g_integral      = 0;
    g_last_error    = 0;
    g_cl_correction = 0;

    if (g_resume_run && g_mode == MODE_AUTO) {
        g_running = 1;
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        strncpy(g_state, "PID_RUN        ", sizeof(g_state));
        printf("\r\n=== TUNING EXIT: Resume AUTO ===\r\n");
        BT_Send("$TUNING_EXIT,RESUME_RUN\r\n");
    } else {
        g_running = 0;
        strncpy(g_state, "STOP           ", sizeof(g_state));
        printf("\r\n=== TUNING EXIT: Stopped ===\r\n");
        BT_Send("$TUNING_EXIT,STOPPED\r\n");
    }
    g_resume_run = 0;
}

/* =========================================================================
 * BT_ProcessCommand
 * =========================================================================*/
void BT_ProcessCommand(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') return;

    uint8_t c    = (uint8_t)cmd[0];
    float   fval = (cmd[1] != '\0') ? atof(&cmd[1]) : -999.0f;
    int     val  = (int)fval;

    if (c == 'T' || c == 't') { EnterTuningMode(); return; }

    /* ---- TUNING MODE ---- */
    if (g_tuning)
    {
        char ack[80];
        if (c == 'Q' || c == 'q') { ExitTuningMode(); return; }
        if (c == 'R' || c == 'r') { Tuning_ResetDefaults(); return; }

        if (cmd[1] != '\0')
        {
            switch (c)
            {
                case 'P': case 'p':
                    g_kp = (fval < 0) ? 0 : fval;
                    snprintf(ack, sizeof(ack), "$TUNE,Kp=%.2f OK\r\n", g_kp);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'I': case 'i':
                    g_ki = (fval < 0) ? 0 : fval;
                    snprintf(ack, sizeof(ack), "$TUNE,Ki=%.3f OK\r\n", g_ki);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'D': case 'd':
                    g_kd = (fval < 0) ? 0 : fval;
                    snprintf(ack, sizeof(ack), "$TUNE,Kd=%.2f OK\r\n", g_kd);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'B': case 'b':
                    g_base_speed = (int8_t)val;
                    snprintf(ack, sizeof(ack), "$TUNE,BASE=%d OK\r\n", g_base_speed);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'O': case 'o':
                    g_turn_lost = (int8_t)val;
                    snprintf(ack, sizeof(ack), "$TUNE,LOST=%d OK\r\n", g_turn_lost);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'V': case 'v':
                    g_manual_speed = (int8_t)val;
                    snprintf(ack, sizeof(ack), "$TUNE,MAN=%d OK\r\n", g_manual_speed);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'X': case 'x':
                    g_manual_timeout = (uint32_t)val;
                    snprintf(ack, sizeof(ack), "$TUNE,TIMEOUT=%lu OK\r\n",
                             (unsigned long)g_manual_timeout);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'W': case 'w':
                    g_w_outer = (fval < 0) ? 0 : fval;
                    snprintf(ack, sizeof(ack), "$TUNE,W_OUT=%.2f OK\r\n", g_w_outer);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'N': case 'n':
                    g_w_inner = (fval < 0) ? 0 : fval;
                    snprintf(ack, sizeof(ack), "$TUNE,W_IN=%.2f OK\r\n", g_w_inner);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'Z': case 'z':
                    g_ir_threshold = (uint16_t)val;
                    snprintf(ack, sizeof(ack), "$TUNE,THR=%d OK\r\n", g_ir_threshold);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'G': case 'g':
                    if (fval >  20.0f) fval =  20.0f;
                    if (fval < -20.0f) fval = -20.0f;
                    g_bias_a = fval;
                    snprintf(ack, sizeof(ack), "$TUNE,BIAS_A=%+.2f%% OK\r\n", g_bias_a);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'H': case 'h':
                    if (fval >  20.0f) fval =  20.0f;
                    if (fval < -20.0f) fval = -20.0f;
                    g_bias_b = fval;
                    snprintf(ack, sizeof(ack), "$TUNE,BIAS_B=%+.2f%% OK\r\n", g_bias_b);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'C': case 'c':
                    g_cl_enabled    = (val != 0) ? 1 : 0;
                    g_cl_correction = 0;
                    g_cl_error_prev = 0;
                    snprintf(ack, sizeof(ack), "$TUNE,CLOSED_LOOP=%s OK\r\n",
                             g_cl_enabled ? "ON" : "OFF");
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'E': case 'e':
                    g_cl_kp = (fval < 0) ? 0 : fval;
                    snprintf(ack, sizeof(ack), "$TUNE,CL_Kp=%.2f OK\r\n", g_cl_kp);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                case 'F': case 'f':
                    g_cl_kd = (fval < 0) ? 0 : fval;
                    snprintf(ack, sizeof(ack), "$TUNE,CL_Kd=%.2f OK\r\n", g_cl_kd);
                    BT_Send(ack); BT_SendTuningMenu(); break;
                default:
                    BT_Send("$ERR,Unknown cmd. Q=exit\r\n"); break;
            }
        }
        else {
            BT_Send("$ERR,In TUNING: cmd+value required\r\n");
        }
        return;
    }

    /* ---- NORMAL MODE ---- */
    if (c == 'A' || c == 'a') { SetMode(MODE_AUTO);   return; }
    if (c == 'M' || c == 'm') { SetMode(MODE_MANUAL); return; }
    if (c == 'S' || c == 's') {
        if (g_mode == MODE_AUTO) ToggleRunning();
        return;
    }
    if (g_mode == MODE_MANUAL) ManualDrive(c);
}

/* =========================================================================
 * SetMode
 * =========================================================================*/
void SetMode(DriveMode_t mode)
{
    if (g_tuning) { g_tuning = 0; g_resume_run = 0; }
    g_mode          = mode;
    g_running       = 0;
    g_manual_active = 0;
    g_cl_correction = 0;
    Motors_Brake();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    strncpy(g_state, "STOP           ", sizeof(g_state));

    if (mode == MODE_AUTO) {
        printf("\r\n>>> MODE: AUTO\r\n");
        BT_Send("$MODE,AUTO\r\n");
    } else {
        printf("\r\n>>> MODE: MANUAL\r\n");
        BT_Send("$MODE,MANUAL\r\n");
    }
}

/* =========================================================================
 * ManualDrive
 * =========================================================================*/
void ManualDrive(uint8_t cmd)
{
    switch (cmd)
    {
        case 'I': case 'i':
            strncpy(g_state, "MAN-FORWARD    ", sizeof(g_state));
            MotorA_SetSpeed( g_manual_speed); MotorB_SetSpeed( g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick(); g_manual_active = 1; break;
        case 'K': case 'k':
            strncpy(g_state, "MAN-BACKWARD   ", sizeof(g_state));
            MotorA_SetSpeed(-g_manual_speed); MotorB_SetSpeed(-g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick(); g_manual_active = 1; break;
        case 'J': case 'j':
            strncpy(g_state, "MAN-LEFT       ", sizeof(g_state));
            MotorA_SetSpeed( g_manual_speed); MotorB_SetSpeed(-g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick(); g_manual_active = 1; break;
        case 'L': case 'l':
            strncpy(g_state, "MAN-RIGHT      ", sizeof(g_state));
            MotorA_SetSpeed(-g_manual_speed); MotorB_SetSpeed( g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick(); g_manual_active = 1; break;
        default:
            strncpy(g_state, "MAN-STOP       ", sizeof(g_state));
            g_manual_active = 0; Motors_Brake(); break;
    }
}

void ManualTimeoutCheck(void)
{
    if (g_mode != MODE_MANUAL || !g_manual_active) return;
    if ((HAL_GetTick() - g_bt_last_cmd_tick) >= g_manual_timeout) {
        g_manual_active = 0;
        strncpy(g_state, "MAN-STOP       ", sizeof(g_state));
        Motors_Brake();
    }
}

/* =========================================================================
 * HAL_UART_RxCpltCallback
 * =========================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    uint8_t c = g_bt_rx_byte;

    if (c == '\r' || c == '\n' || c == ';' || c == ' ') {
        if (g_bt_cmd_len > 0) {
            g_bt_cmd_buf[g_bt_cmd_len] = '\0';
            BT_ProcessCommand(g_bt_cmd_buf);
            g_bt_cmd_len = 0;
        }
    } else if (g_bt_cmd_len < BT_BUF_SIZE - 1) {
        g_bt_cmd_buf[g_bt_cmd_len++] = (char)c;

        uint8_t is_single = 0;
        if (g_bt_cmd_len == 1) {
            if (g_tuning) {
                switch (c) {
                    case 'T': case 't':
                    case 'Q': case 'q':
                    case 'R': case 'r':
                        is_single = 1; break;
                }
            } else {
                switch (c) {
                    case 'A': case 'a': case 'M': case 'm':
                    case 'T': case 't': case 'S': case 's':
                    case 'I': case 'i': case 'K': case 'k':
                    case 'J': case 'j': case 'L': case 'l':
                        is_single = 1; break;
                }
            }
        }

        if (is_single) {
            g_bt_cmd_buf[g_bt_cmd_len] = '\0';
            BT_ProcessCommand(g_bt_cmd_buf);
            g_bt_cmd_len = 0;
        }
    } else {
        g_bt_cmd_buf[g_bt_cmd_len] = '\0';
        BT_ProcessCommand(g_bt_cmd_buf);
        g_bt_cmd_len = 0;
    }
    HAL_UART_Receive_IT(&huart1, &g_bt_rx_byte, 1);
}

/* =========================================================================
 * LineFollow_PID
 *   Black line on white background: low ADC = line detected.
 *   For white line on black background: flip comparison (< to >).
 * =========================================================================*/
void LineFollow_PID(void)
{
    ADC_UpdateIR();

    uint8_t sL2 = (g_ir_L2 < g_ir_threshold) ? 1 : 0;
    uint8_t sL1 = (g_ir_L1 < g_ir_threshold) ? 1 : 0;
    uint8_t sC  = (g_ir_C  < g_ir_threshold) ? 1 : 0;
    uint8_t sR1 = (g_ir_R1 < g_ir_threshold) ? 1 : 0;
    uint8_t sR2 = (g_ir_R2 < g_ir_threshold) ? 1 : 0;

    float sum   = 0;
    int   count = 0;

    if (sL2) { sum +=  g_w_outer; count++; }
    if (sL1) { sum +=  g_w_inner; count++; }
    if (sC ) { sum +=  0.0f;      count++; }
    if (sR1) { sum += -g_w_inner; count++; }
    if (sR2) { sum += -g_w_outer; count++; }

    if (count == 0) {
        strncpy(g_state, "LOST_LINE      ", sizeof(g_state));
        if (g_last_error < -2.0f) {
            MotorA_SetSpeed(-g_turn_lost);
            MotorB_SetSpeed( g_turn_lost);
        } else if (g_last_error > 2.0f) {
            MotorA_SetSpeed( g_turn_lost);
            MotorB_SetSpeed(-g_turn_lost);
        } else {
            MotorA_SetSpeed(g_turn_lost);
            MotorB_SetSpeed(g_turn_lost);
        }
        return;
    }
    else if (count >= 4) {
        strncpy(g_state, "CROSS_DETECT   ", sizeof(g_state));
        g_error = 0;
    }
    else {
        g_error = sum / count;
        strncpy(g_state, "PID_TRACKING   ", sizeof(g_state));
    }

    /* ==== PID ==== */
    uint32_t now = HAL_GetTick();
    float dt = (now - g_last_pid_time) / 1000.0f;
    if (dt <= 0.0f) dt = 0.01f;
    g_last_pid_time = now;

    float P = g_kp * g_error;

    g_integral += g_error * dt;
    float max_i = 50.0f;
    if (g_ki > 0) {
        if (g_integral >  max_i / g_ki) g_integral =  max_i / g_ki;
        if (g_integral < -max_i / g_ki) g_integral = -max_i / g_ki;
    }
    float I = g_ki * g_integral;
    float D = g_kd * (g_error - g_last_error) / dt;

    float output = P + I + D;
    g_last_error  = g_error;

    /* Adaptive base speed: slow down on sharp turns */
    int adaptive_base = g_base_speed;
    int turn_penalty  = (int)(fabsf(output) * 1.5f);
    adaptive_base -= turn_penalty;
    if (adaptive_base < 25) adaptive_base = 25;

    int speedA = adaptive_base + (int)output;
    int speedB = adaptive_base - (int)output;

    MotorA_SetSpeed(speedA);
    MotorB_SetSpeed(speedB);
}

/* =========================================================================
 * UART_PrintStatus
 * =========================================================================*/
void UART_PrintStatus(void)
{
    printf("[%s][%s][%s] "
           "IR:%4d %4d %4d %4d %4d | "
           "%-15s | Err:%5.2f | A:%+4d B:%+4d | "
           "RPM L:%+6.1f R:%+6.1f | CL:%s %+.1f%%\r\n",
           g_mode == MODE_AUTO ? "AUTO  " : "MANUAL",
           g_running ? "RUN" : "STP",
           g_tuning  ? "TUNE" : "    ",
           g_ir_L2, g_ir_L1, g_ir_C, g_ir_R1, g_ir_R2,
           g_state, g_error,
           (int)g_spd_A, (int)g_spd_B,
           g_rpm_left, g_rpm_right,
           g_cl_enabled ? "ON " : "OFF",
           g_cl_correction);
}

/* =========================================================================
 * BT_SendStatus
 * =========================================================================*/
void BT_SendStatus(void)
{
    if (g_tuning) return;
    char buf[160];
    snprintf(buf, sizeof(buf),
             "$%s,%s,%d,%d,%d,%d,%d,%s,%.2f,%+d,%+d,RPM:%.1f/%.1f,CL:%s\r\n",
             g_mode == MODE_AUTO ? "AUTO" : "MAN",
             g_running ? "RUN" : "STP",
             g_ir_L2, g_ir_L1, g_ir_C, g_ir_R1, g_ir_R2,
             g_state, g_error,
             (int)g_spd_A, (int)g_spd_B,
             g_rpm_left, g_rpm_right,
             g_cl_enabled ? "ON" : "OFF");
    BT_Send(buf);
}

/* =========================================================================
 * Interrupt Handlers
 * =========================================================================*/
void EXTI15_10_IRQHandler(void)    { HAL_GPIO_EXTI_IRQHandler(B1_Pin); }
void USART1_IRQHandler(void)       { HAL_UART_IRQHandler(&huart1); }
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != B1_Pin) return;
    uint32_t now = HAL_GetTick();
    if ((now - g_btn_last_tick) < DEBOUNCE_MS) return;
    g_btn_last_tick = now;
    ToggleRunning();
}

/* =========================================================================
 * main()
 * =========================================================================*/
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

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    SetMode(MODE_AUTO);
    HAL_UART_Receive_IT(&huart1, &g_bt_rx_byte, 1);

    printf("\r\n========================================\r\n");
    printf("  Line Follower PID  STM32F401RE\r\n");
    printf("  IR: ADC DMA x5  |  Encoder: TIM2/TIM4\r\n");
    printf("  PPR=%d  GearRatio=%d  CPR=%d\r\n",
           ENCODER_PPR, GEAR_RATIO, ENCODER_COUNTS_PER_REV);
    printf("  Closed-loop: C1=ON  C0=OFF\r\n");
    printf("  Start/Stop : S (BT) or BUTTON\r\n");
    printf("========================================\r\n\r\n");

    BT_Send("$BOOT,LineFollower_F401\r\n");
    BT_SendTuningMenu();

    uint32_t last_print = 0;
    uint32_t loop_tick  = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (now - loop_tick >= 10)  /* 100 Hz */
        {
            loop_tick = now;
            Encoder_UpdateSpeed();

            if (g_mode == MODE_AUTO && g_running && !g_tuning)
                LineFollow_PID();

            if (g_mode == MODE_MANUAL && !g_tuning)
                ManualTimeoutCheck();
        }

        if ((now - last_print) >= UART_MS) {
            last_print = now;
            UART_PrintStatus();
            BT_SendStatus();
        }
    }
}

/* =========================================================================
 * Peripheral Init
 * =========================================================================*/
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------
 * DMA2 - must be init before ADC
 * -------------------------------------------------------------------------*/
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
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* -------------------------------------------------------------------------
 * ADC1 - 5ch scan DMA circular
 * Rank1=CH9(PB1/L2) Rank2=CH14(PC4/L1) Rank3=CH15(PC5/C)
 * Rank4=CH6(PA6/R1) Rank5=CH13(PC3/R2)
 * -------------------------------------------------------------------------*/
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig      = {0};
    GPIO_InitTypeDef       GPIO_InitStruct = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = GPIO_PIN_6;               /* PA6 = R1 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;               /* PB1 = L2 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5; /* PC3=R2 PC4=L1 PC5=C */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;

    sConfig.Channel = ADC_CHANNEL_9;  sConfig.Rank = 1;  /* PB1  L2 */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    sConfig.Channel = ADC_CHANNEL_14; sConfig.Rank = 2;  /* PC4  L1 */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    sConfig.Channel = ADC_CHANNEL_15; sConfig.Rank = 3;  /* PC5  C  */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    sConfig.Channel = ADC_CHANNEL_6;  sConfig.Rank = 4;  /* PA6  R1 */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    sConfig.Channel = ADC_CHANNEL_13; sConfig.Rank = 5;  /* PC3  R2 */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------
 * TIM2 - Left Encoder (PA0=CH1, PA1=CH2, AF1)
 * -------------------------------------------------------------------------*/
static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig      = {0};
    GPIO_InitTypeDef        GPIO_InitStruct = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------
 * TIM3 - PWM Motor (PA7=CH2/MotorB, PB0=CH3/MotorA)
 * GPIO configured inside HAL_TIM_MspPostInit()
 * -------------------------------------------------------------------------*/
static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef      sConfigOC     = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim3);
}

/* -------------------------------------------------------------------------
 * TIM4 - Right Encoder (PB6=CH1, PB7=CH2, AF2)
 * -------------------------------------------------------------------------*/
static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig      = {0};
    GPIO_InitTypeDef        GPIO_InitStruct = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------
 * USART1 - HC-05 @ 38400 (PA9 TX, PA10 RX)
 * -------------------------------------------------------------------------*/
static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 38400;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------
 * USART2 - Debug @ 115200
 * -------------------------------------------------------------------------*/
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* -------------------------------------------------------------------------
 * GPIO Init
 * -------------------------------------------------------------------------*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Default LOW */
    HAL_GPIO_WritePin(GPIOB,
        GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /* Button B1 - EXTI falling */
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* TB6612 Direction + STBY */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* LED LD2 */
    GPIO_InitStruct.Pin = LD2_Pin;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* -------------------------------------------------------------------------
 * Error Handler
 * -------------------------------------------------------------------------*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
