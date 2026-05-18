/* USER CODE BEGIN Header */
/**
 * *****************************************************************************
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
 *   PA0 PA1 = Left  Encoder TIM2
 *   PB6 PB7 = Right Encoder TIM4
 *
 * IR Sensor (TCRT5000 - line den nen trang):
 *   Nen trang -> ADC thap  (phan xa nhieu)
 *   Line den  -> ADC cao   (hap thu anh sang)
 *   => Phat hien line khi ADC > threshold
 *
 *   PB1 = L2  (ADC1_CH9)   -> Rank 1
 *   PC4 = L1  (ADC1_CH14)  -> Rank 2
 *   PC5 = C   (ADC1_CH15)  -> Rank 3
 *   PA6 = R1  (ADC1_CH6)   -> Rank 4
 *   PC3 = R2  (ADC1_CH13)  -> Rank 5
 *   DMA2 Stream0 Ch0 -> g_adc_raw[] circular, KHONG block CPU
 *
 * UART:
 *   PA9  PA10 = USART1 HC-05
 *   PA2  PA3  = USART2 Debug
 *
 * LED:
 *   PA5 = LD2
 *
 * ============================================================
 * LENH BT RUNTIME (TUNING MODE - gui 'T' de vao):
 *   P2.5   -> Kp = 2.5
 *   I0.01  -> Ki = 0.01
 *   D1.2   -> Kd = 1.2
 *   B40    -> BASE_SPEED = 40
 *   O30    -> SPEED khi mat line
 *   V40    -> MANUAL_SPEED = 40
 *   X250   -> MANUAL_TIMEOUT_MS = 250
 *   H500   -> ADC threshold
 *   Z3     -> RAMP_STEP (tang toc moi chu ky 10ms)
 *   N15    -> RAMP_MIN_START (toc do khoi dong)
 *   W1<v>  -> Trong so sensor L2  (vi du: W18.0)
 *   W2<v>  -> Trong so sensor L1  (vi du: W21.5)
 *   W3<v>  -> Trong so sensor C   (vi du: W30.0)
 *   W4<v>  -> Trong so sensor R1  (vi du: W4-1.5)
 *   W5<v>  -> Trong so sensor R2  (vi du: W5-8.0)
 *   R      -> Reset tat ca ve mac dinh
 *   Q      -> Thoat Tuning, tiep tuc chay
 *
 * LENH DIEU KHIEN CHINH:
 *   A  -> AUTO mode
 *   M  -> MANUAL mode
 *   G  -> GO  (AUTO mode)
 *   S  -> STOP
 *   T  -> Vao Tuning mode
 *   I/K/J/L -> Tien/Lui/Trai/Phai (MANUAL mode)
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* =========================================================================
 * Handle ngoai vi
 * =========================================================================*/
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim3;
TIM_HandleTypeDef  htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;

/* =========================================================================
 * Gia tri mac dinh
 * =========================================================================*/
#define DEFAULT_KP               12.0f
#define DEFAULT_KI                0.0f
#define DEFAULT_KD                5.0f
#define DEFAULT_BASE_SPEED        60
#define DEFAULT_TURN_LOST         40
#define DEFAULT_MANUAL_SPEED      60
#define DEFAULT_MANUAL_TIMEOUT   200
#define DEFAULT_ADC_THRESHOLD   1000

/* Soft start mac dinh */
#define DEFAULT_RAMP_STEP         2      /* Tang toc do moi chu ky PID 10ms  */
#define DEFAULT_RAMP_MIN_START   15      /* Toc do khoi dong ban dau (%)      */

/* Trong so sensor mac dinh: Trai duong, Phai am, Trung tam = 0 */
#define DEFAULT_W0   -12.0f    /* L2  */
#define DEFAULT_W1    1.5f    /* L1  */
#define DEFAULT_W2    0.0f     /* C   */
#define DEFAULT_W3   -1.5f    /* R1  */
#define DEFAULT_W4    12.0f    /* R2  */

/* =========================================================================
 * Chieu quay motor
 *   +1 = chieu binh thuong
 *   -1 = dao nguoc
 * =========================================================================*/
#define MOTOR_A_DIR   (+1)
#define MOTOR_B_DIR   (+1)

/* =========================================================================
 * Thong so runtime PID
 * =========================================================================*/
static float    g_kp             = DEFAULT_KP;
static float    g_ki             = DEFAULT_KI;
static float    g_kd             = DEFAULT_KD;
static int8_t   g_base_speed     = DEFAULT_BASE_SPEED;
static int8_t   g_turn_lost      = DEFAULT_TURN_LOST;
static int8_t   g_manual_speed   = DEFAULT_MANUAL_SPEED;
static uint32_t g_manual_timeout = DEFAULT_MANUAL_TIMEOUT;
static uint16_t g_adc_threshold  = DEFAULT_ADC_THRESHOLD;

/* Trong so 5 cam bien */
static float g_weight[5] = {
    DEFAULT_W0, DEFAULT_W1, DEFAULT_W2, DEFAULT_W3, DEFAULT_W4
};

/* Bien PID */
static float    g_error         = 0.0f;
static float    g_last_error    = 0.0f;
static float    g_integral      = 0.0f;
static uint32_t g_last_pid_time = 0;

/* =========================================================================
 * Soft Start / Ramp
 *   g_ramp_speed : toc do hien tai dang duoc ap dung (tang dan -> g_base_speed)
 *   g_ramping    : 1 = dang trong giai doan tang toc, 0 = da dat toc do muc tieu
 *   g_ramp_step  : muc tang moi chu ky PID 10ms (chinh qua lenh Z)
 *   g_ramp_min   : toc do khoi dong (chinh qua lenh N)
 * =========================================================================*/
static uint8_t  g_ramp_speed = 0;
static uint8_t  g_ramping    = 0;
static uint8_t  g_ramp_step  = DEFAULT_RAMP_STEP;
static uint8_t  g_ramp_min   = DEFAULT_RAMP_MIN_START;

/* =========================================================================
 * ADC - TCRT5000 x5
 *   Thu tu DMA: [0]=L2 [1]=L1 [2]=C [3]=R1 [4]=R2
 * =========================================================================*/
#define SENSOR_COUNT  5
static volatile uint16_t g_adc_raw[SENSOR_COUNT];
static uint8_t g_L2, g_L1, g_C, g_R1, g_R2;

/* =========================================================================
 * Encoder
 * =========================================================================*/
static int32_t g_enc_left  = 0;
static int32_t g_enc_right = 0;

/* =========================================================================
 * Hang so khac
 * =========================================================================*/
#define DEBOUNCE_MS   200
#define UART_MS       200

/* =========================================================================
 * Che do hoat dong
 * =========================================================================*/
typedef enum {
    MODE_AUTO   = 0,
    MODE_MANUAL = 1
} DriveMode_t;

/* =========================================================================
 * Buffer nhan lenh BT
 * =========================================================================*/
#define BT_BUF_SIZE  16
static uint8_t g_bt_rx_byte = 0;
static char    g_bt_cmd_buf[BT_BUF_SIZE];
static uint8_t g_bt_cmd_len = 0;

/* =========================================================================
 * Trang thai xe
 * =========================================================================*/
volatile uint8_t      g_running          = 0;
volatile uint32_t     g_btn_last_tick    = 0;
volatile DriveMode_t  g_mode             = MODE_AUTO;
volatile uint32_t     g_bt_last_cmd_tick = 0;
volatile uint8_t      g_manual_active    = 0;
volatile uint8_t      g_tuning           = 0;
static   uint8_t      g_resume_run       = 0;

static int8_t g_spd_A = 0;
static int8_t g_spd_B = 0;
static char   g_state[20] = "STOP";

/* =========================================================================
 * Function prototypes
 * =========================================================================*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

void TB6612_Enable(void);
void TB6612_Disable(void);
void MotorA_SetSpeed(int speed);
void MotorB_SetSpeed(int speed);
void Motors_Stop(void);
void Motors_Brake(void);

void ADC_ReadSensors(void);
void LineFollow_PID(void);
void Ramp_Reset(void);
void Ramp_Update_Manual(void);

void UART_PrintStatus(void);
void BT_Send(const char *str);
void BT_SendStatus(void);
void BT_SendTuningMenu(void);

void SetMode(DriveMode_t mode);
void ManualDrive(uint8_t cmd);
void ManualTimeoutCheck(void);
void BT_ProcessCommand(const char *cmd);
void Tuning_ResetDefaults(void);
void EnterTuningMode(void);
void ExitTuningMode(void);

/* =========================================================================
 * Redirect printf -> UART2 (debug)
 * =========================================================================*/
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* =========================================================================
 * BT helper
 * =========================================================================*/
void BT_Send(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/* =========================================================================
 * TB6612
 * =========================================================================*/
void TB6612_Enable(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   }
void TB6612_Disable(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); }

/* =========================================================================
 * Motor A (Trai) - PWMA = TIM3_CH3 (PB0)
 *   AIN1=PC13  AIN2=PB3
 *   Tien: AIN1=0 AIN2=1  Lui: AIN1=1 AIN2=0  Brake: AIN1=1 AIN2=1
 * =========================================================================*/
void MotorA_SetSpeed(int speed)
{
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;
    g_spd_A = (int8_t)speed;

    int      dir_speed = speed * MOTOR_A_DIR;
    uint32_t pwm       = (uint32_t)(abs(speed) * 65535 / 100);

    if (dir_speed > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
    } else if (dir_speed < 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
}

/* =========================================================================
 * Motor B (Phai) - PWMB = TIM3_CH2 (PA7)
 *   BIN1=PB15  BIN2=PB10
 *   Tien: BIN1=0 BIN2=1  Lui: BIN1=1 BIN2=0  Brake: BIN1=1 BIN2=1
 * =========================================================================*/
void MotorB_SetSpeed(int speed)
{
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;
    g_spd_B = (int8_t)speed;

    int      dir_speed = speed * MOTOR_B_DIR;
    uint32_t pwm       = (uint32_t)(abs(speed) * 65535 / 100);

    if (dir_speed > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    } else if (dir_speed < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}

void Motors_Stop(void)
{
    g_spd_A = 0; g_spd_B = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_15, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}

void Motors_Brake(void)
{
    g_spd_A = 0; g_spd_B = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_15, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}

/* =========================================================================
 * ADC_ReadSensors
 *   DMA cap nhat g_adc_raw[] lien tuc -> ham nay chi phan tich nguong.
 * =========================================================================*/
void ADC_ReadSensors(void)
{
    g_L2 = (g_adc_raw[0] > g_adc_threshold) ? 1 : 0;
    g_L1 = (g_adc_raw[1] > g_adc_threshold) ? 1 : 0;
    g_C  = (g_adc_raw[2] > g_adc_threshold) ? 1 : 0;
    g_R1 = (g_adc_raw[3] > g_adc_threshold) ? 1 : 0;
    g_R2 = (g_adc_raw[4] > g_adc_threshold) ? 1 : 0;
}

/* =========================================================================
 * Soft Start helpers
 *
 * Ramp_Reset():
 *   Goi moi khi bat dau chay (G, nut bam, ExitTuning).
 *   Dat g_ramp_speed = g_ramp_min va bat co g_ramping.
 *   PID se tang dan g_ramp_speed len g_base_speed moi chu ky 10ms.
 *
 * Ramp_Update_Manual():
 *   Goi moi chu ky 10ms khi MANUAL mode dang ramping.
 *   Sau khi dat toc do muc tieu thi g_ramping = 0.
 * =========================================================================*/
void Ramp_Reset(void)
{
    g_ramp_speed = g_ramp_min;
    g_ramping    = 1;
}

void Ramp_Update_Manual(void)
{
    if (!g_ramping || !g_manual_active) return;

    uint8_t target = (uint8_t)abs(g_manual_speed);

    if (g_ramp_speed < target) {
        g_ramp_speed += g_ramp_step;
        if (g_ramp_speed >= target) {
            g_ramp_speed = target;
            g_ramping    = 0;
        }
    } else {
        g_ramp_speed = target;
        g_ramping    = 0;
    }

    /* Cap nhat toc do motor theo chieu dang chay */
    if (g_spd_A > 0 && g_spd_B > 0) {
        /* Tien */
        MotorA_SetSpeed((int)g_ramp_speed);
        MotorB_SetSpeed((int)g_ramp_speed);
    } else if (g_spd_A < 0 && g_spd_B < 0) {
        /* Lui */
        MotorA_SetSpeed(-(int)g_ramp_speed);
        MotorB_SetSpeed(-(int)g_ramp_speed);
    }
    /* Truong hop quay tai cho (J/L): khong ramp, toc do full ngay */
}

/* =========================================================================
 * Tuning
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
    g_adc_threshold  = DEFAULT_ADC_THRESHOLD;
    g_ramp_step      = DEFAULT_RAMP_STEP;
    g_ramp_min       = DEFAULT_RAMP_MIN_START;
    g_weight[0]      = DEFAULT_W0;
    g_weight[1]      = DEFAULT_W1;
    g_weight[2]      = DEFAULT_W2;
    g_weight[3]      = DEFAULT_W3;
    g_weight[4]      = DEFAULT_W4;
    g_integral       = 0.0f;
    g_last_error     = 0.0f;
    g_ramp_speed     = 0;
    g_ramping        = 0;

    printf("[TUNE] Reset ve mac dinh\r\n");
    BT_Send("$TUNE,RESET_OK\r\n");
    BT_SendTuningMenu();
}

void BT_SendTuningMenu(void)
{
    char buf[180];

    BT_Send("$TUNE_MENU_START\r\n");
    BT_Send("$TUNE,---- DUNG XE - TUNING PID ----\r\n");

    snprintf(buf, sizeof(buf),
             "$TUNE,Kp=%.2f  Ki=%.3f  Kd=%.2f\r\n",
             g_kp, g_ki, g_kd);
    BT_Send(buf);

    snprintf(buf, sizeof(buf),
             "$TUNE,BASE=%d  LOST=%d\r\n",
             g_base_speed, g_turn_lost);
    BT_Send(buf);

    snprintf(buf, sizeof(buf),
             "$TUNE,RAMP_STEP=%d  RAMP_MIN=%d\r\n",
             g_ramp_step, g_ramp_min);
    BT_Send(buf);

    snprintf(buf, sizeof(buf),
             "$TUNE,MAN_SPD=%d  TIMEOUT=%lu\r\n",
             g_manual_speed, (unsigned long)g_manual_timeout);
    BT_Send(buf);

    snprintf(buf, sizeof(buf),
             "$TUNE,ADC_THR=%u  raw=[%u,%u,%u,%u,%u]\r\n",
             g_adc_threshold,
             g_adc_raw[0], g_adc_raw[1], g_adc_raw[2],
             g_adc_raw[3], g_adc_raw[4]);
    BT_Send(buf);

    snprintf(buf, sizeof(buf),
             "$TUNE,W=[%.2f,%.2f,%.2f,%.2f,%.2f] (L2,L1,C,R1,R2)\r\n",
             g_weight[0], g_weight[1], g_weight[2],
             g_weight[3], g_weight[4]);
    BT_Send(buf);

    BT_Send("$TUNE,----------------------------------\r\n");
    BT_Send("$TUNE_CMD: P=Kp I=Ki D=Kd\r\n");
    BT_Send("$TUNE_CMD: B=BaseSpd O=LostSpd\r\n");
    BT_Send("$TUNE_CMD: Z=RampStep N=RampMin\r\n");
    BT_Send("$TUNE_CMD: V=ManSpd X=Timeout H=ADCthresh\r\n");
    BT_Send("$TUNE_CMD: W1..W5=TrongSo(L2..R2)\r\n");
    BT_Send("$TUNE_CMD: R=Reset  Q=Thoat+TiepTuc\r\n");
    BT_Send("$TUNE_EX:  P2.5 D1.2 Z3 N10 W18.0 W4-1.5\r\n");
    BT_Send("$TUNE_MENU_END\r\n");

    printf("[TUNE] Kp=%.2f Ki=%.3f Kd=%.2f BASE=%d LOST=%d RAMP=%d/%d THR=%u\r\n",
           g_kp, g_ki, g_kd, g_base_speed, g_turn_lost,
           g_ramp_step, g_ramp_min, g_adc_threshold);
    printf("[TUNE] W=[%.2f,%.2f,%.2f,%.2f,%.2f]\r\n",
           g_weight[0], g_weight[1], g_weight[2],
           g_weight[3], g_weight[4]);
}

void EnterTuningMode(void)
{
    if (g_tuning) {
        ADC_ReadSensors();
        BT_SendTuningMenu();
        return;
    }
    g_resume_run    = (uint8_t)g_running;
    g_running       = 0;
    g_manual_active = 0;
    g_ramping       = 0;
    g_ramp_speed    = 0;
    Motors_Brake();
    strncpy(g_state, "TUNING         ", sizeof(g_state));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    g_tuning = 1;

    ADC_ReadSensors();
    printf("\r\n=== TUNING MODE: XE DUNG - Chinh PID ===\r\n");
    BT_Send("$TUNING_START\r\n");
    BT_SendTuningMenu();
}

void ExitTuningMode(void)
{
    if (!g_tuning) return;
    g_tuning     = 0;
    g_integral   = 0.0f;
    g_last_error = 0.0f;

    if (g_resume_run && g_mode == MODE_AUTO) {
        g_running = 1;
        Ramp_Reset();   /* Tang toc tu tu sau khi thoat tuning */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        strncpy(g_state, "PID_RUN        ", sizeof(g_state));
        printf("\r\n=== TUNING EXIT: Tiep tuc chay AUTO (soft start) ===\r\n");
        BT_Send("$TUNING_EXIT,RESUME_RUN\r\n");
    } else {
        g_running    = 0;
        g_ramp_speed = 0;
        g_ramping    = 0;
        strncpy(g_state, "STOP           ", sizeof(g_state));
        printf("\r\n=== TUNING EXIT: Xe dung ===\r\n");
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
    float   fval = (cmd[1] != '\0') ? atof(&cmd[1]) : -9999.0f;
    int     val  = (int)fval;

    /* Lenh T: vao tuning bat cu luc nao */
    if (c == 'T' || c == 't') { EnterTuningMode(); return; }

    /* ------------------------------------------------------------------ */
    if (g_tuning)
    {
        char ack[120];

        if (c == 'Q' || c == 'q') { ExitTuningMode(); return; }
        if (c == 'R' || c == 'r') { Tuning_ResetDefaults(); return; }

        /* Lenh W: chinh trong so sensor W1..W5 */
        if (c == 'W' || c == 'w') {
            if (cmd[1] >= '1' && cmd[1] <= '5') {
                uint8_t idx  = (uint8_t)(cmd[1] - '1');
                float   wval = (cmd[2] != '\0') ? atof(&cmd[2]) : 0.0f;
                g_weight[idx] = wval;
                snprintf(ack, sizeof(ack),
                         "$TUNE,W%d=%.2f OK\r\n", idx + 1, g_weight[idx]);
                BT_Send(ack);
                BT_SendTuningMenu();
            } else {
                BT_Send("$ERR,W phai co index 1-5. Vi du: W18.0 W4-1.5\r\n");
            }
            return;
        }

        if (fval != -9999.0f) {
            switch (c) {
                case 'P': case 'p':
                    g_kp = fval;
                    snprintf(ack, sizeof(ack), "$TUNE,Kp=%.2f OK\r\n", g_kp);
                    BT_Send(ack); BT_SendTuningMenu(); break;

                case 'I': case 'i':
                    g_ki = fval;
                    snprintf(ack, sizeof(ack), "$TUNE,Ki=%.3f OK\r\n", g_ki);
                    BT_Send(ack); BT_SendTuningMenu(); break;

                case 'D': case 'd':
                    g_kd = fval;
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

                case 'H': case 'h':
                    g_adc_threshold = (uint16_t)val;
                    snprintf(ack, sizeof(ack), "$TUNE,ADC_THR=%u OK\r\n", g_adc_threshold);
                    BT_Send(ack); BT_SendTuningMenu(); break;

                case 'Z': case 'z':
                    /* Ramp step: muc tang toc do moi chu ky 10ms (1..10) */
                    if (val < 1)  val = 1;
                    if (val > 10) val = 10;
                    g_ramp_step = (uint8_t)val;
                snprintf(ack, sizeof(ack), "$TUNE,RAMP_STEP=%d OK\r\n", g_ramp_step);
                BT_Send(ack); BT_SendTuningMenu(); break;

                case 'N': case 'n':
                    /* Ramp min start: toc do khoi dong (5..50) */
                    if (val < 5)  val = 5;
                    if (val > 50) val = 50;
                    g_ramp_min = (uint8_t)val;
                snprintf(ack, sizeof(ack), "$TUNE,RAMP_MIN=%d OK\r\n", g_ramp_min);
                BT_Send(ack); BT_SendTuningMenu(); break;

                default:
                    BT_Send("$ERR,Unknown tune cmd. Q=exit.\r\n"); break;
            }
        } else {
            BT_Send("$ERR,TUNING: them gia tri. Vi du P12.0 D3.5 Z3 N10\r\n");
        }
        return;
    }

    /* ------------------------------------------------------------------ */
    if (c == 'A' || c == 'a') { SetMode(MODE_AUTO);   return; }
    if (c == 'M' || c == 'm') { SetMode(MODE_MANUAL); return; }

    if (c == 'G' || c == 'g') {
        if (g_mode == MODE_AUTO) {
            g_running    = 1;
            g_integral   = 0.0f;
            g_last_error = 0.0f;
            Ramp_Reset();   /* Bat dau tang toc tu tu */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            strncpy(g_state, "PID_RUN        ", sizeof(g_state));
            printf("\r\n=== BT: GO (soft start) ===\r\n");
            BT_Send("$CMD,GO\r\n");
        } else {
            BT_Send("$ERR,Go chi dung trong . Gui A truoc.\r\n");
        }
        return;
    }

    if (c == 'S' || c == 's') {
        g_running       = 0;
        g_manual_active = 0;
        g_ramping       = 0;
        g_ramp_speed    = 0;
        Motors_Brake();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        strncpy(g_state, "STOP           ", sizeof(g_state));
        printf("\r\n=== BT: STOP ===\r\n");
        BT_Send("$CMD,STOP\r\n");
        return;
    }

    if (g_mode == MODE_MANUAL) {
        ManualDrive(c);
    }
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
    g_ramping       = 0;
    g_ramp_speed    = 0;
    Motors_Brake();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    strncpy(g_state, "STOP           ", sizeof(g_state));

    if (mode == MODE_AUTO) {
        printf("\r\n>>> CHE DO: AUTO\r\n");
    } else {
        printf("\r\n>>> CHE DO: MANUAL\r\n");
    }
}

/* =========================================================================
 * ManualDrive - I=Tien K=Lui J=Trai L=Phai
 *
 * Soft start ap dung cho tien/lui (I/K).
 * Quay tai cho (J/L): khong ramp vi toc do thap va can phan hoi ngay.
 * =========================================================================*/
void ManualDrive(uint8_t cmd)
{
    switch (cmd) {
        case 'I': case 'i':
            strncpy(g_state, "MAN-FORWARD    ", sizeof(g_state));
            Ramp_Reset();
            MotorA_SetSpeed((int)g_ramp_speed);
            MotorB_SetSpeed((int)g_ramp_speed);
            g_bt_last_cmd_tick = HAL_GetTick();
            g_manual_active = 1;
            break;

        case 'K': case 'k':
            strncpy(g_state, "MAN-BACKWARD   ", sizeof(g_state));
            Ramp_Reset();
            MotorA_SetSpeed(-(int)g_ramp_speed);
            MotorB_SetSpeed(-(int)g_ramp_speed);
            g_bt_last_cmd_tick = HAL_GetTick();
            g_manual_active = 1;
            break;

        case 'J': case 'j':
            /* Quay tai cho: full speed ngay, khong ramp */
            strncpy(g_state, "MAN-LEFT       ", sizeof(g_state));
            g_ramping = 0;
            MotorA_SetSpeed(-g_manual_speed);
            MotorB_SetSpeed( g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick();
            g_manual_active = 1;
            break;

        case 'L': case 'l':
            /* Quay tai cho: full speed ngay, khong ramp */
            strncpy(g_state, "MAN-RIGHT      ", sizeof(g_state));
            g_ramping = 0;
            MotorA_SetSpeed( g_manual_speed);
            MotorB_SetSpeed(-g_manual_speed);
            g_bt_last_cmd_tick = HAL_GetTick();
            g_manual_active = 1;
            break;

        default:
            strncpy(g_state, "MAN-STOP       ", sizeof(g_state));
            g_manual_active = 0;
            g_ramping       = 0;
            g_ramp_speed    = 0;
            Motors_Brake();
            break;
    }
}

void ManualTimeoutCheck(void)
{
    if (g_mode != MODE_MANUAL || !g_manual_active) return;
    if ((HAL_GetTick() - g_bt_last_cmd_tick) >= g_manual_timeout) {
        g_manual_active = 0;
        g_ramping       = 0;
        g_ramp_speed    = 0;
        strncpy(g_state, "MAN-STOP       ", sizeof(g_state));
        Motors_Brake();
    }
}

/* =========================================================================
 * UART RX Interrupt - nhan lenh BT
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
                    case 'T': case 't': case 'Q': case 'q':
                    case 'R': case 'r':
                    case 'G': case 'g': case 'S': case 's':
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
 *
 * Soft start tich hop:
 *   - Moi lan bat dau (G, nut, ExitTuning) -> Ramp_Reset() -> g_ramp_speed = g_ramp_min
 *   - Moi chu ky 10ms: tang g_ramp_speed them g_ramp_step
 *   - effective_base = g_ramp_speed (thay the g_base_speed truc tiep)
 *   - Khi g_ramp_speed dat g_base_speed: g_ramping = 0, chay binh thuong
 *
 * Thoi gian tang toc ~= (g_base_speed - g_ramp_min) / g_ramp_step * 10ms
 * Vi du mac dinh: (45 - 15) / 2 * 10ms = 150ms
 * =========================================================================*/
void LineFollow_PID(void)
{
    ADC_ReadSensors();

    uint8_t sens[SENSOR_COUNT] = { g_L2, g_L1, g_C, g_R1, g_R2 };
    float   sum   = 0.0f;
    int     count = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (sens[i]) {
            sum += g_weight[i];
            count++;
        }
    }

    /* ---- Cap nhat ramp speed ---- */
    if (g_ramping) {
        g_ramp_speed += g_ramp_step;
        if (g_ramp_speed >= (uint8_t)g_base_speed) {
            g_ramp_speed = (uint8_t)g_base_speed;
            g_ramping    = 0;
            strncpy(g_state, "PID_TRACKING   ", sizeof(g_state));
        }
    } else {
        g_ramp_speed = (uint8_t)g_base_speed;
    }
    int effective_base = (int)g_ramp_speed;

    /* ---- Xu ly cac tinh huong dac biet ---- */
    if (count == 0)
    {
        strncpy(g_state, "LOST_LINE      ", sizeof(g_state));
        /* Khi mat line van giu toc do theo ramp hien tai (khong vuot qua g_turn_lost) */
        int lost_spd = (effective_base < (int)g_turn_lost) ? effective_base : (int)g_turn_lost;
        if (g_last_error < -1.5f) {
            MotorA_SetSpeed(-lost_spd);
            MotorB_SetSpeed( lost_spd);
        } else if (g_last_error > 1.5f) {
            MotorA_SetSpeed( lost_spd);
            MotorB_SetSpeed(-lost_spd);
        } else {
            MotorA_SetSpeed(lost_spd);
            MotorB_SetSpeed(lost_spd);
        }
        return;
    }
    else if (count >= 4)
    {
        strncpy(g_state, g_ramping ? "RAMP_CROSS     " : "CROSS_DETECT   ", sizeof(g_state));
        g_error = 0.0f;
    }
    else
    {
        g_error = sum / (float)count;
        strncpy(g_state, g_ramping ? "RAMP_TRACK     " : "PID_TRACKING   ", sizeof(g_state));
    }

    /* ---- Tinh toan PID ---- */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - g_last_pid_time) / 1000.0f;
    if (dt <= 0.0f) dt = 0.01f;
    g_last_pid_time = now;

    float P = g_kp * g_error;

    g_integral += g_error * dt;
    if (g_ki > 0.0f) {
        float max_i = 50.0f / g_ki;
        if (g_integral >  max_i) g_integral =  max_i;
        if (g_integral < -max_i) g_integral = -max_i;
    }
    float I = g_ki * g_integral;

    float D = g_kd * (g_error - g_last_error) / dt;

    float output = P + I + D;
    g_last_error = g_error;

    /* ---- Adaptive Speed (dua tren effective_base) ---- */
    int adaptive_base = effective_base - (int)(fabs(output) * 1.5f);
    /* San toc do toi thieu: thap hon khi con ramp, cao hon khi da on dinh */
    int spd_floor = g_ramping ? (int)g_ramp_min : 15;
    if (adaptive_base < spd_floor) adaptive_base = spd_floor;

    MotorA_SetSpeed(adaptive_base + (int)output);
    MotorB_SetSpeed(adaptive_base - (int)output);
}

/* =========================================================================
 * Print / BT Status
 * =========================================================================*/
void UART_PrintStatus(void)
{
    g_enc_left  = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    g_enc_right = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    printf("[%s][%s][%s][%s] L2=%d(%d) L1=%d(%d) C=%d(%d) R1=%d(%d) R2=%d(%d)"
    " | %-15s | Err:%5.2f | A:%+4d B:%+4d | Rmp:%d(%d) | EL:%6ld ER:%6ld\r\n",
           g_mode == MODE_AUTO ? "AUTO  " : "MANUAL",
           g_running ? "RUN" : "STP",
           g_tuning  ? "TUNE" : "    ",
           g_ramping ? "RMP" : "   ",
           g_L2, g_adc_raw[0],
           g_L1, g_adc_raw[1],
           g_C,  g_adc_raw[2],
           g_R1, g_adc_raw[3],
           g_R2, g_adc_raw[4],
           g_state, g_error,
           (int)g_spd_A, (int)g_spd_B,
           (int)g_ramp_speed, (int)g_ramping,
           (long)g_enc_left, (long)g_enc_right);
}

void BT_SendStatus(void)
{
    if (g_tuning) return;

    char buf[140];
    snprintf(buf, sizeof(buf),
             "$%s,%s,%d,%d,%d,%d,%d,%s,%.2f,%+d,%+d,%d,%ld,%ld\r\n",
             g_mode == MODE_AUTO ? "AUTO" : "MAN",
             g_running ? "RUN" : "STP",
             g_L2, g_L1, g_C, g_R1, g_R2,
             g_state, g_error,
             (int)g_spd_A, (int)g_spd_B,
             (int)g_ramp_speed,
             (long)g_enc_left, (long)g_enc_right);
    BT_Send(buf);
}

/* =========================================================================
 * Interrupt handlers
 * =========================================================================*/
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13); }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_13) return;
    if (g_mode != MODE_AUTO) return;
    if (g_tuning) return;

    uint32_t now = HAL_GetTick();
    if ((now - g_btn_last_tick) < DEBOUNCE_MS) return;
    g_btn_last_tick = now;

    g_running ^= 1;

    if (g_running) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        g_integral   = 0.0f;
        g_last_error = 0.0f;
        Ramp_Reset();   /* Bat dau tang toc tu tu */
        printf("\r\n=== AUTO: BAT DAU CHAY (soft start) ===\r\n");
        BT_Send("$CMD,START\r\n");
    } else {
        g_ramping    = 0;
        g_ramp_speed = 0;
        Motors_Brake();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        strncpy(g_state, "STOP           ", sizeof(g_state));
        printf("\r\n=== AUTO: DUNG (BRAKE) ===\r\n");
        BT_Send("$CMD,STOP\r\n");
    }
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }

/* =========================================================================
 * MAIN
 * =========================================================================*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    TB6612_Enable();
    Motors_Stop();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_raw, SENSOR_COUNT);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    SetMode(MODE_AUTO);
    HAL_UART_Receive_IT(&huart1, &g_bt_rx_byte, 1);

    printf("\r\n========================================\r\n");
    printf("  Line Follower PID  STM32F411RE\r\n");
    printf("  ADC DMA TCRT5000 x5 + Encoder\r\n");
    printf("  Soft Start: min=%d step=%d\r\n", g_ramp_min, g_ramp_step);
    printf("========================================\r\n\r\n");

    BT_Send("$BOOT,LineFollower_PID_STM32_DMA_SoftStart\r\n");
    BT_SendTuningMenu();

    uint32_t last_print = 0;
    uint32_t loop_tick  = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* Chu ky PID ~10ms (100Hz) */
        if (now - loop_tick >= 10)
        {
            loop_tick = now;

            if (g_mode == MODE_AUTO && g_running && !g_tuning) {
                LineFollow_PID();
            }

            if (g_mode == MODE_MANUAL && !g_tuning) {
                ManualTimeoutCheck();
                /* Cap nhat ramp cho manual (tien/lui) */
                if (g_manual_active && g_ramping) {
                    Ramp_Update_Manual();
                }
            }
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
 * TIM2 - Encoder Trai (PA0=CH1, PA1=CH2)
 * -------------------------------------------------------------------------*/
static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
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
    htim2.Init.Period            = 0xFFFF;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 4;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 4;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

/* -------------------------------------------------------------------------
 * TIM3 - PWM Motor (PA7=CH2, PB0=CH3)
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
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim3);
}

/* -------------------------------------------------------------------------
 * TIM4 - Encoder Phai (PB6=CH1, PB7=CH2)
 * -------------------------------------------------------------------------*/
static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
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
    htim4.Init.Period            = 0xFFFF;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 4;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 4;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
}

/* -------------------------------------------------------------------------
 * MX_ADC1_Init
 *   Scan 5 kenh lien tuc qua DMA circular
 *   Scan order: CH9(L2) CH14(L1) CH15(C) CH6(R1) CH13(R2)
 * -------------------------------------------------------------------------*/
static void MX_ADC1_Init(void)
{
    GPIO_InitTypeDef       GPIO_InitStruct = {0};
    ADC_ChannelConfTypeDef sConfig         = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin  = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = SENSOR_COUNT;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_9;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank    = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank    = 3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank    = 4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.Rank    = 5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

/* -------------------------------------------------------------------------
 * USART1 - HC-05 (PA9=TX, PA10=RX) @ 38400
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
 * USART2 - Debug ST-Link (PA2=TX, PA3=RX) @ 115200
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

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9 | GPIO_PIN_13, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* =========================================================================
 * Error Handler
 * =========================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
