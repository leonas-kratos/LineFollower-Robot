# Line Follower Robot

## Pin Mapping

> **MCU:** STM32F401RE Nucleo-64 <br>
> **Driver:** TB6612FNG <br>
> **Line Sensor:** TCRT5000 × 5 (ADC analog) <br>
> **Encoder:** GA12-N20 <br>
> **Bluetooth:** HC-05 <br>

---

## 1. TB6612FNG — Motor Driver

| TB6612 Pin | STM32 Pin | Description |
|:----------:|:---------:|-------------|
| PWMA | PB0 `TIM3_CH3` | PWM Motor A — **Left** wheel |
| AIN1 | PB13 `GPIO OUT` | Motor A direction |
| AIN2 | PB14 `GPIO OUT` | Motor A direction |
| PWMB | PA7 `TIM3_CH2` | PWM Motor B — **Right** wheel |
| BIN1 | PB15 `GPIO OUT` | Motor B direction |
| BIN2 | PB10 `GPIO OUT` | Motor B direction |
| STBY | PC9 `GPIO OUT` | Driver enable (HIGH = active) |
| VCC | 3.3V | Logic supply |
| VM | 5V–12V | Motor power supply |
| GND | GND | Common ground |

---

## 2. TCRT5000 × 5 — Line Sensors (ADC)

ADC range: **0 – 4095** (12-bit). Default line detection threshold: **2000**.

| Position | STM32 Pin | ADC Channel | Description |
|:--------:|:---------:|:-----------:|-------------|
| L2 — Far Left | PB1 | ADC1_CH9 | Outermost left sensor |
| L1 — Left | PC4 | ADC1_CH14 | Inner left sensor |
| C — Center | PC5 | ADC1_CH15 | Center sensor |
| R1 — Right | PA6 | ADC1_CH6 | Inner right sensor |
| R2 — Far Right | PC3 | ADC1_CH13 | Outermost right sensor |
---

## 3. Encoder × 2 — GA12-N20

Each encoder has two channels: A and B.

### Left Encoder — TIM2

| Encoder Pin | STM32 Pin | Description |
|:-----------:|:---------:|-------------|
| A (CH1) | PA0 `TIM2_CH1` | Channel A |
| B (CH2) | PA1 `TIM2_CH2` | Channel B |
| VCC | 3.3V or 5V | |
| GND | GND | |

### Right Encoder — TIM4

| Encoder Pin | STM32 Pin | Description |
|:-----------:|:---------:|-------------|
| A (CH1) | PB6 `TIM4_CH1` | Channel A |
| B (CH2) | PB7 `TIM4_CH2` | Channel B |
| VCC | 3.3V or 5V | |
| GND | GND | |
---

## 4. HC-05 — Bluetooth UART

| HC-05 Pin | STM32 Pin | Notes |
|:---------:|:---------:|-------|
| TXD | PA10 `USART1_RX` | HC-05 TX → STM32 RX |
| RXD | PA9 `USART1_TX` | HC-05 RX → STM32 TX |

> 📡 **Baudrate:** 38400. Configure HC-05 via AT commands first if not already set.
---

## 5. Button & LEDs — Onboard Nucleo

| Function | STM32 Pin | Notes |
|----------|:---------:|-------|
| Button B1 (Toggle Start/Stop) | PC13 | Onboard blue button, EXTI falling edge |
| LED LD2 (running status) | PA5 | Onboard green LED |
---

## 6. Debug — ST-Link UART

| Function | STM32 Pin | Notes |
|----------|:---------:|-------|
| TX Debug | PA2 `USART2_TX` | Routed automatically through ST-Link on Nucleo |
| RX Debug | PA3 `USART2_RX` | Routed automatically through ST-Link on Nucleo |

> Open Serial Monitor at **115200 baud** to view debug logs.

---
