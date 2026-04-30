# 📌 Sơ đồ nối chân — Line Follower Robot

> **MCU:** STM32F401RE Nucleo-64
> **Driver:** TB6612FNG
> **Cảm biến line:** TCRT5000 × 5 (ADC analog)
> **Encoder:** Quadrature × 2 (11 PPR)
> **Bluetooth:** HC-05

---

## 1. TB6612FNG — Motor Driver

| TB6612 Pin | STM32 Pin | Chức năng |
|:----------:|:---------:|-----------|
| PWMA | PB0 `TIM3_CH3` | PWM Motor A — Bánh **Trái** |
| AIN1 | PB13 `GPIO OUT` | Chiều Motor A |
| AIN2 | PB14 `GPIO OUT` | Chiều Motor A |
| PWMB | PA7 `TIM3_CH2` | PWM Motor B — Bánh **Phải** |
| BIN1 | PB15 `GPIO OUT` | Chiều Motor B |
| BIN2 | PB10 `GPIO OUT` | Chiều Motor B |
| STBY | PC9 `GPIO OUT` | Enable driver (HIGH = hoạt động) |
| VCC | 3.3V | Logic supply |
| VM | 5V–12V | Motor supply |
| GND | GND | Chung GND |

> ⚠️ **Lưu ý:** VM nối nguồn motor riêng (pin/adapter), không dùng 5V từ Nucleo.

---

## 2. TCRT5000 × 5 — Cảm biến dò line (ADC)

Giá trị ADC: **0 – 4095** (12-bit). Ngưỡng nhận line mặc định: **2000**.

| Vị trí | STM32 Pin | ADC Channel | Mô tả |
|:------:|:---------:|:-----------:|-------|
| L2 — Trái ngoài | PB1 | ADC1_CH9 | Cảm biến ngoài cùng bên trái |
| L1 — Trái | PC4 | ADC1_CH14 | Cảm biến trong bên trái |
| C — Giữa | PC5 | ADC1_CH15 | Cảm biến trung tâm |
| R1 — Phải | PA6 | ADC1_CH6 | Cảm biến trong bên phải |
| R2 — Phải ngoài | PC3 | ADC1_CH13 | Cảm biến ngoài cùng bên phải |

Mỗi module TCRT5000 nối:

| TCRT5000 Pin | Nối tới |
|:------------:|---------|
| VCC | 3.3V hoặc 5V |
| GND | GND |
| AO (Analog Out) | STM32 pin tương ứng |
| DO (Digital Out) | Không dùng |

> 💡 **Vạch đen nền trắng:** ADC thấp = thấy line.
> Nếu dùng vạch trắng nền đen thì đảo ngưỡng trong code.

---

## 3. Encoder × 2 — Quadrature (11 PPR)

Mỗi encoder có 2 kênh A và B.

### Encoder Trái — TIM2

| Encoder Pin | STM32 Pin | Chức năng |
|:-----------:|:---------:|-----------|
| A (CH1) | PA0 `TIM2_CH1` | Kênh A |
| B (CH2) | PA1 `TIM2_CH2` | Kênh B |
| VCC | 3.3V hoặc 5V | |
| GND | GND | |

### Encoder Phải — TIM4

| Encoder Pin | STM32 Pin | Chức năng |
|:-----------:|:---------:|-----------|
| A (CH1) | PB6 `TIM4_CH1` | Kênh A |
| B (CH2) | PB7 `TIM4_CH2` | Kênh B |
| VCC | 3.3V hoặc 5V | |
| GND | GND | |

> ⚙️ **Cấu hình:** Quadrature × 4 → 44 counts/vòng (11 PPR × 4).
> Sửa `ENCODER_PPR` trong code nếu dùng encoder khác.

---

## 4. HC-05 — Bluetooth UART

| HC-05 Pin | STM32 Pin | Ghi chú |
|:---------:|:---------:|---------|
| TXD | PA10 `USART1_RX` | HC-05 TX → STM32 RX |
| RXD | PA9 `USART1_TX` | HC-05 RX → STM32 TX |
| VCC | 5V | |
| GND | GND | |

> 📡 **Baudrate:** 38400. Cấu hình HC-05 trước bằng lệnh AT nếu chưa đúng.
> ⚠️ HC-05 dùng mức logic 3.3V cho RXD — PA9 của STM32 xuất 3.3V, nối thẳng được.

---

## 5. Nút nhấn & LED — Onboard Nucleo

| Chức năng | STM32 Pin | Ghi chú |
|-----------|:---------:|---------|
| Nút B1 (Toggle Start/Stop) | PC13 | Nút xanh onboard, EXTI falling |
| LED LD2 (trạng thái chạy) | PA5 | LED xanh onboard |
| LED MODE AUTO | PA11 `GPIO OUT` | LED ngoài (nối qua điện trở 220Ω) |
| LED MODE MANUAL | PB12 `GPIO OUT` | LED ngoài (nối qua điện trở 220Ω) |

---

## 6. Debug — ST-Link UART

| Chức năng | STM32 Pin | Ghi chú |
|-----------|:---------:|---------|
| TX Debug | PA2 `USART2_TX` | Tự động nối qua ST-Link trên Nucleo |
| RX Debug | PA3 `USART2_RX` | Tự động nối qua ST-Link trên Nucleo |

> Mở Serial Monitor ở **115200 baud** để xem log.

---

## 7. Tổng hợp tất cả chân đã dùng

| STM32 Pin | Chức năng | Peripheral |
|:---------:|-----------|:----------:|
| PA0 | Encoder Trái A | TIM2_CH1 |
| PA1 | Encoder Trái B | TIM2_CH2 |
| PA2 | UART Debug TX | USART2 |
| PA3 | UART Debug RX | USART2 |
| PA5 | LED LD2 | GPIO OUT |
| PA6 | IR R1 | ADC1_CH6 |
| PA7 | PWMB Motor B | TIM3_CH2 |
| PA9 | BT TX | USART1 |
| PA10 | BT RX | USART1 |
| PA11 | LED AUTO | GPIO OUT |
| PB0 | PWMA Motor A | TIM3_CH3 |
| PB1 | IR L2 | ADC1_CH9 |
| PB6 | Encoder Phải A | TIM4_CH1 |
| PB7 | Encoder Phải B | TIM4_CH2 |
| PB10 | BIN2 TB6612 | GPIO OUT |
| PB12 | LED MANUAL | GPIO OUT |
| PB13 | AIN1 TB6612 | GPIO OUT |
| PB14 | AIN2 TB6612 | GPIO OUT |
| PB15 | BIN1 TB6612 | GPIO OUT |
| PC3 | IR R2 | ADC1_CH13 |
| PC4 | IR L1 | ADC1_CH14 |
| PC5 | IR C | ADC1_CH15 |
| PC9 | STBY TB6612 | GPIO OUT |
| PC13 | Nút B1 | EXTI |

---

## 8. Sơ đồ nguồn

```
Pin 5V (Nucleo) ──┬── VCC HC-05
                  └── VCC Encoder (nếu dùng 5V)

Pin 3.3V (Nucleo) ─┬── VCC TCRT5000 × 5
                   └── VCC TB6612 (logic)

Nguồn ngoài (7.4V LiPo) ──── VM TB6612 (motor supply)
                         └─── GND chung với Nucleo
```

> ⚠️ **Bắt buộc nối GND chung** giữa Nucleo, TB6612, nguồn ngoài, HC-05 và tất cả cảm biến.
