/*
 * main.c — Controle Papers, Please
 * APS 2 - Computação Embarcada
 *
 * Regras de qualidade obedecidas:
 *   Rule 1.1/1.2 — Globais apenas para ISR, todas volatile
 *   Rule 1.3     — Nenhuma global desnecessária
 *   Rule 3.0-3.3 — ISR curta: sem delay, printf, for, display
 *   Rule 4.1     — FromISR usado dentro de callbacks
 *   Rule 4.2     — API normal usada dentro de tasks
 *   Rule 4.3     — vTaskDelay em todas as tasks
 *   Rule 4.4     — Sem globais de estado; comunicação via filas/semáforos
 *
 * Arquitetura:
 *   imu_task    — lê MPU6050 e envia movimento HID de mouse
 *   btn_task    — consome fila de botões e envia HID teclado/mouse
 *   power_task  — monitora botão power e envia evento via fila
 *   usb_task    — polling do TinyUSB (necessário para HID funcionar)
 *
 *   btn_callback (ISR) — coloca eventos na qButtonEvents
 *
 * Anti-cheat:
 *   - Debounce 50 ms por botão (via volatile timestamp na ISR)
 *   - Rate limiting via semáforo de contagem (máx 20 eventos/s)
 *   - Velocidade do mouse limitada a ±MAX_MOUSE_SPEED
 */

#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "bsp/board.h"
#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ── PINOS ──────────────────────────────────────────────────── */
#define I2C_PORT     i2c0
#define I2C_SDA_PIN  4
#define I2C_SCL_PIN  5

#define BTN_APPROVE_PIN  16
#define BTN_DENY_PIN     17
#define BTN_CLICK_PIN    18
#define BTN_INSPECT_PIN  19
#define BTN_POWER_PIN    20
#define LED_STATUS_PIN   25

/* ── MPU6050 ────────────────────────────────────────────────── */
#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_XOUT_H  0x43
#define GYRO_DEADZONE        10
#define MAX_MOUSE_SPEED      12
#define CALIBRATION_SAMPLES  3000

/* ── HID keycodes ───────────────────────────────────────────── */
#define HID_KEY_A  0x04
#define HID_KEY_I  0x0C
#define HID_KEY_X  0x1B

/* ── ANTI-CHEAT ─────────────────────────────────────────────── */
#define DEBOUNCE_MS          50
#define MAX_BTN_EVENTS_PER_S 20
#define RATE_LIMIT_PERIOD_MS 1000

/* ── TIPOS ──────────────────────────────────────────────────── */
typedef struct {
    uint8_t pin;
    uint8_t state;  /* 1 = pressionado, 0 = solto */
} button_event_t;

typedef enum {
    PWR_ON,
    PWR_OFF
} power_event_t;

/* ── RECURSOS RTOS — inicializados no main() ─────────────────
 * Rule 4.4: comunicação entre ISR e tasks via fila/semáforo,
 * sem variáveis globais de estado.
 * ─────────────────────────────────────────────────────────── */
static QueueHandle_t     qButtonEvents;   /* ISR → btn_task      */
static QueueHandle_t     qPowerEvents;    /* power_task → imu/btn */
static SemaphoreHandle_t hid_mutex;       /* protege TinyUSB HID  */
static SemaphoreHandle_t rate_sem;        /* anti-cheat: conta eventos/s */

/* ── GLOBAIS DE ISR (Rule 1.1, 1.2, 1.3) ────────────────────
 * Apenas timestamps de debounce, modificados na ISR.
 * ─────────────────────────────────────────────────────────── */
static volatile uint32_t last_event_ms[4] = {0U, 0U, 0U, 0U};

/* ── HELPERS ────────────────────────────────────────────────── */
static inline int pin_to_idx(uint8_t pin) {
    switch (pin) {
        case BTN_APPROVE_PIN:  return 0;
        case BTN_DENY_PIN:     return 1;
        case BTN_CLICK_PIN:    return 2;
        case BTN_INSPECT_PIN:  return 3;
        default:               return -1;
    }
}

/* ── MPU6050 ────────────────────────────────────────────────── */
static void mpu6050_init(void) {
    uint8_t buf[2] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

static void mpu6050_read_gyro(int16_t gyro[3]) {
    uint8_t buffer[6];
    uint8_t reg = MPU6050_GYRO_XOUT_H;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    gyro[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    gyro[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    gyro[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
}

/* ── HID HELPERS ────────────────────────────────────────────── */
static void hid_mouse_move(int8_t dx, int8_t dy) {
    if (!tud_hid_ready()) return;
    tud_hid_mouse_report(0, 0x00, dx, dy, 0, 0);
}

static void hid_mouse_button(bool pressed) {
    if (!tud_hid_ready()) return;
    uint8_t buttons = pressed ? MOUSE_BUTTON_LEFT : 0;
    tud_hid_mouse_report(0, buttons, 0, 0, 0, 0);
}

static void hid_key_action(uint8_t keycode, bool pressed) {
    if (!tud_hid_ready()) return;
    if (pressed) {
        uint8_t keycodes[6] = {keycode, 0, 0, 0, 0, 0};
        tud_hid_keyboard_report(1, 0, keycodes);
    } else {
        uint8_t keycodes[6] = {0};
        tud_hid_keyboard_report(1, 0, keycodes);
    }
}

/* ── ISR CALLBACK (Rule 3.0–3.3, Rule 4.1) ─────────────────
 * Apenas: debounce + rate limit + xQueueSendFromISR
 * Sem printf, sem delay, sem for, sem display.
 * ─────────────────────────────────────────────────────────── */
static void btn_callback(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    int idx = pin_to_idx((uint8_t)gpio);

    if (idx < 0) return;

    /* Debounce — Rule 3.3: sem loops */
    if ((now - last_event_ms[idx]) < DEBOUNCE_MS) return;
    last_event_ms[idx] = now;

    /* Rate limiting via semáforo de contagem — Rule 4.1 */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xSemaphoreTakeFromISR(rate_sem, &xHigherPriorityTaskWoken) == pdFALSE) {
        return; /* limite atingido, descarta evento */
    }

    button_event_t event = {
        .pin   = (uint8_t)gpio,
        .state = (events & GPIO_IRQ_EDGE_FALL) ? 1U : 0U
    };
    xQueueSendFromISR(qButtonEvents, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ── TASK: USB polling (TinyUSB) ────────────────────────────── */
static void usb_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ── TASK: Rate limiter — recarrega semáforo a cada 1s ──────── */
static void rate_reset_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(RATE_LIMIT_PERIOD_MS));
        /* Recarrega o semáforo até MAX_BTN_EVENTS_PER_S */
        for (int i = 0; i < MAX_BTN_EVENTS_PER_S; i++) {
            xSemaphoreGive(rate_sem);
        }
    }
}

/* ── TASK: IMU → movimento de mouse ────────────────────────── */
static void imu_task(void *pvParameters) {
    (void)pvParameters;

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    mpu6050_init();

    /* Calibração (igual ao Enzo) */
    int32_t gyro_x_offset = 0;
    int32_t gyro_y_offset = 0;
    int16_t gyro[3];

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        mpu6050_read_gyro(gyro);
        gyro_x_offset += gyro[0];
        gyro_y_offset += gyro[1];
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    gyro_x_offset /= CALIBRATION_SAMPLES;
    gyro_y_offset /= CALIBRATION_SAMPLES;

    bool controller_on = false;
    power_event_t pwr_event;

    while (1) {
        /* Verifica se chegou evento de power sem bloquear */
        if (xQueueReceive(qPowerEvents, &pwr_event, 0) == pdPASS) {
            controller_on = (pwr_event == PWR_ON);
        }

        if (!controller_on) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        mpu6050_read_gyro(gyro);

        int16_t corrected_gx = gyro[0] - (int16_t)gyro_x_offset;
        int16_t corrected_gy = gyro[1] - (int16_t)gyro_y_offset;

        int16_t mouse_dx = -corrected_gy / 100;
        int16_t mouse_dy = -corrected_gx / 100;

        if (abs(corrected_gy) < GYRO_DEADZONE) mouse_dx = 0;
        if (abs(corrected_gx) < GYRO_DEADZONE) mouse_dy = 0;

        /* Clamp anti-cheat */
        if (mouse_dx >  MAX_MOUSE_SPEED) mouse_dx =  MAX_MOUSE_SPEED;
        if (mouse_dx < -MAX_MOUSE_SPEED) mouse_dx = -MAX_MOUSE_SPEED;
        if (mouse_dy >  MAX_MOUSE_SPEED) mouse_dy =  MAX_MOUSE_SPEED;
        if (mouse_dy < -MAX_MOUSE_SPEED) mouse_dy = -MAX_MOUSE_SPEED;

        if (mouse_dx != 0 || mouse_dy != 0) {
            if (xSemaphoreTake(hid_mutex, portMAX_DELAY) == pdTRUE) {
                hid_mouse_move((int8_t)mouse_dx, (int8_t)-mouse_dy);
                xSemaphoreGive(hid_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── TASK: Botões → HID ─────────────────────────────────────── */
static void btn_task(void *pvParameters) {
    (void)pvParameters;

    const uint8_t BTN_PINS[4] = {
        BTN_APPROVE_PIN, BTN_DENY_PIN,
        BTN_CLICK_PIN,   BTN_INSPECT_PIN
    };

    for (int i = 0; i < 4; i++) {
        gpio_init(BTN_PINS[i]);
        gpio_set_dir(BTN_PINS[i], GPIO_IN);
        gpio_pull_up(BTN_PINS[i]);
        gpio_set_irq_enabled_with_callback(
            BTN_PINS[i],
            GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
            true, &btn_callback
        );
    }

    bool controller_on = false;
    button_event_t ev;
    power_event_t pwr_event;

    while (1) {
        /* Verifica evento de power sem bloquear */
        if (xQueueReceive(qPowerEvents, &pwr_event, 0) == pdPASS) {
            controller_on = (pwr_event == PWR_ON);
        }

        if (xQueueReceive(qButtonEvents, &ev, pdMS_TO_TICKS(10)) == pdPASS) {
            if (!controller_on) continue;

            bool pressed = (ev.state == 1U);

            if (xSemaphoreTake(hid_mutex, portMAX_DELAY) == pdTRUE) {
                switch (ev.pin) {
                    case BTN_APPROVE_PIN:
                        hid_key_action(HID_KEY_A, pressed);
                        break;
                    case BTN_DENY_PIN:
                        hid_key_action(HID_KEY_X, pressed);
                        break;
                    case BTN_CLICK_PIN:
                        hid_mouse_button(pressed);
                        break;
                    case BTN_INSPECT_PIN:
                        hid_key_action(HID_KEY_I, pressed);
                        break;
                    default:
                        break;
                }
                xSemaphoreGive(hid_mutex);
            }
        }
    }
}

/* ── TASK: Power + LED ──────────────────────────────────────── */
static void power_task(void *pvParameters) {
    (void)pvParameters;

    gpio_init(BTN_POWER_PIN);
    gpio_set_dir(BTN_POWER_PIN, GPIO_IN);
    gpio_pull_up(BTN_POWER_PIN);

    gpio_init(LED_STATUS_PIN);
    gpio_set_dir(LED_STATUS_PIN, GPIO_OUT);
    gpio_put(LED_STATUS_PIN, 0);

    bool last_btn_state = true;
    bool controller_on  = false;
    TickType_t last_press = 0;

    while (1) {
        bool current = gpio_get(BTN_POWER_PIN);

        if (!current && last_btn_state) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_press) > pdMS_TO_TICKS(300)) {
                controller_on = !controller_on;
                gpio_put(LED_STATUS_PIN, controller_on ? 1 : 0);

                /* Notifica demais tasks via fila — Rule 4.4 */
                power_event_t pwr = controller_on ? PWR_ON : PWR_OFF;
                xQueueOverwrite(qPowerEvents, &pwr);

                last_press = now;
            }
        }
        last_btn_state = current;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ── MAIN ───────────────────────────────────────────────────── */
int main(void) {
    board_init();
    tusb_init();
    stdio_init_all();

    /* Inicialização de todos os recursos RTOS no main (Rule 4.4) */
    qButtonEvents = xQueueCreate(20, sizeof(button_event_t));
    qPowerEvents  = xQueueCreate(1,  sizeof(power_event_t));
    hid_mutex     = xSemaphoreCreateMutex();

    /* Semáforo de contagem para rate limiting anti-cheat
     * Máximo = MAX_BTN_EVENTS_PER_S, começa cheio */
    rate_sem = xSemaphoreCreateCounting(MAX_BTN_EVENTS_PER_S,
                                         MAX_BTN_EVENTS_PER_S);

    xTaskCreate(usb_task,        "USBTask",       256, NULL, 3, NULL);
    xTaskCreate(imu_task,        "IMUTask",        512, NULL, 1, NULL);
    xTaskCreate(btn_task,        "BtnTask",        256, NULL, 1, NULL);
    xTaskCreate(power_task,      "PwrTask",        256, NULL, 2, NULL);
    xTaskCreate(rate_reset_task, "RateResetTask",  128, NULL, 2, NULL);

    vTaskStartScheduler();
    while (1);
}

/* ── CALLBACKS OBRIGATÓRIOS DO TINYUSB ──────────────────────── */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer;   (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type,
                            uint8_t const *buffer, uint16_t bufsize) {
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer;   (void)bufsize;
}