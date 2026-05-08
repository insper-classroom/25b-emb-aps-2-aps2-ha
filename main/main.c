/*
 * main.c — Controle Papers, Please
 * APS 2 - Computação Embarcada
 *
 * A Pico aparece para o PC como dispositivo HID USB (mouse + teclado).
 * Nenhum script Python necessário.
 *
 * Mapeamento:
 *   IMU (MPU6050)  → movimento do mouse
 *   BTN_APPROVE    → tecla 'A'          (carimbar aprovado)
 *   BTN_DENY       → tecla 'X'          (carimbar negado)
 *   BTN_CLICK      → botão esquerdo     (click / drag)
 *   BTN_INSPECT    → tecla 'I'          (interrogar / revistar)
 *   BTN_POWER      → liga/desliga controle + LED status
 *
 * Anti-cheat:
 *   - Debounce 50ms por botão
 *   - Rate limiting: máx 20 eventos/s
 *   - Velocidade do mouse limitada a ±MAX_MOUSE_SPEED
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#define GYRO_DEADZONE        10.0f
#define MAX_MOUSE_SPEED      12

/* ── HID: keycodes HID padrão ───────────────────────────────── */
#define HID_KEY_A  0x04
#define HID_KEY_I  0x0C
#define HID_KEY_X  0x1B

/* ── ANTI-CHEAT ─────────────────────────────────────────────── */
#define DEBOUNCE_MS           50
#define MAX_BTN_EVENTS_PER_S  20

static volatile uint16_t g_btn_event_count = 0;
static volatile bool     g_controller_on   = false;

/* ── FILAS / MUTEX ──────────────────────────────────────────── */
typedef struct {
    uint8_t pin;
    uint8_t state;   // 1 = pressionado, 0 = solto
} button_event_t;

static QueueHandle_t     qButtonEvents;
static SemaphoreHandle_t hid_mutex;

/* ── DEBOUNCE ───────────────────────────────────────────────── */
static uint32_t last_event_ms[4] = {0};

static inline int pin_to_idx(uint8_t pin) {
    switch (pin) {
        case BTN_APPROVE_PIN:  return 0;
        case BTN_DENY_PIN:     return 1;
        case BTN_CLICK_PIN:    return 2;
        case BTN_INSPECT_PIN:  return 3;
        default:               return -1;
    }
}

/* ── MPU6050 (igual ao Enzo) ────────────────────────────────── */
static void mpu6050_init(void) {
    uint8_t buf[] = {MPU6050_PWR_MGMT_1, 0x00};
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

// Envia movimento de mouse via HID
static void hid_mouse_move(int8_t dx, int8_t dy) {
    if (!tud_hid_ready()) return;
    // buttons=0, dx, dy, scroll=0
    tud_hid_mouse_report(0, 0x00, dx, dy, 0, 0);
}

// Clica / segura botão esquerdo do mouse
static void hid_mouse_button(bool pressed) {
    if (!tud_hid_ready()) return;
    uint8_t buttons = pressed ? MOUSE_BUTTON_LEFT : 0;
    tud_hid_mouse_report(0, buttons, 0, 0, 0, 0);
}

// Pressiona / solta uma tecla (keycode HID)
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

/* ── IRQ CALLBACK ───────────────────────────────────────────── */
static void btn_callback(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    int idx = pin_to_idx((uint8_t)gpio);
    if (idx < 0) return;

    if (now - last_event_ms[idx] < DEBOUNCE_MS) return;
    last_event_ms[idx] = now;

    if (g_btn_event_count >= MAX_BTN_EVENTS_PER_S) return;
    g_btn_event_count++;

    button_event_t event = {
        .pin   = (uint8_t)gpio,
        .state = (events & GPIO_IRQ_EDGE_FALL) ? 1 : 0
    };
    xQueueSendFromISR(qButtonEvents, &event, NULL);
}

/* ── TASK: USB HID (TinyUSB precisa de polling) ─────────────── */
static void usb_task(void *pvParameters) {
    while (1) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ── TASK: IMU → movimento de mouse ────────────────────────── */
static void imu_task(void *pvParameters) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    mpu6050_init();

    // Calibração (igual ao Enzo)
    const int CALIBRATION_SAMPLES = 3000;
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

    while (1) {
        if (!g_controller_on) {
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

        // Clamp anti-cheat
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
    const uint8_t BTN_PINS[] = {
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

    button_event_t ev;

    while (1) {
        if (xQueueReceive(qButtonEvents, &ev, portMAX_DELAY) == pdPASS) {
            if (!g_controller_on) continue;

            bool pressed = (ev.state == 1);

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

/* ── TASK: Power + LED + reset rate limiter ─────────────────── */
static void power_task(void *pvParameters) {
    gpio_init(BTN_POWER_PIN);
    gpio_set_dir(BTN_POWER_PIN, GPIO_IN);
    gpio_pull_up(BTN_POWER_PIN);

    gpio_init(LED_STATUS_PIN);
    gpio_set_dir(LED_STATUS_PIN, GPIO_OUT);
    gpio_put(LED_STATUS_PIN, 0);

    bool last_btn_state = true;
    TickType_t last_press = 0;

    while (1) {
        bool current = gpio_get(BTN_POWER_PIN);

        if (!current && last_btn_state) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_press) > pdMS_TO_TICKS(300)) {
                g_controller_on = !g_controller_on;
                gpio_put(LED_STATUS_PIN, g_controller_on ? 1 : 0);
                last_press = now;
            }
        }
        last_btn_state = current;

        // Reset do rate limiter a cada 1 segundo
        static TickType_t last_reset = 0;
        TickType_t now = xTaskGetTickCount();
        if ((now - last_reset) >= pdMS_TO_TICKS(1000)) {
            g_btn_event_count = 0;
            last_reset = now;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ── MAIN ───────────────────────────────────────────────────── */
int main(void) {
    board_init();
    tusb_init();
    stdio_init_all();

    qButtonEvents = xQueueCreate(20, sizeof(button_event_t));
    hid_mutex     = xSemaphoreCreateMutex();

    xTaskCreate(usb_task,   "USBTask",  256, NULL, 3, NULL);  // maior prioridade
    xTaskCreate(imu_task,   "IMUTask",  512, NULL, 1, NULL);
    xTaskCreate(btn_task,   "BtnTask",  256, NULL, 1, NULL);
    xTaskCreate(power_task, "PwrTask",  256, NULL, 2, NULL);

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