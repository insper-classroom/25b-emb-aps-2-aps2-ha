/*
 * main.c — Controle Papers, Please
 * APS 2 + Expert (IA + RTOS) | Computação Embarcada — Insper
 *
 * Arquitetura de tasks:
 *   init_task        (prio 4) — inicializa IMU e periféricos; se auto-deleta
 *   tx_task          (prio 3) — consome xQueueTX e escreve na USB-UART
 *   power_task       (prio 2) — polling do BTN_POWER; atualiza LED; envia PWR,n
 *   rate_reset_task  (prio 2) — recarrega xSemRate 1x/s
 *   imu_task         (prio 1) — lê MPU6050 ~100 Hz; enfileira M,dx,dy e janelas p/ IA
 *   ia_task          (prio 1) — roda inferência Edge Impulse; imprime classe
 *   btn_task         (prio 1) — debounce dos botões; enfileira BD/BU,n
 *
 * Protocolo USB-UART (115200, ASCII, \n):
 *   M,dx,dy\n   — movimento do mouse  (dx/dy clampados em ±12)
 *   BD,n\n      — button down
 *   BU,n\n      — button up
 *   PWR,1\n     — controle ligado
 *   PWR,0\n     — controle desligado
 *
 * NOTA sobre IA:
 *   Este arquivo assume que o modelo Edge Impulse foi exportado como
 *   "C++ Library" e integrado ao projeto (pastas edge-impulse-sdk,
 *   model-parameters, tflite-model). O header ei_run_classifier.h e
 *   a struct ei_impulse_result_t são fornecidos pela biblioteca gerada.
 *   Siga o repositório: https://github.com/insper-embarcados/edgeimpulse-runner
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ── Edge Impulse ───────────────────────────────────────────────────────────
 * Inclua os headers gerados pelo export do Edge Impulse.
 * Se o modelo ainda não foi treinado, deixe IA_ENABLED 0 para compilar
 * sem a biblioteca e testar o restante do firmware.
 * ────────────────────────────────────────────────────────────────────────── */
#define IA_ENABLED 0   /* mude para 1 após integrar a lib do Edge Impulse */

#if IA_ENABLED
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#endif

/* ── Pinos ──────────────────────────────────────────────────────────────── */
#define BTN_APPROVE   13
#define BTN_DENY      15
#define BTN_CLICK     14
#define BTN_INSPECT   12
#define BTN_POWER     11

#define LED_STATUS    17   /* aceso = controle ligado */
#define LED_CALIBRADO 16   /* pisca durante calibração */

/* LEDs de feedback da IA (podem ser o mesmo LED RGB ou 3 pinos distintos) */
#define LED_IA_IDLE   20
#define LED_IA_UPDOWN 21
#define LED_IA_WAVE   22

#define I2C_PORT  i2c0
#define I2C_SDA   8
#define I2C_SCL   9
#define MPU_ADDR  0x68

/* ── Parâmetros ─────────────────────────────────────────────────────────── */
#define CALIB_SAMPLES     200
#define MOUSE_SCALE       12        /* divisor do giroscópio → pixels         */
#define MOUSE_CLAMP       12        /* máximo ±px por evento                  */
#define RATE_LIMIT_HZ     20        /* máx. eventos de botão por segundo       */

/* Janela de amostras para a IA (deve casar com o impulse do Edge Impulse)   */
#define IA_WINDOW_SIZE    50        /* ~500 ms a 100 Hz                        */
#define IA_N_AXES         3         /* ax, ay, az                              */

/* ── Tipos ──────────────────────────────────────────────────────────────── */
typedef struct {
    uint8_t pin;
    bool    pressed;   /* true = pressionado */
    uint32_t ts_ms;
} btn_event_t;

typedef struct {
    float ax, ay, az;
} imu_sample_t;

/* ── Filas e semáforos ──────────────────────────────────────────────────── */
static QueueHandle_t xQueueButtons;   /* ISR  → btn_task  */
static QueueHandle_t xQueueTX;        /* *    → tx_task   */
static QueueHandle_t xQueuePower;     /* power_task → imu/btn/ia */
static QueueHandle_t xQueueIMU;       /* imu_task → ia_task */
static SemaphoreHandle_t xSemRate;    /* rate limiting de botões */

/* ── Estado global da IMU (offsets de calibração) ───────────────────────── */
static volatile int32_t gyro_off_x = 0;
static volatile int32_t gyro_off_y = 0;
static volatile int32_t gyro_off_z = 0;

/* ── Helpers MPU6050 ────────────────────────────────────────────────────── */
static void mpu_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
}

static void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDR, buf, len, false);
}

static void mpu_init(void) {
    mpu_write(0x6B, 0x00);  /* sai do sleep */
    mpu_write(0x1C, 0x00);  /* accel ±2g    */
    mpu_write(0x1B, 0x00);  /* gyro ±250°/s */
}

/* Lê aceleração bruta (registradores 0x3B–0x40) */
static void mpu_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6];
    mpu_read(0x3B, buf, 6);
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

/* Lê giroscópio bruto (registradores 0x43–0x48) */
static void mpu_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[6];
    mpu_read(0x43, buf, 6);
    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ── Callback de botões (ISR) ───────────────────────────────────────────── */
static void btn_callback(uint gpio, uint32_t events) {
    /* Tenta tomar 1 token de rate-limit; descarta evento se cheio */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xSemaphoreTakeFromISR(xSemRate, &xHigherPriorityTaskWoken) != pdTRUE)
        return;

    btn_event_t ev = {
        .pin     = (uint8_t)gpio,
        .pressed = !(gpio_get(gpio)),   /* pull-up: 0 = pressionado */
        .ts_ms   = to_ms_since_boot(get_absolute_time()),
    };
    xQueueSendFromISR(xQueueButtons, &ev, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ── Utilitário: enfileira string na TX ─────────────────────────────────── */
static void tx_send(const char *s) {
    while (*s) {
        xQueueSend(xQueueTX, s, portMAX_DELAY);
        s++;
    }
}

/* ────────────────────────────────────────────────────────────────────────── *
 *  TASKS
 * ────────────────────────────────────────────────────────────────────────── */

/* init_task: configura hardware e calibra IMU, depois se auto-deleta */
static void init_task(void *params) {
    /* I2C */
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    mpu_init();

    /* Verifica WHO_AM_I */
    uint8_t who = 0;
    mpu_read(0x75, &who, 1);
    printf("[INIT] MPU6050 WHO_AM_I=0x%02X %s\n", who, who == 0x68 ? "OK" : "ERRO");

    /* Calibração do giroscópio */
    printf("[INIT] Calibrando giroscópio — mantenha parado...\n");
    int32_t sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int16_t gx, gy, gz;
        mpu_read_gyro(&gx, &gy, &gz);
        sx += gx; sy += gy; sz += gz;
        if (i % 20 == 0)
            gpio_put(LED_CALIBRADO, !gpio_get(LED_CALIBRADO));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    gyro_off_x = sx / CALIB_SAMPLES;
    gyro_off_y = sy / CALIB_SAMPLES;
    gyro_off_z = sz / CALIB_SAMPLES;
    gpio_put(LED_CALIBRADO, 1);
    printf("[INIT] Offsets: gx=%d gy=%d gz=%d\n",
           (int)gyro_off_x, (int)gyro_off_y, (int)gyro_off_z);

    vTaskDelete(NULL);
}

/* tx_task: drena xQueueTX e escreve byte a byte na USB-UART */
static void tx_task(void *params) {
    char c;
    while (true) {
        xQueueReceive(xQueueTX, &c, portMAX_DELAY);
        putchar_raw(c);
    }
}

/* power_task: polling do botão POWER; controla LED de status */
static void power_task(void *params) {
    bool ligado      = false;
    bool last_state  = true;   /* pull-up: true = solto */
    char msg[16];

    while (true) {
        bool cur = gpio_get(BTN_POWER);

        if (!cur && last_state) {          /* borda de descida = pressionado */
            ligado = !ligado;
            gpio_put(LED_STATUS, ligado);

            snprintf(msg, sizeof(msg), "PWR,%d\n", ligado ? 1 : 0);
            tx_send(msg);
            printf("[POWER] Controle %s\n", ligado ? "LIGADO" : "DESLIGADO");

            /* notifica imu_task, btn_task e ia_task via fila (1 slot, overwrite) */
            xQueueOverwrite(xQueuePower, &ligado);

            vTaskDelay(pdMS_TO_TICKS(50));   /* debounce simples */
        }
        last_state = cur;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* rate_reset_task: recarrega xSemRate a 1 Hz (permite RATE_LIMIT_HZ eventos/s) */
static void rate_reset_task(void *params) {
    while (true) {
        for (int i = 0; i < RATE_LIMIT_HZ; i++)
            xSemaphoreGive(xSemRate);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* imu_task: lê MPU6050 ~100 Hz; envia M,dx,dy pela UART e amostras p/ ia_task */
static void imu_task(void *params) {
    bool ligado = false;
    char msg[32];

    /* Aguarda init_task terminar a calibração (espera LED_CALIBRADO acender) */
    while (!gpio_get(LED_CALIBRADO))
        vTaskDelay(pdMS_TO_TICKS(10));

    /* Buffer circular de amostras para a janela da IA */
    static imu_sample_t ia_buf[IA_WINDOW_SIZE];
    int ia_idx = 0;

    const TickType_t period = pdMS_TO_TICKS(10);   /* 100 Hz */
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake, period);

        /* Verifica estado do controle (non-blocking) */
        xQueuePeek(xQueuePower, &ligado, 0);
        if (!ligado) continue;

        /* Lê giroscópio → movimento do cursor */
        int16_t gx, gy, gz;
        mpu_read_gyro(&gx, &gy, &gz);

        int dx = (gx - gyro_off_x) / (32768 / MOUSE_SCALE);
        int dy = (gy - gyro_off_y) / (32768 / MOUSE_SCALE);

        /* Clamp */
        if (dx >  MOUSE_CLAMP) dx =  MOUSE_CLAMP;
        if (dx < -MOUSE_CLAMP) dx = -MOUSE_CLAMP;
        if (dy >  MOUSE_CLAMP) dy =  MOUSE_CLAMP;
        if (dy < -MOUSE_CLAMP) dy = -MOUSE_CLAMP;

        if (dx != 0 || dy != 0) {
            snprintf(msg, sizeof(msg), "M,%d,%d\n", dx, dy);
            tx_send(msg);
        }

        /* Lê aceleração → alimenta janela da IA */
        int16_t ax, ay, az;
        mpu_read_accel(&ax, &ay, &az);

        /* Converte para g (escala ±2g: 16384 LSB/g) */
        ia_buf[ia_idx].ax = ax / 16384.0f;
        ia_buf[ia_idx].ay = ay / 16384.0f;
        ia_buf[ia_idx].az = az / 16384.0f;
        ia_idx = (ia_idx + 1) % IA_WINDOW_SIZE;

        /* Quando completar uma janela, envia cópia para ia_task */
        if (ia_idx == 0) {
            static imu_sample_t snap[IA_WINDOW_SIZE];
            memcpy(snap, ia_buf, sizeof(ia_buf));
            /* Não bloqueia: descarta se ia_task ainda estiver ocupada */
            xQueueSend(xQueueIMU, &snap, 0);
        }
    }
}

/* btn_task: debounce dos botões de ação; enfileira BD/BU,n na UART */
static void btn_task(void *params) {
    /* Mapeamento pino → índice de botão (1-based, conforme protocolo) */
    const uint8_t btn_pins[4]    = {BTN_APPROVE, BTN_DENY, BTN_CLICK, BTN_INSPECT};
    const char   *btn_names[4]   = {"APPROVE",   "DENY",   "CLICK",   "INSPECT"};

    bool ligado = false;
    btn_event_t ev;
    char msg[16];

    /* Timestamps do último evento por pino (debounce) */
    uint32_t last_ts[4] = {0, 0, 0, 0};
    const uint32_t DEBOUNCE_MS = 50;

    while (true) {
        if (xQueueReceive(xQueueButtons, &ev, portMAX_DELAY) != pdTRUE)
            continue;

        xQueuePeek(xQueuePower, &ligado, 0);
        if (!ligado) continue;

        /* Encontra índice */
        int idx = -1;
        for (int i = 0; i < 4; i++) {
            if (btn_pins[i] == ev.pin) { idx = i; break; }
        }
        if (idx < 0) continue;

        /* Debounce por timestamp */
        if (ev.ts_ms - last_ts[idx] < DEBOUNCE_MS) continue;
        last_ts[idx] = ev.ts_ms;

        snprintf(msg, sizeof(msg), "%s,%d\n",
                 ev.pressed ? "BD" : "BU", idx + 1);
        tx_send(msg);
        printf("[BTN] %s %s\n", btn_names[idx], ev.pressed ? "DOWN" : "UP");
    }
}

/* ia_task: inferência Edge Impulse sobre janelas de aceleração da IMU */
static void ia_task(void *params) {
    /* Buffer de entrada para o classificador: IA_WINDOW_SIZE × IA_N_AXES */
    static imu_sample_t window[IA_WINDOW_SIZE];
    bool ligado = false;

    while (true) {
        if (xQueueReceive(xQueueIMU, &window, portMAX_DELAY) != pdTRUE)
            continue;

        xQueuePeek(xQueuePower, &ligado, 0);
        if (!ligado) continue;

#if IA_ENABLED
        /* ── Inferência via Edge Impulse ───────────────────────────────────
         * Monta o sinal no formato esperado pelo impulse.
         * O Edge Impulse espera um array flat: [ax0,ay0,az0, ax1,ay1,az1, ...]
         * ------------------------------------------------------------------ */
        static float features[IA_WINDOW_SIZE * IA_N_AXES];
        for (int i = 0; i < IA_WINDOW_SIZE; i++) {
            features[i * IA_N_AXES + 0] = window[i].ax;
            features[i * IA_N_AXES + 1] = window[i].ay;
            features[i * IA_N_AXES + 2] = window[i].az;
        }

        signal_t signal;
        numpy::signal_from_buffer(features, IA_WINDOW_SIZE * IA_N_AXES, &signal);

        ei_impulse_result_t result;
        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

        if (err != EI_IMPULSE_OK) {
            printf("[IA] Erro na inferência: %d\n", err);
            continue;
        }

        /* Encontra classe vencedora */
        int    best_idx   = 0;
        float  best_score = 0.0f;
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (result.classification[i].value > best_score) {
                best_score = result.classification[i].value;
                best_idx   = (int)i;
            }
        }

        const char *label = result.classification[best_idx].label;
        printf("[IA] %s (%.2f)\n", label, best_score);

        /* LED feedback: acende LED correspondente à classe */
        gpio_put(LED_IA_IDLE,   strcmp(label, "idle")   == 0);
        gpio_put(LED_IA_UPDOWN, strcmp(label, "updown") == 0);
        gpio_put(LED_IA_WAVE,   strcmp(label, "wave")   == 0);

#else
        /* IA_ENABLED == 0: apenas imprime amostra para debug */
        printf("[IA] stub — janela recebida (ax=%.2f ay=%.2f az=%.2f)\n",
               window[0].ax, window[0].ay, window[0].az);
#endif
    }
}

/* ── main ───────────────────────────────────────────────────────────────── */
int main(void) {
    stdio_init_all();
    sleep_ms(2000);   /* aguarda USB-CDC conectar */

    printf("\n=== Controle Papers, Please — APS2 ===\n");

    /* Configura botões de ação com IRQ */
    const uint action_btns[] = {BTN_APPROVE, BTN_DENY, BTN_CLICK, BTN_INSPECT};
    for (int i = 0; i < 4; i++) {
        gpio_init(action_btns[i]);
        gpio_set_dir(action_btns[i], GPIO_IN);
        gpio_pull_up(action_btns[i]);
        gpio_set_irq_enabled_with_callback(action_btns[i],
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, btn_callback);
    }

    /* Botão POWER (polling na power_task) */
    gpio_init(BTN_POWER);
    gpio_set_dir(BTN_POWER, GPIO_IN);
    gpio_pull_up(BTN_POWER);

    /* LEDs */
    const uint leds[] = {LED_STATUS, LED_CALIBRADO, LED_IA_IDLE, LED_IA_UPDOWN, LED_IA_WAVE};
    for (int i = 0; i < 5; i++) {
        gpio_init(leds[i]);
        gpio_set_dir(leds[i], GPIO_OUT);
        gpio_put(leds[i], 0);
    }

    /* Filas e semáforos */
    xQueueButtons = xQueueCreate(16, sizeof(btn_event_t));
    xQueueTX      = xQueueCreate(256, sizeof(char));
    xQueuePower   = xQueueCreate(1,   sizeof(bool));
    xQueueIMU     = xQueueCreate(2,   sizeof(imu_sample_t) * IA_WINDOW_SIZE);
    xSemRate      = xSemaphoreCreateCounting(RATE_LIMIT_HZ, RATE_LIMIT_HZ);

    /* Estado inicial do controle: desligado */
    bool off = false;
    xQueueSend(xQueuePower, &off, 0);

    /* Tasks */
    xTaskCreate(init_task,       "init",       2048, NULL, 4, NULL);
    xTaskCreate(tx_task,         "tx",         512,  NULL, 3, NULL);
    xTaskCreate(power_task,      "power",      512,  NULL, 2, NULL);
    xTaskCreate(rate_reset_task, "rate_reset", 256,  NULL, 2, NULL);
    xTaskCreate(imu_task,        "imu",        1024, NULL, 1, NULL);
    xTaskCreate(ia_task,         "ia",         4096, NULL, 1, NULL);
    xTaskCreate(btn_task,        "btn",        512,  NULL, 1, NULL);

    vTaskStartScheduler();

    /* Nunca deve chegar aqui */
    while (true) {}
    return 0;
}