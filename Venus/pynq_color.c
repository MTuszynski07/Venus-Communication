#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <libpynq.h>

/* =========================================================================
 * TCS3200 color sensor pins — update if your wiring differs
 * ========================================================================= */
#define PIN_S0  IO_A0
#define PIN_S1  IO_A1
#define PIN_S2  IO_A2
#define PIN_S3  IO_A3
#define PIN_OUT IO_A4

/* =========================================================================
 * Calibration constants
 * HOW TO CALIBRATE:
 *   1. Point sensor at a BLACK surface, print the raw fR/fG/fB values and
 *      put them in the _MIN fields below.
 *   2. Point sensor at a WHITE surface and put those values in _MAX.
 * ========================================================================= */
#define CAL_R_MIN  500.0f
#define CAL_R_MAX 3500.0f
#define CAL_G_MIN  400.0f
#define CAL_G_MAX 3200.0f
#define CAL_B_MIN  300.0f
#define CAL_B_MAX 2800.0f

/* =========================================================================
 * Color math
 * ========================================================================= */
typedef struct { float r, g, b; } ColorVector;

static ColorVector get_intensities(float fR, float fG, float fB) {
    ColorVector v;
    v.r = fmaxf(0.0f, fminf(1.0f, (fR - CAL_R_MIN) / (CAL_R_MAX - CAL_R_MIN)));
    v.g = fmaxf(0.0f, fminf(1.0f, (fG - CAL_G_MIN) / (CAL_G_MAX - CAL_G_MIN)));
    v.b = fmaxf(0.0f, fminf(1.0f, (fB - CAL_B_MIN) / (CAL_B_MAX - CAL_B_MIN)));
    return v;
}

static void rgb_to_hsl(float r, float g, float b, float *h, float *s, float *l) {
    float max = fmaxf(r, fmaxf(g, b));
    float min = fminf(r, fminf(g, b));
    float delta = max - min;
    *l = (max + min) / 2.0f;
    if (delta == 0.0f) {
        *h = 0.0f; *s = 0.0f;
    } else {
        *s = (*l > 0.5f) ? delta / (2.0f - max - min) : delta / (max + min);
        if      (max == r) *h = (g - b) / delta + (g < b ? 6.0f : 0.0f);
        else if (max == g) *h = (b - r) / delta + 2.0f;
        else               *h = (r - g) / delta + 4.0f;
        *h *= 60.0f;
    }
}

static const char *get_nearest_color(float fR, float fG, float fB) {
    ColorVector rgb = get_intensities(fR, fG, fB);
    float h, s, l;
    rgb_to_hsl(rgb.r, rgb.g, rgb.b, &h, &s, &l);

    if (l < 0.15f)                        return "Black";
    if (l > 0.75f && s < 0.30f)          return "White";
    if (h >= 330.0f || h < 20.0f)        return "Red";
    if (h >= 80.0f  && h < 160.0f)       return "Green";
    if (h >= 190.0f && h < 270.0f)       return "Blue";
    if (s < 0.20f) return (l < 0.5f) ? "Black" : "White";
    return "Unknown";
}

/* =========================================================================
 * TCS3200 hardware driver
 * ========================================================================= */
static void tcs3200_init(void) {
    gpio_set_direction(PIN_S0, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S1, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S2, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S3, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_OUT, GPIO_DIR_INPUT);
    /* 20 % frequency scaling: S0=HIGH, S1=LOW */
    gpio_set_level(PIN_S0, GPIO_LEVEL_HIGH);
    gpio_set_level(PIN_S1, GPIO_LEVEL_LOW);
}

/* Select which colour filter the sensor reads through */
static void set_filter(int s2, int s3) {
    gpio_set_level(PIN_S2, s2);
    gpio_set_level(PIN_S3, s3);
}

/* Count rising edges on OUT pin for sample_ms milliseconds */
static float measure_frequency(int sample_ms) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int pulses = 0;
    int last = gpio_get_level(PIN_OUT);
    double elapsed = 0.0;
    while (elapsed < sample_ms) {
        int cur = gpio_get_level(PIN_OUT);
        if (last == GPIO_LEVEL_LOW && cur == GPIO_LEVEL_HIGH) pulses++;
        last = cur;
        clock_gettime(CLOCK_MONOTONIC, &now);
        elapsed = (now.tv_sec - start.tv_sec) * 1000.0
                + (now.tv_nsec - start.tv_nsec) / 1.0e6;
    }
    return (float)pulses / (sample_ms / 1000.0f);
}

static void read_rgb(float *fR, float *fG, float *fB) {
    /* TCS3200 filter truth table: (S2,S3) = red(L,L) green(H,H) blue(L,H) */
    set_filter(GPIO_LEVEL_LOW,  GPIO_LEVEL_LOW);  sleep_msec(50);
    *fR = measure_frequency(200);
    set_filter(GPIO_LEVEL_HIGH, GPIO_LEVEL_HIGH); sleep_msec(50);
    *fG = measure_frequency(200);
    set_filter(GPIO_LEVEL_LOW,  GPIO_LEVEL_HIGH); sleep_msec(50);
    *fB = measure_frequency(200);
}

/* =========================================================================
 * UART → ESP32 → MQTT
 * Format (per manual, Fig. 8): [4-byte uint32_t length][payload bytes]
 * The ESP32 firmware forwards the payload as-is to the MQTT /send topic.
 * ========================================================================= */
static void send_json(const char *json) {
    uint32_t len = (uint32_t)strlen(json);
    uart_send(UART0, (uint8_t *)&len, sizeof(uint32_t));
    uart_send(UART0, (uint8_t *)json, (int)len);
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void) {
    pynq_init();

    /* UART0 is connected to the ESP32 connectivity board internally */
    uart_init(UART0);

    tcs3200_init();

    printf("Color sensor started — sending JSON via UART → ESP32 → MQTT\n");

    while (1) {
        float fR, fG, fB;
        read_rgb(&fR, &fG, &fB);

        ColorVector intensity = get_intensities(fR, fG, fB);
        const char *color     = get_nearest_color(fR, fG, fB);

        char json[256];
        snprintf(json, sizeof(json),
                 "{\"color\":\"%s\",\"R\":%.3f,\"G\":%.3f,\"B\":%.3f}",
                 color, intensity.r, intensity.g, intensity.b);

        send_json(json);
        printf("Sent: %s\n", json);

        sleep_msec(1000);
    }

    uart_destroy(UART0);
    pynq_destroy();
    return 0;
}