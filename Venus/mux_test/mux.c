#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <libpynq.h>
#include "vl53l0x.h"

/* ---------- I2C MUX + I2C sensors ---------- */

#define TCA_ADDR 0x70

#define CH_DIST1 0
#define CH_DIST2 2
#define CH_TEMP  4

#define VL53_ADDR 0x29

#define MLX90614_ADDR 0x5A
#define MLX90614_REG_OBJECT 0x07

/* ---------- Ultrasonic ---------- */

#define PIN_TRIG IO_AR13
#define PIN_ECHO IO_AR12

#define TIMEOUT_US 30000
#define NUM_SAMPLES 3
#define SAMPLE_DELAY_MS 20

/* ---------- 4x IR line sensors ---------- */

#define IR_1 IO_AR4
#define IR_2 IO_AR5
#define IR_3 IO_AR6
#define IR_4 IO_AR7

/* ---------- TCS3200 color sensor ---------- */

typedef struct {
    float fRmin, fRmax;
    float fGmin, fGmax;
    float fBmin, fBmax;
} ColorCalibration;

typedef struct {
    float r, g, b;
} ColorVector;

typedef struct {
    int s0, s1, s2, s3, out;
} TCS3200_Pins;

/* ---------- Time helpers ---------- */

long long current_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000000LL + tv.tv_usec;
}

void delay_us(long long us) {
    long long start = current_time_us();
    while (current_time_us() - start < us) {
    }
}

/* ---------- GPIO helpers ---------- */

void set_pin(io_t pin, int value) {
    gpio_set_level(pin, value ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}

int read_pin(io_t pin) {
    return gpio_get_level(pin) == GPIO_LEVEL_HIGH;
}

/* ---------- Ultrasonic functions ---------- */

float read_ultrasonic_once_cm(void) {
    long long start_time;
    long long echo_start;
    long long echo_end;

    set_pin(PIN_TRIG, 0);
    delay_us(5);

    set_pin(PIN_TRIG, 1);
    delay_us(10);
    set_pin(PIN_TRIG, 0);

    start_time = current_time_us();

    while (!read_pin(PIN_ECHO)) {
        if (current_time_us() - start_time > TIMEOUT_US) {
            return -1.0f;
        }
    }

    echo_start = current_time_us();

    while (read_pin(PIN_ECHO)) {
        if (current_time_us() - echo_start > TIMEOUT_US) {
            return -2.0f;
        }
    }

    echo_end = current_time_us();

    long long duration_us = echo_end - echo_start;

    return duration_us / 58.0f;
}

void sort_float_array(float arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

float read_ultrasonic_filtered_cm(void) {
    float values[NUM_SAMPLES];
    int valid_count = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        float d = read_ultrasonic_once_cm();

        if (d > 2.0f && d < 400.0f) {
            values[valid_count] = d;
            valid_count++;
        }

        sleep_msec(SAMPLE_DELAY_MS);
    }

    if (valid_count == 0) {
        return -1.0f;
    }

    sort_float_array(values, valid_count);

    if (valid_count % 2 == 1) {
        return values[valid_count / 2];
    } else {
        return (values[valid_count / 2 - 1] + values[valid_count / 2]) / 2.0f;
    }
}

/* ---------- TCA9548A MUX ---------- */

int tca_select(uint8_t channel) {
    if (channel > 7) return 1;

    uint8_t data = 1 << channel;
    return iic_write_register(IIC0, TCA_ADDR, 0x00, &data, 1);
}

/* ---------- MLX90614 ---------- */

bool mlx_read_object_temp(float *temp_c) {
    uint8_t buf[2];

    if (iic_read_register(IIC0, MLX90614_ADDR, MLX90614_REG_OBJECT, buf, 2)) {
        return true;
    }

    uint16_t raw = ((uint16_t)buf[1] << 8) | buf[0];
    *temp_c = raw * 0.02f - 273.15f;

    return false;
}

/* ---------- VL53L0X ---------- */

int read_distance_mm(vl53x *sensor) {
    uint32_t distance = tofReadDistance(sensor);
    return (int)distance;
}

/* ---------- TCS3200 color logic ---------- */

ColorVector get_intensities(ColorCalibration* cal, float fR, float fG, float fB) {
    ColorVector v;

    v.r = (cal->fRmax != cal->fRmin) ? (fR - cal->fRmin) / (cal->fRmax - cal->fRmin) : 0.0f;
    v.g = (cal->fGmax != cal->fGmin) ? (fG - cal->fGmin) / (cal->fGmax - cal->fGmin) : 0.0f;
    v.b = (cal->fBmax != cal->fBmin) ? (fB - cal->fBmin) / (cal->fBmax - cal->fBmin) : 0.0f;

    v.r = fmaxf(0.0f, fminf(1.0f, v.r));
    v.g = fmaxf(0.0f, fminf(1.0f, v.g));
    v.b = fmaxf(0.0f, fminf(1.0f, v.b));

    return v;
}

void rgb_to_hsl(float r, float g, float b, float *h, float *s, float *l) {
    float max = fmaxf(r, fmaxf(g, b));
    float min = fminf(r, fminf(g, b));
    float delta = max - min;

    *l = (max + min) / 2.0f;

    if (delta == 0) {
        *h = 0;
        *s = 0;
    } else {
        *s = (*l > 0.5f) ? delta / (2.0f - max - min) : delta / (max + min);

        if (max == r) {
            *h = (g - b) / delta + (g < b ? 6.0f : 0.0f);
        } else if (max == g) {
            *h = (b - r) / delta + 2.0f;
        } else {
            *h = (r - g) / delta + 4.0f;
        }

        *h *= 60.0f;
    }
}

const char* get_nearest_color(ColorCalibration* cal, float fR, float fG, float fB) {
    ColorVector rgb = get_intensities(cal, fR, fG, fB);

    float h, s, l;
    rgb_to_hsl(rgb.r, rgb.g, rgb.b, &h, &s, &l);

    if (l < 0.15f) {
        return "Black";
    }

    if (rgb.r > 0.70f && rgb.g > 0.70f && rgb.b > 0.70f) {
        return "White";
    }

    if (h >= 330.0f || h < 20.0f) {
        return "Red";
    }

    if (h >= 80.0f && h < 160.0f) {
        return "Green";
    }

    if (h >= 190.0f && h < 270.0f) {
        return "Blue";
    }

    if (s < 0.20f) {
        return (l < 0.5f) ? "Black" : "White";
    }

    return "Unknown";
}

void tcs3200_init(TCS3200_Pins pins) {
    gpio_set_direction(pins.s0, GPIO_DIR_OUTPUT);
    gpio_set_direction(pins.s1, GPIO_DIR_OUTPUT);
    gpio_set_direction(pins.s2, GPIO_DIR_OUTPUT);
    gpio_set_direction(pins.s3, GPIO_DIR_OUTPUT);
    gpio_set_direction(pins.out, GPIO_DIR_INPUT);

    gpio_set_level(pins.s0, GPIO_LEVEL_HIGH);
    gpio_set_level(pins.s1, GPIO_LEVEL_LOW);
}

void tcs3200_set_filter(TCS3200_Pins pins, const char* color) {
    if (strcmp(color, "red") == 0) {
        gpio_set_level(pins.s2, GPIO_LEVEL_LOW);
        gpio_set_level(pins.s3, GPIO_LEVEL_LOW);
    } else if (strcmp(color, "blue") == 0) {
        gpio_set_level(pins.s2, GPIO_LEVEL_LOW);
        gpio_set_level(pins.s3, GPIO_LEVEL_HIGH);
    } else if (strcmp(color, "green") == 0) {
        gpio_set_level(pins.s2, GPIO_LEVEL_HIGH);
        gpio_set_level(pins.s3, GPIO_LEVEL_HIGH);
    }
}

float tcs3200_measure_frequency(TCS3200_Pins pins, int sample_ms) {
    int pulse_count = 0;

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    double elapsed_ms = 0;
    int last_level = gpio_get_level(pins.out);

    while (elapsed_ms < sample_ms) {
        int current_level = gpio_get_level(pins.out);

        if (last_level == GPIO_LEVEL_LOW && current_level == GPIO_LEVEL_HIGH) {
            pulse_count++;
        }

        last_level = current_level;

        clock_gettime(CLOCK_MONOTONIC, &now);

        elapsed_ms =
            (now.tv_sec - start.tv_sec) * 1000.0 +
            (now.tv_nsec - start.tv_nsec) / 1000000.0;
    }

    return (float)pulse_count / (sample_ms / 1000.0f);
}

void tcs3200_read_rgb(TCS3200_Pins pins, float* fR, float* fG, float* fB) {
    tcs3200_set_filter(pins, "red");
    sleep_msec(50);
    *fR = tcs3200_measure_frequency(pins, 200);

    tcs3200_set_filter(pins, "green");
    sleep_msec(50);
    *fG = tcs3200_measure_frequency(pins, 200);

    tcs3200_set_filter(pins, "blue");
    sleep_msec(50);
    *fB = tcs3200_measure_frequency(pins, 200);
}

/* ---------- Main ---------- */

int main(void) {
    setbuf(stdout, NULL);

    pynq_init();

    printf("ALL SENSOR TEST\n\n");

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);
    sleep_msec(100);

    gpio_set_direction(PIN_TRIG, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_ECHO, GPIO_DIR_INPUT);
    set_pin(PIN_TRIG, 0);

    gpio_set_direction(IR_1, GPIO_DIR_INPUT);
    gpio_set_direction(IR_2, GPIO_DIR_INPUT);
    gpio_set_direction(IR_3, GPIO_DIR_INPUT);
    gpio_set_direction(IR_4, GPIO_DIR_INPUT);

    TCS3200_Pins color_pins = {
        .s0 = IO_A0,
        .s1 = IO_A1,
        .s2 = IO_A2,
        .s3 = IO_A3,
        .out = IO_A4
    };

    tcs3200_init(color_pins);

    ColorCalibration cal = {0};

    printf("Place BLACK object and press Enter...");
    while (getchar() != '\n');

    tcs3200_read_rgb(color_pins,
                     &cal.fRmin,
                     &cal.fGmin,
                     &cal.fBmin);

    printf("Place WHITE object and press Enter...");
    while (getchar() != '\n');

    tcs3200_read_rgb(color_pins,
                     &cal.fRmax,
                     &cal.fGmax,
                     &cal.fBmax);

    vl53x dist1;
    vl53x dist2;

    tca_select(CH_DIST1);
    tofInit(&dist1, IIC0, VL53_ADDR, 0);

    tca_select(CH_DIST2);
    tofInit(&dist2, IIC0, VL53_ADDR, 0);

    while (1) {

        tca_select(CH_DIST1);
        sleep_msec(5);
        int d1 = read_distance_mm(&dist1);

        tca_select(CH_DIST2);
        sleep_msec(5);
        int d2 = read_distance_mm(&dist2);

        float temp_c = -999.0f;

        tca_select(CH_TEMP);
        sleep_msec(5);

        bool temp_failed = mlx_read_object_temp(&temp_c);

        float us = read_ultrasonic_filtered_cm();

        int ir1 = gpio_get_level(IR_1);
        int ir2 = gpio_get_level(IR_2);
        int ir3 = gpio_get_level(IR_3);
        int ir4 = gpio_get_level(IR_4);

        float fR, fG, fB;

        tcs3200_read_rgb(color_pins, &fR, &fG, &fB);

        const char* color =
            get_nearest_color(&cal, fR, fG, fB);

        ColorVector rgb =
            get_intensities(&cal, fR, fG, fB);

        printf("D1=%dmm ", d1);
        printf("D2=%dmm ", d2);

        if (temp_failed) {
            printf("TEMP=FAILED ");
        } else {
            printf("TEMP=%.2fC ", temp_c);
        }

        if (us < 0.0f) {
            printf("US=FAILED ");
        } else {
            printf("US=%.1fcm ", us);
        }

        printf("IR=%d%d%d%d ",
               ir1, ir2, ir3, ir4);

        printf("COLOR=%s ",
               color);

        printf("[R=%.2f G=%.2f B=%.2f]",
               rgb.r,
               rgb.g,
               rgb.b);

        printf("\n");

        sleep_msec(300);
    }

    iic_destroy(IIC0);
    pynq_destroy();

    return 0;
}
