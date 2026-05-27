#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <libpynq.h>

#define PIN_TRIG IO_AR13
#define PIN_ECHO IO_AR12

#define TIMEOUT_US 30000
#define NUM_SAMPLES 7
#define SAMPLE_DELAY_MS 60

long long current_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000000LL + tv.tv_usec;
}

void delay_us(long long us) {
    long long start = current_time_us();
    while (current_time_us() - start < us) {
        // busy wait
    }
}

void set_pin(io_t pin, int value) {
    gpio_set_level(pin, value ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}

int read_pin(io_t pin) {
    return gpio_get_level(pin) == GPIO_LEVEL_HIGH;
}

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

    // HC-SR04 approximation: distance in cm = pulse time / 58
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

        // Keep only reasonable readings
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

    // Median is more stable than a normal average if one reading is bad
    if (valid_count % 2 == 1) {
        return values[valid_count / 2];
    } else {
        return (values[valid_count / 2 - 1] + values[valid_count / 2]) / 2.0f;
    }
}

int main(void) {
    setbuf(stdout, NULL);

    pynq_init();

    printf("HC-SR04 ultrasonic sensor final test\n");
    printf("TRIG = AR13\n");
    printf("ECHO = AR12 through voltage divider\n\n");

    gpio_set_direction(PIN_TRIG, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_ECHO, GPIO_DIR_INPUT);

    set_pin(PIN_TRIG, 0);
    sleep_msec(500);

    while (1) {
        float distance_cm = read_ultrasonic_filtered_cm();

        if (distance_cm < 0.0f) {
            printf("FAILED: no valid ultrasonic reading\n\n");
        } else {
            printf("Filtered distance: %.1f cm\n\n", distance_cm);
        }

        sleep_msec(300);
    }

    pynq_destroy();

    return EXIT_SUCCESS;
}
