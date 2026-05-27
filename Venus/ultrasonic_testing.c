#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <libpynq.h>

#define PIN_TRIG IO_AR9
#define PIN_ECHO IO_AR8

#define TIMEOUT_US 30000

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

float read_ultrasonic_cm(void) {
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

int main(void) {
    setbuf(stdout, NULL);

    pynq_init();

    printf("HC-SR04 ultrasonic sensor test\n");
    printf("TRIG = AR12\n");
    printf("ECHO = AR11 through voltage divider\n\n");

    gpio_set_direction(PIN_TRIG, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_ECHO, GPIO_DIR_INPUT);

    set_pin(PIN_TRIG, 0);
    sleep_msec(500);

    while (1) {
        printf("Idle ECHO level: %d\n", read_pin(PIN_ECHO));

        float distance_cm = read_ultrasonic_cm();

        if (distance_cm == -1.0f) {
            printf("FAILED: ECHO never went HIGH\n\n");
        } else if (distance_cm == -2.0f) {
            printf("FAILED: ECHO stayed HIGH too long\n\n");
        } else {
            printf("Distance: %.1f cm\n\n", distance_cm);
        }

        sleep_msec(500);
    }

    pynq_destroy();

    return EXIT_SUCCESS;
}
