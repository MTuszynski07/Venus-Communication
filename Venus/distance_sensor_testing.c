#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libpynq.h>

#define VL53_ADDR 0x29

#define SYSRANGE_START               0x00
#define SYSTEM_SEQUENCE_CONFIG       0x01
#define SYSTEM_INTERRUPT_CLEAR       0x0B
#define RESULT_INTERRUPT_STATUS      0x13
#define RESULT_RANGE_MM              0x1E
#define IDENTIFICATION_MODEL_ID      0xC0

#define DISTANCE_OFFSET_MM 35     // change this after calibration
#define NUM_SAMPLES 5             // average 5 readings

static bool w8(uint8_t reg, uint8_t val) {
    return iic_write_register(IIC1, VL53_ADDR, reg, &val, 1);
}

static bool r8(uint8_t reg, uint8_t *val) {
    return iic_read_register(IIC1, VL53_ADDR, reg, val, 1);
}

static bool r16(uint8_t reg, uint16_t *val) {
    uint8_t buf[2];

    if (iic_read_register(IIC1, VL53_ADDR, reg, buf, 2)) {
        return true;
    }

    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return false;
}

static uint16_t read_raw_distance_mm(void) {
    uint8_t status;
    int timeout = 100;

    w8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (w8(SYSRANGE_START, 0x01)) {
        return 0xFFFF;
    }

    while (timeout > 0) {
        if (r8(RESULT_INTERRUPT_STATUS, &status)) {
            return 0xFFFF;
        }

        if (status & 0x07) {
            break;
        }

        sleep_msec(10);
        timeout--;
    }

    if (timeout == 0) {
        return 0xFFFF;
    }

    uint16_t distance;

    if (r16(RESULT_RANGE_MM, &distance)) {
        return 0xFFFF;
    }

    w8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return distance;
}

static int corrected_distance_mm(uint16_t raw_mm) {
    int corrected = (int)raw_mm - DISTANCE_OFFSET_MM;

    if (corrected < 0) {
        corrected = 0;
    }

    return corrected;
}

static int average_corrected_distance_mm(void) {
    int sum = 0;
    int valid_count = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint16_t raw = read_raw_distance_mm();

        if (raw != 0xFFFF && raw < 8000) {
            sum += corrected_distance_mm(raw);
            valid_count++;
        }

        sleep_msec(50);
    }

    if (valid_count == 0) {
        return -1;
    }

    return sum / valid_count;
}

int main(void) {
    setbuf(stdout, NULL);

    pynq_init();

    printf("VL53L0X calibrated distance reader\n");

    switchbox_set_pin(IO_AR_SCL, SWB_IIC1_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC1_SDA);

    sleep_msec(500);

    iic_init(IIC1);

    sleep_msec(500);

    uint8_t id = 0;

    if (r8(IDENTIFICATION_MODEL_ID, &id)) {
        printf("ERROR: Could not detect VL53L0X.\n");
        iic_destroy(IIC1);
        pynq_destroy();
        return EXIT_FAILURE;
    }

    printf("Model ID: 0x%02X\n", id);

    if (id != 0xEE) {
        printf("WARNING: Unexpected model ID.\n");
    }

    printf("Using offset: %d mm\n", DISTANCE_OFFSET_MM);
    printf("Averaging %d samples per reading.\n\n", NUM_SAMPLES);

    while (1) {
        uint16_t raw = read_raw_distance_mm();
        int corrected_avg = average_corrected_distance_mm();

        if (raw == 0xFFFF || corrected_avg < 0) {
            printf("Distance read failed\n");
        } else {
            int corrected_single = corrected_distance_mm(raw);

            printf("Raw: %u mm | Corrected: %d mm | Average corrected: %d mm (%.1f cm)\n",
                   raw,
                   corrected_single,
                   corrected_avg,
                   corrected_avg / 10.0f);
        }

        sleep_msec(500);
    }

    iic_destroy(IIC1);
    pynq_destroy();

    return EXIT_SUCCESS;
}
