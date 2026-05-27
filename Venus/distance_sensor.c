#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libpynq.h>

#define VL53_ADDR 0x29

#define SYSRANGE_START               0x00
#define SYSTEM_INTERRUPT_CLEAR       0x0B
#define RESULT_INTERRUPT_STATUS      0x13
#define RESULT_RANGE_MM              0x1E
#define IDENTIFICATION_MODEL_ID      0xC0

#define DISTANCE_OFFSET_MM 35
#define NUM_SAMPLES 5

static bool vl53_w8(uint8_t reg, uint8_t val) {
    return iic_write_register(IIC1, VL53_ADDR, reg, &val, 1);
}

static bool vl53_r8(uint8_t reg, uint8_t *val) {
    return iic_read_register(IIC1, VL53_ADDR, reg, val, 1);
}

static bool vl53_r16(uint8_t reg, uint16_t *val) {
    uint8_t buf[2];

    if (iic_read_register(IIC1, VL53_ADDR, reg, buf, 2)) {
        return true;
    }

    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return false;
}

static uint16_t vl53_read_raw_mm(void) {
    uint8_t status;
    int timeout = 100;

    vl53_w8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (vl53_w8(SYSRANGE_START, 0x01)) {
        return 0xFFFF;
    }

    while (timeout > 0) {
        if (vl53_r8(RESULT_INTERRUPT_STATUS, &status)) {
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

    uint16_t raw_mm;

    if (vl53_r16(RESULT_RANGE_MM, &raw_mm)) {
        return 0xFFFF;
    }

    vl53_w8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return raw_mm;
}

static int vl53_correct_distance_mm(uint16_t raw_mm) {
    int corrected = (int)raw_mm - DISTANCE_OFFSET_MM;

    if (corrected < 0) {
        corrected = 0;
    }

    return corrected;
}

int vl53_init(void) {
    switchbox_set_pin(IO_AR_SCL, SWB_IIC1_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC1_SDA);

    sleep_msec(100);

    iic_init(IIC1);

    sleep_msec(100);

    uint8_t id = 0;

    if (vl53_r8(IDENTIFICATION_MODEL_ID, &id)) {
        return -1;
    }

    if (id != 0xEE) {
        return -2;
    }

    return 0;
}

// Main function you use in the robot code.
// Returns calibrated average distance in mm.
// Returns -1 if all readings failed.
int vl53_get_distance_mm(void) {
    int sum = 0;
    int valid_count = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint16_t raw_mm = vl53_read_raw_mm();

        if (raw_mm != 0xFFFF && raw_mm < 8000) {
            sum += vl53_correct_distance_mm(raw_mm);
            valid_count++;
        }

        sleep_msec(20);
    }

    if (valid_count == 0) {
        return -1;
    }

    return sum / valid_count;
}

int main(void) {
    setbuf(stdout, NULL);

    pynq_init();

    if (vl53_init() != 0) {
        printf("VL53L0X init failed\n");
        iic_destroy(IIC1);
        pynq_destroy();
        return EXIT_FAILURE;
    }

    while (1) {
        int distance_mm = vl53_get_distance_mm();

        if (distance_mm >= 0) {
            printf("Distance: %d mm\n", distance_mm);
        } else {
            printf("Distance read failed\n");
        }

        sleep_msec(100);
    }

    iic_destroy(IIC1);
    pynq_destroy();

    return EXIT_SUCCESS;
}