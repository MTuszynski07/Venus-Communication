#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libpynq.h>

#define MLX90614_ADDR 0x5A

#define MLX90614_REG_AMBIENT 0x06
#define MLX90614_REG_OBJECT  0x07

static bool mlx_read16(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];

    if (iic_read_register(IIC0, MLX90614_ADDR, reg, buf, 2)) {
        return true; // failed
    }

    // MLX90614 sends LSB first, then MSB
    *value = ((uint16_t)buf[1] << 8) | buf[0];

    return false; // success
}

static float mlx_raw_to_celsius(uint16_t raw) {
    return raw * 0.02f - 273.15f;
}

int main(void) {
    setbuf(stdout, NULL);

    pynq_init();

    printf("MLX90614 IR temperature sensor test\n");
    printf("VCC = 3.3V, GND = GND, SDA = AR_SDA, SCL = AR_SCL\n");
    printf("I2C address = 0x%02X\n\n", MLX90614_ADDR);

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    sleep_msec(100);

    iic_init(IIC0);

    sleep_msec(100);

    while (1) {
        uint16_t raw_ambient = 0;
        uint16_t raw_object = 0;

        bool ambient_failed = mlx_read16(MLX90614_REG_AMBIENT, &raw_ambient);
        bool object_failed = mlx_read16(MLX90614_REG_OBJECT, &raw_object);

        if (ambient_failed) {
            printf("FAILED: could not read ambient temperature\n");
        }

        if (object_failed) {
            printf("FAILED: could not read object temperature\n");
        }

        if (!ambient_failed && !object_failed) {
            float ambient_c = mlx_raw_to_celsius(raw_ambient);
            float object_c = mlx_raw_to_celsius(raw_object);

            printf("Ambient: %.2f C | Object: %.2f C\n",
                   ambient_c,
                   object_c);
        }

        printf("\n");
        sleep_msec(500);
    }

    iic_destroy(IIC0);
    pynq_destroy();

    return EXIT_SUCCESS;
}
