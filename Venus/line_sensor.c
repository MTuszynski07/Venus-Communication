#include <libpynq.h>

int main(void)
{
    pynq_init();

    for (int i = IO_AR0; i <= IO_AR7; i++) {
        gpio_set_direction(i, GPIO_DIR_INPUT);
    }

    printf("Move sensor over black tape and white surface\n");

    while (1) {
        printf("AR0=%d  AR1=%d  AR2=%d  AR3=%d  AR4=%d  AR5=%d  AR6=%d  AR7=%d\n",
            (int)gpio_get_level(IO_AR0),
            (int)gpio_get_level(IO_AR1),
            (int)gpio_get_level(IO_AR2),
            (int)gpio_get_level(IO_AR3),
            (int)gpio_get_level(IO_AR4),
            (int)gpio_get_level(IO_AR5),
            (int)gpio_get_level(IO_AR6),
            (int)gpio_get_level(IO_AR7));
        fflush(stdout);
        sleep_msec(10);
    }

    pynq_destroy();
    return EXIT_SUCCESS;
}
