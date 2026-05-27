/*
 * comm.c — Venus Group 36, robot-to-PC communication module
 *
 * See comm.h for the full protocol description.
 *
 * Usage from main():
 *
 *   pynq_init();
 *   comm_init();
 *
 *   comm_send_status("ready");
 *
 *   while (running) {
 *       char cmd[COMM_CMD_BUF_SIZE];
 *       if (comm_poll_command(cmd, sizeof(cmd))) {
 *           // handle cmd ...
 *       }
 *       // ... sensor reads, motion ...
 *       comm_send_telemetry(x, y, theta, air_temp, moving);
 *   }
 *
 *   comm_send_status("complete");
 *   comm_destroy();
 *   pynq_destroy();
 */

#include "comm.h"

#include <libpynq.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ── Robot identity ───────────────────────────────────────────────────────── */
/* Override at compile time:  gcc … -DMODULE_NUMBER='"robot_24_2"'            */
#ifndef MODULE_NUMBER
#define MODULE_NUMBER "robot_24_1"
#endif

/* ── Connectivity-board pin assignments (per course manual) ──────────────── */
#define PIN_UART_RX  IO_AR0   /* ESP32 TX  → PYNQ RX  (switchbox → UART0 RX) */
#define PIN_UART_TX  IO_AR1   /* PYNQ TX   → ESP32 RX (switchbox → UART0 TX) */
#define PIN_ESP_RDY  IO_AR3   /* ESP32 ready-to-accept (input, active HIGH)   */

/* ── Tunables ─────────────────────────────────────────────────────────────── */
#define ESP_READY_TIMEOUT_MS  200U   /* max wait for AR3 before sending anyway */
#define JSON_BUF_SIZE         512U   /* max serialised JSON length             */
#define RX_PAYLOAD_MAX        512U   /* max incoming payload we accept         */
#define MAX_BYTES_PER_POLL     64U   /* UART bytes drained per comm_poll call  */

/* ── Internal receive state machine ──────────────────────────────────────── */
typedef struct {
    uint8_t  len_bytes[4];   /* accumulates the 4-byte LE length header */
    uint8_t  len_idx;        /* how many length bytes received so far    */
    uint32_t payload_len;    /* decoded expected payload length          */
    uint32_t payload_idx;    /* bytes of payload received so far         */
    bool     discarding;     /* true when payload_len > RX_PAYLOAD_MAX   */
    char     buf[RX_PAYLOAD_MAX + 1U]; /* payload accumulation buffer    */
} rx_state_t;

static rx_state_t s_rx;

/* ── Internal helpers ─────────────────────────────────────────────────────── */

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void rx_reset(void) {
    s_rx.len_idx     = 0;
    s_rx.payload_len = 0;
    s_rx.payload_idx = 0;
    s_rx.discarding  = false;
}

/* Wait for the ESP32 to signal it is ready, then transmit one length-prefixed
 * frame.  The frame format is: [4-byte LE uint32 length][payload bytes].     */
static void send_frame(const char *json) {
    for (uint32_t i = 0; i < ESP_READY_TIMEOUT_MS; i++) {
        if (gpio_get_level(PIN_ESP_RDY) == GPIO_LEVEL_HIGH) break;
        sleep_msec(1);
    }

    uint32_t len = (uint32_t)strlen(json);
    uint8_t *len_b = (uint8_t *)&len;
    for (size_t i = 0; i < sizeof(len); i++) uart_send(UART0, len_b[i]);
    for (uint32_t i = 0; i < len; i++)      uart_send(UART0, (uint8_t)json[i]);
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void comm_init(void) {
    switchbox_set_pin(PIN_UART_RX, SWB_UART0_RX);
    switchbox_set_pin(PIN_UART_TX, SWB_UART0_TX);
    gpio_set_direction(PIN_ESP_RDY, GPIO_DIR_INPUT);
    uart_init(UART0);
    rx_reset();

    printf("[comm] init: module=" MODULE_NUMBER "\n");
    fflush(stdout);
}

void comm_destroy(void) {
    uart_destroy(UART0);
    printf("[comm] destroyed\n");
    fflush(stdout);
}

/* ── Outgoing messages ────────────────────────────────────────────────────── */

void comm_send_status(const char *status) {
    char buf[JSON_BUF_SIZE];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"status\",\"robot\":\"" MODULE_NUMBER
        "\",\"t_ms\":%" PRIu64 ",\"status\":\"%s\"}",
        now_ms(), status);
    send_frame(buf);
    printf("[comm] >> status: %s\n", status);
    fflush(stdout);
}

void comm_send_telemetry(double x, double y, double theta,
                         float air_temp, bool moving) {
    char buf[JSON_BUF_SIZE];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"telemetry\",\"robot\":\"" MODULE_NUMBER
        "\",\"t_ms\":%" PRIu64
        ",\"coordX\":%.4f,\"coordY\":%.4f,\"theta\":%.4f"
        ",\"airTemp\":%.2f,\"moving\":%s}",
        now_ms(), x, y, theta,
        (double)air_temp,
        moving ? "true" : "false");
    send_frame(buf);
    /* telemetry is high-frequency — skip console print to reduce noise */
}

void comm_send_rock_sample(double x, double y,
                           const char *color, int size_mm, float temp_c) {
    char buf[JSON_BUF_SIZE];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"rock_sample\",\"robot\":\"" MODULE_NUMBER
        "\",\"t_ms\":%" PRIu64
        ",\"coordX\":%.4f,\"coordY\":%.4f"
        ",\"color\":\"%s\",\"size_mm\":%d,\"temp_c\":%.2f}",
        now_ms(), x, y, color, size_mm, (double)temp_c);
    send_frame(buf);
    printf("[comm] >> rock_sample: %s %d mm %.1f °C @ (%.3f, %.3f)\n",
           color, size_mm, (double)temp_c, x, y);
    fflush(stdout);
}

void comm_send_event(double x, double y, const char *event_type) {
    char buf[JSON_BUF_SIZE];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"event\",\"robot\":\"" MODULE_NUMBER
        "\",\"t_ms\":%" PRIu64
        ",\"coordX\":%.4f,\"coordY\":%.4f,\"event\":\"%s\"}",
        now_ms(), x, y, event_type);
    send_frame(buf);
    printf("[comm] >> event: %s @ (%.3f, %.3f)\n", event_type, x, y);
    fflush(stdout);
}

/* ── Incoming messages ────────────────────────────────────────────────────── */

bool comm_poll_command(char *cmd_out, uint32_t max_len) {
    unsigned int limit = MAX_BYTES_PER_POLL;

    while (limit-- && uart_has_data(UART0)) {
        uint8_t byte = uart_recv(UART0);

        /* --- Accumulate the 4-byte LE length header --- */
        if (s_rx.len_idx < 4U) {
            s_rx.len_bytes[s_rx.len_idx++] = byte;
            if (s_rx.len_idx < 4U) continue;

            /* All 4 length bytes received: decode */
            s_rx.payload_len =
                  (uint32_t)s_rx.len_bytes[0]
                | ((uint32_t)s_rx.len_bytes[1] <<  8)
                | ((uint32_t)s_rx.len_bytes[2] << 16)
                | ((uint32_t)s_rx.len_bytes[3] << 24);
            s_rx.payload_idx = 0;

            if (s_rx.payload_len == 0U) {
                rx_reset();
            } else if (s_rx.payload_len > RX_PAYLOAD_MAX) {
                s_rx.discarding = true;
                printf("[comm] oversized frame (%u bytes), discarding\n",
                       s_rx.payload_len);
                fflush(stdout);
            }
            continue;
        }

        /* --- Discard oversized payloads --- */
        if (s_rx.discarding) {
            if (++s_rx.payload_idx >= s_rx.payload_len) rx_reset();
            continue;
        }

        /* --- Accumulate payload --- */
        s_rx.buf[s_rx.payload_idx++] = (char)byte;
        if (s_rx.payload_idx >= s_rx.payload_len) {
            s_rx.buf[s_rx.payload_len] = '\0';

            /* Copy into caller's buffer (safely) */
            uint32_t copy_len = (s_rx.payload_len < max_len - 1U)
                                 ? s_rx.payload_len : max_len - 1U;
            memcpy(cmd_out, s_rx.buf, copy_len);
            cmd_out[copy_len] = '\0';

            printf("[comm] << %s\n", cmd_out);
            fflush(stdout);

            rx_reset();
            return true;
        }
    }

    return false;
}
