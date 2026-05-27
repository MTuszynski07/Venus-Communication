/*
 * venus_comm_test.c  –  Venus Group 36, communication test
 *
 * What this does:
 *   - Routes AR0/AR1 through the switchbox to UART0 (required – this is how
 *     the ESP32 is physically connected, per the course documentation).
 *   - Checks AR3 (ESP32 ready-to-accept) before every send.
 *   - Sends periodic "ready" and telemetry JSON frames over UART0 → ESP32
 *     → MQTT so the PC can see the robot is alive.
 *   - Listens for a {"cmd":"start_test"} command arriving from the PC via
 *     MQTT → ESP32 → UART0, then replies with "running" / telemetry / "complete".
 *
 * Why the original crashed:
 *   uart_send(UART0, (uint8_t)byte)  ← 2-arg call treated the byte VALUE
 *   as a memory pointer.  The 3-arg buffer API must be used instead:
 *   uart_send(UART0, ptr, len)
 */

#include <libpynq.h>

#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── identity ──────────────────────────────────────────────────────────────── */
#ifndef MODULE_NUMBER
#define MODULE_NUMBER "robot_24_1"
#endif

/* ── pin assignments (per course docs) ────────────────────────────────────── */
#define PIN_UART_RX  IO_AR0   /* ESP32 TX → PYNQ RX  (switchbox → UART0 RX) */
#define PIN_UART_TX  IO_AR1   /* PYNQ TX → ESP32 RX  (switchbox → UART0 TX) */
#define PIN_ESP_RDY  IO_AR3   /* ESP32 ready to accept data   (input, check)  */

/* ── timing ────────────────────────────────────────────────────────────────── */
#define POLL_DELAY_MS        10U
#define READY_PERIOD_MS      2000U
#define TELEMETRY_PERIOD_MS  2000U
#define ESP_READY_TIMEOUT_MS 200U   /* max wait for AR3 high before giving up */
#define MAX_BYTES_PER_POLL   64U

/* ── buffer sizes ──────────────────────────────────────────────────────────── */
#define PAYLOAD_BUF_SIZE  512U
#define JSON_BUF_SIZE     384U

/* ── types ─────────────────────────────────────────────────────────────────── */
typedef enum { PHASE_WAITING = 0, PHASE_COMPLETE } phase_t;

typedef struct {
    uint8_t  len_bytes[4];
    uint8_t  len_idx;
    uint32_t payload_len;
    uint32_t payload_idx;
    bool     discarding;
    char     buf[PAYLOAD_BUF_SIZE + 1U];
} rx_state_t;

typedef struct { double x, y, theta; } pose_t;

/* ── globals ───────────────────────────────────────────────────────────────── */
static volatile sig_atomic_t g_run = 1;

static void on_signal(int s) { (void)s; g_run = 0; }

/* ── helpers ───────────────────────────────────────────────────────────────── */
static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void rx_reset(rx_state_t *r) {
    r->len_idx     = 0;
    r->payload_len = 0;
    r->payload_idx = 0;
    r->discarding  = false;
}

/* ── UART send ─────────────────────────────────────────────────────────────── */

/*
 * Wait up to ESP_READY_TIMEOUT_MS for AR3 (ESP32 ready-to-accept) to go HIGH,
 * then send a length-prefixed frame: [4-byte LE uint32 length][payload bytes].
 * We always send even if AR3 never goes high (avoids blocking indefinitely).
 */
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

static void send_status(const char *status) {
    char buf[JSON_BUF_SIZE];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"status\",\"robot\":\"" MODULE_NUMBER
        "\",\"t_ms\":%" PRIu64 ",\"status\":\"%s\"}",
        now_ms(), status);
    send_frame(buf);
    printf("<< %s\n", buf);
    fflush(stdout);
}

static void send_telemetry(const pose_t *p, bool moving, bool done) {
    char buf[JSON_BUF_SIZE];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"telemetry_test\",\"robot\":\"" MODULE_NUMBER
        "\",\"t_ms\":%" PRIu64 ",\"coordX\":%.6f,\"coordY\":%.6f,"
        "\"theta\":%.6f,\"moving\":%s,\"done\":%s}",
        now_ms(), p->x, p->y, p->theta,
        moving ? "true" : "false",
        done   ? "true" : "false");
    send_frame(buf);
    printf("<< %s\n", buf);
    fflush(stdout);
}

static void send_ready(const pose_t *p) {
    char buf[JSON_BUF_SIZE];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"status\",\"robot\":\"" MODULE_NUMBER
        "\",\"t_ms\":%" PRIu64 ",\"status\":\"ready\","
        "\"coordX\":%.6f,\"coordY\":%.6f,\"theta\":%.6f,"
        "\"moving\":false,\"done\":false}",
        now_ms(), p->x, p->y, p->theta);
    send_frame(buf);
    printf("<< %s\n", buf);
    fflush(stdout);
}

/* ── UART receive ──────────────────────────────────────────────────────────── */

static bool is_start_cmd(const char *s) {
    return strstr(s, "\"cmd\"") != NULL &&
           (strstr(s, "\"start_test\"") != NULL ||
            strstr(s, "\"start\"")      != NULL);
}

static void handle_cmd(const char *cmd, phase_t *phase, const pose_t *p) {
    printf(">> %s\n", cmd);
    fflush(stdout);
    if (*phase == PHASE_WAITING && is_start_cmd(cmd)) {
        send_status("running");
        send_telemetry(p, false, true);
        send_status("complete");
        *phase = PHASE_COMPLETE;
    }
}

/*
 * Drain up to MAX_BYTES_PER_POLL bytes from the UART FIFO each call.
 * Reassembles the length-prefixed frames sent by the ESP32.
 */
static void poll_uart(rx_state_t *r, phase_t *phase, const pose_t *p) {
    unsigned int limit = MAX_BYTES_PER_POLL;

    while (limit-- && uart_has_data(UART0)) {
        uint8_t byte = uart_recv(UART0);

        /* Reading the 4-byte LE length header */
        if (r->len_idx < 4U) {
            r->len_bytes[r->len_idx++] = byte;
            if (r->len_idx < 4U) continue;

            r->payload_len = (uint32_t)r->len_bytes[0]
                           | ((uint32_t)r->len_bytes[1] <<  8)
                           | ((uint32_t)r->len_bytes[2] << 16)
                           | ((uint32_t)r->len_bytes[3] << 24);
            r->payload_idx = 0;

            if      (r->payload_len == 0U)              rx_reset(r);
            else if (r->payload_len > PAYLOAD_BUF_SIZE) r->discarding = true;
            continue;
        }

        /* Drop bytes from an oversized payload */
        if (r->discarding) {
            if (++r->payload_idx >= r->payload_len) {
                printf("!! Discarded oversized frame (%u bytes)\n", r->payload_len);
                fflush(stdout);
                rx_reset(r);
            }
            continue;
        }

        /* Accumulate payload */
        r->buf[r->payload_idx++] = (char)byte;
        if (r->payload_idx >= r->payload_len) {
            r->buf[r->payload_len] = '\0';
            handle_cmd(r->buf, phase, p);
            rx_reset(r);
        }
    }
}

/* ── main ──────────────────────────────────────────────────────────────────── */

int main(void) {
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    printf("venus_comm_test: module=" MODULE_NUMBER "\n");
    fflush(stdout);

    pynq_init();

    /*
     * Route AR0 → UART0 RX and AR1 → UART0 TX through the switchbox.
     * This is mandatory: the ESP32 on the connectivity board is physically
     * wired to AR0/AR1 and the switchbox is the only path to UART0.
     */
    switchbox_set_pin(PIN_UART_RX, SWB_UART0_RX);
    switchbox_set_pin(PIN_UART_TX, SWB_UART0_TX);

    gpio_set_direction(PIN_ESP_RDY, GPIO_DIR_INPUT);

    uart_init(UART0);

    rx_state_t rx    = {0};
    pose_t     pose  = {0.0, 0.0, 0.0};
    phase_t    phase = PHASE_WAITING;

    uint64_t next_ready_ms = now_ms();
    uint64_t next_telem_ms = now_ms();

    while (g_run) {
        uint64_t t = now_ms();

        poll_uart(&rx, &phase, &pose);

        if (phase == PHASE_WAITING && t >= next_ready_ms) {
            send_ready(&pose);
            next_ready_ms = t + READY_PERIOD_MS;
        }

        if (t >= next_telem_ms) {
            send_telemetry(&pose, false, phase == PHASE_COMPLETE);
            next_telem_ms = t + TELEMETRY_PERIOD_MS;
        }

        sleep_msec(POLL_DELAY_MS);
    }

    printf("Stopping.\n");
    fflush(stdout);
    uart_destroy(UART0);
    pynq_destroy();
    return EXIT_SUCCESS;
}
