/*
 * comm.h — Venus Group 36, robot-to-PC communication API
 *
 * Transport layer: PYNQ ↔ ESP32C3 via UART0 (AR0=RX, AR1=TX).
 *                  ESP32C3 ↔ MQTT broker via WiFi.
 *
 * Frame format (both directions):
 *   [4 bytes little-endian uint32 = payload byte count][payload as UTF-8 JSON]
 *
 * JSON message types the robot sends (published to /pynqbridge/{MODULE}/send):
 *
 *   {"type":"status",      "robot":"…", "t_ms":…, "status":"ready|running|complete"}
 *
 *   {"type":"telemetry",   "robot":"…", "t_ms":…,
 *    "coordX":…, "coordY":…, "theta":…,
 *    "airTemp":…, "moving":true|false}
 *
 *   {"type":"rock_sample", "robot":"…", "t_ms":…,
 *    "coordX":…, "coordY":…,
 *    "color":"red|green|blue|white|black", "size_mm":30|60, "temp_c":…}
 *
 *   {"type":"event",       "robot":"…", "t_ms":…,
 *    "coordX":…, "coordY":…,
 *    "event":"cliff|boundary|obstacle"}
 *
 * JSON messages the robot receives (subscribed from /pynqbridge/{MODULE}/recv):
 *   {"cmd":"start"} / {"cmd":"stop"} / {"cmd":"reset"}
 */

#ifndef COMM_H
#define COMM_H

#include <stdbool.h>
#include <stdint.h>

/* Buffer size for an incoming command string (including NUL terminator). */
#define COMM_CMD_BUF_SIZE 512U

/**
 * Initialize the communication module.
 * Routes AR0→UART0 RX and AR1→UART0 TX through the switchbox, sets AR3 as
 * input (ESP32 ready-to-accept), and opens UART0.
 * Call this after pynq_init().
 */
void comm_init(void);

/**
 * Tear down the communication module.
 * Call this before pynq_destroy().
 */
void comm_destroy(void);

/**
 * Send a status message to the PC.
 * @param status  Human-readable status string, e.g. "ready", "running", "complete".
 */
void comm_send_status(const char *status);

/**
 * Send a periodic telemetry frame with the robot's current state.
 * Call this regularly (e.g. every 1–2 s) from the main loop.
 *
 * @param x        Estimated X position in metres (dead-reckoning / odometry).
 * @param y        Estimated Y position in metres.
 * @param theta    Heading in radians (0 = forward, positive = CCW).
 * @param air_temp Ambient temperature in °C from the temperature sensor.
 * @param moving   true while the stepper motors are running.
 */
void comm_send_telemetry(double x, double y, double theta,
                         float air_temp, bool moving);

/**
 * Send a rock-sample detection event.
 * Call this once per rock after the color sensor and distance sensor have
 * both completed their measurements.
 *
 * @param x        Robot X position (metres) at the moment of detection.
 * @param y        Robot Y position (metres) at the moment of detection.
 * @param color    Colour string: "red", "green", "blue", "white", or "black".
 * @param size_mm  Side length of the cube in mm: 30 (small) or 60 (large).
 * @param temp_c   Temperature measured near the sample in °C.
 */
void comm_send_rock_sample(double x, double y,
                           const char *color, int size_mm, float temp_c);

/**
 * Send a terrain-event notification.
 * Call this whenever the robot detects a hazard or boundary.
 *
 * @param x          Robot X position (metres) when the event was detected.
 * @param y          Robot Y position (metres) when the event was detected.
 * @param event_type One of: "cliff", "boundary", "obstacle".
 */
void comm_send_event(double x, double y, const char *event_type);

/**
 * Non-blocking poll for incoming commands from the PC.
 * Drains up to a fixed number of bytes from the UART FIFO per call;
 * returns true and fills @p cmd_out only when a complete frame is assembled.
 *
 * @param cmd_out  Buffer that receives the NUL-terminated JSON command string.
 * @param max_len  Capacity of @p cmd_out — use COMM_CMD_BUF_SIZE.
 * @returns true if a complete command was received, false otherwise.
 */
bool comm_poll_command(char *cmd_out, uint32_t max_len);

#endif /* COMM_H */
