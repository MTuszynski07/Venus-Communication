/*
Copyright (c) 2023 Eindhoven University of Technology

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup MOTION Motion Control library
 *
 * Motion control library for managing stepper-based movement with interrupt support.
 * Allows sensor applications to stop motion and report position for the main controller
 * to handle correction.
 *
 * @{
 */

/**
 * @brief Motion state enumeration
 */
typedef enum {
    MOTION_IDLE,
    MOTION_FORWARD,
    MOTION_SWEEP,
    MOTION_STOPPED
} motion_state_t;

/**
 * @brief Motion context structure
 */
typedef struct {
    motion_state_t state;
    int16_t total_steps_left;
    int16_t total_steps_right;
    int16_t completed_steps_left;
    int16_t completed_steps_right;
    bool interrupt_requested;
} motion_context_t;

/**
 * @brief Initialize motion module
 */
extern void motion_init(void);

/**
 * @brief Get current motion context
 * @returns pointer to motion context
 */
extern motion_context_t* motion_get_context(void);

/**
 * @brief Request motion interrupt (called by sensor applications)
 * This will gracefully stop the current motion
 */
extern void motion_request_interrupt(void);

/**
 * @brief Check if motion has been interrupted
 * @returns true if motion was interrupted, false otherwise
 */
extern bool motion_was_interrupted(void);

/**
 * @brief Get current steps completed
 * @param left pointer to store left wheel steps
 * @param right pointer to store right wheel steps
 */
extern void motion_get_current_steps(int16_t *left, int16_t *right);

/**
 * @brief Get current motion state
 * @returns current motion state
 */
extern motion_state_t motion_get_state(void);

/**
 * @brief Perform forward motion
 * Forward motion can be interrupted by motion_request_interrupt()
 * @returns 0 if completed normally, 1 if interrupted
 */
extern int motion_forward(int16_t forward_steps, uint16_t left_speed, uint16_t right_speed);

/**
 * @brief Perform sweep motion
 * Sweep motion consists of 4 phases: Left, Right, Right, Left
 * Can be interrupted at any phase by motion_request_interrupt()
 * @returns 0 if completed normally, 1 if interrupted
 */
extern int motion_sweep(int16_t full_turn_steps, uint16_t left_speed, uint16_t right_speed);

/**
 * @brief Correct position by moving relative steps
 * Used by main controller to correct position after sensor interrupt
 * @param left_steps steps to move left wheel
 * @param right_steps steps to move right wheel
 * @returns 0 if completed normally
 */
extern int motion_correct_position(int16_t left_steps, int16_t right_steps, uint16_t left_speed, uint16_t right_speed);

/**
 * @brief Reset motion context
 */
extern void motion_reset(void);

/**
 * @}
 */

#endif // MOTION_H
