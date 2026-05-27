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
#include "motion.h"
#include "stepper.h"
#include <unistd.h>

static motion_context_t motion_ctx = {
    .state = MOTION_IDLE,
    .total_steps_left = 0,
    .total_steps_right = 0,
    .completed_steps_left = 0,
    .completed_steps_right = 0,
    .interrupt_requested = false
};

void motion_init(void) {
    motion_reset();
}

motion_context_t* motion_get_context(void) {
    return &motion_ctx;
}

void motion_request_interrupt(void) {
    motion_ctx.interrupt_requested = true;
}

bool motion_was_interrupted(void) {
    return motion_ctx.interrupt_requested;
}

void motion_get_current_steps(int16_t *left, int16_t *right) {
    if (left) *left = motion_ctx.completed_steps_left;
    if (right) *right = motion_ctx.completed_steps_right;
}

motion_state_t motion_get_state(void) {
    return motion_ctx.state;
}

void motion_reset(void) {
    motion_ctx.state = MOTION_IDLE;
    motion_ctx.total_steps_left = 0;
    motion_ctx.total_steps_right = 0;
    motion_ctx.completed_steps_left = 0;
    motion_ctx.completed_steps_right = 0;
    motion_ctx.interrupt_requested = false;
}

int motion_forward(int16_t forward_steps, uint16_t left_speed, uint16_t right_speed) {
    motion_reset();
    motion_ctx.state = MOTION_FORWARD;
    motion_ctx.total_steps_left = forward_steps;
    motion_ctx.total_steps_right = forward_steps;
    motion_ctx.completed_steps_left = 0;
    motion_ctx.completed_steps_right = 0;

    stepper_set_speed(left_speed, right_speed);
    stepper_steps(forward_steps, forward_steps);

    while (!stepper_steps_done()) {
        if (motion_ctx.interrupt_requested) {
            stepper_disable();
            motion_ctx.state = MOTION_STOPPED;
            motion_ctx.completed_steps_left = forward_steps;
            motion_ctx.completed_steps_right = forward_steps;
            stepper_enable();
            return 1;
        }
        sleep(1);
    }

    motion_ctx.completed_steps_left = forward_steps;
    motion_ctx.completed_steps_right = forward_steps;
    motion_ctx.state = MOTION_IDLE;
    return 0;
}

int motion_sweep(int16_t full_turn_steps, uint16_t left_speed, uint16_t right_speed) {
    motion_reset();
    motion_ctx.state = MOTION_SWEEP;
    motion_ctx.total_steps_left = full_turn_steps * 4;
    motion_ctx.total_steps_right = full_turn_steps * 4;
    motion_ctx.completed_steps_left = 0;
    motion_ctx.completed_steps_right = 0;

    stepper_set_speed(left_speed, right_speed);

    // Phase 1: Left sweep
    stepper_steps(-full_turn_steps, full_turn_steps);
    while (!stepper_steps_done()) {
        if (motion_ctx.interrupt_requested) {
            stepper_disable();
            motion_ctx.state = MOTION_STOPPED;
            stepper_enable();
            return 1;
        }
        sleep(1);
    }
    motion_ctx.completed_steps_left += full_turn_steps;
    motion_ctx.completed_steps_right += full_turn_steps;

    // Phase 2: Right sweep
    stepper_steps(full_turn_steps, -full_turn_steps);
    while (!stepper_steps_done()) {
        if (motion_ctx.interrupt_requested) {
            stepper_disable();
            motion_ctx.state = MOTION_STOPPED;
            stepper_enable();
            return 1;
        }
        sleep(1);
    }
    motion_ctx.completed_steps_left += full_turn_steps;
    motion_ctx.completed_steps_right += full_turn_steps;

    // Phase 3: Right sweep (continued)
    stepper_steps(full_turn_steps, -full_turn_steps);
    while (!stepper_steps_done()) {
        if (motion_ctx.interrupt_requested) {
            stepper_disable();
            motion_ctx.state = MOTION_STOPPED;
            stepper_enable();
            return 1;
        }
        sleep(1);
    }
    motion_ctx.completed_steps_left += full_turn_steps;
    motion_ctx.completed_steps_right += full_turn_steps;

    // Phase 4: Left sweep (final)
    stepper_steps(-full_turn_steps, full_turn_steps);
    while (!stepper_steps_done()) {
        if (motion_ctx.interrupt_requested) {
            stepper_disable();
            motion_ctx.state = MOTION_STOPPED;
            stepper_enable();
            return 1;
        }
        sleep(1);
    }
    motion_ctx.completed_steps_left += full_turn_steps;
    motion_ctx.completed_steps_right += full_turn_steps;

    motion_ctx.state = MOTION_IDLE;
    return 0;
}

int motion_correct_position(int16_t left_steps, int16_t right_steps, uint16_t left_speed, uint16_t right_speed) {
    motion_ctx.state = MOTION_IDLE;
    motion_ctx.interrupt_requested = false;

    stepper_set_speed(left_speed, right_speed);
    stepper_steps(left_steps, right_steps);

    while (!stepper_steps_done()) {
        sleep(1);
    }

    motion_ctx.state = MOTION_IDLE;
    return 0;
}
