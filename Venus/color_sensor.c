#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <libpynq.h>

/* ==========================================================================
 * 1. MATHEMATICAL LOGIC (Color Recognition)
 * ========================================================================== */

typedef struct {
    float fRmin, fRmax;
    float fGmin, fGmax;
    float fBmin, fBmax;
} ColorCalibration;

typedef struct {
    float r, g, b;
} ColorVector;

/**
 * Calculates intensity based on measured frequency and calibration.
 */
ColorVector get_intensities(ColorCalibration* cal, float fR, float fG, float fB) {
    ColorVector v;
    v.r = (cal->fRmax != cal->fRmin) ? (fR - cal->fRmin) / (cal->fRmax - cal->fRmin) : 0.0f;
    v.g = (cal->fGmax != cal->fGmin) ? (fG - cal->fGmin) / (cal->fGmax - cal->fGmin) : 0.0f;
    v.b = (cal->fBmax != cal->fBmin) ? (fB - cal->fBmin) / (cal->fBmax - cal->fBmin) : 0.0f;
    
    // Clamp to 0.0 - 1.0
    v.r = fmaxf(0.0f, fminf(1.0f, v.r));
    v.g = fmaxf(0.0f, fminf(1.0f, v.g));
    v.b = fmaxf(0.0f, fminf(1.0f, v.b));
    return v;
}

/**
 * Converts RGB (0.0 to 1.0) to HSL.
 * H is 0-360, S and L are 0.0-1.0.
 */
void rgb_to_hsl(float r, float g, float b, float *h, float *s, float *l) {
    float max = fmaxf(r, fmaxf(g, b));
    float min = fminf(r, fminf(g, b));
    float delta = max - min;

    *l = (max + min) / 2.0f;

    if (delta == 0) {
        *h = 0;
        *s = 0;
    } else {
        *s = (*l > 0.5f) ? delta / (2.0f - max - min) : delta / (max + min);
        if (max == r) {
            *h = (g - b) / delta + (g < b ? 6.0f : 0.0f);
        } else if (max == g) {
            *h = (b - r) / delta + 2.0f;
        } else {
            *h = (r - g) / delta + 4.0f;
        }
        *h *= 60.0f; // Convert to degrees
    }
}

/**
 * Finds the nearest color using HSL logic.
 * Much more sensitive to actual color than RGB distance.
 */
const char* get_nearest_color(ColorCalibration* cal, float fR, float fG, float fB) {
    ColorVector rgb = get_intensities(cal, fR, fG, fB);
    float h, s, l;
    rgb_to_hsl(rgb.r, rgb.g, rgb.b, &h, &s, &l);

    // 1. Detect Black and White first using Lightness and Saturation
    // Decreased black threshold (0.25 -> 0.15) to be more sensitive to dark colors
    if (l < 0.15f) return "Black";
    if (l > 0.75f && s < 0.30f) return "White";

    // 2. Detect Colors using Hue ranges
    if (h >= 330.0f || h < 20.0f) return "Red";
    if (h >= 80.0f  && h < 160.0f) return "Green";
    if (h >= 190.0f && h < 270.0f) return "Blue";

    // 3. Fallback for ambiguous colors
    if (s < 0.20f) return (l < 0.5f) ? "Black" : "White";
    
    return "Unknown";
}

/* ==========================================================================
 * 2. HARDWARE CONTROL (TCS3200 Sensor)
 * ========================================================================== */

typedef struct {
    int s0, s1, s2, s3, out;
} TCS3200_Pins;

void tcs3200_init(TCS3200_Pins pins) {
    gpio_set_direction(pins.s0, GPIO_DIR_OUTPUT);
    gpio_set_direction(pins.s1, GPIO_DIR_OUTPUT);
    gpio_set_direction(pins.s2, GPIO_DIR_OUTPUT);
    gpio_set_direction(pins.s3, GPIO_DIR_OUTPUT);
    gpio_set_direction(pins.out, GPIO_DIR_INPUT);
    
    // Scaling to 20% (S0=HIGH, S1=LOW)
    gpio_set_level(pins.s0, GPIO_LEVEL_HIGH);
    gpio_set_level(pins.s1, GPIO_LEVEL_LOW);
}

void tcs3200_set_filter(TCS3200_Pins pins, const char* color) {
    if (strcmp(color, "red") == 0) {
        gpio_set_level(pins.s2, GPIO_LEVEL_LOW);
        gpio_set_level(pins.s3, GPIO_LEVEL_LOW);
    } else if (strcmp(color, "blue") == 0) {
        gpio_set_level(pins.s2, GPIO_LEVEL_LOW);
        gpio_set_level(pins.s3, GPIO_LEVEL_HIGH);
    } else if (strcmp(color, "clear") == 0) {
        gpio_set_level(pins.s2, GPIO_LEVEL_HIGH);
        gpio_set_level(pins.s3, GPIO_LEVEL_LOW);
    } else if (strcmp(color, "green") == 0) {
        gpio_set_level(pins.s2, GPIO_LEVEL_HIGH);
        gpio_set_level(pins.s3, GPIO_LEVEL_HIGH);
    }
}

float tcs3200_measure_frequency(TCS3200_Pins pins, int sample_ms) {
    int pulse_count = 0;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    double elapsed_ms = 0;
    int last_level = gpio_get_level(pins.out);
    
    while (elapsed_ms < sample_ms) {
        int current_level = gpio_get_level(pins.out);
        if (last_level == GPIO_LEVEL_LOW && current_level == GPIO_LEVEL_HIGH) {
            pulse_count++;
        }
        last_level = current_level;
        
        clock_gettime(CLOCK_MONOTONIC, &now);
        elapsed_ms = (now.tv_sec - start.tv_sec) * 1000.0 + (now.tv_nsec - start.tv_nsec) / 1000000.0;
    }
    return (float)pulse_count / (sample_ms / 1000.0f);
}

void tcs3200_read_rgb(TCS3200_Pins pins, float* fR, float* fG, float* fB) {
    // Increase settling time to 50ms and sample time to 200ms for more precision
    tcs3200_set_filter(pins, "red");
    sleep_msec(50);
    *fR = tcs3200_measure_frequency(pins, 200);
    
    tcs3200_set_filter(pins, "green");
    sleep_msec(50);
    *fG = tcs3200_measure_frequency(pins, 200);
    
    tcs3200_set_filter(pins, "blue");
    sleep_msec(50);
    *fB = tcs3200_measure_frequency(pins, 200);
}

/* ==========================================================================
 * 3. MAIN APPLICATION
 * ========================================================================== */

int main(void) {
    pynq_init();
    
    // Update these to match your wiring!
    TCS3200_Pins pins = {
        .s0 = IO_A0,
        .s1 = IO_A1,
        .s2 = IO_A2,
        .s3 = IO_A3,
        .out = IO_A4
    };
    
    tcs3200_init(pins);
    ColorCalibration cal = {0};
    
    printf("\n--- TCS3200 Color Recognition (C Version) ---\n");
    
    // Calibration Phase
    printf("1. Place a BLACK object and press Enter...");
    while (getchar() != '\n'); // Wait for enter and clear buffer
    tcs3200_read_rgb(pins, &cal.fRmin, &cal.fGmin, &cal.fBmin);
    printf("   Done. R=%.1f, G=%.1f, B=%.1f\n", cal.fRmin, cal.fGmin, cal.fBmin);
    
    printf("2. Place a WHITE object and press Enter...");
    while (getchar() != '\n'); // Wait for enter and clear buffer
    tcs3200_read_rgb(pins, &cal.fRmax, &cal.fGmax, &cal.fBmax);
    printf("   Done. R=%.1f, G=%.1f, B=%.1f\n", cal.fRmax, cal.fGmax, cal.fBmax);
    
    // Measurement Loop
    printf("\n--- Starting Measurements (Ctrl+C to stop) ---\n");
    while (1) {
        float fR, fG, fB;
        tcs3200_read_rgb(pins, &fR, &fG, &fB);
        
        const char* color = get_nearest_color(&cal, fR, fG, fB);
        ColorVector intensities = get_intensities(&cal, fR, fG, fB);
        
        printf("DETECTED: %-10s [R:%.2f G:%.2f B:%.2f]\r", 
               color, intensities.r, intensities.g, intensities.b);
        fflush(stdout);
        
        sleep_msec(500);
    }
    
    pynq_destroy();
    return 0;
}
