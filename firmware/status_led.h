/*
 * Status LED driver for the WS2812 "NeoPixel" on the RP2040-Zero (GP16).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pico/types.h"

typedef enum {
    HEALTH_BOOT,        // Solid blue: initialising.
    HEALTH_OK,          // Green heartbeat blip, 1 Hz.
    HEALTH_DEGRADED,    // Yellow double-blip: recent I2C errors or stale/implausible data.
    HEALTH_NO_SENSOR,   // Fast red blink: LSM303 not detected / not responding.
} health_t;

void status_led_init(uint pin);

/// Set the LED colour directly (0-255 per channel, before brightness scaling).
void status_led_set_rgb(uint8_t r, uint8_t g, uint8_t b);

/// Render the blink pattern for `health` at time `now_ms`. Call often (>= ~50 Hz).
/// If `wdt_flag` is set, a magenta blip is added mid-cycle to show the last reset was
/// caused by the watchdog.
void status_led_update(health_t health, bool wdt_flag, uint32_t now_ms);

const char *health_name(health_t health);
