#include "status_led.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "ws2812.pio.h"

// The LED is very bright and is right next to your eyes on the bench; scale everything down.
static const uint8_t BRIGHTNESS_PCT = 15;

static PIO s_pio = NULL;
static uint s_sm = 0;
static uint32_t s_last_pixel = 0xFFFFFFFFu;  // Force first write.

void status_led_init(uint pin) {
    uint offset;
    // Claims a free PIO + state machine and loads the program.
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &s_pio, &s_sm, &offset, pin, 1, true)) {
        s_pio = NULL;  // LED is cosmetic; carry on without it.
        return;
    }
    ws2812_program_init(s_pio, s_sm, offset, pin, 800000.0f);
    status_led_set_rgb(0, 0, 0);
}

void status_led_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    if (s_pio == NULL) return;

    r = (uint8_t)((r * BRIGHTNESS_PCT) / 100u);
    g = (uint8_t)((g * BRIGHTNESS_PCT) / 100u);
    b = (uint8_t)((b * BRIGHTNESS_PCT) / 100u);

#if STATUS_LED_GRB_ORDER
    const uint32_t pixel = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
#else
    const uint32_t pixel = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
#endif
    if (pixel == s_last_pixel) return;
    s_last_pixel = pixel;

    // TX FIFO is 8 deep (joined) and we only push on change, so this never blocks in practice.
    // Use the non-blocking check anyway so a wedged PIO can't stall the main loop.
    if (!pio_sm_is_tx_fifo_full(s_pio, s_sm)) {
        pio_sm_put(s_pio, s_sm, pixel << 8u);
    } else {
        s_last_pixel = 0xFFFFFFFFu;  // Retry next update.
    }
}

void status_led_update(health_t health, bool wdt_flag, uint32_t now_ms) {
    const uint32_t phase = now_ms % 1000u;

    // Watchdog marker: short magenta blip in the middle of each cycle.
    if (wdt_flag && phase >= 500u && phase < 560u) {
        status_led_set_rgb(255, 0, 255);
        return;
    }

    switch (health) {
        case HEALTH_BOOT:
            status_led_set_rgb(0, 0, 255);
            break;
        case HEALTH_OK:
            if (phase < 60u) status_led_set_rgb(0, 255, 0);
            else status_led_set_rgb(0, 0, 0);
            break;
        case HEALTH_DEGRADED:
            if (phase < 60u || (phase >= 200u && phase < 260u)) status_led_set_rgb(255, 160, 0);
            else status_led_set_rgb(0, 0, 0);
            break;
        case HEALTH_NO_SENSOR:
        default:
            if ((now_ms % 250u) < 125u) status_led_set_rgb(255, 0, 0);
            else status_led_set_rgb(0, 0, 0);
            break;
    }
}

const char *health_name(health_t health) {
    switch (health) {
        case HEALTH_BOOT:      return "boot";
        case HEALTH_OK:        return "ok";
        case HEALTH_DEGRADED:  return "degraded";
        case HEALTH_NO_SENSOR: return "no_sensor";
    }
    return "unknown";
}
