#include "rcp_led_status.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ws2812.h"

#define LED_BRIGHTNESS 32

static void led_init_once(void)
{
    static bool inited = false;
    if (!inited) {
        ws2812_init();
        ws2812_set_brightness(LED_BRIGHTNESS);
        inited = true;
    }
}

void rcp_led_boot(void)
{
    led_init_once();
    ws2812_set(255, 0, 0);   // 🔴 boot
}

void rcp_led_ready(void)
{
    led_init_once();
    ws2812_set(0, 255, 0);   // 🟢 ready
}
