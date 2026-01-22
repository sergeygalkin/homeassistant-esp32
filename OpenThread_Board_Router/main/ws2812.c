#include "ws2812.h"

#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_rom_sys.h"  // esp_rom_delay_us

#define WS2812_GPIO 27

// 10MHz => 0.1us/tick
#define RMT_RES_HZ    (10 * 1000 * 1000)

#define T0H_TICKS     4
#define T0L_TICKS     8
#define T1H_TICKS     7
#define T1L_TICKS     6

#define RESET_US      80

static rmt_channel_handle_t s_tx_chan;
static rmt_encoder_handle_t s_copy_enc;
static uint8_t s_brightness = 255; // 100% по умолчанию
static bool s_inited = false;

static inline rmt_symbol_word_t sym0(void)
{
    return (rmt_symbol_word_t){
        .level0 = 1, .duration0 = T0H_TICKS,
        .level1 = 0, .duration1 = T0L_TICKS,
    };
}

static inline rmt_symbol_word_t sym1(void)
{
    return (rmt_symbol_word_t){
        .level0 = 1, .duration0 = T1H_TICKS,
        .level1 = 0, .duration1 = T1L_TICKS,
    };
}

static void encode_byte(rmt_symbol_word_t *out, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        out[7 - i] = ((byte >> i) & 1) ? sym1() : sym0();
    }
}

void ws2812_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_tx_chan));

    rmt_copy_encoder_config_t enc_cfg;
    memset(&enc_cfg, 0, sizeof(enc_cfg));
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &s_copy_enc));

    ESP_ERROR_CHECK(rmt_enable(s_tx_chan));

    s_inited = true;
}

static inline uint8_t scale(uint8_t v)
{
    // линейное масштабирование
    return (uint8_t)((v * (uint16_t)s_brightness) / 255);
}

void ws2812_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_inited) {
        ws2812_init();
    }
    // WS2812: GRB
    const uint8_t grb[3] = { scale(g), scale(r), scale(b) };

    rmt_symbol_word_t symbols[24];
    encode_byte(&symbols[0],  grb[0]);
    encode_byte(&symbols[8],  grb[1]);
    encode_byte(&symbols[16], grb[2]);

    rmt_transmit_config_t tx_conf = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(rmt_transmit(s_tx_chan, s_copy_enc, symbols, sizeof(symbols), &tx_conf));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_tx_chan, pdMS_TO_TICKS(50)));

    esp_rom_delay_us(RESET_US);
}

void ws2812_set_brightness(uint8_t brightness)
{
    s_brightness = brightness;
}
