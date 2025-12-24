#include "peripherals/StatusLed.h"

#include "hardware/gpio.h"
#include "pio_ws2812/ws2812.h"

#include <algorithm>
#include <cmath>

namespace Doncon::Peripherals {

StatusLed::StatusLed(const Config &config) : m_config(config) {
    gpio_init(m_config.led_enable_pin);
    gpio_set_dir(m_config.led_enable_pin, (bool)GPIO_OUT);
    gpio_put(m_config.led_enable_pin, true);

    ws2812_init(pio0, config.led_pin, m_config.is_rgbw);
    m_leds.resize(m_config.led_count, 0);
}

void StatusLed::setBrightness(const uint8_t brightness) { m_config.brightness = brightness; }
void StatusLed::setEnablePlayerColor(const bool do_enable) { m_config.enable_player_color = do_enable; }

void StatusLed::setInputState(const Utils::InputState &input_state) { m_input_state = input_state; }
void StatusLed::setPlayerColor(const Config::Color &color) { m_player_color = color; }

void StatusLed::update() {
    const float brightness_factor = (float)m_config.brightness / (float)UINT8_MAX;

    // Decay intensities
    m_left_intensity *= 0.97f;
    m_right_intensity *= 0.97f;

    if (m_left_intensity < 0.01f) m_left_intensity = 0.0f;
    if (m_right_intensity < 0.01f) m_right_intensity = 0.0f;

    // Check triggers
    if (m_input_state.drum.don_left.triggered) {
        m_left_intensity = 1.0f;
        m_left_color = m_config.don_left_color;
    } else if (m_input_state.drum.ka_left.triggered) {
        m_left_intensity = 1.0f;
        m_left_color = m_config.ka_left_color;
    }

    if (m_input_state.drum.don_right.triggered) {
        m_right_intensity = 1.0f;
        m_right_color = m_config.don_right_color;
    } else if (m_input_state.drum.ka_right.triggered) {
        m_right_intensity = 1.0f;
        m_right_color = m_config.ka_right_color;
    }

    // Determine Idle Color
    Config::Color idle = m_config.enable_player_color ? m_player_color.value_or(m_config.idle_color) : m_config.idle_color;

    std::vector<Config::Color> frame_colors(m_config.led_count);
    size_t center = m_config.led_count / 2;

    for (size_t i = 0; i < m_config.led_count; ++i) {
        float intensity = 0.0f;
        Config::Color hit_color = {};

        if (m_config.led_count == 1) {
             if (m_left_intensity > m_right_intensity) {
                 intensity = m_left_intensity;
                 hit_color = m_left_color;
             } else {
                 intensity = m_right_intensity;
                 hit_color = m_right_color;
             }
        } else {
             if (i < center) {
                intensity = m_left_intensity;
                hit_color = m_left_color;
             } else {
                intensity = m_right_intensity;
                hit_color = m_right_color;
             }
        }

        // Blend: Hit Color * Intensity + Idle * (1 - Intensity)
        float inv_intensity = 1.0f - intensity;
        
        float r = (float)hit_color.r * intensity + (float)idle.r * inv_intensity;
        float g = (float)hit_color.g * intensity + (float)idle.g * inv_intensity;
        float b = (float)hit_color.b * intensity + (float)idle.b * inv_intensity;

        // Apply global brightness
        frame_colors[i].r = static_cast<uint8_t>(std::min(255.0f, r * brightness_factor));
        frame_colors[i].g = static_cast<uint8_t>(std::min(255.0f, g * brightness_factor));
        frame_colors[i].b = static_cast<uint8_t>(std::min(255.0f, b * brightness_factor));
    }

    // Current Limiting
    uint32_t total_ma = 0;
    // Estimated: 60mA per LED at full white (255, 255, 255)
    // Formula: sum((r+g+b) / 765 * 60)
    for (const auto &c : frame_colors) {
        total_ma += (c.r + c.g + c.b) * 60 / 765; 
    }

    float scale = 1.0f;
    if (total_ma > m_config.max_current_ma && total_ma > 0) {
        scale = (float)m_config.max_current_ma / (float)total_ma;
    }

    // Convert to WS2812 pixel format
    for (size_t i = 0; i < m_config.led_count; ++i) {
        uint8_t r = static_cast<uint8_t>((float)frame_colors[i].r * scale);
        uint8_t g = static_cast<uint8_t>((float)frame_colors[i].g * scale);
        uint8_t b = static_cast<uint8_t>((float)frame_colors[i].b * scale);

        size_t index = m_config.reversed ? (m_config.led_count - 1 - i) : i;
        m_leds[index] = ws2812_rgb_to_gamma_corrected_u32pixel(r, g, b);
    }
    
    ws2812_put_frame(pio0, m_leds.data(), m_leds.size());
}

} // namespace Doncon::Peripherals
