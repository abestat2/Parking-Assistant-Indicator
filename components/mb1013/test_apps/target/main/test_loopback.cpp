// Tier 3: on-target tests. An RMT TX channel on the same GPIO as the MB1013 RX channel
// generates exact pulses, so no sensor or jumper wire is needed.

#include <vector>
#include "unity.h"
#include "driver/rmt_tx.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "MB1013.hpp"

using namespace mb1013;

// Avoid strapping pins (2, 8, 9), SPI flash pins (12-17) and the USB pins (18, 19)
static constexpr gpio_num_t TEST_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t TEST_GPIO_2 = GPIO_NUM_5;

// 10 MHz lets the generator produce sub-microsecond glitches
static constexpr uint32_t TX_RESOLUTION_HZ = 10 * 1000 * 1000;
static constexpr uint32_t TX_TICKS_PER_US = TX_RESOLUTION_HZ / 1000000;
static constexpr uint32_t MAX_SYMBOL_TICKS = 32767;

// Tolerance for a measured width, in mm (1 mm = 1 us)
static constexpr int32_t TOLERANCE_MM = 2;

struct Segment {
    uint32_t level;
    uint32_t ticks; // at TX_RESOLUTION_HZ
};

static Segment high_us(uint32_t us) { return { 1, us * TX_TICKS_PER_US }; }
static Segment low_us(uint32_t us) { return { 0, us * TX_TICKS_PER_US }; }

class PulseGenerator {
public:
    explicit PulseGenerator(gpio_num_t gpio) {
        rmt_tx_channel_config_t tx_config = {};
        tx_config.gpio_num = gpio;
        tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
        tx_config.resolution_hz = TX_RESOLUTION_HZ;
        tx_config.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
        tx_config.trans_queue_depth = 4;
        TEST_ESP_OK(rmt_new_tx_channel(&tx_config, &m_channel));

        rmt_copy_encoder_config_t encoder_config = {};
        TEST_ESP_OK(rmt_new_copy_encoder(&encoder_config, &m_encoder));
        TEST_ESP_OK(rmt_enable(m_channel));
    }

    ~PulseGenerator() {
        rmt_disable(m_channel);
        rmt_del_encoder(m_encoder);
        rmt_del_channel(m_channel);
    }

    // Start sending the segments. loop_count 0 sends once, -1 repeats until the generator is destroyed.
    void send(const std::vector<Segment>& segments, int loop_count = 0) {
        m_symbols = to_symbols(segments);
        rmt_transmit_config_t transmit_config = {};
        transmit_config.loop_count = loop_count;
        TEST_ESP_OK(rmt_transmit(m_channel, m_encoder, m_symbols.data(),
                                 m_symbols.size() * sizeof(rmt_symbol_word_t), &transmit_config));
    }

    void wait_done() {
        TEST_ESP_OK(rmt_tx_wait_all_done(m_channel, 1000));
    }

private:
    // Split segments into chunks that fit the 15-bit duration field and pack them two per symbol.
    // A zero duration would end the transmission early, so every chunk is at least one tick.
    static std::vector<rmt_symbol_word_t> to_symbols(const std::vector<Segment>& segments) {
        std::vector<Segment> chunks;
        for (const Segment& segment : segments) {
            uint32_t remaining = segment.ticks;
            while (remaining > 0) {
                uint32_t ticks = remaining > MAX_SYMBOL_TICKS ? MAX_SYMBOL_TICKS : remaining;
                chunks.push_back({ segment.level, ticks });
                remaining -= ticks;
            }
        }
        // Pad to an even count by splitting the last chunk in two
        if (chunks.size() % 2 != 0) {
            Segment& last = chunks.back();
            TEST_ASSERT_GREATER_THAN_UINT32(1, last.ticks);
            uint32_t half = last.ticks / 2;
            chunks.push_back({ last.level, last.ticks - half });
            chunks[chunks.size() - 2].ticks = half;
        }

        std::vector<rmt_symbol_word_t> symbols;
        for (size_t i = 0; i < chunks.size(); i += 2) {
            rmt_symbol_word_t s = {};
            s.level0 = chunks[i].level;
            s.duration0 = chunks[i].ticks;
            s.level1 = chunks[i + 1].level;
            s.duration1 = chunks[i + 1].ticks;
            symbols.push_back(s);
        }
        return symbols;
    }

    rmt_channel_handle_t m_channel = nullptr;
    rmt_encoder_handle_t m_encoder = nullptr;
    std::vector<rmt_symbol_word_t> m_symbols;
};

// Send one pulse with a short low lead-in, so the read below is armed before the rising edge
static int32_t read_single_pulse(MB1013& sensor, PulseGenerator& generator, const std::vector<Segment>& pulse) {
    std::vector<Segment> segments = { low_us(2000) };
    segments.insert(segments.end(), pulse.begin(), pulse.end());
    segments.push_back(low_us(15000));
    generator.send(segments);
    int32_t range = sensor.read_range_mm();
    generator.wait_done();
    return range;
}

TEST_CASE("target: measures pulse widths across the sensor range", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ESP_OK(sensor.init());
    PulseGenerator generator(TEST_GPIO);

    const uint32_t widths_us[] = { 300, 1000, 2500, 5000 };
    for (uint32_t width : widths_us) {
        int32_t range = read_single_pulse(sensor, generator, { high_us(width) });
        TEST_ASSERT_INT32_WITHIN(TOLERANCE_MM, static_cast<int32_t>(width), range);
    }
}

TEST_CASE("target: glitch shorter than 1 us is ignored", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ESP_OK(sensor.init());
    PulseGenerator generator(TEST_GPIO);

    // 0.5 us glitch, 20 us low, then the real 1000 us pulse
    Segment glitch = { 1, TX_TICKS_PER_US / 2 };
    int32_t range = read_single_pulse(sensor, generator, { glitch, low_us(20), high_us(1000) });
    TEST_ASSERT_INT32_WITHIN(TOLERANCE_MM, 1000, range);
}

TEST_CASE("target: no signal times out as unresolved and the next read recovers", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ESP_OK(sensor.init());
    PulseGenerator generator(TEST_GPIO);

    int64_t start_us = esp_timer_get_time();
    TEST_ASSERT_EQUAL_INT32(RANGE_UNRESOLVED, sensor.read_range_mm());
    int32_t elapsed_ms = static_cast<int32_t>((esp_timer_get_time() - start_us) / 1000);
    TEST_ASSERT_INT32_WITHIN(20, 150, elapsed_ms);

    int32_t range = read_single_pulse(sensor, generator, { high_us(1200) });
    TEST_ASSERT_INT32_WITHIN(TOLERANCE_MM, 1200, range);
}

TEST_CASE("target: pulse longer than the receive window is unresolved", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ESP_OK(sensor.init());
    PulseGenerator generator(TEST_GPIO);

    TEST_ASSERT_EQUAL_INT32(RANGE_UNRESOLVED, read_single_pulse(sensor, generator, { high_us(12000) }));
}

TEST_CASE("target: read started mid-pulse reports in progress, then the next pulse resolves", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ESP_OK(sensor.init());
    PulseGenerator generator(TEST_GPIO);

    // 5 ms pulse, 15 ms gap, 1000 us pulse. Arm 2 ms into the first pulse.
    generator.send({ high_us(5000), low_us(15000), high_us(1000), low_us(15000) });
    esp_rom_delay_us(2000);
    TEST_ASSERT_EQUAL_INT32(RANGE_IN_PROGRESS, sensor.read_range_mm());
    // The in-progress result arrives ~10 ms after the falling edge, before the second pulse starts
    TEST_ASSERT_INT32_WITHIN(TOLERANCE_MM, 1000, sensor.read_range_mm());
    generator.wait_done();
}

TEST_CASE("target: 50 consecutive reads of a repeating signal", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ESP_OK(sensor.init());
    PulseGenerator generator(TEST_GPIO);

    // Mimic the sensor: a 1500 us pulse roughly every 100 ms, repeating
    generator.send({ high_us(1500), low_us(95000) }, -1);

    for (int i = 0; i < 50; i++) {
        int32_t range = sensor.read_range_mm();
        // Reads are not synchronized to the signal, so one may land mid-pulse
        if (range == RANGE_IN_PROGRESS) {
            range = sensor.read_range_mm();
        }
        TEST_ASSERT_INT32_WITHIN(TOLERANCE_MM, 1500, range);
    }
}

TEST_CASE("target: init and deinit 100 times without leaking", "[mb1013][target]")
{
    for (int i = 0; i < 100; i++) {
        MB1013 sensor(TEST_GPIO);
        TEST_ESP_OK(sensor.init());
        sensor.deinit();
    }
    // The leak check runs in tearDown()
}

TEST_CASE("target: init twice is rejected and the sensor keeps working", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ESP_OK(sensor.init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.init());

    PulseGenerator generator(TEST_GPIO);
    TEST_ASSERT_INT32_WITHIN(TOLERANCE_MM, 800, read_single_pulse(sensor, generator, { high_us(800) }));
}

TEST_CASE("target: odd RMT memory size fails init cleanly", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO, SOC_RMT_MEM_WORDS_PER_CHANNEL + 1);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sensor.init());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm());
}

TEST_CASE("target: two sensors fit when each uses one memory block", "[mb1013][target]")
{
    MB1013 sensor_a(TEST_GPIO, SOC_RMT_MEM_WORDS_PER_CHANNEL);
    MB1013 sensor_b(TEST_GPIO_2, SOC_RMT_MEM_WORDS_PER_CHANNEL);
    TEST_ESP_OK(sensor_a.init());
    TEST_ESP_OK(sensor_b.init());
}

TEST_CASE("target: read before init and after deinit reports an error", "[mb1013][target]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm());
    TEST_ESP_OK(sensor.init());
    sensor.deinit();
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm());
}
