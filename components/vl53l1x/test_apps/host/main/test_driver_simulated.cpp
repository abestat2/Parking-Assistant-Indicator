// Tier 2: driver logic, including the unmodified ST ULD, against a simulated sensor.
// SimulatedSensor implements I2cDevice with a register map and the few behaviours the ULD relies
// on (boot status, data-ready interrupt, ranging start/stop, reset). XSHUT goes through the GPIO
// mock, and time is simulated through the FreeRTOS mock.

#include <array>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>
#include "unity.h"
#include "VL53L1X.hpp"

extern "C" {
#include "Mockgpio.h"
#include "Mocktask.h"
}

using namespace vl53l1x;

static_assert(!std::is_copy_constructible_v<VL53L1X>, "VL53L1X must not be copyable");
static_assert(!std::is_copy_assignable_v<VL53L1X>, "VL53L1X must not be copy-assignable");

static constexpr gpio_num_t XSHUT_GPIO = GPIO_NUM_4;

// Registers used by the ULD (see VL53L1X_api.h)
static constexpr uint16_t REG_SOFT_RESET = 0x0000;
static constexpr uint16_t REG_VHV_LOOP_BOUND = 0x0008;
static constexpr uint16_t REG_VHV_INIT = 0x000B;
static constexpr uint16_t REG_GPIO_HV_MUX_CTRL = 0x0030;
static constexpr uint16_t REG_GPIO_TIO_HV_STATUS = 0x0031;
static constexpr uint16_t REG_PHASECAL_TIMEOUT = 0x004B;
static constexpr uint16_t REG_TIMEOUT_MACROP_A_HI = 0x005E;
static constexpr uint16_t REG_INTERMEASUREMENT_PERIOD = 0x006C;
static constexpr uint16_t REG_ROI_XY_SIZE = 0x0080;
static constexpr uint16_t REG_INTERRUPT_CLEAR = 0x0086;
static constexpr uint16_t REG_MODE_START = 0x0087;
static constexpr uint16_t REG_RANGE_STATUS = 0x0089;
static constexpr uint16_t REG_RANGE_MM = 0x0096;
static constexpr uint16_t REG_OSC_CALIBRATE_VAL = 0x00DE;
static constexpr uint16_t REG_FIRMWARE_SYSTEM_STATUS = 0x00E5;
static constexpr uint16_t REG_MODEL_ID = 0x010F;

// First register of the ULD default configuration; a write here means a full initialization
static constexpr uint16_t REG_DEFAULT_CONFIGURATION = 0x002D;

// Raw RESULT__RANGE_STATUS values and the ULD statuses they map to
static constexpr uint8_t RAW_VALID = 9;          // -> 0
static constexpr uint8_t RAW_SIGMA_FAIL = 6;     // -> 1
static constexpr uint8_t RAW_SIGNAL_FAIL = 4;    // -> 2
static constexpr uint8_t RAW_OUT_OF_BOUNDS = 5;  // -> 4

static constexpr int NEVER = 1000000;

struct Write {
    uint16_t index;
    std::vector<uint8_t> data;
};

// A powered sensor, as on a breakout whose XSHUT pull-up holds it on until the host drives XSHUT
class SimulatedSensor : public I2cDevice {
public:
    SimulatedSensor() { power_on(); }

    // Behaviour knobs, applied from the next power-up or measurement
    int boot_polls = 0;          // status reads that report "not booted" after power-up
    int polls_until_ready = 0;   // data-ready reads that report "not ready" for each measurement
    uint16_t model_id = MODEL_ID;
    bool fail_all = false;
    int fail_writes_to = -1;     // register whose writes are not acknowledged
    int fail_write_count = -1;   // how many of those writes fail; -1 for all
    int fail_reads_from = -1;    // register whose reads (starting there) are not acknowledged
    int fail_read_count = -1;    // how many of those reads fail; -1 for all

    bool powered = false;
    bool ranging = false;
    int power_ups = 0;
    std::vector<Write> writes;
    std::array<uint8_t, 0x10000> regs = {};

    // Power-up or reset: registers return to their reset values and the sensor boots again
    void power_on() {
        powered = true;
        power_ups++;
        ranging = false;
        m_ready = false;
        m_boot_polls_left = boot_polls;
        regs.fill(0);
        regs[REG_MODEL_ID] = model_id >> 8;
        regs[REG_MODEL_ID + 1] = model_id & 0xFF;
        // Oscillator calibration read by SetInterMeasurementInMs()
        regs[REG_OSC_CALIBRATE_VAL] = 0x01;
        regs[REG_OSC_CALIBRATE_VAL + 1] = 0xF4;
        // Reset value: active-high interrupt
        regs[REG_GPIO_HV_MUX_CTRL] = 0x01;
        // Reset values of the VHV registers. They differ from the values the ULD's
        // initialization leaves; the on-target tests check this on a real sensor.
        regs[REG_VHV_LOOP_BOUND] = 0x81;
        regs[REG_VHV_INIT] = 0x92;
    }

    void power_off() {
        powered = false;
        ranging = false;
    }

    void set_result(uint8_t raw_status, uint16_t distance_mm) {
        regs[REG_RANGE_STATUS] = raw_status;
        regs[REG_RANGE_MM] = distance_mm >> 8;
        regs[REG_RANGE_MM + 1] = distance_mm & 0xFF;
    }

    uint16_t reg16(uint16_t index) const { return static_cast<uint16_t>(regs[index] << 8 | regs[index + 1]); }
    uint32_t reg32(uint16_t index) const {
        return static_cast<uint32_t>(regs[index]) << 24 | static_cast<uint32_t>(regs[index + 1]) << 16 |
               static_cast<uint32_t>(regs[index + 2]) << 8 | regs[index + 3];
    }

    bool data_ready() const { return m_ready; }

    bool wrote(uint16_t index) const {
        for (const Write& w : writes) {
            if (w.index == index) {
                return true;
            }
        }
        return false;
    }

    esp_err_t write(const uint8_t* data, size_t size) override {
        if (!powered || fail_all || size < 2) {
            return ESP_FAIL;
        }
        uint16_t index = static_cast<uint16_t>(data[0] << 8 | data[1]);
        if (index == fail_writes_to && fail_write_count != 0) {
            if (fail_write_count > 0) {
                fail_write_count--;
            }
            return ESP_FAIL;
        }
        writes.push_back({ index, std::vector<uint8_t>(data + 2, data + size) });
        for (size_t i = 2; i < size; i++) {
            write_register(static_cast<uint16_t>(index + i - 2), data[i]);
        }
        return ESP_OK;
    }

    esp_err_t write_read(const uint8_t* write_data, size_t write_size, uint8_t* read_data, size_t read_size) override {
        if (!powered || fail_all || write_size != 2) {
            return ESP_FAIL;
        }
        uint16_t index = static_cast<uint16_t>(write_data[0] << 8 | write_data[1]);
        if (index == fail_reads_from && fail_read_count != 0) {
            if (fail_read_count > 0) {
                fail_read_count--;
            }
            return ESP_FAIL;
        }
        for (size_t i = 0; i < read_size; i++) {
            read_data[i] = read_register(static_cast<uint16_t>(index + i));
        }
        return ESP_OK;
    }

private:
    void start_measurement() {
        m_ready = false;
        m_polls_left = polls_until_ready;
    }

    void write_register(uint16_t index, uint8_t value) {
        regs[index] = value;
        if (index == REG_SOFT_RESET) {
            // Writing 0 holds the sensor in reset; writing 1 releases it to boot again
            if (value & 0x01) {
                power_on();
            } else {
                ranging = false;
                m_ready = false;
            }
        } else if (index == REG_MODE_START) {
            ranging = value == 0x40;
            if (ranging) {
                start_measurement();
            } else {
                m_ready = false;
            }
        } else if (index == REG_INTERRUPT_CLEAR && (value & 0x01)) {
            m_ready = false;
            if (ranging) {
                start_measurement();
            }
        }
    }

    uint8_t read_register(uint16_t index) {
        if (index == REG_FIRMWARE_SYSTEM_STATUS) {
            if (m_boot_polls_left > 0) {
                m_boot_polls_left--;
                return 0;
            }
            return 1;
        }
        if (index == REG_GPIO_TIO_HV_STATUS) {
            if (ranging && !m_ready) {
                if (m_polls_left > 0) {
                    m_polls_left--;
                } else {
                    m_ready = true;
                }
            }
            // Bit 4 of GPIO_HV_MUX__CTRL set means active-low
            uint8_t active_level = (regs[REG_GPIO_HV_MUX_CTRL] & 0x10) ? 0 : 1;
            uint8_t level = m_ready ? active_level : !active_level;
            return static_cast<uint8_t>((regs[index] & ~0x01) | level);
        }
        return regs[index];
    }

    int m_boot_polls_left = 0;
    int m_polls_left = 0;
    bool m_ready = false;
};

// State shared with the mock callbacks
static SimulatedSensor* s_sensor;
static TickType_t s_ticks;
static int s_delay_calls;
static std::vector<int> s_xshut_levels;
static gpio_config_t s_gpio_config;
static int s_gpio_config_calls;
static esp_err_t s_gpio_config_result;
static esp_err_t s_hold_dis_result;
// XSHUT pin operations in order, to check that the pin never glitches low
static std::vector<std::string> s_gpio_events;

static esp_err_t stub_gpio_set_level(gpio_num_t gpio, uint32_t level, int calls)
{
    TEST_ASSERT_EQUAL(XSHUT_GPIO, gpio);
    s_xshut_levels.push_back(static_cast<int>(level));
    s_gpio_events.push_back(level ? "level 1" : "level 0");
    // XSHUT low powers the sensor down; raising it again boots it from reset
    if (!level) {
        s_sensor->power_off();
    } else if (!s_sensor->powered) {
        s_sensor->power_on();
    }
    return ESP_OK;
}

static esp_err_t stub_gpio_config(const gpio_config_t* config, int calls)
{
    s_gpio_config = *config;
    s_gpio_config_calls++;
    s_gpio_events.push_back("output");
    return s_gpio_config_result;
}

static esp_err_t stub_gpio_hold_dis(gpio_num_t gpio, int calls)
{
    TEST_ASSERT_EQUAL(XSHUT_GPIO, gpio);
    s_gpio_events.push_back("hold released");
    return s_hold_dis_result;
}

static TickType_t stub_tick_count(int calls)
{
    return s_ticks;
}

static void stub_task_delay(const TickType_t ticks, int calls)
{
    TEST_ASSERT_GREATER_THAN_UINT32(0, ticks);
    s_ticks += ticks;
    s_delay_calls++;
}

extern "C" void setUp(void)
{
    Mockgpio_Init();
    Mocktask_Init();
    s_sensor = nullptr;
    s_ticks = 0;
    s_delay_calls = 0;
    s_xshut_levels.clear();
    s_gpio_config = {};
    s_gpio_config_calls = 0;
    s_gpio_config_result = ESP_OK;
    s_hold_dis_result = ESP_OK;
    s_gpio_events.clear();
    xTaskGetTickCount_Stub(stub_tick_count);
    vTaskDelay_Stub(stub_task_delay);
}

extern "C" void tearDown(void)
{
    Mockgpio_Verify();
    Mocktask_Verify();
    Mockgpio_Destroy();
    Mocktask_Destroy();
}

// Drive the simulated sensor's XSHUT through the GPIO mock. init() releases any deep-sleep hold.
static void attach_xshut(SimulatedSensor& sensor)
{
    s_sensor = &sensor;
    gpio_set_level_Stub(stub_gpio_set_level);
    gpio_config_Stub(stub_gpio_config);
    gpio_hold_dis_Stub(stub_gpio_hold_dis);
}

static Config test_config()
{
    Config config;
    config.distance_mode = DistanceMode::Short;
    config.timing_budget_ms = 33;
    config.inter_measurement_ms = 100;
    config.roi_width = 8;
    config.roi_height = 6;
    return config;
}

static void assert_test_config_applied(const SimulatedSensor& sim)
{
    // Short mode, 33 ms budget (short-mode register value), 8x6 ROI
    TEST_ASSERT_EQUAL_UINT8(0x14, sim.regs[REG_PHASECAL_TIMEOUT]);
    TEST_ASSERT_EQUAL_UINT16(0x00D6, sim.reg16(REG_TIMEOUT_MACROP_A_HI));
    TEST_ASSERT_EQUAL_UINT8((6 - 1) << 4 | (8 - 1), sim.regs[REG_ROI_XY_SIZE]);
}

// ---- init() ----

TEST_CASE("simulated: init keeps XSHUT high, then initializes and configures the sensor", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init(test_config()));

    // XSHUT set high before the pin becomes an output, so the sensor is never reset
    TEST_ASSERT_EQUAL(1, s_gpio_config_calls);
    TEST_ASSERT_EQUAL_UINT64(1ULL << XSHUT_GPIO, s_gpio_config.pin_bit_mask);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, s_gpio_config.mode);
    TEST_ASSERT_EQUAL(1, s_xshut_levels.size());
    TEST_ASSERT_EQUAL(1, s_xshut_levels[0]);
    TEST_ASSERT_EQUAL(1, sim.power_ups);

    TEST_ASSERT_TRUE(sensor.is_initialized());
    TEST_ASSERT_FALSE(sensor.is_ranging());
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_EQUAL_UINT8(255, sensor.last_range_status());
    TEST_ASSERT_TRUE(sim.wrote(REG_DEFAULT_CONFIGURATION));
    TEST_ASSERT_EQUAL_UINT8(0x09, sim.regs[REG_VHV_LOOP_BOUND]);
    TEST_ASSERT_EQUAL_UINT8(0x00, sim.regs[REG_VHV_INIT]);

    assert_test_config_applied(sim);
    // 0x1F4 oscillator value x 100 ms x 1.075
    TEST_ASSERT_UINT32_WITHIN(1, 53750, sim.reg32(REG_INTERMEASUREMENT_PERIOD));
}

TEST_CASE("simulated: default configuration is written in one transaction", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    // 0x2D through 0x87 in one write, ending with ranging stopped
    const Write* config_write = nullptr;
    for (const Write& w : sim.writes) {
        if (w.index == REG_DEFAULT_CONFIGURATION) {
            config_write = &w;
        }
    }
    TEST_ASSERT_NOT_NULL(config_write);
    TEST_ASSERT_EQUAL(0x87 - 0x2D + 1, config_write->data.size());
    TEST_ASSERT_EQUAL_HEX8(0x00, config_write->data.back());

    // Default config is long mode with a 50 ms budget and the full ROI
    TEST_ASSERT_EQUAL_UINT8(0x0A, sim.regs[REG_PHASECAL_TIMEOUT]);
    TEST_ASSERT_EQUAL_UINT16(0x00AD, sim.reg16(REG_TIMEOUT_MACROP_A_HI));
    TEST_ASSERT_EQUAL_UINT8(0xFF, sim.regs[REG_ROI_XY_SIZE]);
}

TEST_CASE("simulated: invalid config is rejected without touching the hardware", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim, XSHUT_GPIO);
    Config config;
    config.roi_width = 2;
    // No GPIO stubs are installed, so any GPIO call fails the test
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sensor.init(config));
    TEST_ASSERT_TRUE(sim.writes.empty());
    TEST_ASSERT_FALSE(sensor.is_initialized());
}

TEST_CASE("simulated: second init returns invalid state and keeps the sensor working", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    size_t writes = sim.writes.size();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.init());
    TEST_ASSERT_EQUAL(writes, sim.writes.size());

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    sim.set_result(RAW_VALID, 777);
    TEST_ASSERT_EQUAL_INT32(777, sensor.read_range_mm(100));
}

TEST_CASE("simulated: init waits for the sensor to finish booting", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.boot_polls = 3;
    sim.power_on();
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_TRUE(sensor.is_initialized());
    TEST_ASSERT_EQUAL(3, s_delay_calls);
}

TEST_CASE("simulated: init boots a sensor that was held in reset", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.power_off();
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_TRUE(sim.powered);
    TEST_ASSERT_TRUE(sim.wrote(REG_DEFAULT_CONFIGURATION));
}

TEST_CASE("simulated: unresponsive sensor is reset through XSHUT and recovers", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.boot_polls = NEVER;
    sim.power_on();
    // After the reset it boots normally
    sim.boot_polls = 0;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    // High at init, then a low pulse to reset
    TEST_ASSERT_EQUAL(3, s_xshut_levels.size());
    TEST_ASSERT_EQUAL(1, s_xshut_levels[0]);
    TEST_ASSERT_EQUAL(0, s_xshut_levels[1]);
    TEST_ASSERT_EQUAL(1, s_xshut_levels[2]);
    TEST_ASSERT_TRUE(sim.wrote(REG_DEFAULT_CONFIGURATION));
    TEST_ASSERT_TRUE(sensor.is_initialized());
}

TEST_CASE("simulated: sensor that never boots times out after one reset attempt", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.boot_polls = NEVER;
    sim.power_on();
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sensor.init());

    TEST_ASSERT_FALSE(sensor.is_initialized());
    // Two 100 ms boot timeouts, with the reset pulse between them; XSHUT is left high
    TEST_ASSERT_UINT32_WITHIN(pdMS_TO_TICKS(20) + 2, pdMS_TO_TICKS(200), s_ticks);
    TEST_ASSERT_EQUAL(1, s_xshut_levels.back());

    // The failed init released everything, so it can be retried
    sim.boot_polls = 0;
    sim.power_on();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_TRUE(sensor.is_initialized());
}

TEST_CASE("simulated: device with the wrong model ID is rejected", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.model_id = 0xEEAA;
    sim.power_on();
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
    TEST_ASSERT_FALSE(sim.wrote(REG_DEFAULT_CONFIGURATION));
}

TEST_CASE("simulated: init times out instead of hanging when the first measurement never arrives", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.polls_until_ready = NEVER;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
    // Two 500 ms waits, with the reset pulse between them
    TEST_ASSERT_UINT32_WITHIN(pdMS_TO_TICKS(20) + 2, pdMS_TO_TICKS(1000), s_ticks);
}

TEST_CASE("simulated: I2C failure while writing the configuration fails init", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.fail_writes_to = REG_DEFAULT_CONFIGURATION;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
}

TEST_CASE("simulated: I2C failure while applying the configuration fails init", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.fail_writes_to = REG_ROI_XY_SIZE;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
}

TEST_CASE("simulated: GPIO configuration failure is passed through", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    s_gpio_config_result = ESP_ERR_INVALID_ARG;
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sensor.init());
    TEST_ASSERT_TRUE(sim.writes.empty());

    s_gpio_config_result = ESP_OK;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
}

TEST_CASE("simulated: without XSHUT an unresponsive sensor is recovered by a software reset", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.boot_polls = NEVER;
    sim.power_on();
    sim.boot_polls = 0;
    // No GPIO stubs are installed, so any GPIO call fails the test
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_TRUE(sim.wrote(REG_SOFT_RESET));
    // Construction, the power_on() above, then the software reset
    TEST_ASSERT_EQUAL(3, sim.power_ups);

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.shutdown());
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.prepare_for_deep_sleep());
}

// ---- Reusing an initialized sensor ----

TEST_CASE("simulated: init after an ESP32 restart reuses the initialized sensor", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    {
        // Before the restart: initialized by an earlier boot
        VL53L1X before(sim, XSHUT_GPIO);
        TEST_ASSERT_EQUAL(ESP_OK, before.init());
    }
    // A restart can also happen while the sensor is ranging (a crash, or the watchdog)
    const uint8_t start_ranging[] = { 0x00, 0x87, 0x40 };
    TEST_ASSERT_EQUAL(ESP_OK, sim.write(start_ranging, sizeof(start_ranging)));
    TEST_ASSERT_TRUE(sim.ranging);
    sim.writes.clear();

    TickType_t start = s_ticks;
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init(test_config()));

    // No reset, no default configuration and no initial measurement; only the new configuration
    TEST_ASSERT_EQUAL(1, sim.power_ups);
    TEST_ASSERT_FALSE(sim.wrote(REG_DEFAULT_CONFIGURATION));
    TEST_ASSERT_EQUAL_UINT32(start, s_ticks);
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_FALSE(sim.data_ready());
    assert_test_config_applied(sim);

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    sim.set_result(RAW_VALID, 456);
    TEST_ASSERT_EQUAL_INT32(456, sensor.read_range_mm(100));
}

TEST_CASE("simulated: init after the sensor lost power initializes it fully", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    {
        VL53L1X before(sim, XSHUT_GPIO);
        TEST_ASSERT_EQUAL(ESP_OK, before.init());
    }
    sim.power_off();
    sim.power_on();
    sim.writes.clear();

    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_TRUE(sim.wrote(REG_DEFAULT_CONFIGURATION));
}

// ---- ranging ----

TEST_CASE("simulated: read returns the measured distance and clears the interrupt", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    TEST_ASSERT_TRUE(sim.ranging);
    TEST_ASSERT_TRUE(sensor.is_ranging());

    sim.polls_until_ready = 2;
    sim.set_result(RAW_VALID, 1234);
    TEST_ASSERT_EQUAL_INT32(1234, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(0, sensor.last_range_status());
    TEST_ASSERT_FALSE(sim.data_ready());

    sim.set_result(RAW_VALID, 2500);
    TEST_ASSERT_EQUAL_INT32(2500, sensor.read_range_mm(100));
}

TEST_CASE("simulated: failed measurements are reported with their status", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    sim.set_result(RAW_SIGNAL_FAIL, 4000);
    TEST_ASSERT_EQUAL_INT32(RANGE_OUT_OF_RANGE, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(2, sensor.last_range_status());

    sim.set_result(RAW_OUT_OF_BOUNDS, 4000);
    TEST_ASSERT_EQUAL_INT32(RANGE_OUT_OF_RANGE, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(4, sensor.last_range_status());

    sim.set_result(RAW_SIGMA_FAIL, 900);
    TEST_ASSERT_EQUAL_INT32(RANGE_INVALID, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(1, sensor.last_range_status());
}

TEST_CASE("simulated: read times out when no measurement arrives", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    sim.polls_until_ready = NEVER;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    TickType_t start = s_ticks;
    TEST_ASSERT_EQUAL_INT32(RANGE_TIMEOUT, sensor.read_range_mm(200));
    TEST_ASSERT_UINT32_WITHIN(pdMS_TO_TICKS(20) + 1, pdMS_TO_TICKS(200), s_ticks - start);

    // A zero timeout checks once without waiting
    start = s_ticks;
    TEST_ASSERT_EQUAL_INT32(RANGE_TIMEOUT, sensor.read_range_mm(0));
    TEST_ASSERT_EQUAL_UINT32(start, s_ticks);
}

TEST_CASE("simulated: read before ranging or init reports an error", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.start_ranging());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.stop_ranging());

    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.stop_ranging());
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));
}

TEST_CASE("simulated: I2C failure during a read reports an error", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    sim.fail_all = true;
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));

    sim.fail_all = false;
    sim.set_result(RAW_VALID, 640);
    TEST_ASSERT_EQUAL_INT32(640, sensor.read_range_mm(100));
}

// ---- shutdown() and wake() ----

TEST_CASE("simulated: shutdown is software standby and wake resumes without reinitializing", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init(test_config()));
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    size_t levels = s_xshut_levels.size();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.shutdown());
    // XSHUT untouched, sensor powered but not ranging
    TEST_ASSERT_EQUAL(levels, s_xshut_levels.size());
    TEST_ASSERT_TRUE(sim.powered);
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_FALSE(sensor.is_ranging());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));
    // Shutting down again does nothing
    TEST_ASSERT_EQUAL(ESP_OK, sensor.shutdown());

    sim.writes.clear();
    TickType_t start = s_ticks;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.wake());
    TEST_ASSERT_TRUE(sim.writes.empty());
    TEST_ASSERT_EQUAL_UINT32(start, s_ticks);
    assert_test_config_applied(sim);

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    sim.set_result(RAW_VALID, 321);
    TEST_ASSERT_EQUAL_INT32(321, sensor.read_range_mm(100));
    // Wake while ranging does nothing
    TEST_ASSERT_EQUAL(ESP_OK, sensor.wake());
    TEST_ASSERT_TRUE(sim.ranging);
}

TEST_CASE("simulated: wake reinitializes a sensor that lost power", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init(test_config()));
    TEST_ASSERT_EQUAL(ESP_OK, sensor.shutdown());

    sim.power_off();
    sim.power_on();
    sim.writes.clear();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.wake());
    TEST_ASSERT_TRUE(sim.wrote(REG_DEFAULT_CONFIGURATION));
    assert_test_config_applied(sim);
}

TEST_CASE("simulated: wake on an unresponsive sensor resets it, and fails if it stays silent", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    sim.fail_all = true;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sensor.wake());
    TEST_ASSERT_EQUAL(0, s_xshut_levels[s_xshut_levels.size() - 2]);
    TEST_ASSERT_EQUAL(1, s_xshut_levels.back());
    // Still initialized, so wake can be retried
    TEST_ASSERT_TRUE(sensor.is_initialized());

    sim.fail_all = false;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.wake());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    sim.set_result(RAW_VALID, 999);
    TEST_ASSERT_EQUAL_INT32(999, sensor.read_range_mm(100));
}

TEST_CASE("simulated: shutdown, wake and deep-sleep preparation before init report invalid state", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.shutdown());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.wake());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.prepare_for_deep_sleep());
}

TEST_CASE("simulated: prepare_for_deep_sleep stops ranging and holds XSHUT high", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    gpio_hold_en_ExpectAndReturn(XSHUT_GPIO, ESP_OK);
    gpio_deep_sleep_hold_en_Expect();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.prepare_for_deep_sleep());
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_TRUE(sim.powered);
    TEST_ASSERT_EQUAL(1, s_xshut_levels.back());
}

TEST_CASE("simulated: prepare_for_deep_sleep passes through a hold failure", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    gpio_hold_en_ExpectAndReturn(XSHUT_GPIO, ESP_ERR_NOT_SUPPORTED);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, sensor.prepare_for_deep_sleep());
}

// ---- lifecycle ----

TEST_CASE("simulated: deinit stops ranging, leaves the sensor powered and can be repeated", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    size_t levels = s_xshut_levels.size();
    sensor.deinit();
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_TRUE(sim.powered);
    TEST_ASSERT_FALSE(sensor.is_initialized());
    sensor.deinit();
    TEST_ASSERT_EQUAL(levels, s_xshut_levels.size());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));

    // A deinitialized sensor can be initialized again, reusing the sensor's state
    sim.writes.clear();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_FALSE(sim.wrote(REG_DEFAULT_CONFIGURATION));
}

TEST_CASE("simulated: destructor stops ranging", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    {
        VL53L1X sensor(sim, XSHUT_GPIO);
        TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
        TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    }
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_TRUE(sim.powered);
}

TEST_CASE("simulated: several sensors work side by side up to the limit", "[vl53l1x][host]")
{
    // Without XSHUT so no GPIO calls are made
    SimulatedSensor sims[5];
    VL53L1X s0(sims[0]), s1(sims[1]), s2(sims[2]), s3(sims[3]), s4(sims[4]);
    VL53L1X* sensors[] = { &s0, &s1, &s2, &s3 };
    for (VL53L1X* s : sensors) {
        TEST_ASSERT_EQUAL(ESP_OK, s->init());
        TEST_ASSERT_EQUAL(ESP_OK, s->start_ranging());
    }
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, s4.init());
    TEST_ASSERT_TRUE(sims[4].writes.empty());

    // Each sensor talks only to its own device
    for (int i = 0; i < 4; i++) {
        sims[i].set_result(RAW_VALID, static_cast<uint16_t>(100 * (i + 1)));
    }
    TEST_ASSERT_EQUAL_INT32(300, s2.read_range_mm(100));
    TEST_ASSERT_EQUAL_INT32(100, s0.read_range_mm(100));
    TEST_ASSERT_EQUAL_INT32(400, s3.read_range_mm(100));

    // Releasing one frees a slot
    s1.deinit();
    TEST_ASSERT_EQUAL(ESP_OK, s4.init());
}

// ---- XSHUT handling ----

TEST_CASE("simulated: init raises XSHUT before making it an output, then releases the deep-sleep hold", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    // Releasing a held pin applies the configured state, so the level and direction must be set first
    TEST_ASSERT_EQUAL(3, s_gpio_events.size());
    TEST_ASSERT_EQUAL_STRING("level 1", s_gpio_events[0].c_str());
    TEST_ASSERT_EQUAL_STRING("output", s_gpio_events[1].c_str());
    TEST_ASSERT_EQUAL_STRING("hold released", s_gpio_events[2].c_str());
}

TEST_CASE("simulated: failure releasing the deep-sleep hold is passed through", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    s_hold_dis_result = ESP_ERR_INVALID_ARG;
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
    TEST_ASSERT_TRUE(sim.writes.empty());

    s_hold_dis_result = ESP_OK;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
}

// ---- configurations ----

TEST_CASE("simulated: every valid configuration initializes and is written to the sensor", "[vl53l1x][host]")
{
    struct Budget { uint16_t ms; uint16_t short_a_hi; uint16_t long_a_hi; };
    // RANGE_CONFIG__TIMEOUT_MACROP_A_HI values from VL53L1X_SetTimingBudgetInMs(); 0 = not available
    const Budget budgets[] = {
        { 15, 0x001D, 0 }, { 20, 0x0051, 0x001E }, { 33, 0x00D6, 0x0060 }, { 50, 0x01AE, 0x00AD },
        { 100, 0x02E1, 0x01CC }, { 200, 0x03E1, 0x02D9 }, { 500, 0x0591, 0x048F },
    };
    const DistanceMode modes[] = { DistanceMode::Short, DistanceMode::Long };

    int applied = 0;
    for (DistanceMode mode : modes) {
        for (const Budget& budget : budgets) {
            Config config;
            config.distance_mode = mode;
            config.timing_budget_ms = budget.ms;
            config.inter_measurement_ms = budget.ms;
            uint16_t expected = mode == DistanceMode::Short ? budget.short_a_hi : budget.long_a_hi;
            // validate_config() must accept exactly the combinations the ULD supports
            TEST_ASSERT_EQUAL(expected ? ESP_OK : ESP_ERR_INVALID_ARG, validate_config(config));
            if (!expected) {
                continue;
            }

            SimulatedSensor sim;
            VL53L1X sensor(sim);
            TEST_ASSERT_EQUAL(ESP_OK, sensor.init(config));
            TEST_ASSERT_EQUAL_UINT8(mode == DistanceMode::Short ? 0x14 : 0x0A, sim.regs[REG_PHASECAL_TIMEOUT]);
            TEST_ASSERT_EQUAL_HEX16(expected, sim.reg16(REG_TIMEOUT_MACROP_A_HI));
            applied++;
        }
    }
    TEST_ASSERT_EQUAL(13, applied);
}

TEST_CASE("simulated: region of interest limits are written to the sensor", "[vl53l1x][host]")
{
    struct Roi { uint8_t width, height, reg; };
    const Roi rois[] = { { 4, 4, 0x33 }, { 16, 16, 0xFF }, { 16, 4, 0x3F }, { 4, 16, 0xF3 } };
    for (const Roi& roi : rois) {
        SimulatedSensor sim;
        VL53L1X sensor(sim);
        Config config;
        config.roi_width = roi.width;
        config.roi_height = roi.height;
        TEST_ASSERT_EQUAL(ESP_OK, sensor.init(config));
        TEST_ASSERT_EQUAL_HEX8(roi.reg, sim.regs[REG_ROI_XY_SIZE]);
    }
}

// ---- I2C failures ----

TEST_CASE("simulated: start_ranging clears a stale measurement before starting", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    sim.writes.clear();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    TEST_ASSERT_EQUAL(2, sim.writes.size());
    TEST_ASSERT_EQUAL_HEX16(REG_INTERRUPT_CLEAR, sim.writes[0].index);
    TEST_ASSERT_EQUAL_HEX16(REG_MODE_START, sim.writes[1].index);
    TEST_ASSERT_EQUAL_HEX8(0x40, sim.writes[1].data[0]);
}

TEST_CASE("simulated: start_ranging reports an I2C failure and the sensor is not ranging", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    sim.fail_writes_to = REG_MODE_START;
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.start_ranging());
    TEST_ASSERT_FALSE(sensor.is_ranging());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));
}

TEST_CASE("simulated: stop_ranging I2C failure is reported and the sensor is still treated as ranging", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    sim.fail_writes_to = REG_MODE_START;
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.stop_ranging());
    TEST_ASSERT_TRUE(sensor.is_ranging());
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.shutdown());
    // No GPIO hold expectations are set, so holding XSHUT here would fail the test
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.prepare_for_deep_sleep());

    sim.fail_writes_to = -1;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.shutdown());
    TEST_ASSERT_FALSE(sensor.is_ranging());
}

TEST_CASE("simulated: read reports an error when the result cannot be read", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    sim.set_result(RAW_VALID, 1500);
    sim.fail_reads_from = REG_RANGE_STATUS;
    sim.fail_read_count = 1;
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(255, sensor.last_range_status());

    TEST_ASSERT_EQUAL_INT32(1500, sensor.read_range_mm(100));
}

TEST_CASE("simulated: read reports an error when the interrupt cannot be cleared", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    sim.set_result(RAW_VALID, 1500);
    sim.fail_writes_to = REG_INTERRUPT_CLEAR;
    sim.fail_write_count = 1;
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_INT32(1500, sensor.read_range_mm(100));
}

TEST_CASE("simulated: a failed model ID read is retried after a reset", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.fail_reads_from = REG_MODEL_ID;
    sim.fail_read_count = 1;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(3, s_xshut_levels.size());
    TEST_ASSERT_EQUAL(0, s_xshut_levels[1]);
}

TEST_CASE("simulated: a model ID that can never be read fails init", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.fail_reads_from = REG_MODEL_ID;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
}

TEST_CASE("simulated: I2C failure finishing initialization fails init", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.fail_writes_to = REG_VHV_LOOP_BOUND;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
}

TEST_CASE("simulated: without XSHUT a sensor that never answers fails init with a timeout", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.fail_all = true;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
    TEST_ASSERT_TRUE(sim.writes.empty());
}

// ---- detecting an initialized sensor ----

TEST_CASE("simulated: only both VHV markers together count as already initialized", "[vl53l1x][host]")
{
    struct Markers { uint8_t loop_bound, vhv_init; };
    // Each has one register at the post-initialization value and the other not
    const Markers partial[] = { { 0x09, 0x92 }, { 0x81, 0x00 } };
    for (const Markers& m : partial) {
        SimulatedSensor sim;
        sim.regs[REG_VHV_LOOP_BOUND] = m.loop_bound;
        sim.regs[REG_VHV_INIT] = m.vhv_init;
        VL53L1X sensor(sim);
        TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
        TEST_ASSERT_TRUE(sim.wrote(REG_DEFAULT_CONFIGURATION));
    }
}

TEST_CASE("simulated: failure stopping a reused sensor falls back to a reset and full init", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    {
        VL53L1X before(sim, XSHUT_GPIO);
        TEST_ASSERT_EQUAL(ESP_OK, before.init());
    }
    sim.writes.clear();
    s_xshut_levels.clear();

    sim.fail_writes_to = REG_MODE_START;
    sim.fail_write_count = 1;
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    // High, then the reset pulse
    TEST_ASSERT_EQUAL(3, s_xshut_levels.size());
    TEST_ASSERT_TRUE(sim.wrote(REG_DEFAULT_CONFIGURATION));
}

TEST_CASE("simulated: wake reinitializes a sensor that is still booting after losing power", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init(test_config()));
    TEST_ASSERT_EQUAL(ESP_OK, sensor.shutdown());

    sim.boot_polls = 2;
    sim.power_off();
    sim.power_on();
    sim.writes.clear();
    s_delay_calls = 0;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.wake());
    // The status read during wake used up one "not booted" poll; the boot wait used the other
    TEST_ASSERT_EQUAL(1, s_delay_calls);
    TEST_ASSERT_TRUE(sim.wrote(REG_DEFAULT_CONFIGURATION));
    assert_test_config_applied(sim);
}

TEST_CASE("simulated: wake treats an unreadable status as lost configuration and rechecks", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    sim.writes.clear();

    sim.fail_reads_from = REG_VHV_LOOP_BOUND;
    sim.fail_read_count = 1;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.wake());
    // The second check read the markers fine, so the sensor was reused rather than reinitialized
    TEST_ASSERT_FALSE(sim.wrote(REG_DEFAULT_CONFIGURATION));
}

TEST_CASE("simulated: deinit of a sensor that is not ranging sends nothing", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    sim.writes.clear();
    sensor.deinit();
    TEST_ASSERT_TRUE(sim.writes.empty());
}
