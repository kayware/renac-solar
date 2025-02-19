#include "renac_solar.h"
#include "esphome/core/log.h"

namespace esphome {
    namespace renac_solar {
        static const char *TAG = "renac_solar";
        static const uint8_t MODBUS_READ_MULTIPLE = 0x65;
        static const uint8_t RENAC_REGION_COUNT = 0x04;
        static const float DECIMAL_ONE = 0.1f;
        static const float DECIMAL_TWO = 0.01f;
        static const std::vector<uint8_t> MODBUS_PAYLOAD = {
            0x01,                                           /* Device address */
            MODBUS_READ_MULTIPLE,                           /* Read multi-segment register */
            RENAC_REGION_COUNT,                             /* Number of regions to read */
            0x29, 0x04,                                     /* Region #1 */
            0x00, 0x1B,
            0x29, 0xCC,                                     /* Region #2 */
            0x00, 0x10,
            0x2A, 0x30,                                     /* Region #3 */
            0x00, 0x11,
            0x2E, 0xE0,                                     /* Region #4 */
            0x00, 0x06,
            0xFF, 0xD7                                      /* CRC */
        };

        void RenacSolar::loop() {
            if (!m_waiting_to_update)
                return;
            update();
        }

        void RenacSolar::update() {
            uint32_t now = millis();
            if (now - m_last_send < get_update_interval() / 2)
                return;
            if (waiting_for_response()) {
                m_waiting_to_update = true;
                return;
            }
            m_waiting_to_update = false;
            send_raw(MODBUS_PAYLOAD);
            m_last_send = millis();
        }

        void RenacSolar::on_modbus_data(const std::vector<uint8_t> &data) {
            if (!m_last_send)
                return;
            m_last_send = 0;
            
            if (data.size() < 80) {
                log_error(data);
                return;
            }

            if (data[0] == MODBUS_READ_MULTIPLE)
                parse_registers(data);
            else
                ESP_LOGW(TAG, "Unknown response code: 0x%02X", data[0]);
        }

        void RenacSolar::log_error(const std::vector<uint8_t> &data) {
            if (data.size() == 1) {
                switch (data[0]) {
                case 0x01:
                    ESP_LOGE(TAG, "Got exception: illegal function code!");
                    break;
                case 0x02:
                    ESP_LOGE(TAG, "Got exception: illegal data address!");
                    break;
                case 0x03:
                    ESP_LOGE(TAG, "Got exception: illegal data value!");
                    break;
                case 0x04:
                    ESP_LOGE(TAG, "Got exception: data save failed!");
                    break;
                case 0x06:
                    ESP_LOGE(TAG, "Got exception: slave device busy!");
                    break;
                case 0x08:
                    ESP_LOGE(TAG, "Got exception: storage parity error!");
                    break;
                default:
                    ESP_LOGE(TAG, "Got unknown exception: 0x%02X", data[0]);
                    break;
                }
            } else {
                ESP_LOGW(TAG, "Received %ld bytes, expected a longer response.", data.size());
            }
        }

        void RenacSolar::parse_registers(const std::vector<uint8_t> &data) {
            auto read8 = [&](size_t off) -> uint8_t {
                return data[off];
            };

            auto read16 = [&](size_t off) -> uint16_t {
                return encode_uint16(data[off], data[off + 1]);
            };

            auto read_reg16 = [&](size_t off, float unit) -> float {
                return read16(off) * unit;
            };

            auto read_reg32 = [&](size_t off, float unit) -> float {
                return encode_uint32(data[off], data[off + 1], data[off + 2], data[off + 3]) * unit;
            };

            auto update_sensor = [&](Sensor *s, float value) {
                if (s)
                    s->publish_state(value);
            };

            auto read_generic_info = [&](size_t off) {
                update_sensor(m_inverter_status, read_reg16(off + 0, 1));
                update_sensor(m_today_production, read_reg16(off + 14, DECIMAL_ONE));
                update_sensor(m_total_production, read_reg32(off + 18, DECIMAL_ONE));
                update_sensor(m_pv_active_power, read_reg32(off + 26, 1));
                update_sensor(m_grid_frequency, read_reg16(off + 40, DECIMAL_TWO));

                for (size_t j = 0; j < 3; j++) {
                    update_sensor(m_phases[j].m_voltage, read_reg16(off + 36 + j*6, DECIMAL_ONE));
                    update_sensor(m_phases[j].m_current, read_reg16(off + 38 + j*6, DECIMAL_ONE));
                }
            };

            auto read_string_info = [&](size_t off) {
                for (size_t j = 0; j < 2; j++) {
                    update_sensor(m_strings[j].m_voltage, read_reg16(off + j * 8, DECIMAL_ONE));
                    update_sensor(m_strings[j].m_current, read_reg16(off + 2 + j * 8, DECIMAL_ONE));
                    update_sensor(m_strings[j].m_active_power, read_reg32(off + 4 + j * 8, 1));
                }
            };

            auto read_temperature_info = [&](size_t off) {
                update_sensor(m_ambient_temperature, read_reg16(off + 6, DECIMAL_ONE));
                update_sensor(m_inverter_temperature, read_reg16(off, DECIMAL_ONE));
            };

            auto region_count = data[1];
            for (size_t i = 0, j = 0; j < region_count; j++) {
                auto start_address = read16(i + 2);
                ESP_LOGV(TAG, "Reading region %ld/%ld: %04X", i + 1, region_count, start_address);
                auto num_registers = read8(i + 4);
                switch (start_address) {
                case 0x2904:
                    read_generic_info(i + 5);
                    break;
                case 0x2a30:
                    read_string_info(i + 7);
                    break;
                case 0x2ee0:
                    read_temperature_info(i + 5);
                    break;
                case 0x29cc:
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown register region offset: 0x%04X", start_address);
                    break;
                }

                i += (size_t)num_registers + 3;
            }
        }

        void RenacSolar::dump_config() {
            ESP_LOGCONFIG(TAG, "Renac Solar:");
            ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
        }
    }
}
