#include "renac_solar.h"
#include "esphome/core/log.h"

namespace esphome {
    namespace renac_solar {
        static const char *TAG = "renac_solar";
        static const uint8_t MODBUS_READ_MULTIPLE = 0x65;
        static const std::vector<uint8_t> MODBUS_PAYLOAD = {
            0x01,                                           /* Device address */
            MODBUS_READ_MULTIPLE,                           /* Read multi-segment register */
            0x03,                                           /* Read 3 segments */
            0x29, 0x04,                                     /* Start of region #1 (10500) */
            0x00, 0x26,                                     /* 26 registers in region #1 */
            0x2a, 0x31,                                     /* Start of region #2 (10801) */
            0x00, 0x08,                                     /* 8 registers in region #2 */
            0x2e, 0xe0,                                     /* Start of region #3 (12000) */
            0x00, 0x03                                      /* 3 registers in region #3 */
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
                ESP_LOGW(TAG, "Unknown response code: 0x%02X", data[1]);
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
                update_sensor(m_total_production, read_reg16(off + 18, DECIMAL_ONE));
                update_sensor(m_active_power, read_reg32(off + 26, 1));
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
                    update_sensor(m_strings[j].m_active_power, read_reg32(off + 4 + j * 8, DECIMAL_ONE));
                }
            };

            auto read_temperature_info = [&](size_t off) {

            };

            auto region_count = data[1];
            size_t i = 0;
            while (i < region_count) {
                auto start_address = read16(i);
                auto num_registers = read8(i + 2);
                switch (start_address) {
                case 0x2904:
                    read_generic_info(i + 3);
                    break;
                case 0x2a31:
                    read_string_info(i + 3);
                    break;
                case 0x2ee0:
                    read_temperature_info(i + 3);
                    break;
                default:
                    ESP_LOGW("Unknown register region offset: 0x%04X", start_address);
                    break;
                }

                i += (size_t)num_registers + 3;
            }
        }
    }
}
