#pragma once

/* Based lightly on growatt_solar */

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/modbus/modbus.h"

#include <vector>

namespace esphome {
    namespace renac_solar {
        class RenacSolar : public PollingComponent, public modbus::ModbusDevice {
        public:
            using sensor::Sensor;
            void loop() override;
            void update() override;
            void on_modbus_data(const std::vector<uint8_t> &data) override;
            void dump_config() override;
            void set_inverter_status_sensor(Sensor *sensor) { m_inverter_status = sensor; }
            void set_grid_frequency_sensor(Sensor *sensor) { m_grid_frequency = sensor; }
            void set_pv_active_power_sensor(Sensor *sensor) { m_pv_active_power = sensor; }
            void set_today_production_sensor(Sensor *sensor) { m_today_production = sensor; }
            void set_total_production_sensor(Sensor *sensor) { m_total_production = sensor; }
            void set_inverter_temperature_sensor(Sensor *sensor) { m_inverter_temperature = sensor; }
            void set_ambient_temperature_sensor(Sensor *sensor) { m_ambient_temperature = sensor; }
            void set_phase_voltage_sensor(uint8_t phase, Sensor *sensor) { m_phases[phase].m_voltage = sensor; }
            void set_phase_current_sensor(uint8_t phase, Sensor *sensor) { m_phases[phase].m_current = sensor; }
            void set_phase_active_power_sensor(uint8_t phase, Sensor *sensor) { m_phases[phase].m_active_power = sensor; }
            void set_string_voltage_sensor(uint8_t id, Sensor *sensor) { m_strings[id].m_voltage = sensor; }
            void set_string_current_sensor(uint8_t id, Sensor *sensor) { m_strings[id].m_current = sensor; }
            void set_string_active_power_sensor(uint8_t id, Sensor *sensor) { m_strings[id].m_active_power = sensor; }

            void parse_registers(const std::vector<uint8_t> &data);
            void log_error(const std::vector<uint8_t> &data);
        protected:
            struct SensorGroup {
                Sensor *m_voltage { nullptr };
                Sensor *m_current { nullptr };
                Sensor *m_active_power { nullptr };
            };

            bool m_waiting_to_update;
            uint32_t m_last_send;

            SensorGroup m_phases[3];
            SensorGroup m_strings[2];
            Sensor *m_inverter_status;
            Sensor *m_grid_frequency;
            Sensor *m_pv_active_power;
            Sensor *m_today_production;
            Sensor *m_total_production;
            Sensor *m_inverter_temperature;
            Sensor *m_ambient_temperature;
        };
    }
}
