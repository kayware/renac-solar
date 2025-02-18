#include "renac_solar.h"
#include "esphome/core/log.h"

namespace esphome {
    namespace renac_solar {
        static const char *TAG = "renac_solar";
        static const std::vector<uint8_t> MODBUS_PAYLOAD = {
            0x01,                                           /* Device address */
            0x65,                                           /* Read multi-segment register */
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
    }
}
