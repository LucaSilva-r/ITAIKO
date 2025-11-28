#ifndef UTILS_SERIALCONFIG_H_
#define UTILS_SERIALCONFIG_H_

#include "utils/SettingsStore.h"

#include "pico/stdlib.h"
#include "tusb.h"

namespace Doncon::Utils {

/**
 * @brief Serial configuration interface for runtime parameter adjustment
 *
 * Provides a USB CDC serial protocol for reading and writing settings,
 * compatible with hidtaiko's web configurator (https://kasasiki3.github.io/ver1.3_webapp_rp2040version/).
 *
 * Protocol:
 * - Send "1000" to read all settings (returns "key:value" pairs)
 * - Send "1001" to save settings to flash
 * - Send "1002" to enter write mode
 * - Send "1003" to reload settings from flash
 * - In write mode, send "key:value" pairs (e.g., "0:800")
 *
 * HIDtaiko-compatible Keys (web page order - note keys 0&1 swapped vs kando array!):
 * 0: Don Left Threshold (Left face sensitivity)
 * 1: Ka Left Threshold (Left rim sensitivity)
 * 2: Don Right Threshold (Right face sensitivity)
 * 3: Ka Right Threshold (Right rim sensitivity)
 * 4: Don Debounce - B delay (Next input time after face hit)
 * 5: Kat Debounce - C delay (Rim input acceptance time)
 * 6: Crosstalk Debounce - D delay (Time to ignore rim after face)
 * 7: Key Timeout - H delay (Input limit for simulators)
 * 8: Debounce Delay - A delay (Key hold time)
 */
class SerialConfig {
  public:
    explicit SerialConfig(SettingsStore &settings_store);

    /**
     * @brief Process incoming serial data
     *
     * Call this from main loop when CDC data is available.
     * Non-blocking, processes one command per call.
     */
    void processSerial();

  private:
    SettingsStore &m_settings_store;
    bool m_write_mode;
    int m_write_count;

    enum class Command : int {
        ReadAll = 1000,
        SaveToFlash = 1001,
        EnterWriteMode = 1002,
        ReloadFromFlash = 1003,
    };

    void handleCommand(int command_value);
    void handleWriteData(const char *data);
    void sendAllSettings();
    uint16_t getSettingByKey(int key);
    void setSettingByKey(int key, uint16_t value);
};

} // namespace Doncon::Utils

#endif // UTILS_SERIALCONFIG_H_
