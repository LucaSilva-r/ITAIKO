#include "utils/SerialConfig.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace Doncon::Utils {

SerialConfig::SerialConfig(SettingsStore &settings_store)
    : m_settings_store(settings_store), m_write_mode(false), m_write_count(0) {}

void SerialConfig::processSerial() {
    // Check if CDC is connected and data is available
    if (!tud_cdc_connected() || !tud_cdc_available()) {
        return;
    }

    char buf[64];
    uint32_t count = tud_cdc_read(buf, sizeof(buf) - 1);
    if (count == 0) {
        return;
    }

    buf[count] = '\0'; // Null terminate

    // If in write mode, parse key:value pairs
    if (m_write_mode) {
        handleWriteData(buf);
        return;
    }

    // Otherwise, parse as command
    int command_value = std::atoi(buf);
    handleCommand(command_value);
}

void SerialConfig::handleCommand(int command_value) {
    switch (static_cast<Command>(command_value)) {
    case Command::ReadAll:
        sendAllSettings();
        break;

    case Command::SaveToFlash:
        m_settings_store.store();
        stdio_flush();
        break;

    case Command::EnterWriteMode:
        m_write_mode = true;
        m_write_count = 0;
        break;

    case Command::ReloadFromFlash:
        // SettingsStore loads from flash on construction, so we just echo current values
        sendAllSettings();
        break;
    }
}

void SerialConfig::handleWriteData(const char *data) {
    char buf[32];
    strncpy(buf, data, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *token = strtok(buf, " \n\r");

    while (token != NULL) {
        // Split on ':'
        char *colon = strchr(token, ':');
        if (colon) {
            *colon = '\0';
            int key = atoi(token);
            int value = atoi(colon + 1);

            setSettingByKey(key, value);

            m_write_count++;
            // Exit write mode after receiving 9 values (hidtaiko compatible)
            if (m_write_count >= 9) {
                m_write_mode = false;
                m_write_count = 0;
            }
        }
        token = strtok(NULL, " ");
    }
}

void SerialConfig::sendAllSettings() {
    // Send only 9 values for hidtaiko compatibility
    for (int i = 0; i < 9; i++) {
        uint16_t value = getSettingByKey(i);
        printf("%d:%d\n", i, value);
        stdio_flush();
        sleep_us(5000); // Small delay like hidtaiko
    }
}

uint16_t SerialConfig::getSettingByKey(int key) {
    auto thresholds = m_settings_store.getTriggerThresholds();

    // HIDtaiko-compatible mapping (web page order, not kando array order!)
    switch (key) {
    case 0: // Don Left (face left sensitivity) - swapped!
        return thresholds.don_left;
    case 1: // Ka Left (rim left sensitivity) - swapped!
        return thresholds.ka_left;
    case 2: // Don Right (face right sensitivity)
        return thresholds.don_right;
    case 3: // Ka Right (rim right sensitivity)
        return thresholds.ka_right;
    case 4: // Don Debounce (B delay - next input time after face)
        return m_settings_store.getDonDebounceMs();
    case 5: // Kat Debounce (C delay - rim input acceptance time)
        return m_settings_store.getKatDebounceMs();
    case 6: // Crosstalk Debounce (D delay - time to ignore rim after face)
        return m_settings_store.getCrosstalkDebounceMs();
    case 7: // Key Timeout (H delay - input limit for simulators)
        return m_settings_store.getKeyTimeoutMs();
    case 8: // Debounce Delay (A delay - single hit acceptance time)
        return m_settings_store.getDebounceDelay();
    default:
        return 0;
    }
}

void SerialConfig::setSettingByKey(int key, uint16_t value) {
    auto thresholds = m_settings_store.getTriggerThresholds();

    // HIDtaiko-compatible mapping (web page order, not kando array order!)
    switch (key) {
    case 0: // Don Left (face left sensitivity) - swapped!
        thresholds.don_left = value;
        m_settings_store.setTriggerThresholds(thresholds);
        break;
    case 1: // Ka Left (rim left sensitivity) - swapped!
        thresholds.ka_left = value;
        m_settings_store.setTriggerThresholds(thresholds);
        break;
    case 2: // Don Right (face right sensitivity)
        thresholds.don_right = value;
        m_settings_store.setTriggerThresholds(thresholds);
        break;
    case 3: // Ka Right (rim right sensitivity)
        thresholds.ka_right = value;
        m_settings_store.setTriggerThresholds(thresholds);
        break;
    case 4: // Don Debounce (B delay)
        m_settings_store.setDonDebounceMs(value);
        break;
    case 5: // Kat Debounce (C delay)
        m_settings_store.setKatDebounceMs(value);
        break;
    case 6: // Crosstalk Debounce (D delay)
        m_settings_store.setCrosstalkDebounceMs(value);
        break;
    case 7: // Key Timeout (H delay)
        m_settings_store.setKeyTimeoutMs(value);
        break;
    case 8: // Debounce Delay (A delay)
        m_settings_store.setDebounceDelay(value);
        break;
    default:
        break;
    }
}

} // namespace Doncon::Utils
