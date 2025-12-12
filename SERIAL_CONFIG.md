# Serial Configuration Guide

This document explains how to use the USB serial configuration interface for runtime parameter adjustment during testing.

## Overview

The serial configuration system allows you to read and modify controller settings via USB CDC serial connection without using the OLED menu. This is particularly useful for:
- Quick testing of different threshold values
- Automated configuration scripts
- Bulk parameter adjustments
- Remote configuration
- Web-based configurator integration

## Protocol

The system uses a simple command-based protocol:

### Commands

**Configuration Commands:**
- **1000** - Read all settings (returns 46 key:value pairs)
- **1001** - Save current settings to flash memory
- **1002** - Enter write mode (to send key:value pairs)
- **1003** - Reload settings from flash
- **1004** - Reboot to BOOTSEL mode (firmware update mode)

**Streaming Commands:**
- **2000** - Start streaming sensor data (CSV format, ~100Hz)
- **2001** - Stop streaming sensor data

### Setting Keys

**Trigger Thresholds:**

| Key | Setting | Type | Description |
|-----|---------|------|-------------|
| 0 | Don Left Threshold | uint32 | Left face (don) sensitivity |
| 1 | Ka Left Threshold | uint32 | Left rim (ka) sensitivity |
| 2 | Don Right Threshold | uint32 | Right face (don) sensitivity |
| 3 | Ka Right Threshold | uint32 | Right rim (ka) sensitivity |

**Debounce Settings:**

| Key | Setting | Type | Description |
|-----|---------|------|-------------|
| 4 | Don Debounce | uint16 | Lockout time between don hits (left/right) (ms) |
| 5 | Kat Debounce | uint16 | Lockout time between ka hits (left/right) (ms) |
| 6 | Crosstalk Debounce | uint16 | Time to ignore ka after don hit (ms) |
| 7 | Debounce Delay | uint16 | Same-pad lockout time (can't hit same pad twice) (ms) |
| 8 | Key Timeout | uint16 | How long button appears pressed to OS (ms) |

**Double Trigger Settings:**

| Key | Setting | Type | Description |
|-----|---------|------|-------------|
| 9 | Double Trigger Mode | uint16 | 0=Off, 1=Threshold |
| 10 | Double Trigger Don Left | uint32 | Left face double trigger threshold |
| 11 | Double Trigger Ka Left | uint32 | Left rim double trigger threshold |
| 12 | Double Trigger Don Right | uint32 | Right face double trigger threshold |
| 13 | Double Trigger Ka Right | uint32 | Right rim double trigger threshold |

**Cutoff Thresholds:**

| Key | Setting | Type | Description |
|-----|---------|------|-------------|
| 14 | Cutoff Don Left | uint16 | Don left cutoff value |
| 15 | Cutoff Ka Left | uint16 | Ka left cutoff value |
| 16 | Cutoff Don Right | uint16 | Don right cutoff value |
| 17 | Cutoff Ka Right | uint16 | Ka right cutoff value |

**Keyboard Mappings - Drum P1:**

| Key | Setting | Type | Description |
|-----|---------|------|-------------|
| 18 | Drum P1 Ka Left | uint16 | Keyboard key code for P1 ka left |
| 19 | Drum P1 Don Left | uint16 | Keyboard key code for P1 don left |
| 20 | Drum P1 Don Right | uint16 | Keyboard key code for P1 don right |
| 21 | Drum P1 Ka Right | uint16 | Keyboard key code for P1 ka right |

**Keyboard Mappings - Drum P2:**

| Key | Setting | Type | Description |
|-----|---------|------|-------------|
| 22 | Drum P2 Ka Left | uint16 | Keyboard key code for P2 ka left |
| 23 | Drum P2 Don Left | uint16 | Keyboard key code for P2 don left |
| 24 | Drum P2 Don Right | uint16 | Keyboard key code for P2 don right |
| 25 | Drum P2 Ka Right | uint16 | Keyboard key code for P2 ka right |

**Keyboard Mappings - Controller:**

| Key | Setting | Type | Description |
|-----|---------|------|-------------|
| 26 | Controller Up | uint16 | Keyboard key code for D-pad up |
| 27 | Controller Down | uint16 | Keyboard key code for D-pad down |
| 28 | Controller Left | uint16 | Keyboard key code for D-pad left |
| 29 | Controller Right | uint16 | Keyboard key code for D-pad right |
| 30 | Controller North | uint16 | Keyboard key code for North button (Y/Triangle) |
| 31 | Controller East | uint16 | Keyboard key code for East button (B/Circle) |
| 32 | Controller South | uint16 | Keyboard key code for South button (A/Cross) |
| 33 | Controller West | uint16 | Keyboard key code for West button (X/Square) |
| 34 | Controller L | uint16 | Keyboard key code for L button |
| 35 | Controller R | uint16 | Keyboard key code for R button |
| 36 | Controller Start | uint16 | Keyboard key code for Start |
| 37 | Controller Select | uint16 | Keyboard key code for Select |
| 38 | Controller Home | uint16 | Keyboard key code for Home |
| 39 | Controller Share | uint16 | Keyboard key code for Share |
| 40 | Controller L3 | uint16 | Keyboard key code for L3 |
| 41 | Controller R3 | uint16 | Keyboard key code for R3 |

**ADC Channel Mappings:**

| Key | Setting | Type | Description |
|-----|---------|------|-------------|
| 42 | ADC Channel Don Left | uint16 | ADC channel number for don left (0-3) |
| 43 | ADC Channel Ka Left | uint16 | ADC channel number for ka left (0-3) |
| 44 | ADC Channel Don Right | uint16 | ADC channel number for don right (0-3) |
| 45 | ADC Channel Ka Right | uint16 | ADC channel number for ka right (0-3) |

**Note:** All 46 keys (0-45) can be configured via serial protocol using command-line tools or custom scripts.

## Usage Examples

### Using the Python Test Script

```bash
# Install pyserial if not already installed
pip install pyserial

# Read all current settings
python test_serial_config.py COM3 read

# Set don left threshold to 1000
python test_serial_config.py COM3 set 0 1000

# Save settings to flash
python test_serial_config.py COM3 save

# Reload settings from flash
python test_serial_config.py COM3 reload
```

### Manual Usage (Serial Terminal)

1. Connect to the COM port at 115200 baud
2. Send commands as plain text:

```
1000          # Read all settings
1002          # Enter write mode
0:1000        # Set don left threshold to 1000
1:900         # Set ka left threshold to 900
1001          # Save to flash
```

### Write Mode Details

When you send **1002**, the device enters write mode and accepts key:value pairs:
- Format: `key:value` (e.g., `0:800`)
- Multiple values can be sent space-separated: `0:800 1:900 2:800`
- Write mode exits automatically after receiving at least one value
- Values are applied immediately but not saved to flash until you send **1001**
- The device supports **46 keys total** (0-45)

### Streaming Mode

When you send **2000**, the device starts streaming sensor data:
- **Format:** CSV with triggered flags and raw ADC values
- **Example:** `F,200,T,1000,F,300,F,254` (ka_left, don_left, don_right, ka_right)
- **Rate:** ~100Hz (10ms between samples)
- **Stop:** Send **2001** to stop streaming

**CSV Format:**
```
triggered_ka_left,ka_raw,triggered_don_left,don_left_raw,triggered_don_right,don_right_raw,triggered_ka_right,ka_right_raw
```

**Usage:**
```bash
# Start streaming
python test_serial_config.py COM3 stream

# (In your script, read lines from serial port and parse CSV)

# Stop streaming
python test_serial_config.py COM3 stopstream
```

## Integration with Existing System

The serial configuration system:
- ✅ Integrates with existing `SettingsStore` class
- ✅ Respects the same value ranges and types
- ✅ Works alongside OLED menu system
- ✅ Changes made via serial are immediately applied to the Drum peripheral
- ✅ Changes persist across reboots when saved with command 1001
- ⚠️ Does NOT require menu system or cause settings conflicts
- ⚠️ Changes take effect immediately but must be saved manually

## Feature Summary

| Feature | Description |
|---------|-------------|
| Protocol | Commands 1000-1004 (config), 2000-2001 (streaming) |
| Parameters | 46 configurable keys (0-45) |
| Value Storage | uint32_t for thresholds, uint16_t for other settings |
| Integration | Integrated with SettingsStore for persistence |
| Persistence | Automatic flash wear leveling |
| Streaming Mode | Commands 2000/2001 for live sensor data (~100Hz) |
| Features | Thresholds, debounce, double trigger, cutoffs, keyboard mappings, ADC channels |

## USB Mode Compatibility

Serial configuration requires a USB mode that includes CDC (serial) interface:

✅ **Keyboard P1/P2** - Includes CDC + HID (recommended for PC testing)
✅ **Debug** - CDC only (no controller functionality)
❌ **Other modes** (Switch, PS4, Xbox, MIDI) - HID/Vendor only, no CDC

**To use serial configuration:**
1. Switch to Keyboard mode via OLED menu
2. Connect to PC - it will appear as both a keyboard and COM port
3. Use the web configurator or Python script
4. Settings persist across all modes!

## Troubleshooting

**Settings not persisting:**
- Make sure to send command **1001** to save to flash
- Verify "Settings saved" message appears

**No response from device:**
- Check COM port is correct
- Ensure device is properly connected
- Try reconnecting USB cable
- Verify baudrate is 115200

**Changes not taking effect:**
- Settings are applied immediately when written
- If using the menu system simultaneously, menu changes may override serial changes
- Exit menu before using serial configuration

## Implementation Details

- **Location**: `src/utils/SerialConfig.cpp` and `include/utils/SerialConfig.h`
- **Main Loop**: Called from Core 0 main loop via `serial_config.processSerial()`
- **Non-blocking**: All operations are non-blocking and safe for main loop
- **Thread-safe**: Uses existing SettingsStore which handles thread safety
