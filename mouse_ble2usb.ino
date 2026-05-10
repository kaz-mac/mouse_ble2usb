/*
  mouse_ble2usb.ino
  BLEマウス to USBコンバーター for M5Stack AtomS3U
  AtomS3UとBLEマウスをペアリングして、USB経由でBLEマウスを使えるようにします

  Copyright (c) 2026 Kaz  (https://akibabara.com/blog/)
  Released under the MIT license.
  see https://opensource.org/licenses/MIT
*/
#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLERemoteDescriptor.h>
#include <BLEScan.h>
#include <BLESecurity.h>
#include <esp_system.h>
#include <map>
#include "USB.h"
#include "USBHIDMouse.h"

#define DEBUG_LOG 0  // Set to 1 to enable verbose mouse report logs.

// AtomS3U pins
static constexpr int PIN_DEBUG_UART_RX = 14;  // 2.54mm header G14, connect to USB-UART TX if needed
static constexpr int PIN_DEBUG_UART_TX = 17;  // 2.54mm header G17, connect to USB-UART RX
static constexpr int ATOM_RGB_LED_PIN = 35;   // Built-in WS2812
static constexpr int ATOM_BUTTON_PIN = 41;    // Built-in button, active low

static constexpr uint32_t DEBUG_BAUD = 115200;
static constexpr uint32_t BLE_SCAN_SECONDS = 20;
static constexpr uint32_t BLE_SCAN_CHUNK_SECONDS = 1;
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;
static constexpr uint32_t STATUS_LED_TASK_INTERVAL_MS = 20;
static constexpr size_t MAX_MOUSE_REPORT_LENGTH = 16;
static constexpr size_t MOUSE_REPORT_QUEUE_SIZE = 12;
static constexpr size_t HID_INPUT_REPORT_INFO_SIZE = 12;
static constexpr size_t HID_REPORT_FIELD_SIZE = 32;

static BLEUUID HID_SERVICE_UUID((uint16_t)0x1812);
static BLEUUID BOOT_MOUSE_INPUT_UUID((uint16_t)0x2A33);
static BLEUUID HID_REPORT_UUID((uint16_t)0x2A4D);
static BLEUUID HID_REPORT_MAP_UUID((uint16_t)0x2A4B);
static BLEUUID HID_PROTOCOL_MODE_UUID((uint16_t)0x2A4E);
static BLEUUID CLIENT_CHARACTERISTIC_CONFIG_UUID((uint16_t)0x2902);
static BLEUUID HID_REPORT_REFERENCE_UUID((uint16_t)0x2908);

enum BridgeState {
  STATE_IDLE,
  STATE_SCANNING,
  STATE_CONNECTING,
  STATE_CONNECTED,
  STATE_ERROR,
};

USBHIDMouse Mouse;
HardwareSerial DebugSerial(1);

BLEAdvertisedDevice *targetDevice = nullptr;
BLEClient *bleClient = nullptr;
TaskHandle_t statusLedTaskHandle = nullptr;

volatile BridgeState bridgeState = STATE_IDLE;
volatile bool scanRequested = false;
volatile bool targetFound = false;

uint8_t lastUsbButtons = 0;
uint32_t droppedMouseReports = 0;

struct MouseReport {
  uint8_t data[MAX_MOUSE_REPORT_LENGTH];
  size_t length;
  uint16_t sourceHandle;
};

struct HidInputReportInfo {
  uint16_t handle;
  uint8_t reportId;
  uint8_t reportType;
};

enum HidReportFieldKind {
  HID_FIELD_BUTTON,
  HID_FIELD_X,
  HID_FIELD_Y,
  HID_FIELD_WHEEL,
};

struct HidReportField {
  uint8_t reportId;
  uint16_t bitOffset;
  uint8_t bitSize;
  HidReportFieldKind kind;
  uint8_t buttonMask;
  bool isSigned;
};

MouseReport mouseReportQueue[MOUSE_REPORT_QUEUE_SIZE];
volatile size_t mouseReportReadIndex = 0;
volatile size_t mouseReportWriteIndex = 0;
portMUX_TYPE mouseReportMux = portMUX_INITIALIZER_UNLOCKED;

HidInputReportInfo hidInputReportInfos[HID_INPUT_REPORT_INFO_SIZE];
size_t hidInputReportInfoCount = 0;

HidReportField hidReportFields[HID_REPORT_FIELD_SIZE];
size_t hidReportFieldCount = 0;

void setLed(uint8_t red, uint8_t green, uint8_t blue) {
  neopixelWrite(ATOM_RGB_LED_PIN, red, green, blue);
}

void logLine(const char *message) {
  DebugSerial.println(message);
}

void logAddress(const char *prefix, BLEAdvertisedDevice *device) {
  DebugSerial.print(prefix);
  DebugSerial.println(device->getAddress().toString().c_str());
}

void logResetReason() {
  DebugSerial.print("Reset reason=");
  DebugSerial.println(static_cast<int>(esp_reset_reason()));
}

void logHexReport(uint16_t sourceHandle, const uint8_t *data, size_t length) {
  if (!DEBUG_LOG) {
    return;
  }

  DebugSerial.print("BLE report handle=0x");
  DebugSerial.print(sourceHandle, HEX);
  DebugSerial.print(" len=");
  DebugSerial.print(length);
  DebugSerial.print(" data=");

  for (size_t i = 0; i < length; ++i) {
    if (data[i] < 0x10) {
      DebugSerial.print('0');
    }
    DebugSerial.print(data[i], HEX);
    if (i + 1 < length) {
      DebugSerial.print(' ');
    }
  }

  DebugSerial.println();
}

bool enqueueMouseReport(uint16_t sourceHandle, const uint8_t *data, size_t length) {
  const size_t copyLength = min(length, MAX_MOUSE_REPORT_LENGTH);

  portENTER_CRITICAL(&mouseReportMux);

  size_t nextWriteIndex = (mouseReportWriteIndex + 1) % MOUSE_REPORT_QUEUE_SIZE;
  if (nextWriteIndex == mouseReportReadIndex) {
    mouseReportReadIndex = (mouseReportReadIndex + 1) % MOUSE_REPORT_QUEUE_SIZE;
    ++droppedMouseReports;
  }

  MouseReport &report = mouseReportQueue[mouseReportWriteIndex];
  memcpy(report.data, data, copyLength);
  report.length = copyLength;
  report.sourceHandle = sourceHandle;
  mouseReportWriteIndex = nextWriteIndex;

  portEXIT_CRITICAL(&mouseReportMux);

  return true;
}

bool dequeueMouseReport(MouseReport &report) {
  bool available = false;

  portENTER_CRITICAL(&mouseReportMux);

  if (mouseReportReadIndex != mouseReportWriteIndex) {
    report = mouseReportQueue[mouseReportReadIndex];
    mouseReportReadIndex = (mouseReportReadIndex + 1) % MOUSE_REPORT_QUEUE_SIZE;
    available = true;
  }

  portEXIT_CRITICAL(&mouseReportMux);

  return available;
}

void resetMouseReportQueue() {
  portENTER_CRITICAL(&mouseReportMux);
  mouseReportReadIndex = 0;
  mouseReportWriteIndex = 0;
  droppedMouseReports = 0;
  portEXIT_CRITICAL(&mouseReportMux);
}

void resetHidInputReportInfos() {
  hidInputReportInfoCount = 0;
}

void resetHidReportFields() {
  hidReportFieldCount = 0;
}

void addHidReportField(uint8_t reportId, uint16_t bitOffset, uint8_t bitSize,
                       HidReportFieldKind kind, uint8_t buttonMask, bool isSigned) {
  if (hidReportFieldCount >= HID_REPORT_FIELD_SIZE || bitSize == 0 || bitSize > 16) {
    return;
  }

  hidReportFields[hidReportFieldCount++] = {reportId, bitOffset, bitSize, kind, buttonMask, isSigned};
}

void registerHidInputReportInfo(uint16_t handle, uint8_t reportId, uint8_t reportType) {
  for (size_t i = 0; i < hidInputReportInfoCount; ++i) {
    if (hidInputReportInfos[i].handle == handle) {
      hidInputReportInfos[i].reportId = reportId;
      hidInputReportInfos[i].reportType = reportType;
      return;
    }
  }

  if (hidInputReportInfoCount >= HID_INPUT_REPORT_INFO_SIZE) {
    return;
  }

  hidInputReportInfos[hidInputReportInfoCount++] = {handle, reportId, reportType};
}

const HidInputReportInfo *findHidInputReportInfo(uint16_t handle) {
  for (size_t i = 0; i < hidInputReportInfoCount; ++i) {
    if (hidInputReportInfos[i].handle == handle) {
      return &hidInputReportInfos[i];
    }
  }

  return nullptr;
}

bool hasMovementOnlyReport() {
  for (size_t i = 0; i < hidInputReportInfoCount; ++i) {
    if (hidInputReportInfos[i].reportId == 2 && hidInputReportInfos[i].reportType == 1) {
      return true;
    }
  }

  return false;
}

bool hasHidReportFields(uint8_t reportId) {
  for (size_t i = 0; i < hidReportFieldCount; ++i) {
    if (hidReportFields[i].reportId == reportId) {
      return true;
    }
  }

  return false;
}

uint32_t readReportBits(const uint8_t *data, size_t length, uint16_t bitOffset, uint8_t bitSize) {
  uint32_t value = 0;

  for (uint8_t i = 0; i < bitSize; ++i) {
    const uint16_t sourceBit = bitOffset + i;
    const size_t sourceByte = sourceBit / 8;
    if (sourceByte >= length) {
      break;
    }

    if ((data[sourceByte] >> (sourceBit % 8)) & 0x01) {
      value |= (1UL << i);
    }
  }

  return value;
}

int16_t signExtendReportValue(uint32_t value, uint8_t bitSize) {
  if (bitSize == 0 || bitSize >= 16 || ((value & (1UL << (bitSize - 1))) == 0)) {
    return static_cast<int16_t>(value);
  }

  const uint32_t signMask = 0xFFFFUL << bitSize;
  return static_cast<int16_t>(value | signMask);
}

int8_t clampUsbDelta(int16_t value) {
  if (value > 127) {
    return 127;
  }

  if (value < -127) {
    return -127;
  }

  return static_cast<int8_t>(value);
}

struct ParsedMouseReport {
  uint8_t buttons;
  int16_t dx;
  int16_t dy;
  int8_t wheel;
  const char *formatName;
};

bool parseMousePayload(const uint8_t *payload, size_t length, bool hasReportId, ParsedMouseReport &parsed);
bool parseMouseReportFromMap(uint16_t sourceHandle, const uint8_t *data, size_t length, ParsedMouseReport &parsed);
bool parseMouseReport(uint16_t sourceHandle, const uint8_t *data, size_t length, ParsedMouseReport &parsed);
void updateUsbButtons(uint8_t buttons);

bool looksLikeButtonByte(uint8_t value) {
  // This bridge supports left/right, but allow other low button bits while
  // rejecting values that are clearly not a button bitmap.
  return (value & 0xF0) == 0;
}

bool parseMouseReportFromMap(uint16_t sourceHandle, const uint8_t *data, size_t length, ParsedMouseReport &parsed) {
  const HidInputReportInfo *reportInfo = findHidInputReportInfo(sourceHandle);
  uint8_t reportId = 0;
  const uint8_t *payload = data;
  size_t payloadLength = length;

  if (reportInfo != nullptr && reportInfo->reportType == 1) {
    reportId = reportInfo->reportId;
  } else if (length > 1 && hasHidReportFields(data[0])) {
    reportId = data[0];
    payload = data + 1;
    payloadLength = length - 1;
  }

  if (!hasHidReportFields(reportId)) {
    return false;
  }

  parsed.buttons = 0;
  parsed.dx = 0;
  parsed.dy = 0;
  parsed.wheel = 0;
  parsed.formatName = "hid-report-map";

  bool usedField = false;
  for (size_t i = 0; i < hidReportFieldCount; ++i) {
    const HidReportField &field = hidReportFields[i];
    if (field.reportId != reportId || field.bitOffset + field.bitSize > payloadLength * 8) {
      continue;
    }

    const uint32_t rawValue = readReportBits(payload, payloadLength, field.bitOffset, field.bitSize);
    const int16_t value = field.isSigned ? signExtendReportValue(rawValue, field.bitSize) : static_cast<int16_t>(rawValue);
    usedField = true;

    switch (field.kind) {
      case HID_FIELD_BUTTON:
        if (rawValue != 0) {
          parsed.buttons |= field.buttonMask;
        }
        break;
      case HID_FIELD_X:
        parsed.dx += value;
        break;
      case HID_FIELD_Y:
        parsed.dy += value;
        break;
      case HID_FIELD_WHEEL:
        parsed.wheel += static_cast<int8_t>(value);
        break;
    }
  }

  return usedField;
}

bool parseMousePayload(const uint8_t *payload, size_t length, bool hasReportId, ParsedMouseReport &parsed) {
  if (length < 3 || !looksLikeButtonByte(payload[0])) {
    return false;
  }

  parsed.buttons = payload[0];
  parsed.dx = 0;
  parsed.dy = 0;
  parsed.wheel = 0;

  if (length == 3 || length == 4) {
    parsed.dx = static_cast<int8_t>(payload[1]);
    parsed.dy = static_cast<int8_t>(payload[2]);
    parsed.wheel = (length == 4) ? static_cast<int8_t>(payload[3]) : 0;
    parsed.formatName = hasReportId ? "report-id + 8bit" : "8bit";
    return true;
  }

  if (length == 5 || length >= 6) {
    parsed.dx = static_cast<int16_t>(static_cast<uint16_t>(payload[1]) |
                                     (static_cast<uint16_t>(payload[2]) << 8));
    parsed.dy = static_cast<int16_t>(static_cast<uint16_t>(payload[3]) |
                                     (static_cast<uint16_t>(payload[4]) << 8));
    parsed.wheel = (length >= 6) ? static_cast<int8_t>(payload[5]) : 0;
    parsed.formatName = hasReportId ? "report-id + 16bit" : "16bit";
    return true;
  }

  return false;
}

bool parseMouseReport(uint16_t sourceHandle, const uint8_t *data, size_t length, ParsedMouseReport &parsed) {
  if (length < 3) {
    return false;
  }

  if (parseMouseReportFromMap(sourceHandle, data, length, parsed)) {
    return true;
  }

  const HidInputReportInfo *reportInfo = findHidInputReportInfo(sourceHandle);
  if (reportInfo != nullptr && reportInfo->reportId == 1 && reportInfo->reportType == 1 &&
      length == 3 && hasMovementOnlyReport()) {
    parsed.buttons = data[0];
    parsed.dx = 0;
    parsed.dy = 0;
    parsed.wheel = static_cast<int8_t>(data[1]);
    parsed.formatName = "buttons-wheel";
    return true;
  }

  if (reportInfo != nullptr && reportInfo->reportId == 2 && reportInfo->reportType == 1 && length == 4) {
    parsed.buttons = 0;
    parsed.dx = static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                     (static_cast<uint16_t>(data[1]) << 8));
    parsed.dy = static_cast<int16_t>(static_cast<uint16_t>(data[2]) |
                                     (static_cast<uint16_t>(data[3]) << 8));
    parsed.wheel = 0;
    parsed.formatName = "movement-only 16bit";
    return true;
  }

  // Ambiguous lengths can be either "report ID + 8bit" or a plain 16bit
  // report. If byte 1 looks like a button bitmap, prefer the report-ID form.
  if ((length == 5 || length == 7) && data[0] != 0 && looksLikeButtonByte(data[1])) {
    if (parseMousePayload(data + 1, length - 1, true, parsed)) {
      return true;
    }
  }

  if (parseMousePayload(data, length, false, parsed)) {
    return true;
  }

  if (length > 3 && data[0] != 0 && looksLikeButtonByte(data[1])) {
    return parseMousePayload(data + 1, length - 1, true, parsed);
  }

  return false;
}

void disconnectMouse() {
  logLine("Disconnecting BLE mouse.");
  updateUsbButtons(0);
  resetMouseReportQueue();
  resetHidInputReportInfos();
  resetHidReportFields();

  if (bleClient != nullptr) {
    if (bleClient->isConnected()) {
      bleClient->disconnect();
      delay(100);
    }

    delete bleClient;
    bleClient = nullptr;
  }

  if (targetDevice != nullptr) {
    delete targetDevice;
    targetDevice = nullptr;
  }

  targetFound = false;
  lastUsbButtons = 0;
  bridgeState = STATE_IDLE;
  logLine("BLE mouse disconnected. Press button to scan again.");
}

void updateUsbButtons(uint8_t buttons) {
  const uint8_t supportedButtons = buttons & MOUSE_ALL;  // left, right, middle, backward, forward

  for (uint8_t mask = MOUSE_LEFT; mask <= MOUSE_FORWARD; mask <<= 1) {
    if ((supportedButtons & mask) && !(lastUsbButtons & mask)) {
      Mouse.press(mask);
    } else if (!(supportedButtons & mask) && (lastUsbButtons & mask)) {
      Mouse.release(mask);
    }
  }

  lastUsbButtons = supportedButtons;
}

void handleMouseReport(uint16_t sourceHandle, uint8_t *data, size_t length) {
  if (length < 3) {
    DebugSerial.print("Ignored short BLE report len=");
    DebugSerial.println(length);
    return;
  }

  ParsedMouseReport parsed;
  if (!parseMouseReport(sourceHandle, data, length, parsed)) {
    DebugSerial.print("Ignored unsupported BLE report len=");
    DebugSerial.println(length);
    return;
  }

  if (DEBUG_LOG) {
    DebugSerial.print("Parsed mouse format=");
    DebugSerial.print(parsed.formatName);
    DebugSerial.print(" buttons=");
    DebugSerial.print(parsed.buttons & MOUSE_ALL);
    DebugSerial.print(" dx=");
    DebugSerial.print(parsed.dx);
    DebugSerial.print(" dy=");
    DebugSerial.print(parsed.dy);
    DebugSerial.print(" wheel=");
    DebugSerial.println(parsed.wheel);
  }

  updateUsbButtons(parsed.buttons);

  if (parsed.dx != 0 || parsed.dy != 0 || parsed.wheel != 0) {
    const int8_t usbDx = clampUsbDelta(parsed.dx);
    const int8_t usbDy = clampUsbDelta(parsed.dy);
    if (DEBUG_LOG) {
      DebugSerial.print("USB mouse move dx=");
      DebugSerial.print(usbDx);
      DebugSerial.print(" dy=");
      DebugSerial.print(usbDy);
      DebugSerial.print(" wheel=");
      DebugSerial.println(parsed.wheel);
    }
    Mouse.move(usbDx, usbDy, parsed.wheel);
  }
}

void mouseNotifyCallback(BLERemoteCharacteristic *characteristic, uint8_t *data, size_t length, bool isNotify) {
  (void)isNotify;
  const uint16_t sourceHandle = (characteristic != nullptr) ? characteristic->getHandle() : 0;
  enqueueMouseReport(sourceHandle, data, length);
}

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *client) override {
    (void)client;
    logLine("BLE connected.");
  }

  void onDisconnect(BLEClient *client) override {
    (void)client;
    logLine("BLE disconnected.");
    updateUsbButtons(0);
    resetMouseReportQueue();
    resetHidInputReportInfos();
    resetHidReportFields();
    lastUsbButtons = 0;
    bridgeState = STATE_IDLE;
  }
};

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (targetFound) {
      return;
    }

    if (!advertisedDevice.haveServiceUUID() || !advertisedDevice.isAdvertisingService(HID_SERVICE_UUID)) {
      return;
    }

    if (targetDevice != nullptr) {
      delete targetDevice;
    }

    targetDevice = new BLEAdvertisedDevice(advertisedDevice);
    targetFound = true;

    logAddress("Found BLE HID device: ", targetDevice);
  }
};

ClientCallbacks clientCallbacks;
AdvertisedDeviceCallbacks advertisedDeviceCallbacks;

uint32_t readHidItemUnsigned(const uint8_t *data, size_t size) {
  uint32_t value = 0;
  for (size_t i = 0; i < size; ++i) {
    value |= static_cast<uint32_t>(data[i]) << (8 * i);
  }

  return value;
}

int32_t readHidItemSigned(const uint8_t *data, size_t size) {
  const uint32_t value = readHidItemUnsigned(data, size);
  if (size == 0) {
    return 0;
  }

  const uint8_t bits = size * 8;
  if (bits >= 32) {
    return static_cast<int32_t>(value);
  }

  if ((value & (1UL << (bits - 1))) == 0) {
    return static_cast<int32_t>(value);
  }

  return static_cast<int32_t>(value | (0xFFFFFFFFUL << bits));
}

uint32_t makeFullUsage(uint16_t usagePage, uint32_t usageValue, size_t itemSize) {
  if (itemSize >= 4) {
    return usageValue;
  }

  return (static_cast<uint32_t>(usagePage) << 16) | (usageValue & 0xFFFF);
}

void addFieldForUsage(uint8_t reportId, uint16_t bitOffset, uint8_t bitSize,
                      uint32_t fullUsage, int32_t logicalMinimum) {
  const uint16_t usagePage = static_cast<uint16_t>(fullUsage >> 16);
  const uint16_t usage = static_cast<uint16_t>(fullUsage & 0xFFFF);

  if (usagePage == 0x09 && usage >= 1 && usage <= 5) {
    addHidReportField(reportId, bitOffset, bitSize, HID_FIELD_BUTTON, static_cast<uint8_t>(1 << (usage - 1)), false);
  } else if (usagePage == 0x01 && usage == 0x30) {
    addHidReportField(reportId, bitOffset, bitSize, HID_FIELD_X, 0, logicalMinimum < 0);
  } else if (usagePage == 0x01 && usage == 0x31) {
    addHidReportField(reportId, bitOffset, bitSize, HID_FIELD_Y, 0, logicalMinimum < 0);
  } else if (usagePage == 0x01 && usage == 0x38) {
    addHidReportField(reportId, bitOffset, bitSize, HID_FIELD_WHEEL, 0, logicalMinimum < 0);
  }
}

void parseHidReportMap(const uint8_t *reportMap, size_t length) {
  resetHidReportFields();

  uint16_t usagePage = 0;
  int32_t logicalMinimum = 0;
  uint8_t reportSize = 0;
  uint8_t reportCount = 0;
  uint8_t reportId = 0;
  uint16_t bitOffsets[256] = {0};

  uint32_t localUsages[16];
  size_t localUsageCount = 0;
  uint32_t usageMinimum = 0;
  uint32_t usageMaximum = 0;
  bool hasUsageMinimum = false;
  bool hasUsageMaximum = false;

  size_t index = 0;
  while (index < length) {
    const uint8_t prefix = reportMap[index++];
    if (prefix == 0xFE) {
      if (index + 2 > length) {
        break;
      }
      const uint8_t longItemSize = reportMap[index++];
      ++index;  // long item tag
      index += longItemSize;
      continue;
    }

    const uint8_t sizeCode = prefix & 0x03;
    const size_t itemSize = (sizeCode == 3) ? 4 : sizeCode;
    const uint8_t itemType = (prefix >> 2) & 0x03;
    const uint8_t itemTag = (prefix >> 4) & 0x0F;
    if (index + itemSize > length) {
      break;
    }

    const uint8_t *itemData = reportMap + index;
    const uint32_t unsignedValue = readHidItemUnsigned(itemData, itemSize);
    const int32_t signedValue = readHidItemSigned(itemData, itemSize);
    index += itemSize;

    if (itemType == 0x01) {
      switch (itemTag) {
        case 0x00:
          usagePage = static_cast<uint16_t>(unsignedValue);
          break;
        case 0x01:
          logicalMinimum = signedValue;
          break;
        case 0x07:
          reportSize = static_cast<uint8_t>(unsignedValue);
          break;
        case 0x08:
          reportId = static_cast<uint8_t>(unsignedValue);
          break;
        case 0x09:
          reportCount = static_cast<uint8_t>(unsignedValue);
          break;
      }
    } else if (itemType == 0x02) {
      switch (itemTag) {
        case 0x00:
          if (localUsageCount < 16) {
            localUsages[localUsageCount++] = makeFullUsage(usagePage, unsignedValue, itemSize);
          }
          break;
        case 0x01:
          usageMinimum = makeFullUsage(usagePage, unsignedValue, itemSize);
          hasUsageMinimum = true;
          break;
        case 0x02:
          usageMaximum = makeFullUsage(usagePage, unsignedValue, itemSize);
          hasUsageMaximum = true;
          break;
      }
    } else if (itemType == 0x00 && itemTag == 0x08) {
      const bool isConstant = (unsignedValue & 0x01) != 0;
      for (uint8_t fieldIndex = 0; fieldIndex < reportCount; ++fieldIndex) {
        uint32_t fullUsage = 0;
        if (fieldIndex < localUsageCount) {
          fullUsage = localUsages[fieldIndex];
        } else if (hasUsageMinimum && hasUsageMaximum) {
          fullUsage = usageMinimum + fieldIndex;
          if (fullUsage > usageMaximum) {
            fullUsage = 0;
          }
        }

        if (!isConstant && fullUsage != 0) {
          addFieldForUsage(reportId, bitOffsets[reportId], reportSize, fullUsage, logicalMinimum);
        }

        bitOffsets[reportId] += reportSize;
      }

      localUsageCount = 0;
      hasUsageMinimum = false;
      hasUsageMaximum = false;
    } else if (itemType == 0x00) {
      localUsageCount = 0;
      hasUsageMinimum = false;
      hasUsageMaximum = false;
    }
  }

  DebugSerial.print("Parsed HID report fields=");
  DebugSerial.println(hidReportFieldCount);
}

void readAndParseHidReportMap(BLERemoteService *hidService) {
  BLERemoteCharacteristic *reportMap = hidService->getCharacteristic(HID_REPORT_MAP_UUID);
  if (reportMap == nullptr || !reportMap->canRead()) {
    logLine("HID Report Map not readable. Using fallback parser.");
    resetHidReportFields();
    return;
  }

  String reportMapValue = reportMap->readValue();
  DebugSerial.print("HID Report Map len=");
  DebugSerial.println(reportMapValue.length());
  parseHidReportMap(reinterpret_cast<const uint8_t *>(reportMapValue.c_str()), reportMapValue.length());
}

bool writeReportProtocolMode(BLERemoteService *hidService) {
  BLERemoteCharacteristic *protocolMode = hidService->getCharacteristic(HID_PROTOCOL_MODE_UUID);
  if (protocolMode == nullptr || (!protocolMode->canWrite() && !protocolMode->canWriteNoResponse())) {
    return false;
  }

  uint8_t reportMode = 0x01;
  protocolMode->writeValue(&reportMode, 1, protocolMode->canWrite());
  logLine("Requested HID report protocol mode.");
  return true;
}

bool enableNotifications(BLERemoteCharacteristic *characteristic, const char *label) {
  if (characteristic == nullptr || (!characteristic->canNotify() && !characteristic->canIndicate())) {
    return false;
  }

  DebugSerial.print("Subscribing ");
  DebugSerial.print(label);
  DebugSerial.print(" handle=0x");
  DebugSerial.println(characteristic->getHandle(), HEX);

  const bool useNotify = characteristic->canNotify();
  characteristic->registerForNotify(mouseNotifyCallback, useNotify);

  BLERemoteDescriptor *cccd = characteristic->getDescriptor(CLIENT_CHARACTERISTIC_CONFIG_UUID);
  if (cccd != nullptr) {
    uint8_t enableValue[] = {static_cast<uint8_t>(useNotify ? 0x01 : 0x02), 0x00};
    cccd->writeValue(enableValue, sizeof(enableValue), true);
    DebugSerial.print(useNotify ? "Enabled CCCD notify for " : "Enabled CCCD indicate for ");
    DebugSerial.println(label);
  } else {
    DebugSerial.print("No CCCD descriptor for ");
    DebugSerial.println(label);
  }

  BLERemoteDescriptor *reportReference = characteristic->getDescriptor(HID_REPORT_REFERENCE_UUID);
  if (reportReference != nullptr) {
    String reportReferenceValue = reportReference->readValue();
    if (reportReferenceValue.length() >= 2) {
      const uint8_t reportId = static_cast<uint8_t>(reportReferenceValue[0]);
      const uint8_t reportType = static_cast<uint8_t>(reportReferenceValue[1]);
      registerHidInputReportInfo(characteristic->getHandle(), reportId, reportType);
    }

    DebugSerial.print("Report Reference descriptor exists for ");
    DebugSerial.print(label);
    DebugSerial.print(" value=");
    for (size_t i = 0; i < reportReferenceValue.length(); ++i) {
      const uint8_t value = static_cast<uint8_t>(reportReferenceValue[i]);
      if (value < 0x10) {
        DebugSerial.print('0');
      }
      DebugSerial.print(value, HEX);
      if (i + 1 < reportReferenceValue.length()) {
        DebugSerial.print(' ');
      }
    }
    DebugSerial.println();
  }

  return true;
}

void logCharacteristicInfo(BLERemoteCharacteristic *characteristic) {
  DebugSerial.print("HID characteristic handle=0x");
  DebugSerial.print(characteristic->getHandle(), HEX);
  DebugSerial.print(" uuid=");
  DebugSerial.print(characteristic->getUUID().toString().c_str());
  DebugSerial.print(" props=");
  if (characteristic->canRead()) {
    DebugSerial.print("R");
  }
  if (characteristic->canWrite()) {
    DebugSerial.print("W");
  }
  if (characteristic->canWriteNoResponse()) {
    DebugSerial.print("w");
  }
  if (characteristic->canNotify()) {
    DebugSerial.print("N");
  }
  if (characteristic->canIndicate()) {
    DebugSerial.print("I");
  }
  DebugSerial.println();
}

bool subscribeMouseReports(BLERemoteService *hidService) {
  writeReportProtocolMode(hidService);
  resetHidInputReportInfos();
  readAndParseHidReportMap(hidService);

  bool subscribed = false;

  std::map<uint16_t, BLERemoteCharacteristic *> *characteristics = hidService->getCharacteristicsByHandle();
  for (auto const &entry : *characteristics) {
    BLERemoteCharacteristic *characteristic = entry.second;
    logCharacteristicInfo(characteristic);

    if (enableNotifications(characteristic, "HID notify characteristic")) {
      subscribed = true;
      logLine("Subscribed to HID notify characteristic.");
    }
  }

  return subscribed;
}

void connectToTargetMouse() {
  if (targetDevice == nullptr) {
    bridgeState = STATE_ERROR;
    logLine("No target BLE mouse.");
    return;
  }

  bridgeState = STATE_CONNECTING;
  logAddress("Connecting to ", targetDevice);
  resetMouseReportQueue();
  resetHidInputReportInfos();
  resetHidReportFields();
  lastUsbButtons = 0;

  if (bleClient != nullptr) {
    if (bleClient->isConnected()) {
      bleClient->disconnect();
      delay(100);
    }

    delete bleClient;
    bleClient = nullptr;
  }

  bleClient = BLEDevice::createClient();
  bleClient->setClientCallbacks(&clientCallbacks);

  if (!bleClient->connect(targetDevice)) {
    bridgeState = STATE_ERROR;
    logLine("BLE connect failed.");
    return;
  }

  BLERemoteService *hidService = bleClient->getService(HID_SERVICE_UUID);
  if (hidService == nullptr) {
    bridgeState = STATE_ERROR;
    logLine("HID service not found.");
    bleClient->disconnect();
    return;
  }

  if (!subscribeMouseReports(hidService)) {
    bridgeState = STATE_ERROR;
    logLine("Mouse input report not found.");
    bleClient->disconnect();
    return;
  }

  bridgeState = STATE_CONNECTED;
  logLine("BLE mouse is ready. Forwarding to USB HID.");
}

void scanAndConnectMouse() {
  bridgeState = STATE_SCANNING;
  targetFound = false;

  if (targetDevice != nullptr) {
    delete targetDevice;
    targetDevice = nullptr;
  }

  BLEScan *scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&advertisedDeviceCallbacks, true);
  scan->setActiveScan(true);
  scan->setInterval(80);
  scan->setWindow(60);

  logLine("Scanning for BLE HID mouse. Put the mouse in pairing mode.");
  for (uint32_t elapsedSeconds = 0; elapsedSeconds < BLE_SCAN_SECONDS && !targetFound; elapsedSeconds += BLE_SCAN_CHUNK_SECONDS) {
    scan->start(BLE_SCAN_CHUNK_SECONDS, false);
    scan->clearResults();
    delay(1);
  }

  if (targetFound && targetDevice != nullptr) {
    connectToTargetMouse();
  } else {
    bridgeState = STATE_IDLE;
    logLine("No BLE HID mouse found. Press button to scan again.");
  }
}

void handleButton() {
  static bool lastReading = false;
  static bool stablePressed = false;
  static uint32_t lastChangeMs = 0;

  const bool reading = (digitalRead(ATOM_BUTTON_PIN) == LOW);
  const uint32_t now = millis();

  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = now;
  }

  if (now - lastChangeMs < BUTTON_DEBOUNCE_MS || reading == stablePressed) {
    return;
  }

  stablePressed = reading;
  if (!stablePressed) {
    return;
  }

  if (bridgeState == STATE_CONNECTED) {
    disconnectMouse();
    return;
  }

  if (bridgeState == STATE_SCANNING || bridgeState == STATE_CONNECTING) {
    logLine("BLE scan/connect already in progress.");
    return;
  }

  scanRequested = true;
}

void updateStatusLed() {
  static uint32_t lastBlinkMs = 0;
  static bool blinkOn = false;

  const uint32_t now = millis();
  if (now - lastBlinkMs >= 300) {
    lastBlinkMs = now;
    blinkOn = !blinkOn;
  }

  switch (bridgeState) {
    case STATE_IDLE:
      setLed(30, 0, 0);  // red: waiting for button
      break;
    case STATE_SCANNING:
      setLed(0, 0, blinkOn ? 40 : 0);  // blue blink: scanning
      break;
    case STATE_CONNECTING:
      setLed(40, 24 , 0);  // yellow
      break;
    case STATE_CONNECTED:
      setLed(0, 30, 0);  // green
      break;
    case STATE_ERROR:
      setLed(blinkOn ? 40 : 0, 0, 0);  // red blink
      break;
  }
}

void statusLedTask(void *parameter) {
  (void)parameter;

  for (;;) {
    updateStatusLed();
    vTaskDelay(pdMS_TO_TICKS(STATUS_LED_TASK_INTERVAL_MS));
  }
}

void processMouseReports() {
  MouseReport report;
  while (dequeueMouseReport(report)) {
    logHexReport(report.sourceHandle, report.data, report.length);
    handleMouseReport(report.sourceHandle, report.data, report.length);
  }

  static uint32_t lastDropLogMs = 0;
  const uint32_t now = millis();
  if (DEBUG_LOG && droppedMouseReports > 0 && now - lastDropLogMs > 1000) {
    lastDropLogMs = now;
    DebugSerial.print("Dropped BLE mouse reports=");
    DebugSerial.println(droppedMouseReports);
  }
}

void setupBleSecurity() {
  BLEDevice::init("AtomS3U BLE Mouse Bridge");

  BLESecurity *security = new BLESecurity();
  security->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
  security->setCapability(ESP_IO_CAP_NONE);
  security->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
}

void setup() {
  pinMode(ATOM_BUTTON_PIN, INPUT_PULLUP);
  setLed(0, 0, 0);

  DebugSerial.begin(DEBUG_BAUD, SERIAL_8N1, PIN_DEBUG_UART_RX, PIN_DEBUG_UART_TX);
  delay(300);

  logLine("");
  logLine("AtomS3U BLE mouse to USB HID bridge - Step 2");
  logResetReason();
  logLine("Starting USB HID mouse...");

  Mouse.begin();
  USB.begin();

  logLine("Starting BLE client...");
  setupBleSecurity();

  bridgeState = STATE_IDLE;
  if (statusLedTaskHandle == nullptr) {
    const BaseType_t created = xTaskCreate(statusLedTask, "status_led", 2048, nullptr, 1, &statusLedTaskHandle);
    if (created != pdPASS) {
      statusLedTaskHandle = nullptr;
      logLine("Failed to start status LED task.");
    }
  }

  logLine("Press AtomS3U button to scan and pair with a BLE mouse.");
}

void loop() {
  handleButton();
  processMouseReports();

  if (scanRequested) {
    scanRequested = false;
    scanAndConnectMouse();
  }

  delay(5);
}
