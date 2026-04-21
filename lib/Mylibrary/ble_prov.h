#pragma once
/**
 * BLE Wi-Fi Provisioning
 *
 * 使用 Arduino WiFiProv 库（ESP-IDF network_provisioning 的封装）。
 * 配套手机 App：ESP BLE Provisioning（乐鑫官方，iOS/Android）
 *
 * 流程：
 *   1. 首次启动（NVS 无凭证）→ 进入 BLE 配网模式，屏幕显示二维码
 *   2. 手机 App 扫码 → 输入 Wi-Fi 密码 → 发送给设备
 *   3. 设备连接 Wi-Fi 成功 → 凭证自动保存到 NVS
 *   4. 后续启动直接从 NVS 读取凭证，跳过配网
 *
 * 长按 BOOT 按钮（GPIO 0）3 秒可清除凭证，重新进入配网模式。
 */

#include <WiFi.h>
#include <WiFiProv.h>

// BLE 设备名（Espressif App 要求 PROV_ 前缀）
#ifndef BLE_PROV_SERVICE_NAME
#define BLE_PROV_SERVICE_NAME "PROV_AIBOX"
#endif

// Proof of Possession 配对密码（空字符串 = 无密码）
#ifndef BLE_PROV_POP
#define BLE_PROV_POP ""
#endif

// 长按清除凭证的 GPIO（BOOT 按钮）
#ifndef BLE_PROV_RESET_GPIO
#define BLE_PROV_RESET_GPIO 0
#endif

// ─────────────────────────────────────────────
// 内部状态
// ─────────────────────────────────────────────
namespace _ble_prov_impl {

static volatile bool _wifi_connected = false;
static volatile bool _prov_failed    = false;
static volatile bool _prov_ended     = false;

static void (*_on_prov_start_cb)(const char *qr_payload, const char *dev_name) = nullptr;

static void wifi_event_cb(arduino_event_t *event) {
    switch (event->event_id) {
        case ARDUINO_EVENT_PROV_START:
            Serial.println("[BLE_PROV] 配网已启动，等待手机连接...");
            if (_on_prov_start_cb) {
                char qr_payload[128];
                snprintf(qr_payload, sizeof(qr_payload),
                    "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"ble\"}",
                    BLE_PROV_SERVICE_NAME, BLE_PROV_POP);
                _on_prov_start_cb(qr_payload, BLE_PROV_SERVICE_NAME);
            }
            break;
        case ARDUINO_EVENT_PROV_CRED_RECV:
            Serial.printf("[BLE_PROV] 收到凭证: SSID=%s\n",
                (const char *)event->event_info.prov_cred_recv.ssid);
            break;
        case ARDUINO_EVENT_PROV_CRED_FAIL:
            Serial.println("[BLE_PROV] 凭证验证失败");
            _prov_failed = true;
            break;
        case ARDUINO_EVENT_PROV_CRED_SUCCESS:
            Serial.println("[BLE_PROV] Wi-Fi 验证成功！");
            break;
        case ARDUINO_EVENT_PROV_END:
            Serial.println("[BLE_PROV] 配网结束，BLE 资源已释放");
            _prov_ended = true;
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.printf("[BLE_PROV] 已连接，IP: %s\n",
                IPAddress(event->event_info.got_ip.ip_info.ip.addr).toString().c_str());
            _wifi_connected = true;
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            _wifi_connected = false;
            break;
        default:
            break;
    }
}

static bool reset_requested() {
    pinMode(BLE_PROV_RESET_GPIO, INPUT_PULLUP);
    if (digitalRead(BLE_PROV_RESET_GPIO) == LOW) {
        Serial.println("[BLE_PROV] 检测到 BOOT 按钮，等待 3 秒确认重置...");
        uint32_t t = millis();
        while (digitalRead(BLE_PROV_RESET_GPIO) == LOW) {
            if (millis() - t > 3000) {
                Serial.println("[BLE_PROV] 确认重置！");
                return true;
            }
            delay(50);
        }
    }
    return false;
}

} // namespace _ble_prov_impl

// ─────────────────────────────────────────────
// 公共 API
// ─────────────────────────────────────────────

/**
 * 启动配网流程（非阻塞，立即返回）。
 * 内部通过 WiFi.onEvent 异步处理配网事件。
 *
 * @param on_prov_start  进入 BLE 配网时回调（携带 qr_payload 和 dev_name）
 * @param force_reset    true = 强制清除 NVS 凭证重新配网
 */
static void ble_prov_start(
    void (*on_prov_start)(const char *qr_payload, const char *dev_name) = nullptr,
    bool force_reset = false)
{
    using namespace _ble_prov_impl;

    _on_prov_start_cb = on_prov_start;
    _wifi_connected   = false;
    _prov_failed      = false;
    _prov_ended       = false;

    WiFi.onEvent(wifi_event_cb);
    WiFi.mode(WIFI_STA);

    uint8_t uuid[16] = {0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
                        0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02};

    // beginProvision 内部检查 NVS：
    //   有凭证且 reset=false → 直接连接，不启动 BLE
    //   无凭证或 reset=true  → 启动 BLE 配网广播
    WiFiProv.beginProvision(
        NETWORK_PROV_SCHEME_BLE,
        NETWORK_PROV_SCHEME_HANDLER_FREE_BLE,  // 只释放 BLE 协议栈，不动 BT controller
        NETWORK_PROV_SECURITY_1,
        BLE_PROV_POP[0] ? BLE_PROV_POP : nullptr,
        BLE_PROV_SERVICE_NAME,
        nullptr,
        uuid,
        force_reset
    );
}

/**
 * 阻塞等待 WiFi 连接成功。
 *
 * @param tick_cb    等待期间持续调用（用于刷新 UI）
 * @param timeout_ms 超时毫秒，0 = 永久等待
 * @return true  已连接
 * @return false 超时
 */
static bool ble_prov_wait_connected(void (*tick_cb)() = nullptr, uint32_t timeout_ms = 0) {
    uint32_t start = millis();
    while (!_ble_prov_impl::_wifi_connected) {
        delay(50);
        if (tick_cb) tick_cb();
        if (timeout_ms > 0 && (millis() - start) > timeout_ms) {
            Serial.println("[BLE_PROV] 等待连接超时！");
            return false;
        }
    }
    return true;
}

/**
 * 等待 BLE 配网流程彻底结束（PROV_END），确保 BLE 栈资源已释放。
 * 在启动 I2S / ESP_SR 等大内存消费者之前调用。
 *
 * @param tick_cb    等待期间持续调用
 * @param timeout_ms 超时毫秒
 */
static void ble_prov_wait_done(void (*tick_cb)() = nullptr, uint32_t timeout_ms = 10000) {
    uint32_t start = millis();
    while (!_ble_prov_impl::_prov_ended && (millis() - start < timeout_ms)) {
        delay(100);
        if (tick_cb) tick_cb();
    }
    if (_ble_prov_impl::_prov_ended) {
        Serial.println("[BLE_PROV] BLE 资源已释放");
    } else {
        Serial.println("[BLE_PROV] 警告: PROV_END 未收到，BLE 可能仍占用内存");
    }
    // 额外等待让 BLE 栈完成内部清理，再启动 I2S
    delay(500);
    Serial.printf("[BLE_PROV] 当前堆: %d\n", ESP.getFreeHeap());
}

/**
 * 检测长按 BOOT 按钮，返回是否需要重置配网。
 * 在 ble_prov_start() 之前调用。
 */
static inline bool ble_prov_check_reset() {
    return _ble_prov_impl::reset_requested();
}

/**
 * 清除已保存的 Wi-Fi 凭证并重启（重启后进入配网模式）。
 */
static inline void ble_prov_clear_credentials() {
    Serial.println("[BLE_PROV] 清除凭证并重启...");
    WiFi.disconnect(true, true);
    delay(500);
    esp_restart();
}
