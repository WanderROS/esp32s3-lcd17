#pragma once
/**
 * BLE Wi-Fi Provisioning
 *
 * 使用 ESP-IDF network_provisioning 组件 + BLE 传输层。
 * 配套手机 App：ESP BLE Provisioning（乐鑫官方，iOS/Android）
 *
 * 流程：
 *   1. 首次启动（NVS 无凭证）→ 进入 BLE 配网模式，屏幕显示设备名
 *   2. 手机 App 扫描设备 → 输入 Wi-Fi 密码 → 发送给设备
 *   3. 设备连接 Wi-Fi 成功 → 凭证写入 NVS → 进入正常模式
 *   4. 后续启动直接从 NVS 读取凭证，跳过配网
 *
 * 长按 BOOT 按钮（GPIO 0）3 秒可清除凭证，重新进入配网模式。
 */

#include <WiFi.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <network_provisioning/manager.h>
#include <network_provisioning/scheme_ble.h>

// BLE 广播名前缀（手机 App 搜索时显示）
#ifndef BLE_PROV_DEVICE_NAME
#define BLE_PROV_DEVICE_NAME "AIBOX"
#endif

// 长按清除凭证的 GPIO（BOOT 按钮）
#ifndef BLE_PROV_RESET_GPIO
#define BLE_PROV_RESET_GPIO 0
#endif

// NVS 命名空间 & 键名
#define NVS_NS   "wifi_cred"
#define NVS_SSID "ssid"
#define NVS_PASS "pass"

// ─────────────────────────────────────────────
// 内部实现
// ─────────────────────────────────────────────
namespace _ble_prov_impl {

static Preferences _prefs;

static bool load_credentials(String &ssid, String &pass) {
    _prefs.begin(NVS_NS, true);
    ssid = _prefs.getString(NVS_SSID, "");
    pass = _prefs.getString(NVS_PASS, "");
    _prefs.end();
    return ssid.length() > 0;
}

static void save_credentials(const char *ssid, const char *pass) {
    _prefs.begin(NVS_NS, false);
    _prefs.putString(NVS_SSID, ssid);
    _prefs.putString(NVS_PASS, pass);
    _prefs.end();
}

static void clear_credentials() {
    _prefs.begin(NVS_NS, false);
    _prefs.clear();
    _prefs.end();
    Serial.println("[BLE_PROV] 凭证已清除");
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

// network_provisioning 事件回调
static void prov_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == NETWORK_PROV_EVENT) {
        switch (id) {
            case NETWORK_PROV_START:
                Serial.println("[BLE_PROV] 配网已启动，等待手机连接...");
                break;
            case NETWORK_PROV_WIFI_CRED_RECV: {
                wifi_sta_config_t *cfg = (wifi_sta_config_t *)data;
                Serial.printf("[BLE_PROV] 收到凭证: SSID=%s\n", cfg->ssid);
                save_credentials((const char *)cfg->ssid, (const char *)cfg->password);
                break;
            }
            case NETWORK_PROV_WIFI_CRED_FAIL: {
                network_prov_wifi_sta_fail_reason_t *reason =
                    (network_prov_wifi_sta_fail_reason_t *)data;
                Serial.printf("[BLE_PROV] 凭证验证失败: %s\n",
                    (*reason == NETWORK_PROV_WIFI_STA_AUTH_ERROR) ? "认证错误" : "AP 未找到");
                clear_credentials();
                network_prov_mgr_reset_wifi_sm_state_on_failure();
                break;
            }
            case NETWORK_PROV_WIFI_CRED_SUCCESS:
                Serial.println("[BLE_PROV] Wi-Fi 验证成功！");
                break;
            case NETWORK_PROV_END:
                Serial.println("[BLE_PROV] 配网结束，释放资源");
                network_prov_mgr_deinit();
                break;
            default:
                break;
        }
    }
}

} // namespace _ble_prov_impl

// ─────────────────────────────────────────────
// 公共 API
// ─────────────────────────────────────────────

/**
 * 初始化并执行配网流程（阻塞直到 Wi-Fi 连接成功）。
 *
 * @param status_cb  可选回调，用于更新 UI 状态文字
 * @param timeout_ms 配网等待超时（毫秒），0 = 永久等待
 * @return true  Wi-Fi 已连接
 * @return false 超时或连接失败
 */
static bool ble_prov_connect(void (*status_cb)(const char *) = nullptr,
                             uint32_t timeout_ms = 0) {
    using namespace _ble_prov_impl;

    // 1. 检测长按重置
    if (reset_requested()) {
        clear_credentials();
    }

    // 2. 尝试从 NVS 读取已保存凭证
    String saved_ssid, saved_pass;
    if (load_credentials(saved_ssid, saved_pass)) {
        Serial.printf("[BLE_PROV] 使用已保存凭证: %s\n", saved_ssid.c_str());
        if (status_cb) status_cb("连接 WiFi...");

        WiFi.mode(WIFI_STA);
        WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
        uint32_t deadline = millis() + 15000;
        while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
            delay(200);
            Serial.print(".");
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[BLE_PROV] 已连接，IP: %s\n", WiFi.localIP().toString().c_str());
            // 释放 BT 内存供 SR 任务使用
            btStop();
            esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
            Serial.printf("[BLE_PROV] BT 内存已释放，当前堆: %d\n", esp_get_free_heap_size());
            if (status_cb) status_cb("WiFi 已连接");
            return true;
        }
        Serial.println("[BLE_PROV] 已保存凭证连接失败，进入配网模式");
        clear_credentials();
    }

    // 3. 进入 BLE 配网模式
    Serial.println("[BLE_PROV] 进入 BLE 配网模式");
    if (status_cb) status_cb("BLE 配网中...");

    // 确保 default event loop 已创建（Arduino 框架可能未创建）
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        Serial.printf("[BLE_PROV] event loop 创建失败: 0x%x\n", err);
        return false;
    }

    // 确保 WiFi 已初始化
    WiFi.mode(WIFI_STA);

    ESP_ERROR_CHECK(esp_event_handler_register(
        NETWORK_PROV_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, nullptr));

    network_prov_mgr_config_t config = {
        .scheme               = network_prov_scheme_ble,
        .scheme_event_handler = NETWORK_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };
    ESP_ERROR_CHECK(network_prov_mgr_init(config));

    // 生成唯一设备名（后缀 MAC 后 3 字节）
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char dev_name[32];
    snprintf(dev_name, sizeof(dev_name), "%s_%02X%02X%02X",
             BLE_PROV_DEVICE_NAME, mac[3], mac[4], mac[5]);
    Serial.printf("[BLE_PROV] 设备名: %s\n", dev_name);

    if (status_cb) {
        char msg[64];
        snprintf(msg, sizeof(msg), "BLE: %s", dev_name);
        status_cb(msg);
    }

    // 打印二维码（用 ESP BLE Provisioning App 扫描）
    // 格式: {"ver":"v1","name":"<设备名>","pop":"","transport":"ble"}
    char qr_payload[128];
    snprintf(qr_payload, sizeof(qr_payload),
             "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"\",\"transport\":\"ble\"}",
             dev_name);
    Serial.println("[BLE_PROV] 使用 ESP BLE Provisioning App 扫描以下二维码配网:");
    Serial.printf("[BLE_PROV] QR Payload: %s\n", qr_payload);
    // 生成文本二维码（需要 esp_qrcode 组件，此处打印链接替代）
    Serial.printf("[BLE_PROV] 或在 App 中手动输入设备名: %s\n", dev_name);

    // 启动配网（SECURITY_1，无 PoP）
    ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(
        NETWORK_PROV_SECURITY_1, nullptr, dev_name, nullptr));

    // 4. 等待配网完成
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("~");
        if (timeout_ms > 0 && (millis() - start) > timeout_ms) {
            Serial.println("\n[BLE_PROV] 配网超时！");
            network_prov_mgr_deinit();
            return false;
        }
    }
    Serial.println();
    Serial.printf("[BLE_PROV] 配网成功，IP: %s\n", WiFi.localIP().toString().c_str());

    // 释放 BLE/BT 占用的内存，归还给堆供 SR 任务使用
    btStop();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    Serial.printf("[BLE_PROV] BT 内存已释放，当前堆: %d\n", esp_get_free_heap_size());

    if (status_cb) status_cb("WiFi 已连接");
    return true;
}

/**
 * 清除已保存的 Wi-Fi 凭证（可在设置菜单中调用）
 */
static inline void ble_prov_clear_credentials() {
    _ble_prov_impl::clear_credentials();
}
