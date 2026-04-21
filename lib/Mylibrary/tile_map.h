#pragma once
/**
 * tile_map.h — 在线瓦片地图下载 + SD 缓存
 *
 * 瓦片来源：ArcGIS World Street Map（JPEG，无需 Key）
 * 格式：JPEG，256×256 px
 * 缓存路径：SD 卡 /tiles/{z}_{x}_{y}.jpg
 */

#include <Arduino.h>
#include <HTTPClient.h>
#include <SD_MMC.h>
#include <math.h>

// ── 瓦片参数 ──────────────────────────────────────────────
#define TILE_SIZE   256
#define MAP_ZOOM_DEFAULT 13   // 13 级覆盖范围更广，ArcGIS 中国区域数据完整

// ArcGIS 卫星影像瓦片（JPEG，无需 Key，中国覆盖完整）
#define TILE_URL_FMT \
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%d/%d/%d"

#define TILE_CACHE_DIR "/tiles_v2"  // 换源后用新目录，避免读到旧缓存
#define TILE_MAX_BYTES (48 * 1024)   // 单张瓦片最大 48 KB

// ── 坐标转换 ──────────────────────────────────────────────
struct TileXY { int x, y, z; };

// WGS-84 经纬度 → 瓦片编号（OSM/高德通用 Web Mercator）
inline TileXY latLonToTile(double lat, double lon, int zoom) {
    double n = pow(2.0, zoom);
    int tx = (int)floor((lon + 180.0) / 360.0 * n);
    double lat_r = lat * M_PI / 180.0;
    int ty = (int)floor((1.0 - log(tan(lat_r) + 1.0 / cos(lat_r)) / M_PI) / 2.0 * n);
    return {tx, ty, zoom};
}

// 经纬度在中心瓦片内的像素偏移（0~255）
inline void tilePixelOffset(double lat, double lon, int zoom, int &px, int &py) {
    double n = pow(2.0, zoom);
    double fx = (lon + 180.0) / 360.0 * n;
    double lat_r = lat * M_PI / 180.0;
    double fy = (1.0 - log(tan(lat_r) + 1.0 / cos(lat_r)) / M_PI) / 2.0 * n;
    TileXY t = latLonToTile(lat, lon, zoom);
    px = (int)((fx - t.x) * TILE_SIZE);
    py = (int)((fy - t.y) * TILE_SIZE);
}

// ── TileMap 类 ────────────────────────────────────────────
class TileMap {
public:
    bool sdReady = false;

    void begin() {
        // SD_MMC 由 main.cpp 已初始化，这里只确保缓存目录存在
        if (SD_MMC.cardType() != CARD_NONE) {
            sdReady = true;
            if (!SD_MMC.exists(TILE_CACHE_DIR)) {
                SD_MMC.mkdir(TILE_CACHE_DIR);
            }
            Serial.println("[MAP] SD 缓存目录就绪: " TILE_CACHE_DIR);
        } else {
            Serial.println("[MAP] 警告：SD 卡未就绪，瓦片将不缓存");
        }
    }

    /**
     * 获取瓦片 JPEG 数据（优先 SD 缓存，否则 WiFi 下载）
     * @param outLen  返回实际字节数，失败时为 0
     * @return  PSRAM 中的 uint8_t 缓冲区，调用方负责 free()；失败返回 nullptr
     */
    uint8_t* getTile(int x, int y, int z, size_t &outLen) {
        char path[64];
        snprintf(path, sizeof(path), "%s/%d_%d_%d.jpg", TILE_CACHE_DIR, z, x, y);

        // 1. 读 SD 缓存
        if (sdReady && SD_MMC.exists(path)) {
            File f = SD_MMC.open(path, FILE_READ);
            if (f) {
                outLen = f.size();
                uint8_t *buf = (uint8_t*)ps_malloc(outLen);
                if (buf) {
                    f.read(buf, outLen);
                    f.close();
                    Serial.printf("[MAP] 缓存命中 %s (%d B)\n", path, outLen);
                    return buf;
                }
                f.close();
            }
        }

        // 2. WiFi 下载
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[MAP] WiFi 未连接，无法下载瓦片");
            outLen = 0;
            return nullptr;
        }

        char url[256];
        snprintf(url, sizeof(url), TILE_URL_FMT, z, y, x);  // ArcGIS 顺序: z/y/x
        Serial.printf("[MAP] 下载 %s\n", url);

        HTTPClient http;
        http.begin(url);
        http.setTimeout(8000);
        http.addHeader("User-Agent", "ESP32MapClient/1.0");
        int code = http.GET();
        if (code != 200) {
            Serial.printf("[MAP] HTTP %d\n", code);
            http.end();
            outLen = 0;
            return nullptr;
        }

        int contentLen = http.getSize();
        size_t allocLen = (contentLen > 0 && contentLen <= TILE_MAX_BYTES)
                          ? contentLen : TILE_MAX_BYTES;
        uint8_t *buf = (uint8_t*)ps_malloc(allocLen);
        if (!buf) {
            Serial.println("[MAP] PSRAM 分配失败");
            http.end();
            outLen = 0;
            return nullptr;
        }

        WiFiClient *stream = http.getStreamPtr();
        size_t got = 0;
        uint32_t deadline = millis() + 8000;
        while (millis() < deadline && got < allocLen) {
            int avail = stream->available();
            if (avail > 0) {
                int chunk = min(avail, (int)(allocLen - got));
                got += stream->read(buf + got, chunk);
            } else {
                if (!http.connected()) break;
                delay(2);
            }
        }
        http.end();
        outLen = got;

        if (got == 0) {
            free(buf);
            Serial.println("[MAP] 下载失败，0 字节");
            return nullptr;
        }
        Serial.printf("[MAP] 下载完成 %d B\n", got);

        // 3. 写入 SD 缓存
        if (sdReady) {
            File f = SD_MMC.open(path, FILE_WRITE);
            if (f) { f.write(buf, got); f.close(); }
        }

        return buf;
    }
};
