#pragma once
/**
 * tile_map.h — 在线瓦片地图下载 + SD 缓存
 *
 * 瓦片来源：高德地图街道图（PNG，无需 Key，中国覆盖完整）
 * 格式：PNG，256×256 px
 * 坐标系：GCJ-02（火星坐标），与 OSM/WGS-84 有约 100-500m 偏移
 */

#include <Arduino.h>
#include <HTTPClient.h>
#include <SD_MMC.h>
#include <math.h>

// ── 瓦片参数 ──────────────────────────────────────────────
#define TILE_SIZE        256
#define MAP_ZOOM_DEFAULT 16

// 高德地图街道瓦片（PNG，中国数据完整，无需 Key）
// style=7: 标准街道图  style=8: 路网（更简洁）
#define TILE_URL_FMT \
    "https://webrd01.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x=%d&y=%d&z=%d"
//   参数顺序：x / y / z（标准 OSM 坐标，无需翻转）

#define TILE_CACHE_DIR  "/tiles_amap2"  // 高德街道图缓存目录
#define TILE_MAX_BYTES  (64 * 1024)    // PNG 稍大，给到 64 KB
#define TILE_IS_PNG     1              // 标记为 PNG，map_screen.h 用 pngle 解码

// ── 坐标转换 ──────────────────────────────────────────────
struct TileXY { int x, y, z; };

// WGS-84 → GCJ-02（火星坐标）标准转换
static const double GCJ_A  = 6378245.0;
static const double GCJ_EE = 0.00669342162296594323;

inline bool _isOutOfChina(double lat, double lon) {
    return lon < 72.004 || lon > 137.8347 || lat < 0.8293 || lat > 55.8271;
}

inline void wgs84ToGcj02(double lat, double lon, double &glat, double &glon) {
    if (_isOutOfChina(lat, lon)) { glat = lat; glon = lon; return; }

    double dLat = -100.0 + 2.0*(lon-105.0) + 3.0*(lat-35.0)
                + 0.2*(lat-35.0)*(lat-35.0) + 0.1*(lon-105.0)*(lat-35.0)
                + 0.2*sqrt(fabs(lon-105.0));
    dLat += (20.0*sin(6.0*(lon-105.0)*M_PI) + 20.0*sin(2.0*(lon-105.0)*M_PI)) * 2.0/3.0;
    dLat += (20.0*sin((lat-35.0)*M_PI)      + 40.0*sin((lat-35.0)/3.0*M_PI))  * 2.0/3.0;
    dLat += (160.0*sin((lat-35.0)/12.0*M_PI)+ 320.0*sin((lat-35.0)*M_PI/30.0))* 2.0/3.0;

    double dLon = 300.0 + (lon-105.0) + 2.0*(lat-35.0)
                + 0.1*(lon-105.0)*(lon-105.0) + 0.1*(lon-105.0)*(lat-35.0)
                + 0.1*sqrt(fabs(lon-105.0));
    dLon += (20.0*sin(6.0*(lon-105.0)*M_PI) + 20.0*sin(2.0*(lon-105.0)*M_PI)) * 2.0/3.0;
    dLon += (20.0*sin((lon-105.0)*M_PI)     + 40.0*sin((lon-105.0)/3.0*M_PI)) * 2.0/3.0;
    dLon += (150.0*sin((lon-105.0)/12.0*M_PI)+300.0*sin((lon-105.0)/30.0*M_PI))* 2.0/3.0;

    double radlat   = lat * M_PI / 180.0;
    double magic    = sin(radlat);
    magic = 1.0 - GCJ_EE * magic * magic;
    double sqrtmagic = sqrt(magic);

    glat = lat + (dLat * 180.0) / ((GCJ_A * (1.0 - GCJ_EE)) / (magic * sqrtmagic) * M_PI);
    glon = lon + (dLon * 180.0) / (GCJ_A / sqrtmagic * cos(radlat) * M_PI);
}

// WGS-84 经纬度 → 瓦片编号（先转 GCJ-02 再计算）
inline TileXY latLonToTile(double lat, double lon, int zoom) {
    double glat, glon;
    wgs84ToGcj02(lat, lon, glat, glon);
    double n = pow(2.0, zoom);
    int tx = (int)floor((glon + 180.0) / 360.0 * n);
    double lat_r = glat * M_PI / 180.0;
    int ty = (int)floor((1.0 - log(tan(lat_r) + 1.0 / cos(lat_r)) / M_PI) / 2.0 * n);
    return {tx, ty, zoom};
}

// 经纬度在中心瓦片内的像素偏移（先转 GCJ-02）
inline void tilePixelOffset(double lat, double lon, int zoom, int &px, int &py) {
    double glat, glon;
    wgs84ToGcj02(lat, lon, glat, glon);
    double n = pow(2.0, zoom);
    double fx = (glon + 180.0) / 360.0 * n;
    double lat_r = glat * M_PI / 180.0;
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
        // 高德地图：标准 OSM 坐标，参数顺序 x/y/z
        snprintf(url, sizeof(url), TILE_URL_FMT, x, y, z);
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
