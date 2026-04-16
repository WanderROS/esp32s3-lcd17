#pragma once
/**
 * LVGL 内存文件系统驱动
 * 将字体文件从 Flash (SPIFFS) 整体加载到 PSRAM，注册为内存盘符
 * lv_font_load("A:f") 从内存读取，速度极快
 */
#include <lvgl.h>
#include <SPIFFS.h>
#include "esp_heap_caps.h"

struct MemFile {
    const uint8_t *data;
    size_t         size;
    size_t         pos;
    bool           owned;
};

static void *_mfs_open(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) {
    (void)mode; (void)path;
    MemFile *src = (MemFile *)drv->user_data;
    if (!src || !src->data) return NULL;
    MemFile *f = new MemFile{src->data, src->size, 0, false};
    return (void *)f;
}

static lv_fs_res_t _mfs_close(lv_fs_drv_t *drv, void *file_p) {
    delete (MemFile *)file_p;
    return LV_FS_RES_OK;
}

static lv_fs_res_t _mfs_read(lv_fs_drv_t *drv, void *file_p,
                              void *buf, uint32_t btr, uint32_t *br) {
    MemFile *f = (MemFile *)file_p;
    size_t avail = f->size - f->pos;
    *br = (btr < avail) ? btr : (uint32_t)avail;
    memcpy(buf, f->data + f->pos, *br);
    f->pos += *br;
    return LV_FS_RES_OK;
}

static lv_fs_res_t _mfs_seek(lv_fs_drv_t *drv, void *file_p,
                              uint32_t pos, lv_fs_whence_t whence) {
    MemFile *f = (MemFile *)file_p;
    if      (whence == LV_FS_SEEK_SET) f->pos = pos;
    else if (whence == LV_FS_SEEK_CUR) f->pos += pos;
    else if (whence == LV_FS_SEEK_END) f->pos = (size_t)((int)f->size + (int)pos);
    if (f->pos > f->size) f->pos = f->size;
    return LV_FS_RES_OK;
}

static lv_fs_res_t _mfs_tell(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) {
    *pos_p = (uint32_t)((MemFile *)file_p)->pos;
    return LV_FS_RES_OK;
}

/**
 * 从 Flash (SPIFFS) 读取字体文件到 PSRAM，然后用内存 FS 加载字体
 * @param flash_path  SPIFFS 路径，如 "/fonts/cn16.bin"
 * @param letter      临时盘符（每个字体用不同字母，如 'A','B','C'）
 * @return lv_font_t* 或 NULL
 */
static lv_font_t *load_font_from_flash(const char *flash_path, char letter) {
    // 列出 SPIFFS 所有文件，方便调试路径问题
    static bool listed = false;
    if (!listed) {
        listed = true;
        File root = SPIFFS.open("/");
        File entry = root.openNextFile();
        Serial.println("[FONT] SPIFFS 文件列表:");
        while (entry) {
            Serial.printf("  %s (%u bytes)\n", entry.name(), (unsigned)entry.size());
            entry = root.openNextFile();
        }
    }

    File f = SPIFFS.open(flash_path, FILE_READ);
    if (!f) {
        // 尝试不带前导斜杠
        String alt = String(flash_path);
        if (alt.startsWith("/")) alt = alt.substring(1);
        f = SPIFFS.open(alt.c_str(), FILE_READ);
        if (!f) {
            Serial.printf("[FONT] SPIFFS 打开失败: %s\n", flash_path);
            return NULL;
        }
    }

    // SPIFFS 的 size() 有时返回 0，用 seek 到末尾获取真实大小
    f.seek(0, SeekEnd);
    size_t fsize = f.position();
    f.seek(0, SeekSet);

    if (fsize == 0) {
        Serial.printf("[FONT] 文件大小为 0: %s\n", flash_path);
        f.close();
        return NULL;
    }

    uint8_t *buf = (uint8_t *)heap_caps_malloc(fsize, MALLOC_CAP_SPIRAM);
    if (!buf) {
        Serial.printf("[FONT] PSRAM 分配失败，需要 %u bytes，可用 %u bytes\n",
                      (unsigned)fsize,
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        f.close();
        return NULL;
    }

    uint32_t t0 = millis();
    size_t n = f.read(buf, fsize);
    f.close();
    Serial.printf("[FONT] %s 读取 %u bytes，耗时 %lu ms\n",
                  flash_path, (unsigned)n, millis() - t0);

    lv_fs_drv_t *drv = new lv_fs_drv_t();
    lv_fs_drv_init(drv);
    drv->letter    = letter;
    drv->open_cb   = _mfs_open;
    drv->close_cb  = _mfs_close;
    drv->read_cb   = _mfs_read;
    drv->seek_cb   = _mfs_seek;
    drv->tell_cb   = _mfs_tell;
    drv->user_data = new MemFile{buf, n, 0, true};
    lv_fs_drv_register(drv);

    char font_path[4] = {letter, ':', 'f', 0};
    uint32_t t1 = millis();
    lv_font_t *font = lv_font_load(font_path);
    Serial.printf("[FONT] lv_font_load(%s) 耗时 %lu ms, font=%p\n",
                  flash_path, millis() - t1, font);
    return font;
}
