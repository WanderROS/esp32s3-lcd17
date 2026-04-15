#pragma once
/**
 * LVGL 内存文件系统驱动
 * 将文件整体加载到 PSRAM，注册为 'M' 盘符
 * lv_font_load("M:cn16.bin") 从内存读取，速度极快
 */
#include <lvgl.h>
#include <SD_MMC.h>
#include "esp_heap_caps.h"

struct MemFile {
    const uint8_t *data;
    size_t         size;
    size_t         pos;
    bool           owned;  // 是否需要 free
};

static void *_mfs_open(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) {
    // path 格式: "cn16.bin"，对应 SD 卡 /fonts/cn16.bin
    // 数据指针存在 drv->user_data（由调用者预先加载）
    // 这里不支持写，只支持读
    (void)mode;
    // user_data 是 MemFile* 数组的头，通过 path 匹配
    // 简化：直接把 MemFile* 存在 drv->user_data，path 忽略
    MemFile *src = (MemFile *)drv->user_data;
    if (!src || !src->data) return NULL;

    MemFile *f = new MemFile();
    f->data  = src->data;
    f->size  = src->size;
    f->pos   = 0;
    f->owned = false;
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
    *br = (btr < avail) ? btr : avail;
    memcpy(buf, f->data + f->pos, *br);
    f->pos += *br;
    return LV_FS_RES_OK;
}

static lv_fs_res_t _mfs_seek(lv_fs_drv_t *drv, void *file_p,
                              uint32_t pos, lv_fs_whence_t whence) {
    MemFile *f = (MemFile *)file_p;
    if      (whence == LV_FS_SEEK_SET) f->pos = pos;
    else if (whence == LV_FS_SEEK_CUR) f->pos += pos;
    else if (whence == LV_FS_SEEK_END) f->pos = f->size + pos;
    if (f->pos > f->size) f->pos = f->size;
    return LV_FS_RES_OK;
}

static lv_fs_res_t _mfs_tell(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) {
    *pos_p = ((MemFile *)file_p)->pos;
    return LV_FS_RES_OK;
}

/**
 * 从 SD 卡读取字体文件到 PSRAM，然后用内存 FS 加载字体
 * @param sd_path  SD 卡路径，如 "/fonts/cn16.bin"
 * @param letter   临时盘符（每个字体用不同字母，如 'A','B','C'）
 * @return lv_font_t* 或 NULL
 */
static lv_font_t *load_font_from_sd(const char *sd_path, char letter) {
    // 1. 读取 SD 文件到 PSRAM
    File f = SD_MMC.open(sd_path, FILE_READ);
    if (!f) {
        Serial.printf("[FONT] SD 打开失败: %s\n", sd_path);
        return NULL;
    }
    size_t fsize = f.size();
    uint8_t *buf = (uint8_t *)heap_caps_malloc(fsize, MALLOC_CAP_SPIRAM);
    if (!buf) {
        Serial.printf("[FONT] PSRAM 分配失败: %u bytes\n", (unsigned)fsize);
        f.close();
        return NULL;
    }

    uint32_t t0 = millis();
    size_t n = f.read(buf, fsize);
    f.close();
    Serial.printf("[FONT] %s 读取 %u bytes，耗时 %lu ms\n",
                  sd_path, (unsigned)n, millis() - t0);

    // 2. 注册内存 FS 驱动（每个字体一个盘符）
    lv_fs_drv_t *drv = new lv_fs_drv_t();
    lv_fs_drv_init(drv);
    drv->letter   = letter;
    drv->open_cb  = _mfs_open;
    drv->close_cb = _mfs_close;
    drv->read_cb  = _mfs_read;
    drv->seek_cb  = _mfs_seek;
    drv->tell_cb  = _mfs_tell;

    // 把内存数据指针存在 user_data
    MemFile *mf = new MemFile{buf, n, 0, true};
    drv->user_data = mf;
    lv_fs_drv_register(drv);

    // 3. 用内存 FS 加载字体（路径随意，open_cb 忽略 path）
    char font_path[8] = {letter, ':', 'f', 0};
    uint32_t t1 = millis();
    lv_font_t *font = lv_font_load(font_path);
    Serial.printf("[FONT] lv_font_load 耗时 %lu ms\n", millis() - t1);

    return font;
}
