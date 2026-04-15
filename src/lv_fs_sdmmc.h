#pragma once
/**
 * LVGL 文件系统驱动 —— 对接 SD_MMC
 * 带 4KB 读缓存，大幅减少 SD 卡随机读次数，加速 lv_font_load
 */
#include <lvgl.h>
#include <SD_MMC.h>

#define LV_FS_CACHE_SIZE 4096  // 每个文件 4KB 读缓存

struct LvFsFile {
    File    file;
    uint8_t cache[LV_FS_CACHE_SIZE];
    size_t  cache_start = 0;   // 缓存对应的文件偏移
    size_t  cache_len   = 0;   // 缓存中有效字节数
};

static void *_fs_open(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) {
    String full = String("/") + path;
    const char *flags = (mode == LV_FS_MODE_WR) ? FILE_WRITE : FILE_READ;

    LvFsFile *f = new LvFsFile();
    if (!f) return NULL;

    f->file = SD_MMC.open(full.c_str(), flags);
    if (!f->file) {
        Serial.printf("[LVGL-FS] 打开失败: %s\n", full.c_str());
        delete f;
        return NULL;
    }
    return (void *)f;
}

static lv_fs_res_t _fs_close(lv_fs_drv_t *drv, void *file_p) {
    if (!file_p) return LV_FS_RES_OK;
    LvFsFile *f = (LvFsFile *)file_p;
    f->file.close();
    delete f;
    return LV_FS_RES_OK;
}

static lv_fs_res_t _fs_read(lv_fs_drv_t *drv, void *file_p,
                             void *buf, uint32_t btr, uint32_t *br) {
    if (!file_p) return LV_FS_RES_INV_PARAM;
    LvFsFile *f = (LvFsFile *)file_p;
    size_t pos = f->file.position();
    *br = 0;

    uint8_t *out = (uint8_t *)buf;
    size_t remaining = btr;

    while (remaining > 0) {
        // 检查缓存是否命中
        if (f->cache_len > 0 &&
            pos >= f->cache_start &&
            pos < f->cache_start + f->cache_len) {
            size_t offset  = pos - f->cache_start;
            size_t avail   = f->cache_len - offset;
            size_t to_copy = remaining < avail ? remaining : avail;
            memcpy(out, f->cache + offset, to_copy);
            out       += to_copy;
            pos       += to_copy;
            *br       += to_copy;
            remaining -= to_copy;
            f->file.seek(pos);
        } else {
            // 缓存未命中，填充缓存
            f->file.seek(pos);
            f->cache_start = pos;
            f->cache_len   = f->file.read(f->cache, LV_FS_CACHE_SIZE);
            if (f->cache_len == 0) break;  // EOF
        }
    }
    return LV_FS_RES_OK;
}

static lv_fs_res_t _fs_write(lv_fs_drv_t *drv, void *file_p,
                              const void *buf, uint32_t btw, uint32_t *bw) {
    if (!file_p) return LV_FS_RES_INV_PARAM;
    LvFsFile *f = (LvFsFile *)file_p;
    *bw = f->file.write((const uint8_t *)buf, btw);
    f->cache_len = 0;  // 写入后使缓存失效
    return LV_FS_RES_OK;
}

static lv_fs_res_t _fs_seek(lv_fs_drv_t *drv, void *file_p,
                             uint32_t pos, lv_fs_whence_t whence) {
    if (!file_p) return LV_FS_RES_INV_PARAM;
    LvFsFile *f = (LvFsFile *)file_p;
    SeekMode m = SeekSet;
    if      (whence == LV_FS_SEEK_CUR) m = SeekCur;
    else if (whence == LV_FS_SEEK_END) m = SeekEnd;
    f->file.seek(pos, m);
    return LV_FS_RES_OK;
}

static lv_fs_res_t _fs_tell(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) {
    if (!file_p) return LV_FS_RES_INV_PARAM;
    LvFsFile *f = (LvFsFile *)file_p;
    *pos_p = f->file.position();
    return LV_FS_RES_OK;
}

inline void lv_fs_sdmmc_init(void) {
    static lv_fs_drv_t drv;
    lv_fs_drv_init(&drv);
    drv.letter   = 'S';
    drv.open_cb  = _fs_open;
    drv.close_cb = _fs_close;
    drv.read_cb  = _fs_read;
    drv.write_cb = _fs_write;
    drv.seek_cb  = _fs_seek;
    drv.tell_cb  = _fs_tell;
    lv_fs_drv_register(&drv);
    Serial.println("[LVGL-FS] SD_MMC 驱动注册完成 (盘符 'S')");
}
