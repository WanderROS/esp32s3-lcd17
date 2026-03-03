/**
 * @file lv_mem_core_psram.c
 * Custom LVGL memory allocator that uses ESP32 PSRAM (SPIRAM)
 * to keep internal DMA-capable RAM free for display and SDMMC.
 */

#include "lvgl.h"
#if LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM

#include "esp_heap_caps.h"
#include <stdlib.h>

void lv_mem_init(void)
{
}

void lv_mem_deinit(void)
{
}

lv_mem_pool_t lv_mem_add_pool(void * mem, size_t bytes)
{
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    LV_UNUSED(pool);
}

void * lv_malloc_core(size_t size)
{
    /*
     * 小于阈值的分配走内部RAM（widget/style/小缓冲区访问频繁，需要速度）
     * 大于阈值的分配走PSRAM（图片缓存/字体数据/大缓冲区，节省内部RAM）
     */
    if(size < 512) {
        void * p = malloc(size);  /* 内部RAM优先 */
        if(p) return p;
    }
    /* 大块或内部RAM不足时走PSRAM */
    void * p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if(p == NULL) {
        p = malloc(size);
    }
    return p;
}

void * lv_realloc_core(void * p, size_t new_size)
{
    if(new_size < 512) {
        void * new_p = realloc(p, new_size);
        if(new_p) return new_p;
    }
    void * new_p = heap_caps_realloc(p, new_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if(new_p == NULL) {
        new_p = realloc(p, new_size);
    }
    return new_p;
}

void lv_free_core(void * p)
{
    free(p);
}

void lv_mem_monitor_core(lv_mem_monitor_t * mon_p)
{
    LV_UNUSED(mon_p);
}

lv_result_t lv_mem_test_core(void)
{
    return LV_RESULT_OK;
}

#endif /* LV_STDLIB_CUSTOM */
