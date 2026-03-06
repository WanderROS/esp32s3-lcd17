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
    /* 全部走 PSRAM，保留内部 RAM 给 BLE/WiFi 协议栈使用 */
    void * p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if(p == NULL) {
        p = malloc(size);  /* PSRAM 不足时降级到内部 RAM */
    }
    return p;
}

void * lv_realloc_core(void * p, size_t new_size)
{
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
