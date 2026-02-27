/**
 * @file menu.h
 * @brief 功能菜单列表模块 - 在 LVGL Editor 生成的屏幕上动态添加菜单
 */

#ifndef MENU_H
#define MENU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

/**
 * 菜单项定义
 */
typedef struct {
    const char *icon;       // LVGL 内置符号或 NULL
    const char *text;       // 菜单项文字
    lv_event_cb_t cb;       // 点击回调函数，可为 NULL
    void *user_data;        // 回调的用户数据
} menu_item_t;

/**
 * 在指定的父对象（通常是 XML 生成的屏幕）上创建功能菜单列表
 * @param parent 父对象，如 main_page_create() 返回的屏幕
 * @param items  菜单项数组
 * @param count  菜单项数量
 * @param font   菜单文字字体，传 NULL 则使用默认字体
 * @return 创建的 lv_list 对象
 */
lv_obj_t * menu_create(lv_obj_t *parent, const menu_item_t *items, uint32_t count, const lv_font_t *font);

#ifdef __cplusplus
}
#endif

#endif /* MENU_H */
