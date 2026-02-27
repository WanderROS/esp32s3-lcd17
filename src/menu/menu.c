/**
 * @file menu.c
 * @brief 功能菜单列表实现
 */

#include "menu.h"

lv_obj_t * menu_create(lv_obj_t *parent, const menu_item_t *items, uint32_t count, const lv_font_t *font)
{
    lv_obj_t *list = lv_list_create(parent);
    lv_obj_set_size(list, 220, 180);
    lv_obj_set_align(list, LV_ALIGN_BOTTOM_MID);
    lv_obj_set_y(list, -10);

    /* 半透明背景，和 XML 生成的背景图融合 */
    lv_obj_set_style_bg_opa(list, LV_OPA_70, 0);
    lv_obj_set_style_bg_color(list, lv_color_hex(0x1a1a2e), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_radius(list, 12, 0);
    lv_obj_set_style_pad_ver(list, 4, 0);

    for (uint32_t i = 0; i < count; i++) {
        lv_obj_t *btn = lv_list_add_button(list, items[i].icon, items[i].text);

        /* 菜单项样式 */
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_text_color(btn, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_40, LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, lv_color_white(), LV_STATE_PRESSED);

        /* 设置文字字体 */
        if (font) {
            lv_obj_t *label = lv_obj_get_child(btn, -1); /* 最后一个子对象是 label */
            if (label) {
                lv_obj_set_style_text_font(label, font, 0);
            }
        }

        if (items[i].cb) {
            lv_obj_add_event_cb(btn, items[i].cb, LV_EVENT_CLICKED, items[i].user_data);
        }
    }

    return list;
}
