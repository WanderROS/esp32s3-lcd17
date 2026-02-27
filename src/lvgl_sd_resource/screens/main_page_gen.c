/**
 * @file main_page_gen.c
 * @brief Template source file for LVGL objects
 */

/*********************
 *      INCLUDES
 *********************/

#include "main_page_gen.h"
#include "lvgl_sd_resource.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/***********************
 *  STATIC VARIABLES
 **********************/

/***********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_obj_t * main_page_create(void)
{
    LV_TRACE_OBJ_CREATE("begin");


    static bool style_inited = false;

    if (!style_inited) {

        style_inited = true;
    }

    lv_obj_t * lv_obj_0 = lv_obj_create(NULL);
    lv_obj_set_name_static(lv_obj_0, "main_page_#");

    lv_obj_t * lv_image_0 = lv_image_create(lv_obj_0);
    lv_image_set_src(lv_image_0, weather_location_2_bg);
    lv_obj_set_align(lv_image_0, LV_ALIGN_CENTER);
    lv_obj_set_x(lv_image_0, 0);
    lv_obj_set_y(lv_image_0, 0);
    
    lv_obj_t * lv_label_0 = lv_label_create(lv_obj_0);
    lv_obj_set_style_text_font(lv_label_0, chinese_24, 0);
    lv_label_set_text(lv_label_0, "刘雪清牛掰");
    lv_obj_set_align(lv_label_0, LV_ALIGN_CENTER);
    lv_obj_set_x(lv_label_0, -9);
    lv_obj_set_y(lv_label_0, -94);
    
    lv_obj_t * lv_label_1 = lv_label_create(lv_obj_0);
    lv_label_set_text(lv_label_1, "English");
    lv_obj_set_style_text_font(lv_label_1, english_24, 0);
    lv_obj_set_align(lv_label_1, LV_ALIGN_CENTER);
    lv_obj_set_y(lv_label_1, -132);
    lv_obj_set_x(lv_label_1, -5);
    
    lv_obj_t * lv_label_2 = lv_label_create(lv_obj_0);
    lv_obj_set_style_text_font(lv_label_2, chinese_48, 0);
    lv_label_set_text(lv_label_2, "汉字");
    lv_obj_set_align(lv_label_2, LV_ALIGN_CENTER);
    lv_obj_set_y(lv_label_2, -45);
    lv_obj_set_width(lv_label_2, 96);
    lv_obj_set_height(lv_label_2, 65);
    lv_obj_set_x(lv_label_2, -6);

    LV_TRACE_OBJ_CREATE("finished");

    return lv_obj_0;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

