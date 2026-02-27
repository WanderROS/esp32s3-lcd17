/**
 * @file lvgl_sd_resource_gen.h
 */

#ifndef LVGL_SD_RESOURCE_GEN_H
#define LVGL_SD_RESOURCE_GEN_H

#ifndef UI_SUBJECT_STRING_LENGTH
#define UI_SUBJECT_STRING_LENGTH 256
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL VARIABLES
 **********************/

/*-------------------
 * Permanent screens
 *------------------*/

/*----------------
 * Global styles
 *----------------*/

/*----------------
 * Fonts
 *----------------*/

extern lv_font_t * chinese_24;

extern lv_font_t * chinese_48;

extern lv_font_t * english_24;

/*----------------
 * Images
 *----------------*/

extern const void * icon_plus;
extern const void * icon_minus;
extern const void * light_temp_arc_bg;
extern const void * icon_heart;
extern const void * icon_play;
extern const void * icon_pause;
extern const void * icon_skip_back;
extern const void * icon_skip_forward;
extern const void * icon_volume_max;
extern const void * icon_volume_min;
extern const void * icon_volume_none;
extern const void * song_cover_1;
extern const void * weather_location_1_bg;
extern const void * weather_location_2_bg;
extern const void * icon_cloudy;
extern const void * icon_sunny;
extern const void * icon_pin;
extern const void * icon_theme;

/*----------------
 * Subjects
 *----------------*/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/*----------------
 * Event Callbacks
 *----------------*/

/**
 * Initialize the component library
 */

void lvgl_sd_resource_init_gen(const char * asset_path);

/**********************
 *      MACROS
 **********************/

/**********************
 *   POST INCLUDES
 **********************/

/*Include all the widget and components of this library*/
#include "screens/main_page_gen.h"

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LVGL_SD_RESOURCE_GEN_H*/