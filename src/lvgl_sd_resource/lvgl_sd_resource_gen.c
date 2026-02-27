/**
 * @file lvgl_sd_resource_gen.c
 */

/*********************
 *      INCLUDES
 *********************/

#include "lvgl_sd_resource_gen.h"

#if LV_USE_XML
#endif /* LV_USE_XML */

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/

/*----------------
 * Translations
 *----------------*/

/**********************
 *  GLOBAL VARIABLES
 **********************/

/*--------------------
 *  Permanent screens
 *-------------------*/

/*----------------
 * Global styles
 *----------------*/

/*----------------
 * Fonts
 *----------------*/

lv_font_t * chinese_24;
lv_font_t * chinese_48;
lv_font_t * english_24;

/*----------------
 * Images
 *----------------*/

const void * icon_plus;
const void * icon_minus;
const void * light_temp_arc_bg;
const void * icon_heart;
const void * icon_play;
const void * icon_pause;
const void * icon_skip_back;
const void * icon_skip_forward;
const void * icon_volume_max;
const void * icon_volume_min;
const void * icon_volume_none;
const void * song_cover_1;
const void * weather_location_1_bg;
const void * weather_location_2_bg;
const void * icon_cloudy;
const void * icon_sunny;
const void * icon_pin;
const void * icon_theme;

/*----------------
 * Subjects
 *----------------*/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lvgl_sd_resource_init_gen(const char * asset_path)
{
    char buf[256];

    /*----------------
     * Global styles
     *----------------*/

    /*----------------
     * Fonts
     *----------------*/

    /* create tiny ttf font "chinese_24" from file */
    lv_snprintf(buf, 256, "%s%s", asset_path, "fonts/QingNiaoHuaGuangJianMeiHei-2.ttf");
    chinese_24 = lv_tiny_ttf_create_file(buf, 24);
    /* create tiny ttf font "chinese_48" from file */
    lv_snprintf(buf, 256, "%s%s", asset_path, "fonts/QingNiaoHuaGuangJianMeiHei-2.ttf");
    chinese_48 = lv_tiny_ttf_create_file(buf, 48);
    /* create tiny ttf font "english_24" from file */
    lv_snprintf(buf, 256, "%s%s", asset_path, "fonts/NotoSansTC-VariableFont_wght.ttf");
    english_24 = lv_tiny_ttf_create_file(buf, 24);


    /*----------------
     * Images
     *----------------*/
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_plus.png");
    icon_plus = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_minus.png");
    icon_minus = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/light_temp_arc_bg.png");
    light_temp_arc_bg = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_heart.png");
    icon_heart = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_play.png");
    icon_play = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_pause.png");
    icon_pause = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_skip_back.png");
    icon_skip_back = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_skip_forward.png");
    icon_skip_forward = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_volume_max.png");
    icon_volume_max = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_volume_min.png");
    icon_volume_min = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_volume_none.png");
    icon_volume_none = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/song_cover_1.png");
    song_cover_1 = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/weather_location_1_bg.png");
    weather_location_1_bg = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/weather_location_2_bg.png");
    weather_location_2_bg = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_cloudy.png");
    icon_cloudy = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_sunny.png");
    icon_sunny = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_pin.png");
    icon_pin = lv_strdup(buf);
    lv_snprintf(buf, 256, "%s%s", asset_path, "images/icon_theme.png");
    icon_theme = lv_strdup(buf);

    /*----------------
     * Subjects
     *----------------*/
    /*----------------
     * Translations
     *----------------*/

#if LV_USE_XML
    /* Register widgets */

    /* Register fonts */
    lv_xml_register_font(NULL, "chinese_24", chinese_24);
    lv_xml_register_font(NULL, "chinese_48", chinese_48);
    lv_xml_register_font(NULL, "english_24", english_24);

    /* Register subjects */

    /* Register callbacks */
#endif

    /* Register all the global assets so that they won't be created again when globals.xml is parsed.
     * While running in the editor skip this step to update the preview when the XML changes */
#if LV_USE_XML && !defined(LV_EDITOR_PREVIEW)
    /* Register images */
    lv_xml_register_image(NULL, "icon_plus", icon_plus);
    lv_xml_register_image(NULL, "icon_minus", icon_minus);
    lv_xml_register_image(NULL, "light_temp_arc_bg", light_temp_arc_bg);
    lv_xml_register_image(NULL, "icon_heart", icon_heart);
    lv_xml_register_image(NULL, "icon_play", icon_play);
    lv_xml_register_image(NULL, "icon_pause", icon_pause);
    lv_xml_register_image(NULL, "icon_skip_back", icon_skip_back);
    lv_xml_register_image(NULL, "icon_skip_forward", icon_skip_forward);
    lv_xml_register_image(NULL, "icon_volume_max", icon_volume_max);
    lv_xml_register_image(NULL, "icon_volume_min", icon_volume_min);
    lv_xml_register_image(NULL, "icon_volume_none", icon_volume_none);
    lv_xml_register_image(NULL, "song_cover_1", song_cover_1);
    lv_xml_register_image(NULL, "weather_location_1_bg", weather_location_1_bg);
    lv_xml_register_image(NULL, "weather_location_2_bg", weather_location_2_bg);
    lv_xml_register_image(NULL, "icon_cloudy", icon_cloudy);
    lv_xml_register_image(NULL, "icon_sunny", icon_sunny);
    lv_xml_register_image(NULL, "icon_pin", icon_pin);
    lv_xml_register_image(NULL, "icon_theme", icon_theme);
#endif

#if LV_USE_XML == 0
    /*--------------------
     *  Permanent screens
     *-------------------*/
    /* If XML is enabled it's assumed that the permanent screens are created
     * manaully from XML using lv_xml_create() */
#endif
}

/* Callbacks */

/**********************
 *   STATIC FUNCTIONS
 **********************/