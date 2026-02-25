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
lv_font_t * english_24;

/*----------------
 * Images
 *----------------*/

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
    /* create tiny ttf font "english_24" from file */
    lv_snprintf(buf, 256, "%s%s", asset_path, "fonts/NotoSansTC-VariableFont_wght.ttf");
    english_24 = lv_tiny_ttf_create_file(buf, 24);


    /*----------------
     * Images
     *----------------*/
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
    lv_xml_register_font(NULL, "english_24", english_24);

    /* Register subjects */

    /* Register callbacks */
#endif

    /* Register all the global assets so that they won't be created again when globals.xml is parsed.
     * While running in the editor skip this step to update the preview when the XML changes */
#if LV_USE_XML && !defined(LV_EDITOR_PREVIEW)
    /* Register images */
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