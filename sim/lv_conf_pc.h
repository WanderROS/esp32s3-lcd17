/**
 * @file lv_conf_pc.h
 * PC 模拟器专用 LVGL 配置（基于 SDL2）
 * 对应嵌入式端: include/lv_conf.h
 */

/* clang-format off */
#if 1

#ifndef LV_CONF_H
#define LV_CONF_H

#if 0 && defined(__ASSEMBLY__)
#endif

/*====================
   COLOR SETTINGS
 *====================*/

/**
 * PC 端使用 32 位色深（XRGB8888），SDL2 原生支持，无需字节序转换。
 * 嵌入式端是 16 位 RGB565，两端独立。
 */
#define LV_COLOR_DEPTH 32

/*=========================
   STDLIB WRAPPER SETTINGS
 *=========================*/

/* PC 端直接用标准 C 库，不需要自定义 PSRAM 分配器 */
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_CLIB
#define LV_USE_STDLIB_STRING    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_BUILTIN

#define LV_STDINT_INCLUDE       <stdint.h>
#define LV_STDDEF_INCLUDE       <stddef.h>
#define LV_STDBOOL_INCLUDE      <stdbool.h>
#define LV_INTTYPES_INCLUDE     <inttypes.h>
#define LV_LIMITS_INCLUDE       <limits.h>
#define LV_STDARG_INCLUDE       <stdarg.h>

/*====================
   HAL SETTINGS
 *====================*/

#define LV_DEF_REFR_PERIOD  33      /**< [ms] */
#define LV_DPI_DEF          130     /**< [px/inch] */

/*=================
 * OPERATING SYSTEM
 *=================*/

/* PC 端单线程运行，与嵌入式端保持一致 */
#define LV_USE_OS   LV_OS_NONE

/*========================
 * RENDERING CONFIGURATION
 *========================*/

#define LV_DRAW_BUF_STRIDE_ALIGN    1
#define LV_DRAW_BUF_ALIGN           4
/* LV_DRAW_TRANSFORM_USE_MATRIX 依赖 LV_USE_MATRIX，必须同时启用 */
#define LV_USE_MATRIX               1
#define LV_DRAW_TRANSFORM_USE_MATRIX 1
#define LV_DRAW_LAYER_SIMPLE_BUF_SIZE    (16 * 1024)
#define LV_DRAW_LAYER_MAX_MEMORY    0
#define LV_DRAW_THREAD_STACK_SIZE   (8 * 1024)
#define LV_DRAW_THREAD_PRIO         LV_THREAD_PRIO_HIGH

#define LV_USE_DRAW_SW 1
#if LV_USE_DRAW_SW == 1
    #define LV_DRAW_SW_SUPPORT_RGB565           1
    #define LV_DRAW_SW_SUPPORT_RGB565_SWAPPED   1
    #define LV_DRAW_SW_SUPPORT_RGB565A8         1
    #define LV_DRAW_SW_SUPPORT_RGB888           1
    #define LV_DRAW_SW_SUPPORT_XRGB8888         1
    #define LV_DRAW_SW_SUPPORT_ARGB8888         1
    #define LV_DRAW_SW_SUPPORT_ARGB8888_PREMULTIPLIED 1
    #define LV_DRAW_SW_SUPPORT_L8               1
    #define LV_DRAW_SW_SUPPORT_AL88             1
    #define LV_DRAW_SW_SUPPORT_A8               1
    #define LV_DRAW_SW_SUPPORT_I1               1
    #define LV_DRAW_SW_I1_LUM_THRESHOLD         127
    #define LV_DRAW_SW_DRAW_UNIT_CNT            1
    #define LV_USE_DRAW_ARM2D_SYNC              0
#endif

#define LV_USE_DRAW_VGLITE          0
#define LV_USE_DRAW_PXP             0
#define LV_USE_DRAW_DAVE2D          0
#define LV_USE_DRAW_DMA2D           0
#define LV_USE_DRAW_OPENGLES        0
#define LV_USE_DRAW_SDL             0

/*=======================
 * FEATURE CONFIGURATION
 *=======================*/

#define LV_GRADIENT_MAX_STOPS      4
#define LV_GRAD_CACHE_DEF_SIZE     0

/*===================
   LOGGING
 *===================*/

#define LV_USE_LOG                  1
#if LV_USE_LOG
    #define LV_LOG_LEVEL            LV_LOG_LEVEL_WARN
    #define LV_LOG_PRINTF           1   /* PC 端直接 printf 输出 */
    #define LV_LOG_TIMESTAMP        1
    #define LV_LOG_LEVEL_TRACE      0
#endif

/*=================
   OTHERS
 *===============*/

#define LV_SPRINTF_CUSTOM           0
/* LV_USE_FLOAT 必须先于 LV_USE_MATRIX 定义 */
#define LV_USE_FLOAT                1
#define LV_USE_OBJ_NAME             1
#define LV_USE_OBJ_ID               0
#define LV_USE_OBJ_PROPERTY         0
#define LV_USE_OBJ_PROPERTY_NAME    0
#define LV_USE_VG_LITE_THORVG       0

/*============
   LAYOUTS
 *============*/

#define LV_USE_FLEX                 1
#define LV_USE_GRID                 1

/*===========
   WIDGETS
 *===========*/

#define LV_USE_ANIMIMG              1
#define LV_USE_ARC                  1
#define LV_USE_BAR                  1
#define LV_USE_BUTTON               1
#define LV_USE_BUTTONMATRIX         1
#define LV_USE_CALENDAR             1
#if LV_USE_CALENDAR
    #define LV_CALENDAR_WEEK_STARTS_MONDAY  0
    #define LV_CALENDAR_DEFAULT_MONTH_NAMES {"January", "February", "March",  "April", "May",  "June", "July", "August", "September", "October", "November", "December"}
    #define LV_CALENDAR_DEFAULT_DAY_NAMES   {"Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"}
#endif
#define LV_USE_CANVAS               1
#define LV_USE_CHART                1
#define LV_USE_CHECKBOX             1
#define LV_USE_DROPDOWN             1
#define LV_USE_IMAGE                1
#define LV_USE_IMAGEBUTTON          1
#define LV_USE_KEYBOARD             1
#define LV_USE_LABEL                1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION     1
    #define LV_LABEL_LONG_TXT_HINT      1
    #define LV_LABEL_WAIT_CHAR_COUNT    3
#endif
#define LV_USE_LED                  1
#define LV_USE_LINE                 1
#define LV_USE_LIST                 1
#define LV_USE_LOTTIE               0
#define LV_USE_MENU                 1
#define LV_USE_MSGBOX               1
#define LV_USE_ROLLER               1
#define LV_USE_SCALE                1
#define LV_USE_SLIDER               1
#define LV_USE_SPAN                 1
#if LV_USE_SPAN
    #define LV_SPAN_SNIPPET_STACK_SIZE  64
#endif
#define LV_USE_SPINBOX              1
#define LV_USE_SPINNER              1
#define LV_USE_SWITCH               1
#define LV_USE_TABLE                1
#define LV_USE_TABVIEW              1
#define LV_USE_TEXTAREA             1
#if LV_USE_TEXTAREA
    #define LV_TEXTAREA_DEF_PWD_SHOW_TIME    1500
#endif
#define LV_USE_TILEVIEW             1
#define LV_USE_WIN                  1

/*=========
   THEMES
 *=========*/

#define LV_USE_THEME_DEFAULT        1
#if LV_USE_THEME_DEFAULT
    #define LV_THEME_DEFAULT_DARK   0
    #define LV_THEME_DEFAULT_GROW   1
    #define LV_THEME_DEFAULT_TRANSITION_TIME    80
#endif
#define LV_USE_THEME_SIMPLE         1
#define LV_USE_THEME_MONO           1

/*=============
   FONT USAGE
 *=============*/

#define LV_FONT_MONTSERRAT_8        0
#define LV_FONT_MONTSERRAT_10       0
#define LV_FONT_MONTSERRAT_12       1
#define LV_FONT_MONTSERRAT_14       1
#define LV_FONT_MONTSERRAT_16       0
#define LV_FONT_MONTSERRAT_18       0
#define LV_FONT_MONTSERRAT_20       1
#define LV_FONT_MONTSERRAT_22       0
#define LV_FONT_MONTSERRAT_24       1
#define LV_FONT_MONTSERRAT_26       0
#define LV_FONT_MONTSERRAT_28       0
#define LV_FONT_MONTSERRAT_30       0
#define LV_FONT_MONTSERRAT_32       0
#define LV_FONT_MONTSERRAT_34       0
#define LV_FONT_MONTSERRAT_36       0
#define LV_FONT_MONTSERRAT_38       0
#define LV_FONT_MONTSERRAT_40       1
#define LV_FONT_MONTSERRAT_42       0
#define LV_FONT_MONTSERRAT_44       0
#define LV_FONT_MONTSERRAT_46       0
#define LV_FONT_MONTSERRAT_48       0
#define LV_FONT_UNSCII_8            0
#define LV_FONT_UNSCII_16           0
#define LV_FONT_MONTSERRAT_12_SUBPX      0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SIMSUN_14_CJK            0
#define LV_FONT_SIMSUN_16_CJK            0

#define LV_FONT_DEFAULT             &lv_font_montserrat_14
#define LV_FONT_FMT_TXT_LARGE       0
#define LV_USE_FONT_PLACEHOLDER     1
#define LV_USE_FONT_COMPRESSED      0

/*=============
   OTHERS
 *=============*/

#define LV_USE_SNAPSHOT             0
#define LV_USE_REFR_DEBUG           0
#define LV_USE_PERF_MONITOR         0
#define LV_USE_MEM_MONITOR          0

/*=================
   ANIMATION
 *=================*/

#define LV_ANIM_TIMELINE_MAX_PLAYBACK_SEGMENT 10

/*=================
   OBSERVER
 *=================*/

/* UI 数据绑定系统依赖 Observer */
#define LV_USE_OBSERVER             1

/*=====================
   FILE SYSTEM DRIVERS
 *=====================*/

/* 使用自定义 FS 驱动（与嵌入式端同一套代码，路径映射到本地文件系统） */
#define LV_USE_FS_STDIO             0
#define LV_USE_FS_POSIX             0
#define LV_USE_FS_WIN32             0
#define LV_USE_FS_FATFS             0
#define LV_USE_FS_MEMFS             0
#define LV_USE_FS_LittleFS          0
#define LV_USE_FS_ARDUINO_SD        0

/*====================
   IMAGE DECODERS
 *====================*/

/* PNG 图片资源使用 lodepng */
#define LV_USE_LODEPNG              1
#define LV_USE_LIBPNG               0
#define LV_USE_BMP                  0
#define LV_USE_RLE                  0
#define LV_USE_GIF                  0
#define LV_GIF_CACHE_DECODE_DATA    0
#define LV_USE_BARCODE              0
#define LV_USE_QRCODE               0
#define LV_USE_TJPGD                0
#define LV_USE_LIBJPEG_TURBO        0
#define LV_USE_SVG                  0
#define LV_USE_SVG_ANIMATION        0
#define LV_SVG_ENABLE_FLOAT_LAYOUT  0

/* TinyTTF 用于运行时加载字体文件（binfont） */
#define LV_USE_TINY_TTF             1
#if LV_USE_TINY_TTF
    #define LV_TINY_TTF_FILE_SUPPORT    1
    #define LV_TINY_TTF_CACHE_GLYPH_CNT 64
#endif

/*=================
   CACHE
 *=================*/

/* PC 端内存充足，给图片缓存 16MB */
#define LV_CACHE_DEF_SIZE           (16 * 1024 * 1024)
#define LV_CACHE_LOCK_TYPE          LV_CACHE_LOCK_NONE
#define LV_IMAGE_HEADER_CACHE_DEF_CNT 100
#define LV_DRAW_BUF_CACHE_DEF_CNT   0

/*=================
   XML
 *=================*/

#define LV_USE_XML                  0

/*============
   DEMOS
 *============*/

#define LV_BUILD_DEMOS              1
#define LV_BUILD_EXAMPLES           1

#endif /* LV_CONF_H */
#endif /* End enable content */
