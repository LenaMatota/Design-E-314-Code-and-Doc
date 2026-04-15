#include<stdint.h>

#ifndef __SSD1306_FONTS_H__
#define __SSD1306_FONTS_H__

#include "ssd1306_conf.h"

typedef struct {
    const uint8_t width;                  // Font width in pixels
    const uint8_t height;                 // Font height in pixels
    const uint16_t *data;                 // Pointer to font data
    const uint8_t *char_width;            // Pointer to character widths (NULL if fixed width)
} FontDef;



#ifdef SSD1306_INCLUDE_FONT_6x8
extern FontDef Font_6x8;
#endif
#ifdef SSD1306_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif
#ifdef SSD1306_INCLUDE_FONT_11x18
extern FontDef Font_11x18;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x26
extern FontDef Font_16x26;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x24
extern FontDef Font_16x24;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x15
/** Generated Roboto Thin 15
 * @copyright Google https://github.com/googlefonts/roboto
 * @license This font is licensed under the Apache License, Version 2.0.
*/
extern FontDef Font_16x15;
#endif

#endif // __SSD1306_FONTS_H__
