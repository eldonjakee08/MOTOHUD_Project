/*
 * ssd1306_ssd1315.h
 *
 *  Created on: May 17, 2025
 *      Author: eldon
 */

#ifndef DRIVER_SSD1306_SSD1315_H_
#define DRIVER_SSD1306_SSD1315_H_

#include "stm32wbxx_hal.h" //change this according to your STM32 series
#include "glyph_assets.h"	//glyph assets stored in flash

#include <stddef.h>
#include <string.h>
#include <stdint.h>

/*
 * User defined OLED parameters
 */
#define WIDTH					128					//input OLED width pixels
#define HEIGHT 					64					//input OLED height pixels
#define OLED_ADDR				(0x3C << 1)			//input OLED device address left shift by 1 for R/W bit
#define I2C_OLED_HANDLE         hi2c1				//input OLED I2C handler name
#define PAGE_LIMIT				HEIGHT / 8          //number of pages OLED has available, each page is 8 pixels high

#define OLED_BUFFER_SIZE 		( (WIDTH * HEIGHT) / 8 )
extern I2C_HandleTypeDef hi2c1; //change this according to your I2C handler name


// Enumeration for screen colors
typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} OLED_COLOR;


//enum for turning the display ON or OFF
typedef enum {
	ON,
	OFF
}ON_OFF;


//@Glyph_Drawing_Modes
typedef enum {
  GLYPH_MODE_OVERWRITE,
  GLYPH_MODE_SET,
  GLYPH_MODE_AND_MASK,
  GLYPH_MODE_TOGGLE,
  GLYPH_MODE_CLEAR
} GlyphRederMode_t;


/*
 * function prototypes of APIs
 */

//APIs for communication with OLED
void oled_WriteCommand(uint8_t *commandByte, size_t command_length);
void oled_WriteData(uint8_t *pBuffer, size_t bufferSize);

//APIs for drawing into the OLED
void oled_drawBMP(uint8_t x_start, uint8_t y_start, const uint8_t *glyph, uint8_t glyph_width, uint8_t glyph_height, GlyphRederMode_t mode);
void oled_RenderGlyph(const Glyph_Metadata_t *glyph_meta, GlyphArray_Index_t GlyphIndex,GlyphRederMode_t mode);
void oled_clearGlyphRegion(uint8_t x_start, uint8_t y_start, uint8_t glyph_width, uint8_t glyph_height);
void oled_ClearGlyph(const Glyph_Metadata_t *glyph_meta);
void oled_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color);
void oled_Fill(OLED_COLOR color);
void oled_UpdateScreen(void);


//Other APIs for OLED
void oled_init(void);
void oled_SetDisplayOn(ON_OFF ONorOff);
void oled_SetContrast(uint8_t contrast);

#endif /* DRIVER_SSD1306_SSD1315_H_ */
