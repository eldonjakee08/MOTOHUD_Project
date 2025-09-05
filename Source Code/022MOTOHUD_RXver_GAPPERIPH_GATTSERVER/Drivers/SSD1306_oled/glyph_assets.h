/*
 * glyph_assets.h
 *
 *  Created on: Aug 31, 2025
 *      Author: eldon
 */

#ifndef SSD1306_OLED_GLYPH_ASSETS_H_
#define SSD1306_OLED_GLYPH_ASSETS_H_

#include <stdint.h>

/*
 * Enumeration of glyph indexes if glyph bitmap array contains multiple glyphs
 * Ex. speedometer_font_16x24 array contains digits 0-9, so we can define an enum for the indexes
 * 	  	rpm_bar_gauge_9x44 array contains multiple levels of the gauge (1 to 9), so we can define an enum for the indexes
 */
typedef enum {
	GLYPH_GAUGE_LEVEL_0 = 0,
	GLYPH_GAUGE_LEVEL_1 = 1,
	GLYPH_GAUGE_LEVEL_2 = 2,
	GLYPH_GAUGE_LEVEL_3 = 3,
	GLYPH_GAUGE_LEVEL_4 = 4,
	GLYPH_GAUGE_LEVEL_5 = 5,
	GLYPH_GAUGE_LEVEL_6 = 6,
	GLYPH_GAUGE_LEVEL_7 = 7,
	GLYPH_GAUGE_LEVEL_8 = 8,
	GLYPH_GAUGE_LEVEL_9 = 9,
	GLYPH_DIGIT_0 = 0,
	GLYPH_DIGIT_1 = 1,
	GLYPH_DIGIT_2 = 2,
	GLYPH_DIGIT_3 = 3,
	GLYPH_DIGIT_4 = 4,
	GLYPH_DIGIT_5 = 5,
	GLYPH_DIGIT_6 = 6,
	GLYPH_DIGIT_7 = 7,
	GLYPH_DIGIT_8 = 8,
	GLYPH_DIGIT_9 = 9,
	GLYPH_V_CHAR = 10,
	GLYPH_SINGLE_ELEMENT = 12
}GlyphArray_Index_t;

/*
 * Structure to hold metadata for a glyph or set of glyphs in a bitmap array
 */
typedef struct {
	const uint8_t *bitmap;					//pointer to the bitmap array
	uint8_t width;							//width of the glyph in pixels
	uint8_t height;							//height of the glyph in pixels
	uint8_t x_coordinate;					//x coordinate of the glyph
	uint8_t y_coordinate;					//y coordinate of the glyph
	uint8_t elements_per_glpyh;				//this specifies how many array elements a single glyph consumes
	GlyphArray_Index_t glyph_index_limit; 	//max index of the glyph array, set to 0 if bitmap contains only a single glyph

} Glyph_Metadata_t;

/*********************************************************
 * Glyph Metadata externs
 *********************************************************/
// Bluetooth icons
extern const Glyph_Metadata_t bluetooth_broadcasting_icon;
extern const Glyph_Metadata_t bluetooth_connected_icon;

// Fuel bar gauge levels
extern const Glyph_Metadata_t fuel_bar_gauge;

// RPM bar gauge levels
extern const Glyph_Metadata_t rpm_bar_gauge;

//rpm digits
extern const Glyph_Metadata_t rpm_integer_digit_1;
extern const Glyph_Metadata_t rpm_decimal_digit_2;

//turn signals
extern const Glyph_Metadata_t left_turn_signal;
extern const Glyph_Metadata_t right_turn_signal;

//speedometer digits
extern const Glyph_Metadata_t speedometer_digit_1;
extern const Glyph_Metadata_t speedometer_digit_2;
extern const Glyph_Metadata_t speedometer_digit_3;

//time digits
extern const Glyph_Metadata_t time_hour_digit_1;
extern const Glyph_Metadata_t time_hour_digit_2;
extern const Glyph_Metadata_t time_minute_digit_1;
extern const Glyph_Metadata_t time_minute_digit_2;

//battery voltage digits
extern const Glyph_Metadata_t battery_integer_digit_1;
extern const Glyph_Metadata_t battery_decimal_digit_1;
extern const Glyph_Metadata_t battery_V_char;

/**********************************************
 * Icon & Glyph Assets
 **********************************************/
extern const uint8_t bluetooth_broadcasting_12x12[24]; 	//bluetooth broadcasting icon 12x12 pixels
extern const uint8_t bluetooth_connected_12x12[24]; 	//bluetooth connected icon 12x12 pixels
extern const uint8_t left_signal_arrow_9x7[9];         	//left signal arrow 9x7 pixels
extern const uint8_t right_signal_arrow_9x7[9];        	//right signal arrow 9x7 pixels
extern const uint8_t fuel_bar_gauge_9x35[315];			//fuel bar gauge 9x35 pixels
extern const uint8_t rpm_bar_gauge_9x44[540];			//rpm bar gauge 9x44 pixels

/*****************************************************
 * Font Assets
 *****************************************************/
extern const uint8_t speedometer_font_16x24[480]; 		//speedometer fonts 16x24 pixels
extern const uint8_t rpm_font_5x7[50]; 					//rpm font 5x7 pixels
extern const uint8_t topbar_font_6x12[132];				//top bar font 6x12 pixels


/*****************************************************
 * Bonus Bitmap Assets
 *****************************************************/
extern const uint8_t cedric_bitmap_128x64[1024];		//cedric bitmap 128x64 pixels
extern const uint8_t center_bitmap_128x64[1024];		//center bitmap 128x64 pixels


/*****************************************************
 * Miscellaneous Glyphs for debug
 *****************************************************/
extern const uint8_t rando_12x21[36];
extern const uint8_t allwhite_12x37[60];

#endif /* SSD1306_OLED_GLYPH_ASSETS_H_ */
