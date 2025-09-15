
/*************************************************************************************
 * SSD1306/SSD1315 OLED Driver for STM32 ARM Cortex-M4 processor
 * Author: Eldon Jake Enerio
 * Note: Currently ported for STM32WB5x series, change the HAL library according to your STM32 series
 *************************************************************************************/


#include "ssd1306_ssd1315.h"

//command bytes buffer array
static uint8_t command_bytes[3];

/*********************************************
 * OLED Screen Buffer, 128X64px
 * Stored in RAM
 ****************************************/
static uint8_t OLED_ScreenBuffer[OLED_BUFFER_SIZE] = {
		0xff, 0x01, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0x01, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0x01,
		0x41, 0x01, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0x01, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0x01, 0x01,
		0x01, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0x01, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0x01, 0xff,
		0xff, 0x00, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x00,
		0x04, 0x00, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x20,
		0x00, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0xff,
		0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xff, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xff,
		0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
		0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xc0, 0x30, 0x00, 0xf0, 0x90, 0xf0, 0x00, 0xf0,
		0x40, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
		0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03,
		0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
		0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x20, 0xa0, 0xa0,
		0x20, 0x20, 0xa0, 0xa0, 0xa0, 0x20, 0xa0, 0x20, 0x20, 0x20, 0xa0, 0x20, 0x20, 0x20, 0xa0, 0xa0,
		0xa0, 0xa0, 0xa0, 0x20, 0x20, 0x20, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0x20, 0x20, 0x20, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xff, 0x00, 0x80, 0xc0, 0x40, 0x40, 0x40, 0xc0, 0x80, 0x80, 0x00, 0x00, 0xff,
		0xff, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xff, 0x80, 0xbf, 0x84,
		0xbb, 0x80, 0xbf, 0x84, 0x87, 0x80, 0xbf, 0x81, 0x82, 0x81, 0xbf, 0x80, 0x94, 0x80, 0xbf, 0xa0,
		0xa0, 0xa0, 0xbf, 0x80, 0xa0, 0x80, 0xbf, 0xa0, 0xa0, 0xa0, 0xbf, 0x80, 0xa8, 0x90, 0xa8, 0x80,
		0xb8, 0x80, 0xb8, 0x90, 0xa8, 0x80, 0xff, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0xff, 0x80, 0xbf, 0xff, 0xc2, 0xea, 0xea, 0xff, 0xbf, 0xa1, 0x9f, 0x80, 0xff
};

/*
 * @brief: Write command bytes to the OLED through I2C peripheral
 *
 * @param: commandByte: pointer to the command byte array
 *
 * @param: command_length: length of the command byte array
 */
void oled_WriteCommand(uint8_t *commandByte, size_t command_length) {

	/*
	 * Sends the command bytes through polling mode
	 * DMA mode for future optimizations
	 */

	//HAL_I2C_Mem_Write_DMA(&I2C_OLED_HANDLE, OLED_ADDR, 0x00, 1, commandByte, command_length);
	HAL_I2C_Mem_Write(&I2C_OLED_HANDLE, OLED_ADDR, 0x00, 1, commandByte, command_length, HAL_MAX_DELAY);

}


/*
 * @brief: Write into the GDDRAM of the OLED from the oled_screenbuffer through I2C peripheral
 *
 * @param: pBuffer: pointer to the data buffer to be sent to the OLED
 * @param: bufferSize: size of the data buffer to be sent to the OLED
 */
void oled_WriteData(uint8_t *pBuffer, size_t bufferSize) {

	/*
	 * Writes oled_screenbuffer data to GDDRAM in polling mode.
	 * DMA mode for future optimizations
	 */

	HAL_I2C_Mem_Write(&I2C_OLED_HANDLE, OLED_ADDR, 0x40, 1, pBuffer, bufferSize, HAL_MAX_DELAY);
	//HAL_I2C_Mem_Write_DMA(&I2C_OLED_HANDLE, OLED_ADDR, 0x40, 1, pBuffer, bufferSize);
}



/*
 * @brief Initializes the SSD1306/SSD1315 OLED screen
 */
void oled_init(void) {

	uint8_t startline = 0;


	//Sets display to OFF for initialization
	oled_SetDisplayOn(OFF);
	HAL_Delay(10); //wait for 10ms

	command_bytes[0] = 0x20; //command for setting memory addressing mode
	command_bytes[1] = 0x00; //0x00 = horizontal addressing mode, 0x01 = vertical addressing mode, 0x02 = page addressing mode
	oled_WriteCommand(command_bytes, 2); //set memory addressing mode

	command_bytes[0] = 0x21; //command for setting column address
	command_bytes[1] = 0x00; //column start address
	command_bytes[2] = 0x7F; //column end address
	oled_WriteCommand(command_bytes, 3); //Set column address for horizontal addressing mode

	command_bytes[0] = 0x22; //command for setting page address
	command_bytes[1] = 0x00; //page start address
	command_bytes[2] = 0x07; //page end address
	oled_WriteCommand(command_bytes, 3); //Set page address for horizontal addressing mode

	command_bytes[0] = 0xC8; //command for setting COM output scan direction
	oled_WriteCommand(command_bytes, 1); //Set COM output scan direction, 0xC8 = top to bottom, 0xC0 = bottom to top (inverted)

	command_bytes[0] = 0x40 | startline; //command for setting display start line
	oled_WriteCommand(command_bytes, 1); //Set start line address, startline values 0 t 63.

	oled_SetContrast(0xFF);

	command_bytes[0] = 0xA1; //command for setting segment re-map
	oled_WriteCommand(command_bytes, 1); //No horizontal mirroring, 0XA1 = no horizontal mirroring, col 0 to col 127
	//0xA0 = horizontal mirroring, col 127 to col 0

	command_bytes[0] = 0xA6; //command for setting display mode;
	oled_WriteCommand(command_bytes, 1); //Set normal color, 0xA6 = normal color, 0xA7 = inverted color display

	//Set multiplex ratio A.K.A number of rows to display on screen
	command_bytes[0] = 0xA8; //command for setting multiplex ratio
	command_bytes[1] = 0x3F; //0x3F = 63 dec, meaning rows 0 to rows 63 are displayed on the screen
	oled_WriteCommand(command_bytes, 2); //Set multiplex ratio

	command_bytes[0] = 0XA4;
	oled_WriteCommand(command_bytes, 1); //Normal operation, 0xA4 = output follow RAM content
	//0xA5 = output does not follow RAM content. Turns ON all pixels irregardless of RAM content

	//Set display offset
	command_bytes[0] = 0XD3; //command for setting display offset
	command_bytes[1] = 0X00; //0x00 = 0, means no vertical display offset; 0x3F = 63 max value vertical display offset
	oled_WriteCommand(command_bytes, 2); //If there is vertical offset, the display is shifted up by n rows according to programmed offset value.

	//Sets oscillator freq & display clock divide ratio
	command_bytes[0] = 0xD5; //command for setting the display clock divide ratio/oscillator freq
	command_bytes[1] = 0XF0; //0xF0 means F = 1111b(15dec), highest nibble which sets the oscillator frequency in Hz
	oled_WriteCommand(command_bytes, 2); //0xF0 mean 0 = divide ratio values 0 to 15, both setting determine the Frame Freq (Hz)= Fosc / (D + 1) / 128

	command_bytes[0] = 0xD9; //command for setting the pre-charge period
	command_bytes[1] = 0x22; //each nibble corresponds to phase 2 Most Sig Nibble & phase 1 Least Sig Nibble. Duration is in DCLK cycles
	oled_WriteCommand(command_bytes, 2); //command for setting the pre-charge period
	//default setting is 2 DCLK cycles or 0x22.

	command_bytes[0] = 0xDA; //command for setting COM pins hardware configuration;
	command_bytes[1] = 0x12; //0x12 = 0001 0010, Bit 5 = 0, Sequential COM pin config; Bit 4 = 1, COM pin left/right remap ; Bit 1 = 1 (reset value 0x2)
	oled_WriteCommand(command_bytes, 2); //COM pins hardware config, NOTE: in datasheet Bit 4 should be Bit 5 and Bit 5 should be bit 4. Baliktad in data sheet. PISTE!


	command_bytes[0] = 0XDB; //command for setting VCOMH deselect level
	command_bytes[1] = 0x20; //0x20, 0.77xVcc, at Vcc 3.3V estimate OLED voltage drop is only at 0.759V
	oled_WriteCommand(command_bytes, 2); //set VCOMH voltage level, VCOMH is the COM pin voltage level when its idle state (not pulled to GND)
	//0x20, 0.77xVcc, at Vcc 3.3V estimate OLED voltage drop is only at 0.759V which is lower than OLED threshold voltage of ~1.3V

	command_bytes[0] = 0X8D; //command for setting charge pump
	command_bytes[1] = 0X14; //0x14, enables the charge pump during display ON
	oled_WriteCommand(command_bytes, 2);

	oled_UpdateScreen(); //update oled with the splashscreen

	oled_SetDisplayOn(ON); 	 //turn on OLED panel
}


/*
 * @breif: Draws a pixel at the specified (x,y) coordinate in the OLED screen buffer
 *
 * @param: x		- x coordinate of the pixel, 0 to 127
 * @param: y		- y coordinate of the pixel, 0 to 63
 * @param: color	- OLED_COLOR ENUM to draw the pixel in Black or White
 */
void oled_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color) {

	if(x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
		// Don't write outside the buffer
		return;
	}

	// Draw in the right color
	if(color == White) {
		//set the pixel (turn on the pixel)
		OLED_ScreenBuffer[x + ((y / 8) * WIDTH)] |= 1 << (y % 8);
	} else {
		//color is Black (turn off the pixel)
		OLED_ScreenBuffer[x + ((y / 8) * WIDTH)] &= ~(1 << (y % 8));
	}
}

/*
 * @breif: Fills the entire OLED screen buffer with the specified color
 *
 * @param: color - OLED_COLOR ENUM to fill the screen buffer with Black or White
 */
void oled_Fill(OLED_COLOR color) {
	memset(OLED_ScreenBuffer, (color == Black) ? 0x00 : 0xFF, sizeof(OLED_ScreenBuffer));
}


/*
 * @breif: Dumps the OLED screen buffer to the OLED GDDRAM through I2C peripheral
 *
 */
void oled_UpdateScreen(void) {

	//dump all screen buffer data into GDDRAM
	oled_WriteData(OLED_ScreenBuffer,OLED_BUFFER_SIZE);
}


/*
 * @breif: Turns the OLED display ON or OFF
 *
 * @param: ONorOff - ON_OFF ENUM to turn the display ON or OFF
 */
void oled_SetDisplayOn(ON_OFF ONorOff) {

	if(ONorOff == ON){
		command_bytes[0] = 0xAF;
		oled_WriteCommand(command_bytes, 1);	//OLED ON command
	}
	else{
		command_bytes[0] = 0xAE;
		oled_WriteCommand(command_bytes, 1);	//OLED OFF command
	}
}


/*
 * @breif: Sets the contrast level of the OLED display
 *
 * @param: contrast - Contrast value from 0x00 to 0xFF
 */
void oled_SetContrast(uint8_t contrast)	{

	command_bytes[0] = 0x81; //command for setting contrast
	command_bytes[1] = contrast; //contrast value 0x00 to 0xFF

	oled_WriteCommand(command_bytes, 2);

}

/****************************************************************************************************************
 * @breif: Renders a monochrome bitmap glyph to the OLED screen at specified (x,y) coordinate.
 *
 * @param: x_start		- 	starting x coordinate of the glyph, 0 to 127
 *
 * @param: y_start 		- 	starting y coordinate of the glyph, 0 to 63
 *
 * @param: glyph 		- 	pointer to the glyph bitmap array
 *
 * @param: glyph_width 	- 	width of the glyph in pixels, 1 to 128
 *
 * @param: glyph_height - 	height of the glyph in pixels, 1 to 64
 *
 * @param: mode			- 	GlyphRederMode_t ENUM for rendering mode. Options are:
 * 							GLYPH_MODE_OVERWRITE 	- 	Clears the glyph region before rendering the glyph
 * 							GLYPH_MODE_SET 			- 	Sets the pixels of the glyph to ON state
 *							GLYPH_MODE_AND_MASK 	- 	Applies AND mask to the glyph region before rendering the glyph
 *							GLYPH_MODE_TOGGLE  		- 	Toggles the pixels of the glyph
 *							GLYPH_MODE_CLEAR   		- 	Clears the pixels of the glyph region to OFF state
 *
 * @note: x_start & y_start uses zero based counting. The top left corner of the screen is (0,0).
 * @note: The glyph's top left corner is the x_start & y_start coordinate.
 *
 *******************************************************************************************************************/
void oled_drawBMP(uint8_t x_start, uint8_t y_start, const uint8_t *glyph, uint8_t glyph_width, uint8_t glyph_height, GlyphRederMode_t mode)
{
	//this flag is used to check if the glyph region has been cleared
	uint8_t glyph_region_cleared = 0;

	uint8_t y_end = (y_start + glyph_height) - 1; //y end coordinate
	uint8_t x_end = (x_start + glyph_width) - 1; //x end coordinate

	/*
	 * calculate bit offset if y coordinate is not page aligned (meaning not divisible by 8)
	 * if y is paged aligned bit_offset should be equal to 0
	 */
	uint8_t bit_offset = (y_start % 8);

	/*
	 * Calculates how many pages the glyph needs to render
	 * The +7 is to round up the division A.K.A Ceiling Division,
	 * to make sure any remainder bits that spilled to the next page are accounted for
	 *
	 * Ex. glyph_height = 17, so you have 1 bit that spilled to the next page in total of 3 pages needed, 8(page1) + 8(page2) + 1(page3)
	 * 		If you divide 17/8 = 2.125, you only get 2, so you add +7 to numerator to round up the division
	 * 		17 + 7 = 24, 24/8 = 3, which is the correct number of pages needed to render the glyph
	 */
	uint8_t pages_needed = (glyph_height + 7) / 8;

	uint8_t start_page = y_start / 8; 	//starting page location
	uint8_t end_page = start_page + pages_needed;	//end page location

	//boundary check
	if(x_start < 0 || x_start >= WIDTH || y_start < 0 || y_start >= HEIGHT || x_end >= WIDTH || y_end >= HEIGHT) {
		// Don't write outside the buffer
		return;
	}

	//loops from start_page to (end_page -1) of the glyph
	//skips the last page since it will be handled by the carry_glyph_byte
	for (uint8_t page_counter = start_page; page_counter < end_page; page_counter++) {

		//pre-calculate glyph page offset to optimize the inner loop
		uint8_t glyph_page_offset = (page_counter - start_page) * glyph_width;

		for (uint8_t x_counter = x_start; x_counter <= x_end; x_counter++) {
			/*
			 * x_counter: current column of the glyph to render, moves through the glyph array horizontally bound by glyph_width
			 * x_start: starting x coordinate of the glyph, used to offset the glyph array index to start from 0
			 * glyph_page_offset: used to offset the glyph array index to jump to the next page of the glyph
			 */
			uint8_t glyph_byte = glyph[ (x_counter - x_start) + glyph_page_offset];

			//main glyph byte value with applied offset if y is not page aligned (meaning not divisible by 8)
			uint8_t main_glyph_byte = glyph_byte << bit_offset;

			/*
			 * if y value is not page aligned (not divisible by 8), there will be pixels that spill to the next page
			 * this handles the pixels that overflowed to next page and make sure they are rendered correctly
			 *
			 * Ex. byte = 10011101, y = 9,so bit_offset = 9 % 8 = 1,
			 * so you'd shift the MSB to the next page, to get the bit displayed to the next page, right shift by 8 - bit_offset.
			 * so, carry = 10011101 >> (8 - 1) = 1, this bit (pixel) will then be written to next page of OLED GDDRAM
			 */
			uint8_t carry_glyph_byte = glyph_byte >> (8 - bit_offset);

			/*
			 * calculates the actual index in OLED_buffer where the glyph byte will be stored and rendered
			 * x_counter: actual column in GDDRAM where the glyph byte will be stored
			 * (page_counter * WIDTH) = page starting index in the OLED buffer
			 * upon adding the two, you get the actual index in OLED_buffer
			 *
			 * Ex. x_start = 10, page_counter = 1
			 * index = (10) + (1 * 128) = 138, so the glyph byte will be stored in OLED_buffer[138]
			 */
			uint16_t index = (x_counter) + (page_counter * WIDTH);

			switch (mode) {
			case GLYPH_MODE_OVERWRITE:

				/*
				 * Clears the whole region where the glyph will render (x + glyph_width) * (y + glyph_height)
				 * To ensure no ghosting effect if glyph is being rendered multiple times
				 */
				if (glyph_region_cleared == 0) {
					oled_clearGlyphRegion(x_start, y_start, glyph_width, glyph_height);
					glyph_region_cleared = 1; //set flag to prevent clearing the page again
				}

				OLED_ScreenBuffer[index] |= main_glyph_byte;
				break;
			case GLYPH_MODE_SET:
				OLED_ScreenBuffer[index] |= main_glyph_byte;
				break;
			case GLYPH_MODE_TOGGLE:
				OLED_ScreenBuffer[index] ^= main_glyph_byte;
				break;
			case GLYPH_MODE_AND_MASK:
				OLED_ScreenBuffer[index] &= ~(main_glyph_byte);
				OLED_ScreenBuffer[index] |= main_glyph_byte;
				break;
			case GLYPH_MODE_CLEAR:
				oled_clearGlyphRegion(x_start, y_start, glyph_width, glyph_height);
				break;
			}

			/*
			 * Handle pixels that spilled into the next page
			 * If carry_glpyh_byte = 0, meaning no pixels spilled on to next page and y coordinate is page aligned (divisible by 8)
			 * If carry_glpyh_byte != 0, meaning there are pixels that spilled to next page and needs to be rendered
			 *
			 * added condition (page_counter + 1) < 8 to prevent writing outside the OLED_ScreenBuffer & OLED GDDRAM boundaries
			 */
			if (carry_glyph_byte != 0 && (page_counter + 1) < PAGE_LIMIT) {

				/*
				 * similar to previous calculation of index but this time +1 to the page number
				 * to jump to the next page index
				 */
				uint16_t next_index = (x_counter) + ((page_counter + 1) * WIDTH);

				switch (mode) {
				case GLYPH_MODE_OVERWRITE:
					OLED_ScreenBuffer[next_index] |= carry_glyph_byte;
					break;
				case GLYPH_MODE_SET:
					OLED_ScreenBuffer[next_index] |= carry_glyph_byte;
					break;
				case GLYPH_MODE_TOGGLE:
					OLED_ScreenBuffer[next_index] ^= carry_glyph_byte;
					break;
				case GLYPH_MODE_AND_MASK:
					OLED_ScreenBuffer[next_index] &= ~(carry_glyph_byte);
					OLED_ScreenBuffer[next_index] |= carry_glyph_byte;
					break;
				case GLYPH_MODE_CLEAR:
					//do nothing since the whole glyph region has already been cleared
					break;
				}
			}

			//uncomment if for debugging purposes
			//oled_UpdateScreen();
		}

	}

	//oled_UpdateScreen();
}


/****************************************************************************************************************
 * @breif: Clears the retangular region that a glyph occupies in the OLED.
 *
 * @param: x_start - starting x coordinate of the glyph region, 0 to 127
 *
 * @param: y_start - starting y coordinate of the glyph region, 0 to 63
 *
 * @param: glyph_width - width of the glyph in pixels, 1 to 128
 *
 * @param: glyph_height - height of the glyph in pixels, 1 to 64
 *
 * @note: x_start & y_start uses zero based counting.
 *
 *******************************************************************************************************************/
void oled_clearGlyphRegion(uint8_t x_start, uint8_t y_start, uint8_t glyph_width, uint8_t glyph_height)	{


	uint8_t y_end = (y_start + glyph_height) - 1; //y end coordinate
	uint8_t x_end = (x_start + glyph_width) - 1; //x end coordinate

	uint8_t page_start = y_start / 8; 	//starting page location
	uint8_t page_end = y_end / 8; 	//ending page location

	uint8_t mask;
	uint8_t start_mask = 0xFF << (y_start % 8); //pre-calculate mask for the starting page
	uint8_t end_mask = 0xFF >> (8 - ((y_end % 8) + 1) ); //pre-calculate mask for the ending page

	//boundary check
	if(x_start < 0 || x_start >= WIDTH || y_start < 0 || y_start >= HEIGHT ||  x_end >= WIDTH || y_end >= HEIGHT) {
		// Don't write outside the buffer
		return;
	}

	//loop from start page to end page of region
	for (uint8_t page_counter = page_start; page_counter <= page_end; page_counter++) {

		//determine the mask to use for clearing the pixels in the glyph region

		//if glyph fits within a single page
		if(page_start == page_end && page_counter == page_end) {
			mask = start_mask & end_mask;
		}

		/*
		 * at 1st page, if glyph is not page aligned, mask only the bits (pixels) from y_start to page y_end
		 * Ex. if y_start = 18, so bit_offset = 18 % 8 = 2, so mask = 11111111 << 2 = 11111100
		 * this will only clear the pixels from bit 2 to bit 7 (y = 18 to y = 23) of page 2
		 * leaving bit 0 and bit 1 (y = 16 & 17, of page 2) untouched
		 */
		else if (page_counter == page_start) {
			mask = start_mask;
		}

		/*
		 * if in the middle pages, meaning the whole page is occupied by the glyph region,
		 * clear the whole 8 bits (pixels) of the page (page aligned clearing),
		 */
		else if(page_counter > page_start && page_counter < page_end) {
			mask = 0xFF;
		}
		/*
		 * if in the last page, mask only the bits (pixels) from page y_start to glyph y_end
		 * Ex. if y_page_start = 40 & page_y_end = 41, so mask = 11111111 >> (8 - ((41 - 40) + 1) ) = 00000011
		 *     this will only clear the pixels from bit 0 to bit 1 (y = 40 to y = 41) of page 5
		 *     leaving bit 2 to bit 7 (y = 42 to y = 47, of page 5) untouched
		 */
		else if(page_counter == page_end) {

			mask = end_mask;
		}

		//use pointer arithmetic to not have to recalculate the index in every iteration of the inner loop. Optimize performance.
		uint8_t *OLED_bufferptr = &OLED_ScreenBuffer[x_start + (page_counter * WIDTH)];

		//loop from 0 to glyph_width to clear the pixels in the glyph region
		for (uint8_t x_counter = 0; x_counter < glyph_width; x_counter++) {

			OLED_bufferptr[x_counter] &= ~(mask); //inverse the mask to only clear the related pixels
		}
	}
}


/*********************************************************************************
 * @breif: Renders a glyph to the OLED screen using the glyph metadata structure
 *
 * @param: glyph_meta 	- 	pointer to the Glyph_Metadata_t structure containing glyph information.
 * 							Ref  @Glyph_Metadata_Structures for available glyphs
 * @param: mode			- 	GlyphRederMode_t ENUM for rendering mode. Options are:
 * 							GLYPH_MODE_OVERWRITE 	- 	Clears the glyph region before rendering the glyph
 * 							GLYPH_MODE_SET 			- 	Sets the pixels of the glyph to ON state
 *							GLYPH_MODE_AND_MASK 	- 	Applies AND mask to the glyph region before rendering the glyph
 *							GLYPH_MODE_TOGGLE  		- 	Toggles the pixels of the glyph
 *							GLYPH_MODE_CLEAR   		- 	Clears the pixels of the glyph region to OFF state
 * @param: GlyphIndex	-	Index of the glyph in the glyph bitmap array to render, this selects which glyph to render if the bitmap array has multiple glyphs stored inside
 * 							If glyph bitmap array only contains one glyph, set this parameter to 0.
 *******************************************************************************/
void oled_RenderGlyph(const Glyph_Metadata_t *glyph_meta, GlyphArray_Index_t GlyphIndex,GlyphRederMode_t mode) {

	//Check if glyph bitmao array has multiple glyphs stored inside
	if(glyph_meta->elements_per_glpyh > 0 && glyph_meta->glyph_index_limit > 0){

		//boundary check for glyph index
		if (GlyphIndex > glyph_meta->glyph_index_limit) {
			//invalid glyph index, out of bounds, return
			return;
		}
		//if within range, render the glyph.
		oled_drawBMP(glyph_meta->x_coordinate, glyph_meta->y_coordinate,&(glyph_meta->bitmap[GlyphIndex * (glyph_meta->elements_per_glpyh)]), glyph_meta->width, glyph_meta->height, mode);
	}else {
		//if only one glyph is stored in the bitmap array
		oled_drawBMP(glyph_meta->x_coordinate, glyph_meta->y_coordinate, glyph_meta->bitmap, glyph_meta->width, glyph_meta->height, mode);
	}
}


/***********************************************************************************
 * @breif: Clears a glyph in the OLED screen buffer using the glyph metadata structure
 *
 * @param: glyph_meta 	- 	pointer to the Glyph_Metadata_t structure containing glyph information
 **********************************************************************************/
void oled_ClearGlyph(const Glyph_Metadata_t *glyph_meta) {

	oled_clearGlyphRegion(glyph_meta->x_coordinate, glyph_meta->y_coordinate,glyph_meta->width, glyph_meta->height);

}
