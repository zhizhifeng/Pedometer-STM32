//  LED  - backlight
//  SCK  - SPI clock signal
// DC/RS - Data/Command control signal
//  RST  - Reset signal
//  CS   - Chip selection signal

#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>

// LCD important parameters
typedef struct
{
    uint16_t width;   // LCD width
    uint16_t height;  // LCD height
    uint16_t id;      // LCD ID
    uint8_t dir;      // horizontal or vertical screen display direction, 1 or 0
    uint16_t wramcmd; // start write to display RAM
    uint16_t setxcmd; // set x position
    uint16_t setycmd; // set y position
} _lcd_dev;

// LCD Handler
extern _lcd_dev hlcd;

/**
 * @brief Defines the rotation direction of the LCD screen.
 *        0 - 0 degree rotation
 *        1 - 90 degree rotation
 *        2 - 180 degree rotation
 *        3 - 270 degree rotation
 */
#define USE_HORIZONTAL 0


/**
 * @brief Defines the LCD screen size.
 * 
 */
#define LCD_W 240
#define LCD_H 320

/* Define the driver to be used(TODO)*/
/*------------------------------------
#ifdef HAL_SPI_MODULE_ENABLED
// extern SPI_HandleTypeDef hspi1;
// #define SPI_SEND(data) HAL_SPI_Transmit(&hspi1, &(data), 1, HAL_MAX_DELAY)
// #elif defined USE_LL_LIB
// extern SPI_TypeDef *SPI1;
// #define SPI_SEND(data) LL_SPI_TransmitData8(SPI1, data)
// #else
// #error "Either USE_HAL_LIB or USE_LL_LIB must be defined"
// #endif
------------------------------------*/

// color
#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40 
#define BRRED 0XFC07 
#define GRAY 0X8430  
#define DARKBLUE 0X01CF
#define LIGHTBLUE 0X7D7C
#define GRAYBLUE 0X5458
#define LIGHTGREEN 0X841F
#define LIGHTGRAY 0XEF5B
#define LGRAY 0XC618
#define LGRAYBLUE 0XA651
#define LBBLUE 0X2B12

void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteData16(uint16_t Data);
void LCD_WR_DATA(uint8_t data);
void LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd);
void LCD_Init(void);
void LCD_direction(uint8_t direction);
void LCD_Clear(uint16_t Color);

// Undeclared functions
void LCD_WriteRAM(uint16_t RGB_Code);
void LCD_SetParam(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
uint16_t LCD_ReadRAM(void);
uint16_t LCD_ReadPoint(uint16_t x, uint16_t y);
uint16_t LCD_RD_DATA(void);
uint16_t LCD_BGR2RGB(uint16_t c);

#endif
