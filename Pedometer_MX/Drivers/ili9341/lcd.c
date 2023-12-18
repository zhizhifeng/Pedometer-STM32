#include "lcd.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

// Manage important parameters of the LCD
// Default is portrait mode
_lcd_dev hlcd;

// Pen color, background color
uint16_t POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;
uint16_t DeviceCode;

/**
 * @brief Write an 8-bit command to the LCD screen
 *
 * @param data Command value to be written
 */
void LCD_WR_REG(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Write an 8-bit data to the LCD screen
 *
 * @param data Data value to be written
 */
void LCD_WR_DATA(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Write data into registers
 *
 * @param LCD_Reg Register address
 * @param LCD_RegValue Data to be written
 */
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
    LCD_WR_REG(LCD_Reg);
    LCD_WR_DATA(LCD_RegValue);
}

/**
 * @brief Write GRAM
 *
 */
void LCD_WriteRAM_Prepare(void)
{
    LCD_WR_REG(hlcd.wramcmd);
}

/**
 * @brief Write an 16-bit data to the LCD screen
 *
 * @param Data Data to be written
 */
void LCD_WriteData16(uint16_t Data)
{
    uint8_t high_byte = (Data >> 8) & 0xFF;
    uint8_t low_byte = Data & 0xFF;
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi1, &high_byte, 1, HAL_MAX_DELAY); // Send high byte first
    HAL_SPI_Transmit(&hspi1, &low_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Clear the LCD screen with a specified color
 *
 * @param Color The color to be filled
 */
void LCD_Clear(uint16_t Color)
{
    unsigned int i, m;
    LCD_SetWindows(0, 0, hlcd.width - 1, hlcd.height - 1);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    for (i = 0; i < hlcd.height; i++)
    {
        for (m = 0; m < hlcd.width; m++)
        {
            LCD_WriteData16(Color);
        }
    }
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Reset LCD screen
 *
 */
void LCD_RESET(void)
{
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
}

/**
 * @brief Initialize LCD screen
 *
 */
void LCD_Init(void)
{
    LCD_RESET();
    /* Initialize ILI9341 */
    LCD_WR_REG(0xCF);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0xD9); // C1
    LCD_WR_DATA(0X30);
    LCD_WR_REG(0xED);
    LCD_WR_DATA(0x64);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0X12);
    LCD_WR_DATA(0X81);
    LCD_WR_REG(0xE8);
    LCD_WR_DATA(0x85);
    LCD_WR_DATA(0x10);
    LCD_WR_DATA(0x7A);
    LCD_WR_REG(0xCB);
    LCD_WR_DATA(0x39);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x34);
    LCD_WR_DATA(0x02);
    LCD_WR_REG(0xF7);
    LCD_WR_DATA(0x20);
    LCD_WR_REG(0xEA);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0xC0);  // Power control
    LCD_WR_DATA(0x1B); // VRH[5:0]
    LCD_WR_REG(0xC1);  // Power control
    LCD_WR_DATA(0x12); // SAP[2:0];BT[3:0] //0x01
    LCD_WR_REG(0xC5);  // VCM control
    LCD_WR_DATA(0x26); // 3F
    LCD_WR_DATA(0x26); // 3C
    LCD_WR_REG(0xC7);  // VCM control2
    LCD_WR_DATA(0XB0);
    LCD_WR_REG(0x36); // Memory Access Control
    LCD_WR_DATA(0x08);
    LCD_WR_REG(0x3A);
    LCD_WR_DATA(0x55);
    LCD_WR_REG(0xB1);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x1A);
    LCD_WR_REG(0xB6); // Display Function Control
    LCD_WR_DATA(0x0A);
    LCD_WR_DATA(0xA2);
    LCD_WR_REG(0xF2); // 3Gamma Function Disable
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0x26); // Gamma curve selected
    LCD_WR_DATA(0x01);
    LCD_WR_REG(0xE0); // Set Gamma
    LCD_WR_DATA(0x1F);
    LCD_WR_DATA(0x24);
    LCD_WR_DATA(0x24);
    LCD_WR_DATA(0x0D);
    LCD_WR_DATA(0x12);
    LCD_WR_DATA(0x09);
    LCD_WR_DATA(0x52);
    LCD_WR_DATA(0xB7);
    LCD_WR_DATA(0x3F);
    LCD_WR_DATA(0x0C);
    LCD_WR_DATA(0x15);
    LCD_WR_DATA(0x06);
    LCD_WR_DATA(0x0E);
    LCD_WR_DATA(0x08);
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0XE1); // Set Gamma
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x1B);
    LCD_WR_DATA(0x1B);
    LCD_WR_DATA(0x02);
    LCD_WR_DATA(0x0E);
    LCD_WR_DATA(0x06);
    LCD_WR_DATA(0x2E);
    LCD_WR_DATA(0x48);
    LCD_WR_DATA(0x3F);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0x0A);
    LCD_WR_DATA(0x09);
    LCD_WR_DATA(0x31);
    LCD_WR_DATA(0x37);
    LCD_WR_DATA(0x1F);

    LCD_WR_REG(0x2B);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x01);
    LCD_WR_DATA(0x3f);
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0xef);
    LCD_WR_REG(0x11); // Exit Sleep
    HAL_Delay(120);
    LCD_WR_REG(0x29); // display on

    LCD_direction(USE_HORIZONTAL); // Set the display direction of LCD screen
    // LCD_LED = 1;                // Set the backlight
    LCD_Clear(WHITE); // Clear the screen
}

/**
 * @brief Set the LCD display window
 *
 * @param xStar The bebinning x coordinate of the LCD display window
 * @param yStar The bebinning y coordinate of the LCD display window
 * @param xEnd The endning x coordinate of the LCD display window
 * @param yEnd The endning y coordinate of the LCD display window
 */
void LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd)
{
    LCD_WR_REG(hlcd.setxcmd);
    LCD_WR_DATA(xStar >> 8);
    LCD_WR_DATA(0x00FF & xStar);
    LCD_WR_DATA(xEnd >> 8);
    LCD_WR_DATA(0x00FF & xEnd);

    LCD_WR_REG(hlcd.setycmd);
    LCD_WR_DATA(yStar >> 8);
    LCD_WR_DATA(0x00FF & yStar);
    LCD_WR_DATA(yEnd >> 8);
    LCD_WR_DATA(0x00FF & yEnd);

    LCD_WriteRAM_Prepare(); // Start writing GRAM
}

/**
 * @brief Set the display direction of LCD screen
 *
 * @param direction 0-0 degree; 1-90 degree; 2-180 degree; 3-270 degree
 */
void LCD_direction(uint8_t direction)
{
    hlcd.setxcmd = 0x2A;
    hlcd.setycmd = 0x2B;
    hlcd.wramcmd = 0x2C;
    switch (direction)
    {
    case 0:
        hlcd.width = LCD_W;
        hlcd.height = LCD_H;
        LCD_WriteReg(0x36, (1 << 3) | (0 << 6) | (0 << 7)); // BGR==1,MY==0,MX==0,MV==0
        break;
    case 1:
        hlcd.width = LCD_H;
        hlcd.height = LCD_W;
        LCD_WriteReg(0x36, (1 << 3) | (0 << 7) | (1 << 6) | (1 << 5)); // BGR==1,MY==1,MX==0,MV==1
        break;
    case 2:
        hlcd.width = LCD_W;
        hlcd.height = LCD_H;
        LCD_WriteReg(0x36, (1 << 3) | (1 << 6) | (1 << 7)); // BGR==1,MY==0,MX==0,MV==0
        break;
    case 3:
        hlcd.width = LCD_H;
        hlcd.height = LCD_W;
        LCD_WriteReg(0x36, (1 << 3) | (1 << 7) | (1 << 5)); // BGR==1,MY==1,MX==0,MV==1
        break;
    default:
        break;
    }
}
