#include "lcd.h"
#include "string.h"
#include "font.h"
#include "gui.h"

/**
 * @brief Draw a point in LCD screen
 *
 * @param x the x coordinate of the point
 * @param y the y coordinate of the point
 * @param color the color value of the point
 */
void GUI_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_SetWindows(x, y, x, y);
    LCD_WriteData16(color);
}

/**
 * @brief Fill the specified area with a specified color
 *
 * @param sx the bebinning x coordinate of the specified area
 * @param sy the bebinning y coordinate of the specified area
 * @param ex the ending x coordinate of the specified area
 * @param ey the ending y coordinate of the specified area
 * @param color the filled color value
 */
void GUI_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    uint16_t i, j;
    uint16_t width = ex - sx + 1;   // fill area width
    uint16_t height = ey - sy + 1;  // fill area height
    LCD_SetWindows(sx, sy, ex, ey); // set fill area
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
            LCD_WriteData16(color); // write color value
    }
    LCD_SetWindows(0, 0, hlcd.width - 1, hlcd.height - 1); // reset full screen
}

/**
 * @brief Draw a line between two points
 *
 * @param x1 the beginning x coordinate of the line
 * @param y1 the beginning y coordinate of the line
 * @param x2 the ending x coordinate of the line
 * @param y2 the ending y coordinate of the line
 */
void GUI_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int dir_x, dir_y, uRow, uCol;

    delta_x = x2 - x1; // calculate the increment of x and y
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)
        dir_x = 1; // set the increment direction
    else if (delta_x == 0)
        dir_x = 0; // vertical line
    else
    {
        dir_x = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        dir_y = 1;
    else if (delta_y == 0)
        dir_y = 0; // horizontal line
    else
    {
        dir_y = -1;
        delta_y = -delta_y;
    }
    /* Choose the basic incremental axis */
    if (delta_x > delta_y)
        distance = delta_x;
    else
        distance = delta_y;
    for (t = 0; t <= distance + 1; t++)
    {
        GUI_DrawPoint(uRow, uCol, WHITE);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += dir_x;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += dir_y;
        }
    }
}

/**
 * @brief Draw a rectangle
 *
 * @param x1 the beginning x coordinate of the rectangle
 * @param y1 the beginning y coordinate of the rectangle
 * @param x2 the ending x coordinate of the rectangle
 * @param y2 the ending y coordinate of the rectangle
 */
void GUI_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    GUI_DrawLine(x1, y1, x2, y1);
    GUI_DrawLine(x1, y1, x1, y2);
    GUI_DrawLine(x1, y2, x2, y2);
    GUI_DrawLine(x2, y1, x2, y2);
}

/**
 * @brief Draw a filled rectangle
 *
 * @param x1 the beginning x coordinate of the rectangle
 * @param y1 the beginning y coordinate of the rectangle
 * @param x2 the ending x coordinate of the rectangle
 * @param y2 the ending y coordinate of the rectangle
 */
void GUI_DrawFilledRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    GUI_Fill(x1, y1, x2, y2, color);
}

/**
 * @brief 8 symmetry circle drawing algorithm (internal call)
 *
 * @param xc the x coordinate of the Circular center
 * @param yc the y coordinate of the Circular center
 * @param x the x coordinate relative to the Circular center
 * @param y the y coordinate relative to the Circular center
 * @param color the color value of the circle
 */
void _draw_circle_8(int xc, int yc, int x, int y, uint16_t color)
{
    GUI_DrawPoint(xc + x, yc + y, color);

    GUI_DrawPoint(xc - x, yc + y, color);

    GUI_DrawPoint(xc + x, yc - y, color);

    GUI_DrawPoint(xc - x, yc - y, color);

    GUI_DrawPoint(xc + y, yc + x, color);

    GUI_DrawPoint(xc - y, yc + x, color);

    GUI_DrawPoint(xc + y, yc - x, color);

    GUI_DrawPoint(xc - y, yc - x, color);
}

/**
 * @brief Draw a circle of specified size at a specified location
 *
 * @param xc the x coordinate of the Circular center
 * @param yc the y coordinate of the Circular center
 * @param color the color value of the circle
 * @param r the radius of the circle
 * @param fill 1-filling, 0-no filling
 */
void GUI_DrawCircle(int xc, int yc, uint16_t color, int r, int fill)
{
    int x = 0, y = r, yi, d;

    d = 3 - 2 * r;

    if (fill)
    {
        // Draw a filled circle
        while (x <= y)
        {
            for (yi = x; yi <= y; yi++)
                _draw_circle_8(xc, yc, x, yi, color);

            if (d < 0)
            {
                d = d + 4 * x + 6;
            }
            else
            {
                d = d + 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    }
    else
    {
        // Draw a hollow circle
        while (x <= y)
        {
            _draw_circle_8(xc, yc, x, y, color);
            if (d < 0)
            {
                d = d + 4 * x + 6;
            }
            else
            {
                d = d + 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    }
}

/**
 * @brief Draw a triangle at a specified position
 *
 * @param x0 the beginning x coordinate of the triangular edge
 * @param y0 the beginning y coordinate of the triangular edge
 * @param x1 the vertex x coordinate of the triangular
 * @param y1 the vertex y coordinate of the triangular
 * @param x2 the ending x coordinate of the triangular edge
 * @param y2 the ending y coordinate of the triangular edge
 */
void GUI_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    GUI_DrawLine(x0, y0, x1, y1);
    GUI_DrawLine(x1, y1, x2, y2);
    GUI_DrawLine(x2, y2, x0, y0);
}

/**
 * @brief Swap two numbers (internal call)
 *
 * @param a the first number
 * @param b the second number
 */
static void _swap(uint16_t *a, uint16_t *b)
{
    uint16_t tmp;
    tmp = *a;
    *a = *b;
    *b = tmp;
}

/**
 * @brief Draw a filled triangle at a specified position
 *
 * @param x0 the beginning x coordinate of the triangular edge
 * @param y0 the beginning y coordinate of the triangular edge
 * @param x1 the vertex x coordinate of the triangular
 * @param y1 the vertex y coordinate of the triangular
 * @param x2 the ending x coordinate of the triangular edge
 * @param y2 the ending y coordinate of the triangular edge
 */
void GUI_DrawFilledTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t a, b, y, last;
    int dx01, dy01, dx02, dy02, dx12, dy12;
    long sa = 0;
    long sb = 0;
    if (y0 > y1)
    {
        _swap(&y0, &y1);
        _swap(&x0, &x1);
    }
    if (y1 > y2)
    {
        _swap(&y2, &y1);
        _swap(&x2, &x1);
    }
    if (y0 > y1)
    {
        _swap(&y0, &y1);
        _swap(&x0, &x1);
    }
    if (y0 == y2)
    {
        a = b = x0;
        if (x1 < a)
        {
            a = x1;
        }
        else if (x1 > b)
        {
            b = x1;
        }
        if (x2 < a)
        {
            a = x2;
        }
        else if (x2 > b)
        {
            b = x2;
        }
        GUI_Fill(a, y0, b, y0, color);
        return;
    }
    dx01 = x1 - x0;
    dy01 = y1 - y0;
    dx02 = x2 - x0;
    dy02 = y2 - y0;
    dx12 = x2 - x1;
    dy12 = y2 - y1;

    if (y1 == y2)
    {
        last = y1;
    }
    else
    {
        last = y1 - 1;
    }
    for (y = y0; y <= last; y++)
    {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if (a > b)
        {
            _swap(&a, &b);
        }
        GUI_Fill(a, y, b, y, color);
    }
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++)
    {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if (a > b)
        {
            _swap(&a, &b);
        }
        GUI_Fill(a, y, b, y, color);
    }
}

/**
 * @brief Display a ASCII character
 *
 * @param x the bebinning x coordinate of the character display position
 * @param y the bebinning y coordinate of the character display position
 * @param front_color the color value of display character
 * @param back_color the background color of display character
 * @param num the ascii code of display character(0~94)
 * @param size the size of display character
 * @param mode 0-no overlying, 1-overlying
 */
void GUI_DrawChar(uint16_t x, uint16_t y, uint16_t front_color, uint16_t back_color, uint8_t num, uint8_t size, uint8_t mode)
{
    uint8_t temp;
    uint8_t pos, t;

    num = num - ' ';                                      // Get the offset value of the character
    LCD_SetWindows(x, y, x + size / 2 - 1, y + size - 1); // Set the display area
    if (!mode)                                            // Use non-overlying mode
    {
        for (pos = 0; pos < size; pos++)
        {
            if (size == 12)
                temp = asc2_1206[num][pos]; // Use 1206 font
            else
                temp = asc2_1608[num][pos]; // Use 1608 font
            for (t = 0; t < size / 2; t++)
            {
                if (temp & 0x01)
                    LCD_WriteData16(front_color);
                else
                    LCD_WriteData16(back_color);
                temp >>= 1;
            }
        }
    }
    else // 叠加方式
    {
        for (pos = 0; pos < size; pos++)
        {
            if (size == 12)
                temp = asc2_1206[num][pos];
            else
                temp = asc2_1608[num][pos];
            for (t = 0; t < size / 2; t++)
            {
                if (temp & 0x01)
                    GUI_DrawPoint(x + t, y + pos, front_color);
                temp >>= 1;
            }
        }
    }
    LCD_SetWindows(0, 0, hlcd.width - 1, hlcd.height - 1);
}

/**
 * @brief Get the nth power of m (internal call)
 * 
 * @param m the multiplier
 * @param n the power
 * @return uint32_t the nth power of m
 */
uint32_t mypow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
        result *= m;
    return result;
}

/**
 * @brief Draw a number
 * 
 * @param x the beginning x coordinate of the number
 * @param y the beginning y coordinate of the number
 * @param num the number(0~4294967295)
 * @param len the length of the display number
 * @param size the size of display number
 * @param front_color the color value of display number
 * @param back_color the background color of display number
 */
void GUI_DrawNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t front_color, uint16_t back_color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                GUI_DrawChar(x + (size / 2) * t, y, front_color, back_color, ' ', size, 0);
                continue;
            }
            else
                enshow = 1;
        }
        GUI_DrawChar(x + (size / 2) * t, y, front_color, back_color, temp + '0', size, 0);
    }
}

/**
 * @brief Display a single Chinese character
 *
 * @param x the beginning x coordinate of the Chinese character
 * @param y the beginning y coordinate of the Chinese character
 * @param front_color the color value of Chinese character
 * @param back_color the background color of Chinese character
 * @param pchar the start address of the Chinese character
 * @param mode the display mode of Chinese character
 * @param size only 16, 24, 32 are valid
 */
void GUI_DrawFont(uint16_t x, uint16_t y, uint16_t front_color, uint16_t back_color, uint8_t *pchar, uint8_t mode, uint8_t size)
{
    if (size == 16)
    {
        GUI_DrawFont16(x, y, front_color, back_color, pchar, mode);
    }
    else if (size == 24)
    {
        GUI_DrawFont24(x, y, front_color, back_color, pchar, mode);
    }
    else if (size == 32)
    {
        GUI_DrawFont32(x, y, front_color, back_color, pchar, mode);
    }
    else
    {
        return;
    }
    LCD_SetWindows(0, 0, hlcd.width - 1, hlcd.height - 1);
}

/**
 * @brief Display a single 16x16 Chinese character
 *
 * @param x the beginning x coordinate of the Chinese character
 * @param y the beginning y coordinate of the Chinese character
 * @param front_color the color value of Chinese character
 * @param back_color the background color of Chinese character
 * @param pchar the start address of the Chinese character
 * @param mode the display mode of Chinese character
 */
void GUI_DrawFont16(uint16_t x, uint16_t y, uint16_t front_color, uint16_t back_color, uint8_t *pchar, uint8_t mode)
{
    uint8_t i, j;
    uint8_t k;
    uint8_t set_size;
    uint16_t x0 = x;
    set_size = sizeof(myFonts16) / sizeof(FontTypeDef16);

    for (k = 0; k < set_size; k++)
    {
        if ((myFonts16[k].UTF8Code[0] == *(pchar)) && (myFonts16[k].UTF8Code[1] == *(pchar + 1)) && (myFonts16[k].UTF8Code[2] == *(pchar + 2)))
        {
            LCD_SetWindows(x, y, x + 16 - 1, y + 16 - 1);
            for (i = 0; i < 16 * 2; i++)
            {
                for (j = 0; j < 8; j++)
                {
                    if (!mode)  // Non-overlying mode
                    {
                        if (myFonts16[k].Data[i] & (0x80 >> j))
                            LCD_WriteData16(front_color);
                        else
                            LCD_WriteData16(back_color);
                    }
                    else
                    {
                        if (myFonts16[k].Data[i] & (0x80 >> j))
                            GUI_DrawPoint(x, y, front_color);
                        x++;
                        if ((x - x0) == 16)
                        {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue; // Exit immediately after finding the corresponding dot matrix library to prevent multiple Chinese characters from being affected by repeated modeling
    }
}

void GUI_DrawFont24(uint16_t x, uint16_t y, uint16_t front_color, uint16_t back_color, uint8_t *pchar, uint8_t mode)
{
    uint8_t i, j;
    uint16_t k;
    uint16_t set_size;
    uint16_t x0 = x;
    set_size = sizeof(myFonts24) / sizeof(FontTypeDef24);

    for (k = 0; k < set_size; k++)
    {
        if ((myFonts24[k].UTF8Code[0] == *(pchar)) && (myFonts24[k].UTF8Code[1] == *(pchar + 1)) && (myFonts24[k].UTF8Code[2] == *(pchar + 2)))
        {
            LCD_SetWindows(x, y, x + 24 - 1, y + 24 - 1);
            for (i = 0; i < 24 * 3; i++)
            {
                for (j = 0; j < 8; j++)
                {
                    if (!mode)
                    {
                        if (myFonts24[k].Data[i] & (0x80 >> j))
                            LCD_WriteData16(front_color);
                        else
                            LCD_WriteData16(back_color);
                    }
                    else
                    {
                        if (myFonts24[k].Data[i] & (0x80 >> j))
                            GUI_DrawPoint(x, y, front_color);
                        x++;
                        if ((x - x0) == 24)
                        {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;
    }
}

void GUI_DrawFont32(uint16_t x, uint16_t y, uint16_t front_color, uint16_t back_color, uint8_t *pchar, uint8_t mode)
{
    uint8_t i, j;
    uint16_t k;
    uint16_t set_size;
    uint16_t x0 = x;
    set_size = sizeof(myFonts32) / sizeof(FontTypeDef32);
    for (k = 0; k < set_size; k++)
    {
        if ((myFonts32[k].UTF8Code[0] == *(pchar)) && (myFonts32[k].UTF8Code[1] == *(pchar + 1)) && (myFonts32[k].UTF8Code[2] == *(pchar + 2)))
        {
            LCD_SetWindows(x, y, x + 32 - 1, y + 32 - 1);
            for (i = 0; i < 32 * 4; i++)
            {
                for (j = 0; j < 8; j++)
                {
                    if (!mode)
                    {
                        if (myFonts32[k].Data[i] & (0x80 >> j))
                            LCD_WriteData16(front_color);
                        else
                            LCD_WriteData16(back_color);
                    }
                    else
                    {
                        if (myFonts32[k].Data[i] & (0x80 >> j))
                            GUI_DrawPoint(x, y, front_color);
                        x++;
                        if ((x - x0) == 32)
                        {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;
    }
}

/**
 * @brief Draw a string
 *
 * @param x the beginning x coordinate of the string
 * @param y the beginning y coordinate of the string
 * @param front_color the color value of the string
 * @param back_color the background color of the string
 * @param str the start address of the string
 * @param size the size of the string
 * @param mode the display mode of the string
 */
void GUI_DrawStr(uint16_t x, uint16_t y, uint16_t front_color, uint16_t back_color, uint8_t *str, uint8_t size, uint8_t mode)
{
    uint16_t x0 = x;
    uint8_t ifChn = 0; // Whether there is Chinese
    while (*str != 0)  // Not the end of the string
    {
        if (!ifChn)
        {
            if (x > (hlcd.width - size / 2) || y > (hlcd.height - size))
                return;
            if (*str > 0x80)
                ifChn = 1; // Chinese
            else           // ASCII
            {
                if (*str == 0x0D) // Tab
                {
                    y += size;
                    x = x0;
                    str++;
                }
                else
                {
                    if (size > 16) // Replace the size with 16 if the size is greater than 16
                    {
                        GUI_DrawChar(x, y, front_color, back_color, *str, 16, mode);
                        x += 8; // Half of the character
                    }
                    else
                    {
                        GUI_DrawChar(x, y, front_color, back_color, *str, size, mode);
                        x += size / 2; // Half of the character
                    }
                }
                str++;
            }
        }
        else // Chinese
        {
            if (x > (hlcd.width - size) || y > (hlcd.height - size))
                return;
            ifChn = 0; // Clear Chinese flag
            GUI_DrawFont(x, y, front_color, back_color, str, mode, size);
            str += 3;
            x += size; // The Chinese character size
        }
    }
}

/**
 * @brief Draw an image
 *
 * @param x the beginning x coordinate of the image
 * @param y the beginning y coordinate of the image
 * @param p the start address of image array
 * @param width the width of the image
 * @param height the height of the image
 */
void GUI_DrawPic(uint16_t x, uint16_t y, const unsigned char *p, uint16_t width, uint16_t height)
{
    int i;
    unsigned char picH, picL;
    LCD_SetWindows(x, y, x + width - 1, y + height - 1);
    for (i = 0; i < width * height; i++)
    {
        picL = *(p + i * 2); // low byte first
        picH = *(p + i * 2 + 1);
        LCD_WriteData16(picH << 8 | picL);
    }
}