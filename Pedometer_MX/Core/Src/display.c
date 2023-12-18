#include "display.h"
#include "lcd.h"
#include "gui.h"
#include "pic.h"
#include <stdio.h>
#include <string.h>

extern _lcd_dev hlcd;

static void DrawPage(void);

void Display_Page_Initialize(void)
{
    LCD_Init();
    DrawPage();
}

void Display_Step_Update(uint8_t step)
{
    char str[10];
    sprintf(str, "%d", step);
    GUI_DrawStr(70 - 4 * strlen(str), DATA_HEIGHT, BLACK, WHITE, (uint8_t *)str, 32, 0);
}

void Display_Temperature_Update(float temperature)
{
    char str[10];
    sprintf(str, "%.2f C", temperature);
    GUI_DrawStr(160 - 4 * strlen(str), DATA_HEIGHT, BLACK, WHITE, (uint8_t *)str, 32, 0);
}

void Display_Humidity_Update(float humidity)
{
    char str[10];
    sprintf(str, "%.2f %%", humidity);
    GUI_DrawStr(250 - 4 * strlen(str), DATA_HEIGHT, BLACK, WHITE, (uint8_t *)str, 32, 0);
}

static void DrawPage(void)
{
    LCD_Clear(WHITE);
    LCD_direction(1);
    GUI_Fill(0, 0, hlcd.width, 20, BLUE);
    GUI_DrawStr(hlcd.width / 2 - 4 * 9, 2, WHITE, BLUE, (uint8_t *)"Pedometer", 16, 1);

    GUI_DrawPic(45, 60, (unsigned char *)gImage_step, 50, 50);
    GUI_DrawPic(135, 60, (unsigned char *)gImage_temperature, 50, 50);
    GUI_DrawPic(225, 60, (unsigned char *)gImage_humidity, 50, 50);

    GUI_DrawStr(70 - 24, TITLE_HEIGHT, BLACK, WHITE, (uint8_t *)"步数", 24, 1);
    GUI_DrawStr(160 - 24, TITLE_HEIGHT, BLACK, WHITE, (uint8_t *)"温度", 24, 1);
    GUI_DrawStr(250 - 24, TITLE_HEIGHT, BLACK, WHITE, (uint8_t *)"湿度", 24, 1);

    GUI_DrawStr(70 - 4 * 1, DATA_HEIGHT, BLACK, WHITE, (uint8_t *)"0", 32, 1);
    GUI_DrawStr(160 - 4 * 3, DATA_HEIGHT, BLACK, WHITE, (uint8_t *)"NaN", 32, 1);
    GUI_DrawStr(250 - 4 * 3, DATA_HEIGHT, BLACK, WHITE, (uint8_t *)"NaN", 32, 1);
}