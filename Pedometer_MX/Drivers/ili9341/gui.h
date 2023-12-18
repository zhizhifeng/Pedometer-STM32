#ifndef __GUI_H__
#define __GUI_H__
#include <stdint.h>

void GUI_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
void GUI_DrawTriangle(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void GUI_DrawStr(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *str,uint8_t size,uint8_t mode);
void GUI_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void GUI_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
void GUI_DrawPic(uint16_t x, uint16_t y, const unsigned char *p, uint16_t width, uint16_t height);
void GUI_DrawNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t front_color, uint16_t back_color);
void GUI_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void GUI_DrawFont32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s,uint8_t mode);
void GUI_DrawFont24(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s,uint8_t mode);
void GUI_DrawFont16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s,uint8_t mode);
void GUI_DrawFilledTriangle(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2, uint16_t color);
void GUI_DrawFilledRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void GUI_DrawCircle(int xc, int yc,uint16_t c,int r, int fill);
void GUI_DrawChar(uint16_t x,uint16_t y,uint16_t fc, uint16_t bc, uint8_t num,uint8_t size,uint8_t mode);
#endif

