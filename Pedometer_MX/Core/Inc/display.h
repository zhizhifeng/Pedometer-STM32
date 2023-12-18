#ifndef __DISPLAY_H
#define __DISPLAY_H
#include <stdint.h>

#define TITLE_HEIGHT  130
#define DATA_HEIGHT   180

void Display_Page_Initialize(void);
void Display_Step_Update(uint8_t step);
void Display_Temperature_Update(float temperature);
void Display_Humidity_Update(float humidity);
#endif