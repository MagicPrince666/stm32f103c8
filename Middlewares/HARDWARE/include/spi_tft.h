#ifndef __SPI_TFT_H
#define __SPI_TFT_H	

#include "sys.h"

#define LCD_BL PCout(4)
#define LCD_RS PCout(5)
#define LCD_CS PAout(4)

void test_color(void);
void Lcd_Initialize(void);

#endif
