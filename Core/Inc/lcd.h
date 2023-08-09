//OLD VERSION
/*
#pragma once

#include <stdint.h>
#define BLACK     0x0000
#define RED       0xf800
#define GREEN     0x07e0
#define BLUE      0x001f
#define YELLOW    0xffe0
#define MAGENTA   0xf81f
#define CYAN      0x07ff
#define WHITE     0xffff

void lcd_fill_box(int x, int y, int width, int height, uint16_t color);

void lcd_init(void);

void lcd_put_pixel(int x, int y, uint16_t color);
*/

#pragma once

#include <stdint.h>

#define LCD_WIDTH	160
#define LCD_HEIGHT	128

//Definicje kolorów z odwróconą kolejnością bajtów
#define BLACK			0x0000
#define RED			0x00f8
#define GREEN			0xe007
#define BLUE			0x1f00
#define YELLOW			0xe0ff
#define MAGENTA			0x1ff8
#define CYAN			0xff07
#define WHITE			0xffff

void lcd_init(void); // Inicjalizacja wyświetlacza
void lcd_put_pixel(int x, int y, uint16_t color);
void lcd_copy(void); // Przesłanie zawartości bufora
void lcd_fillBox(int x, int y, int width, int height, uint16_t color);
