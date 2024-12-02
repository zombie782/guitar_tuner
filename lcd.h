#ifndef LCD_H
#define LCD_H

#define US 4

volatile unsigned char i2c_data[255];
volatile unsigned i2c_i;

void lcd_init();

void lcd_write_char(char c);
void lcd_write(char str[], unsigned char size);

void lcd_line2();

#endif
