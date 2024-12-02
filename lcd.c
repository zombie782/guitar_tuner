/*
 * lcd.c
 *
 *  Created on: Nov 29, 2024
 *      Author: milez
 */

#include <msp430fr2433.h>
#include "lcd.h"

inline void lcd_ready(){
    while(!(UCB0IFG | UCSTPIFG)){}
}

inline void send(unsigned char byte_cnt){
    UCB0CTLW0 |= UCSWRST; // set sw rst
    UCB0TBCNT = byte_cnt;
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE |= (UCTXIE0 | UCNACKIE); // enable interrupts
    UCB0CTLW0 |= (UCTXSTT | UCTR);
}

void lcd_init(){

    __delay_cycles(40000u * US);
    i2c_i = 0;
    i2c_data[0] = 0x00;
    i2c_data[1] = 0x38;
    send(2);
    __delay_cycles(2000u * US);

    lcd_ready();
    i2c_i = 0;
    i2c_data[1] = 0x0c;
    send(2);
    __delay_cycles(2000u * US);

    lcd_ready();
    i2c_i = 0;
    i2c_data[1] = 0x01;
    send(2);
    __delay_cycles(10000u * US);

    lcd_ready();
    i2c_i = 0;
    i2c_data[1] = 0x06;
    send(2);
    __delay_cycles(2000u * US);
}

void lcd_write_char(char c){
    lcd_ready();

    i2c_i = 0;
    i2c_data[0] = 0x40;
    i2c_data[1] = c;

    send(2);
    __delay_cycles(2000u * US);
}


void lcd_write(char str[], unsigned char size){

    unsigned char i;
    for(i = 0; i != size; ++i){
        lcd_write_char(str[i]);
    }

}

void lcd_line2(){
    lcd_ready();

    i2c_i = 0;
    i2c_data[0] = 0x00;
    i2c_data[1] = 0xc0;

    send(2);
    __delay_cycles(2000u * US);
}
