#include <msp430fr2433.h>
#include "note_map.h"
#include "lcd.h"

/**
 * main.c
 */

#define TROUGH_THRESHOLD 50
#define CLOCKS_FOR_SECOND 0x67d0
#define MIN_FREQUENCY 33

enum State{
    START,
    ADC_START,
    ADC_RESULT,
    ADC_WAITING,
    TIMER_CHECK,
    GET_NOTE,
    LCD_INIT,
    LCD_WRITE
};

volatile enum State state;
volatile unsigned short adc_cur;

void init(){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    //Clock initialization (MCLK = 4 MHz, SMCLK = 1 MHz)
    CSCTL1 |= DCORSEL1;
    CSCTL1 &= ~DCORSEL0;
    CSCTL5 |= DIVS1;

    //ADC initialization, uses MODCLK
    //SYSCFG2 |= (ADCPCTL2 | ADCPCTL1 | ADCPCTL0); //enable ADC1, VEREF- and VEREF+
    SYSCFG2 |= ADCPCTL1;
    //ADCMCTL0 |= (ADCSREF2 | ADCSREF1 | ADCSREF0);
    ADCMCTL0 |= ADCINCH0;
    ADCCTL0 |= ADCON;
    ADCCTL1 &= ~(ADCSHS1 | ADCSHS0); //set ADCSC as trigger
    ADCCTL1 |= ADCSHP; //pulse mode
    ADCIE |= ADCIE0; //enable interrupts

    TA0CCR0 = CLOCKS_FOR_SECOND; //sample audio for one second (value found through trial and error)

    UCB0CTLW0 |= UCSWRST; // set sw rst
    UCB0CTLW0 |= UCMST; // set master
    UCB0CTLW0 |= (UCMODE1 | UCMODE0); // i2c mode
    UCB0CTLW1 |= UCASTP1; // automatic stop condition
    UCB0BRW = 0x0040; // SMCLK / 64
    UCB0I2CSA = 0x3c; // set slave address
    P1SEL0 |= (BIT3 | BIT2); // configure pins for i2c
    P1SEL1 &= ~(BIT3 | BIT2);

    //P1DIR |= BIT7; //debug

    state = START;

    __enable_interrupt();
}

void update_period_cnt(unsigned short *periods_ptr, unsigned char *periods_flags_ptr, unsigned short *adc_prev_ptr, unsigned short *trough_ptr){

    if(adc_cur > *adc_prev_ptr){
        //if down and it becomes up, record trough
        if(~*periods_flags_ptr & BIT0){
            *trough_ptr = *adc_prev_ptr;
        }
        *periods_flags_ptr |= BIT0; //set up bit
    }
    else if(adc_cur < *adc_prev_ptr){
        *periods_flags_ptr &= ~(BIT1 | BIT0); //clear both up bit and processed bit
    }

    //if up and not processed and we're higher than the threshold, increment periods and set processed bit
    if((*periods_flags_ptr & BIT0) && (~*periods_flags_ptr & BIT1) && (adc_cur - *trough_ptr > TROUGH_THRESHOLD)){
        ++*periods_ptr;
        *periods_flags_ptr |= BIT1;
    }

    *adc_prev_ptr = adc_cur;
}

inline unsigned short frequency(unsigned short periods){
    return periods;
}

inline unsigned char musical_note(unsigned short frequency){
    unsigned char l = 0, r = 95, m;
    while(l <= r && r != 255 && l != 96){
        m = (l + r) >> 1;
        if(frequency > NOTE_MAPS[m].frequency){
            l = m + 1;
        }
        else if(frequency < NOTE_MAPS[m].frequency){
            r = m - 1;
        }
        else
            break;
    }
    return m;
}

void to_string(unsigned short val, char* buf){
    unsigned char cnt = 0;
    while(val >= 1000){
        val -= 1000;
        ++cnt;
    }
    buf[0] = '0' + cnt;

    cnt = 0;
    while(val >= 100){
        val -= 100;
        ++cnt;
    }
    buf[1] = '0' + cnt;

    cnt = 0;
    while(val >= 10){
        val -= 10;
        ++cnt;
    }
    buf[2] = '0' + cnt;

    buf[3] = '0' + val;
}

//volatile unsigned short voltages[1000]; //debug
//volatile unsigned short i = 0; //debug

int main(void){
    init();

    unsigned short freq = 0;
    unsigned short adc_prev;
    unsigned short periods;
    unsigned char periods_flags; //BIT1 = processed period for current up cycle, BIT0 = up or down (1 for up)
    unsigned short trough;

    unsigned char freq_index;
    note_map note0, note1;
    char lcd_buf[16];

	while(1){
	    switch(state){
	    case START:
	        adc_prev = 1023;
	        periods = 0;
	        periods_flags = 0;
	        trough = 520;

	        //Timer initialization, uses SMCLK, ultimately at ~62.5 kHz after divider settings
	        TA0CTL |= TACLR;

	        TA0CTL |= TASSEL1;
	        TA0CTL |= (ID1 | ID0);
	        TA0EX0 |= TAIDEX0;
	        TA0CTL |= MC0; //up mode

	        ADCCTL0 |= ADCENC;

	        state = ADC_START;
	        break;
	    case ADC_START:
	        ADCCTL0 |= ADCSC;
	        state = ADC_WAITING;
	        break;
	    case ADC_WAITING:
	        break;
	    case ADC_RESULT:
	        update_period_cnt(&periods, &periods_flags, &adc_prev, &trough);

	        //debug
	        //voltages[i++] = adc_cur;
	        //if(i == 1000) i = 0;

	        state = TIMER_CHECK;
	        break;
	    case TIMER_CHECK:
	        if(TA0CTL & TAIFG){
	            //P1OUT ^= BIT7; //debug
	            TA0CTL &= ~MC0;
	            TA0CTL &= ~TAIFG;
	            state = GET_NOTE;
	        }
	        else{
	            state = ADC_START;
	        }
	        break;
	    case GET_NOTE:
	        ADCCTL0 &= ~ADCENC;
	        freq = frequency(periods);
	        if(freq < MIN_FREQUENCY){
	            state = START;
	            break;
	        }
	        freq_index = musical_note(freq);
	        if(NOTE_MAPS[freq_index].frequency > freq)
	            --freq_index;
	        note0 = NOTE_MAPS[freq_index];
	        note1 = NOTE_MAPS[freq_index + 1];
	        state = LCD_INIT;
	        break;
	    case LCD_INIT:
	        lcd_init();
	        state = LCD_WRITE;
	        break;
	    case LCD_WRITE:
	        lcd_buf[0] = note0.lcd_rep[0];
	        lcd_buf[1] = note0.lcd_rep[1];
	        lcd_buf[2] = ':';
	        to_string(note0.frequency, lcd_buf + 3);
	        lcd_buf[7] = ',';
	        lcd_buf[8] = ' ';

            lcd_buf[9] = note1.lcd_rep[0];
            lcd_buf[10] = note1.lcd_rep[1];
            lcd_buf[11] = ':';
            to_string(note1.frequency, lcd_buf + 12);

	        lcd_write(lcd_buf, 16);

	        lcd_line2();

	        lcd_buf[0] = 'Y';
	        lcd_buf[1] = 'o';
	        lcd_buf[2] = 'u';
	        lcd_buf[3] = 'r';
	        lcd_buf[4] = 's';
	        lcd_buf[5] = ':';
	        lcd_buf[6] = ' ';
	        to_string(freq, lcd_buf + 7);

	        lcd_write(lcd_buf, 11);

	        state = START;
	        break;
	    }
	}
}

//unused
/*
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A_ISR(){
    //P1OUT ^= BIT7; //debug

    if(state == ADC_START || state == ADC_RESULT || state == ADC_WAITING)
        state = GET_NOTE;

    TA0CTL &= ~MC0;
    TA0CTL &= ~TAIFG;
}
*/

#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(){
    adc_cur = ADCMEM0;
    state = ADC_RESULT;
}

#pragma vector = USCI_B0_VECTOR
__interrupt void I2C_ISR(){
    switch(UCB0IV){
    case USCI_I2C_UCTXIFG0:
        UCB0TXBUF = i2c_data[i2c_i++];
        break;
    case USCI_I2C_UCNACKIFG:
        UCB0CTLW0 |= (UCTXSTT | UCTR);
        break;
    }
}
