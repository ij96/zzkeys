/* Keyboard example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * Copyright (c) 2008 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_keyboard.h"

// clock prescaler
#define CPU_PRESCALE(n)		(CLKPR = 0x80, CLKPR = (n))

// LED
#define LED_ON(port,pin)	port |= 1<<pin
#define LED_OFF(port,pin)	port &= ~(1<<pin)

// key RGB LED

// BPM RGB LED
#define BPM_RGB_R		OCR1A
#define BPM_RGB_G		OCR1B
#define BPM_RGB_B		OCR1C

// key values
#define KEY0_VAL		KEY_Q
#define KEY1_VAL		KEY_W
#define KEY2_VAL		KEY_SPACE

// port, port_prev, pin
#define KEY0_PARAM		b,b_prev,PB4
#define KEY1_PARAM		d,d_prev,PD3
#define KEY2_PARAM		d,d_prev,PD7

// indirection
#define KEY_PRESS_(port,port_prev,pin)		((port & 1<<pin) == 0 && (port_prev & 1<<pin) != 0)
#define KEY_RELEASE_(port,port_prev,pin)	((port & 1<<pin) != 0 && (port_prev & 1<<pin) == 0)
#define KEY_PRESS(...)						KEY_PRESS_(__VA_ARGS__)
#define KEY_RELEASE(...)					KEY_RELEASE_(__VA_ARGS__)

uint16_t second_count = 0;	// 1 count = 1/61 second
uint8_t press_count = 0;	// number of key presses

void pwm_init(void){
	// initialise TCCR0
	TCCR1A |= (1<<WGM10);	// phase correct PWM
	TCCR1A |= (1<<COM1A1);	// clear on compare match A
	TCCR1A |= (1<<COM1B1);	// clear on compare match B
	TCCR1A |= (1<<COM1C1);	// clear on compare match C
	TCCR1B |= (1<<CS00);	// clock no prescaling
	// set OC1 pins as output pins
	DDRB |= (1<<PB5);		// OC1A
	DDRB |= (1<<PB6);		// OC1B
	DDRB |= (1<<PB7);		// OC1C
}

#define PRESS_COUNT_LIST_LEN	4

uint8_t press_count_list[PRESS_COUNT_LIST_LEN];
uint8_t press_count_list_pos = 0;
uint8_t press_count_per_sec = 0;

void bpm_counter_init(void){
	// TIMER0: overflow interrupt
	// Configure timer 0 to generate a timer overflow interrupt every
	// 256*1024 clock cycles, or approx 61 Hz when using 16 MHz clock
	TCCR0A = 0b00000000;
	TCCR0B = 0b00000101;
	TIMSK0 = (1<<TOIE0);
	
	uint8_t i = 0;
	for(i = 0; i<PRESS_COUNT_LIST_LEN; i++){
		press_count_list[i] = 0;
	}
}

int main(void){
	uint8_t b, d;
	uint8_t b_prev = 0xFF, d_prev = 0xFF;
	
	// set for 16 MHz clock
	CPU_PRESCALE(0);

	// initialise timer for BPM counter
	bpm_counter_init();
	
	// Configure all port B and port D pins as inputs with pullup resistors.
	// See the "Using I/O Pins" page for details.
	// http://www.pjrc.com/teensy/pins.html
	DDRB = 0b00000000;
	DDRD = 0b00000000;
	PORTB = 0b11111111;
	PORTD = 0b11111111;

	
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured());

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	// PWM setup
	pwm_init();
	OCR1B = 0;

	while (1) {
		// read all port B and port D pins
		b = PINB;
		d = PIND;
		
		// check if key status changed (pressed/released)
		// only KEY0 and KEY1 press will increase press_count
		if (KEY_PRESS(KEY0_PARAM)) {
			usb_keyboard_press(KEY0_VAL, 0, 0);
			//LED_ON(PORTD,6);
			press_count++;
		}
		if (KEY_RELEASE(KEY0_PARAM)) {
			usb_keyboard_press(0, 0, 0);
			//LED_OFF(PORTD,6);
		}
		if (KEY_PRESS(KEY1_PARAM)) {
			usb_keyboard_press(KEY1_VAL, 0, 1);
			//LED_ON(PORTD,7);
			press_count++;
		}
		if (KEY_RELEASE(KEY1_PARAM)) {
			usb_keyboard_press(0, 0, 1);
			//LED_OFF(PORTD,7);
		}
		if (KEY_PRESS(KEY2_PARAM)) {
			usb_keyboard_press(KEY2_VAL, 0, 2);
		}
		if (KEY_RELEASE(KEY2_PARAM)) {
			usb_keyboard_press(0, 0, 2);
		}
		// now the current pins will be the previous, and
		// wait a short delay so we're not highly sensitive
		// to mechanical "bounce".
		b_prev = b;
		d_prev = d;
		_delay_ms(2);
	}
}

void RGB_set_colour(uint8_t r, uint8_t g, uint8_t b){
	BPM_RGB_R = r;
	BPM_RGB_G = g;
	BPM_RGB_B = b;
}
/*
void RGB_set_bpm(uint8_t press_count){
	if(press_count<2){
		RGB_set_colour();
	}
}*/

// This interrupt routine is ran approx. 61 times per second.
ISR(TIMER0_OVF_vect){
	second_count++;
	if (second_count > 10){	// approx. every 1/3 second
		second_count = 0;
		press_count_list_pos = (press_count_list_pos + 1)%PRESS_COUNT_LIST_LEN;
		press_count_list[press_count_list_pos] = press_count;
		press_count = 0;
		
		press_count_per_sec = 0;
		uint8_t i = 0;
		for(i = 0;i<PRESS_COUNT_LIST_LEN;i++){
			if(i!=press_count_list_pos){
				press_count_per_sec += press_count_list[press_count_list_pos];
			}
		}
		
		RGB_set_colour(press_count_per_sec,press_count_list_pos<<2,255); 
	}
}