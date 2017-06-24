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

#define LED_CONFIG()		DDRF = 0xFF
#define LED_ON(n)			PORTF |= 1<<n
#define LED_OFF(n)			PORTF &= ~(1<<n)
#define CPU_PRESCALE(n)		(CLKPR = 0x80, CLKPR = (n))

#define KEY0_VAL		KEY_X
#define KEY1_VAL		KEY_Z
#define KEY2_VAL		KEY_SPACE

// port, port_prev, mask
#define KEY0_PARAM		b,b_prev,1<<4
#define KEY1_PARAM		d,d_prev,1<<3
#define KEY2_PARAM		b,b_prev,1<<5

// indirection
#define KEY_PRESS_(port,port_prev,mask)		((port & mask) == 0 && (port_prev & mask) != 0)
#define KEY_RELEASE_(port,port_prev,mask)	((port & mask) != 0 && (port_prev & mask) == 0)
#define KEY_PRESS(...)						KEY_PRESS_(__VA_ARGS__)
#define KEY_RELEASE(...)					KEY_RELEASE_(__VA_ARGS__)

uint16_t second_count = 0;	// 1 count = 1/61 second
uint16_t keypresses = 0;	// number of key presses

int main(void)
{
	uint8_t b, d;
	uint8_t b_prev = 0xFF, d_prev = 0xFF;

	// set for 16 MHz clock
	CPU_PRESCALE(0);

	// Configure all port B and port D pins as inputs with pullup resistors.
	// See the "Using I/O Pins" page for details.
	// http://www.pjrc.com/teensy/pins.html
	DDRD = 0x00;
	DDRB = 0x00;
	PORTB = 0xFF;
	PORTD = 0xFF;
	
	// Configure LED
	LED_CONFIG();
	PORTF = 0x00;

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured());

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	// Configure timer 0 to generate a timer overflow interrupt every
	// 256*1024 clock cycles, or approx 61 Hz when using 16 MHz clock
	TCCR0A = 0x00;
	TCCR0B = 0x05;
	TIMSK0 = (1<<TOIE0);

	while (1) {
		// read all port B and port D pins
		b = PINB;
		d = PIND;
		
		// check if key status changed (pressed/released)
		if (KEY_PRESS(KEY0_PARAM)) { //high to low (press)
			usb_keyboard_press(KEY0_VAL, 0);
			LED_ON(0);
			keypresses++;
		}
		if (KEY_RELEASE(KEY0_PARAM)) { //low to high (release)
			usb_keyboard_press(0, 0);
			LED_OFF(0);
		}
		if (KEY_PRESS(KEY1_PARAM)) {
			usb_keyboard_press(KEY1_VAL, 0);
			LED_ON(1);
			keypresses++;
		}
		if (KEY_RELEASE(KEY1_PARAM)) {
			usb_keyboard_press(0, 0);
			LED_OFF(1);
		}
		if (KEY_PRESS(KEY2_PARAM)) {
			usb_keyboard_press(KEY2_VAL, 0);
			LED_ON(5);
		}
		if (KEY_RELEASE(KEY2_PARAM)) {
			usb_keyboard_press(0, 0);
			LED_OFF(5);
		}
		// now the current pins will be the previous, and
		// wait a short delay so we're not highly sensitive
		// to mechanical "bounce".
		b_prev = b;
		d_prev = d;
		_delay_ms(2);
	}
}

// This interrupt routine is ran approx. 61 times per second.
ISR(TIMER0_OVF_vect){
	second_count++;
	if (second_count > 61){
		if (keypresses > 3){
			LED_ON(4);
		}
		else{
			LED_OFF(4);
		}
		second_count = 0;
		keypresses = 0;
	}
}
