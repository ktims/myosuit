#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define F_CPU 8000000

FUSES = {
	.low			= (FUSE_SUT_CKSEL4 & FUSE_SUT_CKSEL3 & FUSE_SUT_CKSEL2 & FUSE_SUT_CKSEL0),
	.high			= (FUSE_SPIEN),
	.extended	= 0xff,
};

#define RS485_BAUD 38400 
#define RX_BUFSIZ 8
#define TX_BUFSIZ 8

#define LED0_REG	OCR2B
#define LED1_REG	OCR1A
#define LED2_REG	OCR1B
#define LED3_REG	OCR2A

volatile char rx_buf[RX_BUFSIZ];
volatile char rx_siz = 0;
volatile char rx_pos = 0;

char myaddr, bytesleft;

enum { CMDWAIT, DATA, IGNORE } state;

union {
	struct {
		char cmd;
		char data[4];
	} s;
	char b[5];
} cmdbuf;

/*ISR(USART0_RX_vect) {
	char value;

	value = UDR0;

	if (rx_siz <= rx_pos) {
		rx_siz = 0;
		rx_pos = 0;
	}

	if (RX_BUFSIZ -1 != rx_siz)
		rx_buf[rx_siz++] = value;
}*/

void init_io(void) {
	// PORTA:
	//  0 - ADC REF					input|nopu
	//  1 - ADC input 0 (ADC1)		input|nopu
	//  2 - ADC input 1 (ADC2)		input|nopu
	//  3 - ADC input 3 (ADC3)		input|nopu
	//  4 - PWM LED 1 (TOCC3)		output | OC1A
	//  5 - PWM LED 2 (TOCC4)		output | OC1B
	//  6 - PWM LED 3 (TOCC5)		output | OC2A
	//  7 - USART TX					output
	//
	// PORTB:
	//  0 - ADC input 2 (ADC11)	input|nopu
	//  1 - GPIO TX driver EN		output
	//  2 - USART RX					input|nopu
	//  3 - LED0 (soft pwm :()		output
	
	OCR1A = 0;
	OCR1B = 0;
	OCR2A = 0;

	DDRA = _BV(DDRA4) | _BV(DDRA5) | _BV(DDRA6) | _BV(DDRA7);
	DDRB = _BV(DDRB1) | _BV(DDRB3);

	// Map outputs to correct pins
	TOCPMSA0 = _BV(TOCC3S0);
	TOCPMSA1 = _BV(TOCC4S0) | _BV(TOCC5S1);

	// Run in non-inverting phase correct PWM mode
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
	TCCR2A = _BV(COM2A1) | _BV(WGM20);

	// Use the main system clock for PWM
	TCCR1B = _BV(CS10);
	TCCR2B = _BV(CS20);

	// Enable the muxed outputs
	TOCPMCOE = _BV(TOCC3OE) | _BV(TOCC4OE) | _BV(TOCC5OE);
}

void init_usart(void) {
	REMAP = _BV(0); // NB: U0MAP symbol in this version of avr-libc is incorrect defined
	UCSR0B = _BV(RXEN0) | _BV(TXEN0); // | _BV(UCSZ02) - enable 9-bit serial
	#define BAUD RS485_BAUD
	#include <util/setbaud.h>
	UBRR0H = UBRRH_VALUE;
   UBRR0L = UBRRL_VALUE;
   #if USE_2X
   UCSR0A |= (1 << U2X0);
   #else
   UCSR0A &= ~(1 << U2X0);
   #endif
}

void usart_putc(char c) {
//	PORTB |= _BV(1);
	UDR0 = c;
	loop_until_bit_is_set(UCSR0A, TXC0);
	UCSR0A |= _BV(TXC0);
//	PORTB &= ~_BV(1);
}

void usart_puts(char *s) {
	while (*s)
		usart_putc(*s++);
}

void process_cmd() {
	switch(cmdbuf.s.cmd) {
		case 0x01:
			break;
	}
}

void process_uart() {
	while (rx_siz > rx_pos) {
		char c = rx_buf[rx_pos++];
		char addr;
		char cmd;

		switch(state) {
			case IGNORE:
				if (--bytesleft == 0)
					state = CMDWAIT;
				break;

			case CMDWAIT:
				addr = 0xf0 & c;
				cmd = 0x0f & c;
				bytesleft = 5;
				if (addr == myaddr || addr == 0xff)
					state = DATA;
				else 
					state = IGNORE;
				break;
			case DATA:
				cmdbuf.b[bytesleft-5] = c;
				if (--bytesleft == 0) {
					state = CMDWAIT;
					process_cmd();
				}
				break;

		}
	}
}

void set_led(uint8_t pos, uint8_t val) {
	switch (pos) {
		case 0:
			LED0_REG = val;
			return;
		case 1:
			LED1_REG = val;
			return;
		case 2:
			LED2_REG = val;
			return;
		case 3:
			LED3_REG = val;
			return;
	}
}

void init_adc(void) {

}

int main(void) {
	init_io();
	init_usart();

	uint8_t i = 0;

	// pwm runs at 31.25KHz. do a startup show.
	for (i = 0; i < 255; i++) {
		set_led(1, i);
		_delay_ms(10);
	}
	for (i = 0; i < 255; i++) {
		set_led(2, i);
		_delay_ms(10);
	}
	for (i = 0; i < 255; i++) {
		set_led(3, i);
		_delay_ms(10);
	}

	return 0;
}
