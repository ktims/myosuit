#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cie1931.h"

#define F_CPU 8000000

FUSES = {
	.low			= (FUSE_SUT_CKSEL4 & FUSE_SUT_CKSEL3 & FUSE_SUT_CKSEL2 & FUSE_SUT_CKSEL0),
	.high			= (FUSE_SPIEN & FUSE_EESAVE & FUSE_BODLEVEL1 & FUSE_BODLEVEL0),
	.extended	= (FUSE_SELFPRGEN),
};

#define RS485_BAUD 38400
#define RX_BUFSIZ 8
#define TX_BUFSIZ 8

// milliseconds - round down to nearest 128uS
#define CMD_TIMEOUT 10

#define PWM_BITS 16

#define LED0_REG	OCR2B
#define LED1_REG	OCR1A
#define LED2_REG	OCR1B
#define LED3_REG	OCR2A

#define EEPROM_OFFSET 	((const void *)0x00)
#define EEPROM_OSCCAL   ((const void *)0x3f)

// Average N samples for each ADC reading. ADC is 10-bit, accumulator is 16-bit
// ~ this can bet set up to 2**6 (64)
// Should be a power of 2 for ISR efficiency
#define ADC_AVG 	16

#define CIE_VAL(x) (pgm_read_word_near(&(cie[x])))

volatile char rx_buf[RX_BUFSIZ];
volatile uint8_t rx_siz = 0;
volatile uint8_t rx_pos = 0;

uint16_t adc_vals[4];

uint8_t myaddr;

enum { CMDWAIT, DATA, IGNORE } state;

union {
	struct {
		char cmd;
		char data[4];
	} s;
	char b[5];
} cmdbuf;

// ADMUX values for each ADC channel we use
const uint8_t adc_channels[] = {
	0x01,
	0x02,
	0x03,
	0x04
};

// Current ADC input
uint8_t cur_adc;
// ISR will set this to current ADC value after averaging
uint16_t adc_return;
// ISR can flag main when data is available
uint8_t adc_flag;

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
	
	// Set up timeout compare match
	OCR0A = (CMD_TIMEOUT / 0.128);
	TIMSK0 = _BV(OCIE0A);

	// Map PWM outputs to correct pins
	TOCPMSA0 = _BV(TOCC3S0);
	TOCPMSA1 = _BV(TOCC4S0) | _BV(TOCC5S1);

	// Set 16-bit PWM top
	ICR1 = (1<<PWM_BITS) - 1;
	ICR2 = (1<<PWM_BITS) - 1;

	// Run in non-inverting 16-bit fast PWM mode
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
	TCCR2A = _BV(COM2A1) | _BV(WGM21);

	// Use the main system clock for PWM
	TCCR1B = _BV(CS10) | _BV(WGM12) | _BV(WGM13);
	TCCR2B = _BV(CS20) | _BV(WGM22) | _BV(WGM23);

	// Enable the muxed outputs
	TOCPMCOE = _BV(TOCC3OE) | _BV(TOCC4OE) | _BV(TOCC5OE);
}

void init_osc(void) {
	uint8_t cal = eeprom_read_byte(EEPROM_OSCCAL);
	if (cal != 0xff)
		OSCCAL0 = cal;
}

void init_usart(void) {
	REMAP = _BV(0); // NB: U0MAP symbol in this version of avr-libc is incorrect defined
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0); // | _BV(UCSZ02) - enable 9-bit serial
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

void begin_timeout(void) {
	// Run at Clkio / 1024 = 7.8125khz
	// match range 128us - 32.64ms
	TCCR0B = _BV(CS02) | _BV(CS00);
}
void cancel_timeout(void) {
	TCCR0B = 0;
	TCNT0 = 0;
}

inline void usart_tx_enable(void) {
	PORTB |= _BV(1);
}
inline void usart_tx_disable(void) {
	PORTB &= _BV(1);
}

void usart_putc(char c) {
	UDR0 = c;
	loop_until_bit_is_set(UCSR0A, TXC0);
	UCSR0A |= _BV(TXC0);
//	_delay_ms(1);
}

void usart_putc_auto(char c) {
	usart_tx_enable();
	usart_putc(c);
	usart_tx_disable();
}

void usart_puts(char *s) {
	usart_tx_enable();
	while (*s) {
		loop_until_bit_is_set(UCSR0A, UDRE0);
		UDR0 = *s++;
		UCSR0A |= _BV(TXC0);
	}
	loop_until_bit_is_set(UCSR0A, TXC0);
	usart_tx_disable();
}

void load_my_addr(void) {
	uint8_t a = eeprom_read_byte(EEPROM_OFFSET);
	if (a == 0xff)
		myaddr = 0;
	else
		myaddr = a & 0xf0;
}

void set_my_addr(uint8_t a) {
	myaddr = a;
	eeprom_update_byte(EEPROM_OFFSET, a);
}

void set_led(uint8_t pos, uint8_t val) {
	switch (pos) {
		case 0:
			LED0_REG = CIE_VAL(val);
			return;
		case 1:
			LED1_REG = CIE_VAL(val);
			return;
		case 2:
			LED2_REG = CIE_VAL(val);
			return;
		case 3:
			LED3_REG = CIE_VAL(val);
			return;
	}
}

void set_all_leds(uint8_t val) {
	LED0_REG = CIE_VAL(val);
	LED1_REG = LED0_REG;
	LED2_REG = LED0_REG;
	LED3_REG = LED0_REG;
}

void send_adc_vals(void) {
			uint16_t cur[4];
			char ret[5];
			ATOMIC_BLOCK(ATOMIC_FORCEON) {
				memcpy(cur, adc_vals, sizeof(cur));
			}
			// Pack for the trip
			//   a9 a8 a7 a6  a5 a4 a3 a2
			//   a1 a0 b9 b8  b7 b6 b5 b4
			//   b3 b2 b1 b0  c9 c8 c7 c6
			//   c5 c4 c3 c2  c1 c0 d9 d8
			//   d7 d6 d5 d4  d3 d2 d1 d0
			ret[0] = cur[0] >> 2;
			ret[1] = (cur[0] << 6) | (cur[1] >> 4);
			ret[2] = (cur[1] << 4) | (cur[2] >> 6);
			ret[3] = (cur[2] << 2) | (cur[3] >> 8);
			ret[4] = cur[3];

			usart_tx_enable();
			for (uint8_t i = 0; i < sizeof(ret); i++)
				usart_putc(ret[i]);
			usart_tx_disable();
}

void set_osccal(uint8_t value, uint8_t save) {
	ICR1 = 79;
	ICR2 = 79;
	OCR1A = 39;
	OCR1B = 39;
	OCR2B = 39;

	OSCCAL0 = value;
	if (save)
		eeprom_write_byte(EEPROM_OSCCAL, value);
}

void set_flags(uint8_t data[]) {
	//TODO: implement this
}

/* cmds: (XX = don't care)
 *  0x01 - set all LEDs. data 4 bytes [LED0, LED1, LED2, LED3] PWM values
 *  0x02 - set my address. data 4 bytes [ADDR, XX, XX, XX]. top 4 bits from ADDR written to my
 *  		  address
 *  0x03 - return current ADC values
 *  		RETURNS: 4 10-bit values packed in 5 bytes [IN0,IN1,IN2,IN3]
 *  0x04 - set OSCCAL value [VAL, SAVE, XX, XX] and put us into OSCCAL mode
 *  				- Sets PWM to output 50% duty cycle at exactly 100KHz
 *  		- if SAVE is 0 only set register, else save to EEPROM
 *  0x05 - set config flags [FLAGS0, FLAGS1, FLAGS2, SAVE]
 *  		FLAGS0:
 *  			MSB 7 - gamma correction enabled
 */
void process_cmd(void) {
	switch(cmdbuf.s.cmd) {
		case 0x01:
			set_led(0, cmdbuf.s.data[0]);
			set_led(1, cmdbuf.s.data[1]);
			set_led(2, cmdbuf.s.data[2]);
			set_led(3, cmdbuf.s.data[3]);
			break;
		case 0x02:
			set_my_addr(cmdbuf.s.data[0] & 0xf0);
			break;
		case 0x03:
			send_adc_vals();
			break;
		case 0x04:
			set_osccal(cmdbuf.s.data[0], cmdbuf.s.data[1]);
			break;
		case 0x05:
			set_flags(cmdbuf.s.data);
			break;
	}
}

void process_usart(void) {
	static uint8_t bytesleft;
	while (rx_siz > rx_pos) {
		char c = rx_buf[rx_pos++];
		char addr;

		switch(state) {
			case IGNORE:
				if (--bytesleft == 0)
					state = CMDWAIT;
				break;

			case CMDWAIT:
				addr = 0xf0 & c;
				cmdbuf.b[0] = 0x0f & c;
				bytesleft = 4;
				if (addr == myaddr || addr == 0xf0)
					state = DATA;
				else 
					state = IGNORE;

				begin_timeout();
				break;

			case DATA:
				cmdbuf.b[5-bytesleft] = c;
				if (--bytesleft == 0) {
					cancel_timeout();
					state = CMDWAIT;
					process_cmd();
				}
				break;

		}
	}
}

void init_adc(void) {
	cur_adc = 0;
	ADMUXB = _BV(REFS1) | _BV(REFS0);
	ADCSRA = _BV(ADEN) | _BV(ADIE);
	//DIDR0 = _BV(ADC1D) | _BV(ADC2D) | _BV(ADC3D) | _BV(ADC4D);
}

void run_adc(uint8_t channel) {
	if (channel > 3)
		return;
	cur_adc = channel;
	adc_flag = 0;
	ADMUXA = adc_channels[cur_adc];
	ADCSRA |= _BV(ADSC) | _BV(ADATE);
}

ISR(ADC_vect) {
	static uint8_t count = 0;
	static uint16_t accum = 0;

	if (count == 0)
		accum = 0;

	accum += ADC;

	if (++count == ADC_AVG) {
		adc_vals[cur_adc] = accum / ADC_AVG;
		adc_flag = 1;
		count = 0;
	} 
	// On the second-to-last sample, disable auto triggering so main can change cur_adc after the run
	// this can be rolled into the ISR if desired...
	else if (count == ADC_AVG-1) {
		ADCSRA &= ~_BV(ADATE);
	}
}

ISR(USART0_RX_vect) {
	char value;

	value = UDR0;

	if (rx_siz <= rx_pos) {
		rx_siz = 0;
		rx_pos = 0;
	}

	if (RX_BUFSIZ-1 != rx_siz)
		rx_buf[rx_siz++] = value;
}

// The host is taking its time with the command - give up
ISR(TIMER0_COMPA_vect) {
	state = CMDWAIT;
}


void dit(void) {
	set_all_leds(255);
	_delay_ms(60);
	set_all_leds(0);
	_delay_ms(60);
}

void dah(void) {
	set_all_leds(255);
	_delay_ms(180);
	set_all_leds(0);
	_delay_ms(60);
}

void wank(void) {
	uint8_t i;
	// pwm runs at 31.25KHz. do a startup show.
	for (i = 0; i < 255; i++) {
		set_led(0, i);
		_delay_ms(2);
	}
	for (i = 0; i < 255; i++) {
		set_led(1, i);
		_delay_ms(2);
	}
	for (i = 0; i < 255; i++) {
		set_led(2, i);
		_delay_ms(2);
	}
	for (i = 0; i < 255; i++) {
		set_led(3, i);
		_delay_ms(2);
	}

	usart_putc_auto('Y');
	dah(); dit(); dah(); dah();
	_delay_ms(180);

	usart_putc_auto('M');
	dah(); dah();
	_delay_ms(180);

	usart_putc_auto('C');
	dah(); dit(); dah(); dit();
	_delay_ms(180);

	usart_putc_auto('A');
	dit(); dah();
	_delay_ms(180);

}


int main(void) {
	load_my_addr();
	init_osc();
	init_io();
	init_usart();
	init_adc();

	wank();

	run_adc(0);

	// GO!
	sei();

	for (;;) {
		if (adc_flag) {
			run_adc((cur_adc+1) & 0x03);
		}
		process_usart();
	}


	return 0;
}
