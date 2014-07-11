#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define F_CPU 8000000

FUSES = {
	.low			= (FUSE_SUT_CKSEL4 & FUSE_SUT_CKSEL3 & FUSE_SUT_CKSEL2 & FUSE_SUT_CKSEL0),
	.high			= (FUSE_SPIEN & FUSE_EESAVE),
	.extended	= 0xff,
};

#define RS485_BAUD 38400
#define RX_BUFSIZ 8
#define TX_BUFSIZ 8

#define LED0_REG	OCR2B
#define LED1_REG	OCR1A
#define LED2_REG	OCR1B
#define LED3_REG	OCR2A

#define EEPROM_OFFSET 	0x00
#define EEPROM_OSCCAL   0x3f

// Average N samples for each ADC reading. ADC is 10-bit, accumulator is 16-bit
// ~ this can bet set up to 2**6 (64)
// Should be a power of 2 for ISR efficiency
#define ADC_AVG 	16

volatile char rx_buf[RX_BUFSIZ];
volatile uint8_t rx_siz = 0;
volatile uint8_t rx_pos = 0;

uint8_t myaddr, bytesleft;

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

void init_osc(void) {
	uint8_t cal = eeprom_read_byte((const void *)EEPROM_OSCCAL);
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

void usart_putc(char c) {
	PORTB |= _BV(1);
	UDR0 = c;
	loop_until_bit_is_set(UCSR0A, TXC0);
	UCSR0A |= _BV(TXC0);
	_delay_ms(1);
	PORTB &= ~_BV(1);
}

void usart_puts(char *s) {
	while (*s)
		usart_putc(*s++);
}

void load_my_addr(void) {
	uint8_t a = eeprom_read_byte((const void *)(EEPROM_OFFSET));
	if (a == 0xff)
		myaddr = 0;
	else
		myaddr = a & 0xf0;
}

void set_my_addr(uint8_t a) {
	myaddr = a;
	eeprom_update_byte((uint8_t *)(EEPROM_OFFSET), a);
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

void set_all_leds(uint8_t val) {
	LED0_REG = val;
	LED1_REG = val;
	LED2_REG = val;
	LED3_REG = val;
}

/* cmds: (XX = don't care)
 *  0x01 - set all LEDs. data 4 bytes [LED0, LED1, LED2, LED3] PWM values
 *  		RETURNS: 0xff
 *  0x02 - set my address. data 4 bytes [ADDR, XX, XX, XX]. top 4 bits from ADDR written to my
 *  		  address
 *  		RETURNS: 0xff on success
 */
void process_cmd(void) {
	switch(cmdbuf.s.cmd) {
		case 0x01:
			set_led(0, cmdbuf.s.data[0]);
			set_led(1, cmdbuf.s.data[1]);
			set_led(2, cmdbuf.s.data[2]);
			set_led(3, cmdbuf.s.data[3]);
			usart_putc(0xff);
			break;
		case 0x02:
			set_my_addr(cmdbuf.s.data[0] & 0xf0);
			usart_putc(0xff);
			break;
	}
}

void process_usart(void) {
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
				cmdbuf.b[0] = 0x0f & c;
				bytesleft = 4;
				if (addr == myaddr || addr == 0xf0)
					state = DATA;
				else 
					state = IGNORE;
				break;

			case DATA:
				cmdbuf.b[5-bytesleft] = c;
				if (--bytesleft == 0) {
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
		adc_return = accum / ADC_AVG;
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


inline void dit(void) {
	set_all_leds(255);
	_delay_ms(60);
	set_all_leds(0);
	_delay_ms(60);
}

inline void dah(void) {
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

	usart_putc('Y');
	dah(); dit(); dah(); dah();
	_delay_ms(180);

	usart_putc('M');
	dah(); dah();
	_delay_ms(180);

	usart_putc('C');
	dah(); dit(); dah(); dit();
	_delay_ms(180);

	usart_putc('A');
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

	char serbuf[16];
	uint16_t adc_vals[4];
	
	run_adc(0);

	// GO!
	sei();

	for (;;) {
		if (adc_flag) {
			adc_vals[cur_adc] = adc_return;
			run_adc((cur_adc+1) & 0x03);
		}
		process_usart();
	}


	return 0;
}
