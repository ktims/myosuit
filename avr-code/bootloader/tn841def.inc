#ifndef _TN841DEF_INC_
#define _TN841DEF_INC_

#pragma partinc 0

.device ATtiny841
#pragma AVRPART ADMIN PART_NAME ATtiny841
.equ SIGNATURE_000 = 0x1e
.equ SIGNATURE_001 = 0x93
.equ SIGNATURE_002 = 0x15

#pragma AVRPART CORE CORE_VERSION V2
#pragma AVRPART CORE NEW_INSTRUCTIONS lpm rd,z+

.equ SREG 	= 0x3f
.equ SPH 	= 0x3e
.equ SPL 	= 0x3d

.equ ADCSRB  = 0x04
.equ ADTS0   = 0
.equ ADTS1   = 1
.equ ADTS2   = 2
.equ ADLAR   = 3

.equ ADCSRA  = 0x05
.equ ADPS0   = 0
.equ ADPS1   = 1
.equ ADPS2   = 2
.equ ADIE    = 3
.equ ADIF    = 4
.equ ADATE   = 5
.equ ADSC    = 6
.equ ADEN    = 7

.equ ADCL    = 0x06
.equ ADCH    = 0x07

.equ ADMUXB  = 0x08
.equ GSEL0   = 0
.equ GSEL1   = 1
.equ REFS0   = 5
.equ REFS1   = 6
.equ REFS2   = 7

.equ ADMUXA  = 0x09
.equ MUX0    = 0
.equ MUX1    = 1
.equ MUX2    = 2
.equ MUX3    = 3
.equ MUX4    = 4
.equ MUX5    = 5

.equ ACSR0A  = 0x0A
.equ ACIS0   = 0
.equ ACIS1   = 1
.equ ACIC0   = 2
.equ ACIE0   = 3
.equ ACI0    = 4
.equ ACO0    = 5
.equ ACPMUX2 = 6
.equ ACD0    = 7

.equ ACSR0B  = 0x0B
.equ ACPMUX0 = 0
.equ ACPMUX1 = 1
.equ ACNMUX0 = 2
.equ ACNMUX1 = 3
.equ ACOE0   = 4
.equ HLEV0   = 6
.equ HSEL0   = 7

.equ ACSR1A  = 0x0C
.equ ACIC1   = 2
.equ ACIE1   = 3
.equ ACI1    = 4
.equ ACO1    = 5
.equ ACBG1   = 6
.equ ACD1    = 7

.equ ACSR1B  = 0x0D
.equ ACME1   = 2
.equ ACOE1   = 4
.equ HLEV1   = 6
.equ HSEL1   = 7

.equ TIFR1   = 0x0E
.equ TOV1    = 0
.equ OCF1A   = 1
.equ OCF1B   = 2
.equ ICF1    = 5

.equ TIMSK1  = 0x0F
.equ TOIE1   = 0
.equ OCIE1A  = 1
.equ OCIE1B  = 2
.equ ICIE1   = 5

.equ TIFR2   = 0x10
.equ TOV2    = 0
.equ OCF2A   = 1
.equ OCF2B   = 2
.equ ICF2    = 5

.equ TIMSK2  = 0x11
.equ TOIE2   = 0
.equ OCIE2A  = 1
.equ OCIE2B  = 2
.equ ICIE2   = 5

.equ PCMSK0  = 0x12

.equ GPIOR0  = 0x13

.equ GPIOR1  = 0x14

.equ GPIOR2  = 0x15

.equ PINB    = 0x16
.equ PINB3   = 3
.equ PINB2   = 2
.equ PINB1   = 1
.equ PINB0   = 0

.equ DDRB    = 0x17
.equ DDRB3   = 3
.equ DDRB2   = 2
.equ DDRB1   = 1
.equ DDRB0   = 0

.equ PORTB   = 0x18
.equ PORTB3  = 3
.equ PORTB2  = 2
.equ PORTB1  = 1
.equ PORTB0  = 0

.equ PINA    = 0x19
.equ PINA7   = 7
.equ PINA6   = 6
.equ PINA5   = 5
.equ PINA4   = 4
.equ PINA3   = 3
.equ PINA2   = 2
.equ PINA1   = 1
.equ PINA0   = 0

.equ DDRA    = 0x1A
.equ DDRA7   = 7
.equ DDRA6   = 6
.equ DDRA5   = 5
.equ DDRA4   = 4
.equ DDRA3   = 3
.equ DDRA2   = 2
.equ DDRA1   = 1
.equ DDRA0   = 0

.equ PORTA   = 0x1B
.equ PORTA7  = 7
.equ PORTA6  = 6
.equ PORTA5  = 5
.equ PORTA4  = 4
.equ PORTA3  = 3
.equ PORTA2  = 2
.equ PORTA1  = 1
.equ PORTA0  = 0

.equ EECR    = 0x1C
.equ EERE    = 0
.equ EEPE    = 1
.equ EEMPE   = 2
.equ EERIE   = 3
.equ EEPM0   = 4
.equ EEPM1   = 5

.equ EEDR    = 0x1D

.equ EEARL   = 0x1E
.equ EEARH   = 0x1F

.equ PCMSK1  = 0x20

.equ WDTCSR  = 0x21
.equ WDE     = 3
.equ WDP0    = 0
.equ WDP1    = 1
.equ WDP2    = 2
.equ WDP3    = 5
.equ WDIE    = 6
.equ WDIF    = 7

.equ TCCR1C  = 0x22
.equ FOC1B   = 6
.equ FOC1A   = 7

.equ GTCCR   = 0x23
.equ PSR     = 0
.equ TSM     = 7

.equ ICR1L   = 0x24
.equ ICR1H   = 0x25

; Reserved [0x26..0x27] 

.equ OCR1BL  = 0x28
.equ OCR1BH  = 0x29

.equ OCR1AL  = 0x2A
.equ OCR1AH  = 0x2B

.equ TCNT1L  = 0x2C
.equ TCNT1H  = 0x2D

.equ TCCR1B  = 0x2E
.equ CS10    = 0
.equ CS11    = 1
.equ CS12    = 2
.equ WGM12   = 3
.equ WGM13   = 4
.equ ICES1   = 6
.equ ICNC1   = 7

.equ TCCR1A  = 0x2F
.equ WGM10   = 0
.equ WGM11   = 1
.equ COM1B0  = 4
.equ COM1B1  = 5
.equ COM1A0  = 6
.equ COM1A1  = 7

.equ TCCR0A  = 0x30
.equ WGM00   = 0
.equ WGM01   = 1
.equ COM0B0  = 4
.equ COM0B1  = 5
.equ COM0A0  = 6
.equ COM0A1  = 7

; Reserved [0x31] 

.equ TCNT0   = 0x32

.equ TCCR0B  = 0x33
.equ CS00    = 0
.equ CS01    = 1
.equ CS02    = 2
.equ WGM02   = 3
.equ FOC0B   = 6
.equ FOC0A   = 7

.equ MCUSR   = 0x34
.equ PORF    = 0
.equ EXTRF   = 1
.equ BORF    = 2
.equ WDRF    = 3

.equ MCUCR   = 0x35
.equ ISC00   = 0
.equ ISC01   = 1
.equ SM0     = 3
.equ SM1     = 4
.equ SE      = 5

.equ OCR0A   = 0x36

.equ SPMCR  = 0x37
.equ SPMEN   = 0
.equ PGERS   = 1
.equ PGWRT   = 2
.equ RFLB    = 3
.equ CTPB    = 4
.equ RSIG    = 5

.equ TIFR0   = 0x38
.equ TOV0    = 0
.equ OCF0A   = 1
.equ OCF0B   = 2

.equ TIMSK0  = 0x39
.equ TOIE0   = 0
.equ OCIE0A  = 1
.equ OCIE0B  = 2

.equ GIFR    = 0x3A
.equ PCIF0   = 4
.equ PCIF1   = 5
.equ INTF0   = 6

.equ GIMSK   = 0x3B
.equ PCIE0   = 4
.equ PCIE1   = 5
.equ INT0    = 6

.equ OCR0B   = 0x3C

; SP [0x3D..0x3E] 

; SREG [0x3F] 

.equ DIDR0   = 0x60
.equ ADC0D   = 0
.equ ADC1D   = 1
.equ ADC2D   = 2
.equ ADC3D   = 3
.equ ADC4D   = 4
.equ ADC5D   = 5
.equ ADC6D   = 6
.equ ADC7D   = 7

.equ DIDR1   = 0x61
.equ ADC11D  = 0
.equ ADC10D  = 1
.equ ADC8D   = 2
.equ ADC9D   = 3

.equ PUEB    = 0x62

.equ PUEA    = 0x63

.equ PORTCR  = 0x64
.equ BBMB    = 1
.equ BBMA    = 0

.equ REMAP   = 0x65
.equ U0MAP   = 1
.equ SPIMAP  = 0

.equ TOCPMCOE = 0x66
.equ TOCC0OE = 0
.equ TOCC1OE = 1
.equ TOCC2OE = 2
.equ TOCC3OE = 3
.equ TOCC4OE = 4
.equ TOCC5OE = 5
.equ TOCC6OE = 6
.equ TOCC7OE = 7

.equ TOCPMSA0 = 0x67
.equ TOCC0S0 = 0
.equ TOCC0S1 = 1
.equ TOCC1S0 = 2
.equ TOCC1S1 = 3
.equ TOCC2S0 = 4
.equ TOCC2S1 = 5
.equ TOCC3S0 = 6
.equ TOCC3S1 = 7

.equ TOCPMSA1 = 0x68
.equ TOCC4S0 = 0
.equ TOCC4S1 = 1
.equ TOCC5S0 = 2
.equ TOCC5S1 = 3
.equ TOCC6S0 = 4
.equ TOCC6S1 = 5
.equ TOCC7S0 = 6
.equ TOCC7S1 = 7

; Reserved [0x69] 

.equ PHDE    = 0x6A
.equ PHDEA0  = 0
.equ PHDEA1  = 1

; Reserved [0x6B..0x6F] 

.equ PRR     = 0x70
.equ PRADC   = 0
.equ PRTIM0  = 1
.equ PRTIM1  = 2
.equ PRTIM2  = 3
.equ PRSPI   = 4
.equ PRUSART0 = 5
.equ PRUSART1 = 6
.equ PRTWI   = 7

.equ CCP     = 0x71

.equ CLKCR   = 0x72
.equ CKSEL0  = 0
.equ CKSEL1  = 1
.equ CKSEL2  = 2
.equ CKSEL3  = 3
.equ SUT     = 4
.equ CKOUTC  = 5
.equ CSTR    = 6
.equ OSCRDY  = 7

.equ CLKPR   = 0x73
.equ CLKPS0  = 0
.equ CLKPS1  = 1
.equ CLKPS2  = 2
.equ CLKPS3  = 3

.equ OSCCAL0 = 0x74

.equ OSCTCAL0A = 0x75

.equ OSCTCAL0B = 0x76

.equ OSCCAL1 = 0x77

; Reserved [0x78..0x7F] 

.equ UDR0    = 0x80

.equ UBRR0L  = 0x81
.equ UBRR0H  = 0x82

.equ UCSR0D  = 0x83
.equ SFDE0   = 5
.equ RXS0    = 6
.equ RXSIE0  = 7

.equ UCSR0C  = 0x84
.equ UCPOL0  = 0
.equ UCSZ00  = 1
.equ UCSZ01  = 2
.equ USBS0   = 3
.equ UPM00   = 4
.equ UPM01   = 5
.equ UMSEL00 = 6
.equ UMSEL01 = 7

.equ UCSR0B  = 0x85
.equ TXB80   = 0
.equ RXB80   = 1
.equ UCSZ02  = 2
.equ TXEN0   = 3
.equ RXEN0   = 4
.equ UDRIE0  = 5
.equ TXCIE0  = 6
.equ RXCIE0  = 7

.equ UCSR0A  = 0x86
.equ MPCM0   = 0
.equ U2X0    = 1
.equ UPE0    = 2
.equ DOR0    = 3
.equ FE0     = 4
.equ UDRE0   = 5
.equ TXC0    = 6
.equ RXC0    = 7

; Reserved [0x87..0x8F] 

.equ UDR1    = 0x90

.equ UBRR1L  = 0x91
.equ UBRR1H  = 0x92

.equ UCSR1D  = 0x93
.equ SFDE1   = 5
.equ RXS1    = 6
.equ RXSIE1  = 7

.equ UCSR1C  = 0x94
.equ UCPOL1  = 0
.equ UCSZ10  = 1
.equ UCSZ11  = 2
.equ USBS1   = 3
.equ UPM10   = 4
.equ UPM11   = 5
.equ UMSEL10 = 6
.equ UMSEL11 = 7

.equ UCSR1B  = 0x95
.equ TXB81   = 0
.equ RXB81   = 1
.equ UCSZ12  = 2
.equ TXEN1   = 3
.equ RXEN1   = 4
.equ UDRIE1  = 5
.equ TXCIE1  = 6
.equ RXCIE1  = 7

.equ UCSR1A  = 0x96
.equ MPCM1   = 0
.equ U2X1    = 1
.equ UPE1    = 2
.equ DOR1    = 3
.equ FE1     = 4
.equ UDRE1   = 5
.equ TXC1    = 6
.equ RXC1    = 7

; Reserved [0x97..0x9F] 

.equ TWSD    = 0xA0
.equ TWSD0   = 0
.equ TWSD1   = 1
.equ TWSD2   = 2
.equ TWSD3   = 3
.equ TWSD4   = 4
.equ TWSD5   = 5
.equ TWSD6   = 6
.equ TWSD7   = 7

.equ TWSAM   = 0xA1
.equ TWAE    = 0
.equ TWSAM1  = 1
.equ TWSAM2  = 2
.equ TWSAM3  = 3
.equ TWSAM4  = 4
.equ TWSAM5  = 5
.equ TWSAM6  = 6
.equ TWSAM7  = 7

.equ TWSA    = 0xA2

.equ TWSSRA  = 0xA3
.equ TWAS    = 0
.equ TWDIR   = 1
.equ TWBE    = 2
.equ TWC     = 3
.equ TWRA    = 4
.equ TWCH    = 5
.equ TWASIF  = 6
.equ TWDIF   = 7

.equ TWSCRB  = 0xA4
.equ TWCMD0  = 0
.equ TWCMD1  = 1
.equ TWAA    = 2
.equ TWHNM   = 3

.equ TWSCRA  = 0xA5
.equ TWSME   = 0
.equ TWPME   = 1
.equ TWSIE   = 2
.equ TWEN    = 3
.equ TWASIE  = 4
.equ TWDIE   = 5
.equ TWSHE   = 7

; Reserved [0xA6..0xAF] 

.equ SPDR    = 0xB0

.equ SPSR    = 0xB1
.equ SPI2X   = 0
.equ WCOL    = 6
.equ SPIF    = 7

.equ SPCR    = 0xB2
.equ SPR0    = 0
.equ SPR1    = 1
.equ CPHA    = 2
.equ CPOL    = 3
.equ MSTR    = 4
.equ DORD    = 5
.equ SPE     = 6
.equ SPIE    = 7

; Reserved [0xB3..0xBF] 

.equ ICR2L   = 0xC0
.equ ICR2H   = 0xC1

.equ OCR2BL  = 0xC2
.equ OCR2BH  = 0xC3

.equ OCR2AL  = 0xC4
.equ OCR2AH  = 0xC5

.equ TCNT2L  = 0xC6
.equ TCNT2H  = 0xC7

.equ TCCR2C  = 0xC8
.equ FOC2B   = 6
.equ FOC2A   = 7

.equ TCCR2B  = 0xC9
.equ CS20    = 0
.equ CS21    = 1
.equ CS22    = 2
.equ WGM22   = 3
.equ WGM23   = 4
.equ ICES2   = 6
.equ ICNC2   = 7

.equ TCCR2A  = 0xCA
.equ WGM20   = 0
.equ WGM21   = 1
.equ COM2B0  = 4
.equ COM2B1  = 5
.equ COM2A0  = 6
.equ COM2A1  = 7

; Interrupt vectors 
; Vector 0 is the reset vector 
; External Interrupt Request 0 
.equ INT0addr            = 1

; Pin Change Interrupt Request 0 
.equ PCI0addr            = 2

; Pin Change Interrupt Request 1 
.equ PCI1addr            = 3

; Watchdog Time-out Interrupt 
.equ WDTaddr            = 4

; Timer/Counter1 Capture Event 
.equ TIMER1_CAPTaddr            = 5

; Timer/Counter1 Compare Match A 
.equ OC1Aaddr            = 6

; Timer/Counter1 Compare Match B 
.equ OC1Baddr            = 7

; Timer/Counter1 Overflow 
.equ OVF1addr            = 8

; TimerCounter0 Compare Match A 
.equ OC0Aaddr            = 9

; TimerCounter0 Compare Match B 
.equ OC0Baddr            = 10

; Timer/Couner0 Overflow 
.equ OVF0addr            = 11

; Analog Comparator 0 
.equ ACI0addr            = 12

; ADC Conversion Complete 
.equ ADCCaddr            = 13

; EEPROM Ready 
.equ ERDYaddr            = 14

; Analog Comparator 1 
.equ ACI1addr            = 15

; Timer/Counter2 Capture Event 
.equ TIMER2_CAPTaddr            = 16

; Timer/Counter2 Compare Match A 
.equ OC2Aaddr            = 17

; Timer/Counter2 Compare Match B 
.equ OC2Baddr            = 18

; Timer/Counter2 Overflow 
.equ OVF2addr            = 19

; Serial Peripheral Interface 
.equ SPIaddr            = 20

; USART0, Start 
.equ USART0_STARTaddr            = 21

; USART0, Rx Complete 
.equ USART0_RXaddr            = 22

; USART0 Data Register Empty 
.equ USART0_UDREaddr            = 23

; USART0, Tx Complete 
.equ USART0_TXaddr            = 24

; USART1, Start 
.equ USART1_STARTaddr            = 25

; USART1, Rx Complete 
.equ USART1_RXaddr            = 26

; USART1 Data Register Empty 
.equ USART1_UDREaddr            = 27

; USART1, Tx Complete 
.equ USART1_TXaddr            = 28

; Two-wire Serial Interface 
.equ TWI_SLAVEaddr            = 29

.equ INT_VECTORS_SIZE = 60


; Constants 

.equ FLASHSTART   = 0x0000
.equ FLASHEND     = 0x1FFF
.equ SRAM_START     = 0x0100
.equ SRAM_SIZE      = 512
.equ RAMEND       = 0x02FF
.equ EEPROMSTART     = 0
.equ EEPROMSIZE      = 512
.equ EEPROMPAGESIZE  = 4
.equ EEPROMEND       = 0x01FF
.equ XRAMEND      = RAMEND


; Fuses 

.equ FUSE_MEMORY_SIZE = 3

; Low Fuse Byte 
.equ SUT_CKSEL0 = 0
.equ SUT_CKSEL1 = 1
.equ SUT_CKSEL2 = 2
.equ SUT_CKSEL3 = 3
.equ SUT_CKSEL4 = 4
.equ CKOUT  =     6
.equ CKDIV8 =     7

; High Fuse Byte 
.equ BODLEVEL0 =  0
.equ BODLEVEL1 =  1
.equ BODLEVEL2 =  2
.equ EESAVE    =  3
.equ WDTON     =  4
.equ SPIEN     =  5
.equ DWEN      =  6
.equ RSTDISBL  =  7

; Extended Fuse Byte 
.equ SELFPRGEN  = 0
.equ BODACT0    = 1
.equ BODACT1    = 2
.equ BODPD0     = 3
.equ BODPD1     = 4
.equ ULPOSCSEL0 = 5
.equ ULPOSCSEL1 = 6
.equ ULPOSCSEL2 = 7

.def XH = r27
.def XL = r26
.def YH = r29
.def YL = r28
.def ZH = r31
.def ZL = r30


#endif ; #ifdef _AVR_ATTINY841_H_INCLUDED 

