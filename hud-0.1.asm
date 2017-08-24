
; Panteltje (c) h501s_hud.asm based on lac.asm (laser arrow control .asm) Copyright Jan Panteltje 2013-always

; 32 bit signed integer math copyright PETER HEMSLEY the rest is copyright Jan Panteltje
; and released under the GPL. GPL violaters will be procecuted.
; If you use this code, you are required to release SOURCE CODE.


; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.

; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.

; You should have received a copy of the GNU General Public License
; along with this program; if not, write to the Free Software
; Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.



; Serial link listens at 57600 Baud 8 data bits 1 stop bit to Hubsan H501S remote controller AFC_RX testpoint.
; this data is decoded and transformed to on screen messages.


; This code is assembled with gpasm in Linux.
; This code likely will not assemble without changes in Microchip mplab.



;                           **** set TABS to 4 to read any of this ****



; CHANGES:
;
; 0.1:
; This is a first release.


; uncommwent this for full GPS display, else only number of satellites
#define FULL_GPS

; uncomment this to see the data buffer
;#define DEBUG_BUFFER

; uncomment this to see the decoding at the serial output
;#define DEBUG_DECODE

; comment this out not to send any decoded data to SAA5281 display memory
#define TO_SAA

; uncomment for debug
;#define REPORT_COM_ERRORS
;#define REPORT_CHECKSUM_ERRORS

; please note that the SAA5281 is a PAL chip, in NTSC it works but column and line numbers are different, see screen layout below


; set timer0 timeout
; timer 0 will timeout when no serial character from AFC_RX is received for 20 ms, this indicates the end of a data block, any time a byte comes in timer0 is reset.
; on timer 0 overflow we process the data in the RX buffer, the first byte should always be 0xff, if so do a checksum, and if OK decode the field and send it to the display.
; after that we reset the data pointer to the start of the rx buffer, ready for the next sequence of bytes.
;
; we need abiut 20 ms so 50 Hz
; default clock speed (1 second tick)
; timer0 clock = (64000000 / 4) / 256 = 62500 ticks
; 1 second = 62500 tick, so 2/100 second = (2 / 100) * 62500 = 1250 ticks
; for a 20 ms second tick we need to preset timer 1 to 65535 - 1250 =  65535 - 1250 = 64285
#define TIMER0_PRELOAD                  D'64285'



; position on screeen of data in characters

; Hubsan H501S is NTSC!!!
; As SAA5281 does not support NTSC, weird xpos and ypos numbers  have to be used
; uncomment this to test on a PAL system
;#define PAL
#ifdef PAL

#define VOLTS_X					D'0'
#define VOLTS_Y					D'21'

#define HEADING_X				D'5'
#define HEADING_Y				D'21'

#define DRONE_POSITION_X		D'0'
#define DRONE_POSITION_Y		D'19'

#define REMOTE_POSITION_X		D'0'
#define REMOTE_POSITION_Y		D'20'

#define ALTITUDE_X				D'11'
#define ALTITUDE_Y				D'21'

#define AIRSPEED_X				D'20'
#define AIRSPEED_Y				D'21'

#define DISTANCE_X				D'30'
#define DISTANCE_Y				D'21'

#define BUTTON_X				D'21'
#define BUTTON_Y				D'22'

#define REQUEST_X				D'0'
#define REQUEST_Y				D'1'

#define STATUS_X				D'0'
#define STATUS_Y				D'22'

#else ; NTSC

#define VOLTS_X					D'0'
#define VOLTS_Y					D'13'

#define HEADING_X				D'5'
#define HEADING_Y				D'13'

#define DRONE_POSITION_X		D'0'
#define DRONE_POSITION_Y		D'11'

#define REMOTE_POSITION_X		D'0'
#define REMOTE_POSITION_Y		D'12'

#define ALTITUDE_X				D'11'
#define ALTITUDE_Y				D'13'

#define AIRSPEED_X				D'20'
#define AIRSPEED_Y				D'13'

#define DISTANCE_X				D'30'
#define DISTANCE_Y				D'13'

#define BUTTON_X				D'21'
#define BUTTON_Y				D'14'

#define REQUEST_X				D'0'
#define REQUEST_Y				D'20'

#define STATUS_X				D'0'
#define STATUS_Y				D'14'

; screen layout in NTSC
; y  x
;    37 38 39 00 01
; 19 black	Hubsan camera icon is here
; 18 black	
; 17 video
; ...
; ...
; 12 video
; 13 video
; 14 black	
; 15 black	bottom line, may not be visible

#endif ; NTSC




; SAAA5281
#define SAA5281_ADDRESS		D'34'

; PIC
#define VREF	D'4096'		; mV, internal x 4

; For speed, either 16 MHz without PLL 19200 Bd, or 64 MHz with PLL 115200 Bd
#define USE_PLL


TRUE    equ     1
FALSE   equ     0
MSB		equ		7


; include PIC register definitions.
	include <P18F14K22.INC>
	PROCESSOR	PIC18F14K22

;	Oscillator	Selection	bits:
;	CONFIG	FOSC = LP			; LP	oscillator
;	CONFIG	FOSC = XT			; XT	oscillator
;	CONFIG	FOSC = HS			; HS oscillator
;	CONFIG	FOSC = ERCCLKOUT	; External RC oscillator, CLKOUT function on OSC2
;	CONFIG	FOSC = ECCLKOUTH	; EC, CLKOUT function on OSC2 (high)
;	CONFIG	FOSC = ECH			; EC (high)
;	CONFIG	FOSC = ERC			; External RC oscillator
	CONFIG	FOSC = IRC			; Internal RC oscillator
;	CONFIG	FOSC = IRCCLKOUT	; Internal RC oscillator, CLKOUT function on OSC2
;	CONFIG	FOSC = ECCLKOUTM	; EC, CLKOUT function on OSC2 (medium)
;	CONFIG	FOSC = ECM			; EC (medium)
;	CONFIG	FOSC = ECCLKOUTL	; EC, CLKOUT function on OSC2 (low)
;	CONFIG	FOSC = ECL			; EC (low)
; 4 X PLL Enable bit:
	CONFIG	PLLEN = OFF			; PLL is under software control
;	CONFIG	PLLEN = ON			; Oscillator multiplied by 4
; Primary Clock Enable Bit:
	CONFIG	PCLKEN = OFF		; Primary clock is under software control
;	CONFIG	PCLKEN = ON			; Primary clock enabled
; Fail-Safe Clock Monitor Enable bit:
	CONFIG	FCMEN = OFF			; Fail-Safe Clock Monitor disabled
;	CONFIG	FCMEN = ON			; Fail-Safe Clock Monitor enabled
; Internal/External Oscillator Switchover bit:
;;	CONFIG	IESO = OFF			; Oscillator Switchover mode disabled
;	CONFIG	IESO = ON			; Oscillator Switchover mode enabled
; Power-up Timer Enable bit:
;	CONFIG	PWRTEN = ON			; PWRT enabled
	CONFIG	PWRTEN = OFF		; PWRT disabled
; Brown-out Reset Enable bits:
;	CONFIG	BOREN = OFF			; Brown-out Reset disabled in hardware and software
;	CONFIG	BOREN = ON			; Brown-out Reset enabled and controlled by software (SBOREN is enabled)
;	CONFIG	BOREN = NOSLP		; Brown-out Reset enabled in hardware only and disabled in Sleep mode (SBOREN is disabled)
	CONFIG	BOREN = SBORDIS		; Brown-out Reset enabled in hardware only (SBOREN is disabled)
; Brown Out Voltage:
	CONFIG	BORV = 30			; VBOR set to 2.85 V nominal
;	CONFIG	BORV = 27			; VBOR set to 2.5 V nominal
;	CONFIG	BORV = 22			; VBOR set to 2.2 V nominal
;	CONFIG	BORV = 19			; VBOR set to 1.9 V nominal
; Watchdog Timer Enable bit:
	CONFIG	WDTEN = OFF			; WDT is controlled by SWDTEN bit of the WDTCON register
;	CONFIG	WDTEN = ON			; WDT is always enabled. SWDTEN bit has no effect.
; Watchdog Timer Postscale Select bits:
;	CONFIG	WDTPS = 1			; 1:1
;	CONFIG	WDTPS = 2			; 1:2
;	CONFIG	WDTPS = 4			; 1:4
;	CONFIG	WDTPS = 8			; 1:8
;	CONFIG	WDTPS = 16			; 1:16
;	CONFIG	WDTPS = 32			; 1:32
;	CONFIG	WDTPS = 64			; 1:64
;	CONFIG	WDTPS = 128			; 1:128
;	CONFIG	WDTPS = 256			; 1:256
;	CONFIG	WDTPS = 512			; 1:512
	CONFIG	WDTPS = 1024		; 1:1024			x 4 ms = about 4 seconds, enough time to print the help menu at this baudrate.
;	CONFIG	WDTPS = 2048		; 1:2048
;	CONFIG	WDTPS = 4096		; 1:4096
;	CONFIG	WDTPS = 8192		; 1:8192
;	CONFIG	WDTPS = 16384		; 1:16384
;	CONFIG	WDTPS = 32768		; 1:32768
; MCLR Pin Enable bit:
	CONFIG	MCLRE = OFF			; RA3 input pin enabled	CONFIG MCLR disabled
;	CONFIG	MCLRE = ON			; MCLR pin enabled, RA3 input pin disabled
; HFINTOSC Fast Start-up bit:
	CONFIG	HFOFST = OFF		; The system clock is held off until the HFINTOSC is stable.
;	CONFIG	HFOFST = ON			; HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.
; Stack Full/Underflow Reset Enable bit:
;	CONFIG	STVREN = OFF		; Stack full/underflow will not cause Reset
	CONFIG	STVREN = ON			; Stack full/underflow will cause Reset
; Single-Supply ICSP Enable bit:
	CONFIG	LVP = OFF			; Single-Supply ICSP disabled
;	CONFIG	LVP = ON			; Single-Supply ICSP enabled
; Boot Block Size Select Bit:
	CONFIG	BBSIZ = OFF			; 1kW boot block size
;	CONFIG	BBSIZ = ON			; 2kW boot block size
; Extended Instruction Set Enable bit:
	CONFIG	XINST = OFF			; Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
;	CONFIG	XINST = ON			; Instruction set extension and Indexed Addressing mode enabled
; Code Protection bit:
;	CONFIG	CP0 = ON			; Block 0 code-protected
	CONFIG	CP0 = OFF			; Block 0 not code-protected
;	Code Protection bit:
;	CONFIG	CP1 = ON			; Block 1 code-protected
	CONFIG	CP1 = OFF			; Block 1 not code-protected
; Boot Block Code Protection bit:
;	CONFIG	CPB = ON			; Boot block code-protected
	CONFIG	CPB = OFF			; Boot block not code-protected
; Data EEPROM Code Protection bit:
;	CONFIG	CPD = ON			; Data EEPROM code-protected
	CONFIG	CPD = OFF			; Data EEPROM not code-protected
; Write Protection bit:
;	CONFIG	WRT0 = ON			; Block 0 write-protected
	CONFIG	WRT0 = OFF			; Block 0 not write-protected
; Write Protection bit:
;	CONFIG	WRT1 = ON			; Block 1 write-protected
	CONFIG	WRT1 = OFF			; Block 1 not write-protected
; Boot Block Write Protection bit:
;	CONFIG	WRTB = ON			; Boot block write-protected
	CONFIG	WRTB = OFF			; Boot block not write-protected
; Configuration Register Write Protection bit:
;	CONFIG	WRTC = ON			; Configuration registers write-protected
	CONFIG	WRTC = OFF			; Configuration registers not write-protected
; Data EEPROM Write Protection bit:
;	CONFIG	WRTD = ON			; Data EEPROM write-protected
	CONFIG	WRTD = OFF			; Data EEPROM not write-protected
; Table Read Protection bit:
;	CONFIG	EBTR0 = ON			; Block 0 protected from table reads executed in other blocks
	CONFIG	EBTR0 = OFF			; Block 0 not protected from table reads executed in other blocks
; Table Read Protection bit:
;	CONFIG	EBTR1 = ON			; Block 1 protected from table reads executed in other blocks
	CONFIG	EBTR1 = OFF			; Block 1 not protected from table reads executed in other blocks
;	Boot Block Table Read Protection bit:
;	CONFIG	EBTRB = ON			; Boot block protected from table reads executed in other blocks
	CONFIG	EBTRB = OFF			; Boot block not protected from table reads executed in other blocks



; RS232 commands 115200 Bd 1 start bit, 8 data bits, 1 stop bit, type h for help.


; for SAA5281 display
#define END_BOX         D'10'
#define START_BOX       D'11'
#define NORMAL_HEIGHT   D'12'
#define DOUBLE_HEIGHT   D'13'

; for i2c addressing (data)
#define COMMAND_START			0x100
#define TEXT_COMMAND_START		0x110
#define TEXT_START				TEXT_COMMAND_START+1

#define DATA_BUFFER_START		0x180	; max 25 bytes needed it seems
#define DATA_BUFFER_END			0x1a0



; add more bytes here



; RS232 
#define SIGNAL_BELL									D'7'


; variables
	CBLOCK 0x00

	temp1							; [LCD]print_w_ascii_dec						
	flags1							; cleared on reset	
	count						
	temp_u
	temp_h						
	temp_l						
	temp4							; print_32_ascii_dec
	flags2							; some of these flags are left over reset
	eeprom_address				
	eeprom_data
	temp_w						
	temp_s						
	AARGB4							; used in print 16 in debug, do not use for anythng else.
	AARGB3
	AARGB7
	AARGB6
	AARGB5
	sign
	REGA3
	REGA2
	REGA1
	REGA0
	REGB3
	REGB2
	REGB1
	REGB0
	REGC3
	REGC2
	REGC1
	REGC0
	MCOUNT
	temp3					; print_adc, select_baudrate
	temp_x					; hex printing only
	temp_x2					; hex printimg only
	delay_count1
	flags4
	iic_delay_cnt
	iic_loop_cnt
	iic_temp
	iic_byte_cnt
	iic_device_address
	acqmem
	dspmem
	xpos
	ypos
	txt_len
	flags5
	checksum
	buffer_counter
	volts
	heading_h
	heading_l
	latitude_3
	latitude_2
	latitude_1
	latitude_0
	longitude_3
	longitude_2
	longitude_1
	longitude_0
	satellites
	d_latitude_3
	d_latitude_2
	d_latitude_1
	d_latitude_0
	d_longitude_3
	d_longitude_2
	d_longitude_1
	d_longitude_0
	d_satellites
; 69
	



; 96 is overwritten!

	ENDC

; variables kept in RAM accessed by 'poke' and 'peek':

; define  flags1
#define ONE_MINUTE_FLAG						flags1,0
#define SIGNED_FLAG							flags1,1
#define FIRST_ZERO_SUPPRESSED_FLAG		  	flags1,2
#define T_NEGATIVE_FLAG						flags1,3
#define SAVE_SETTINGS_FLAG					flags1,4
#define DEBUG_FLAG							flags1,5
#define ONE_SECOND_FLAG						flags1,6
#define NEGATIVE_ALTITUDE_FLAG				flags1,7


; define flags2
#define REPORT_FLAG							flags2,0
#define WATCHDOG_RESET_FLAG					flags2,1
#define BROWNOUT_RESET_FLAG					flags2,2

				
; define flags3, flags3 is saved in EEPROM


; define flags4
#define	NO_SND_BYTE_ERROR_MSG_FLAG			flags4,0
#define	LAST_BYTE_FLAG						flags4,1
#define SAVE_POSITION_FLAG					flags4,2
#define CHECKSUM_FLAG						flags4,3
#define IN_TEXT_FLAG						flags4,4
#define BOX_FLAG							flags4,5
#define SHORT_BOX_FLAG						flags4,6
#define DOUBLE_HEIGHT_FLAG					flags4,7


; define flags5
#define DIVIDE_BY_TEN_FLAG					flags5,0
#define DIVIDE_BY_HUNDRED_FLAG				flags5,1
#define DIVIDE_BY_THOUSAND_FLAG				flags5,2
#define	DIVIDE_BY_TEN_THOUSAND_FLAG			flags5,3
#define DIVIDE_BY_HUNDRED_THOUSAND_FLAG		flags5,4
#define DIVIDE_BY_ONE_MILLION_FLAG			flags5,5
#define DIVIDE_BY_HUNDRED_MILLION_FLAG		flags5,6
#define HAVE_DOT_FLAG						flags5,7


to_saa macro pointer
	clrf	acqmem
	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

; get text length
	lfsr	FSR0, pointer
	movff	POSTINC0, txt_len

; iic
	movlw	SAA5281_ADDRESS
	movwf	iic_device_address

; point one byte before text, overwrite what is there
	lfsr	FSR0, pointer

; select R11
	movlw	D'11'				; R11
	movwf	POSTINC0

; send text string, FSR0 now pointing to DATA_BUFFER_START+D'7'
	movff	txt_len, iic_byte_cnt	; length of text data
	incf	iic_byte_cnt			; plus R11 select byte

; point to iic command, will increment pointer 1x
	lfsr	FSR0, pointer
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM
	endm




div32_by_2 macro
; clr carry 
; rotate right REGB1 into carry
; rotate right REGB0 into carry  
; rotate right REGC1 into carry  
; rotate right REGC0 into carry  
	bcf		STATUS, C
	rrcf	REGB1
	rrcf	REGB0
	rrcf	REGC1
	rrcf	REGC0
	endm



; conversion macros pc16 to pic18
rlf	macro	destination1, destination2
	rlcf	destination1, destination2
	endm

rrf	macro	destination1, destination2
	rrcf	destination1, destination2
	endm

movfw macro source
	movf	source, W
	endm

; macros to save and restore W and status register in interrupt.
save_w_stat macro
	movwf   temp_w
	swapf   STATUS,W
	clrf	STATUS			; extra force bank 0 clears IRP, RP1, RP0
	movwf   temp_s
	endm

restore_w_stat macro
	swapf   temp_s,W
	movwf   STATUS
	swapf   temp_w,F
	swapf   temp_w,W
	endm

poke macro address
	lfsr	FSR1, address
	movwf	INDF1
	endm

peek macro address, register
	lfsr	FSR1, address 
	movff	INDF1, register
	endm


; code start
code_start:
	org	0
	goto	reset_entry

; first interrupt vector
	org	8
	goto	int_8
;	retfie

	org 0x18
;	goto	int_18
	retfie


; main program start
reset_entry:
; set internal OSC speed, the output on pin 3 (OSC2) is this clock / 4. 

; test reset possibilities
; test normal (power up) reset

detect_bor_reset:
	btfsc	RCON, NOT_BOR					; '0' is BOR reset
	bra		end_bor_reset	
; NOT_BOR brown out reset happened.
	bsf		RCON, NOT_BOR
; signal in main to user
	bsf		BROWNOUT_RESET_FLAG
end_bor_reset:


detect_watchdog_reset:
	btfsc	RCON, NOT_TO
	bra		end_watchdog_reset	
; NOT_TO watchdog reset happened	
	bsf		RCON, NOT_TO
; signal in main to user
	bsf		WATCHDOG_RESET_FLAG
end_watchdog_reset:


set_oscillator:

;							; bit 7          IDLEN: Idle Enable bit
;							;               1 = Device enters Idle mode on SLEEP instruction
;							;               0 = Device enters Sleep mode on SLEEP instruction
	bcf	OSCCON, IDLEN
;							; bit 6-4        IRCF<2:0>: Internal Oscillator Frequency Select bits
;							;               111 = 16 MHz
;							;               110 = 8 MHz
;							;               101 = 4 MHz
;							;               100 = 2 MHz
;							;               011 = 1 MHz(3)
;							;               010 = 500 kHz
;							;               001 = 250 kHz
;							;               000 = 31 kHz(2)
	bsf	OSCCON, IRCF2
	bsf	OSCCON, IRCF1			; 16 MHz
	bsf	OSCCON, IRCF0
	
;							; bit 3          OSTS: Oscillator Start-up Time-out Status bit(1)
;							;               1 = Device is running from the clock defined by FOSC<2:0> of the CONFIG1 register
;							;               0 = Device is running from the internal oscillator (HFINTOSC or LFINTOSC)
	bcf	OSCCON, OSTS				; internal osc.

;							; bit 2          HFIOFS: HFINTOSC Frequency Stable bit
;							;               1 = HFINTOSC frequency is stable
;							;               0 = HFINTOSC frequency is not stable
;	bcf	OSCCON, HFIOFS			; read only

;							; bit 1-0        SCS<1:0>: System Clock Select bits
;							;               1x = Internal oscillator block
;							;               01 = Secondary (Timer1) oscillator
;							;               00 = Primary clock (determined by CONFIG1H[FOSC<3:0>]).
;							; Note 1:    Reset state depends on state of the IESO Configuration bit.
;							;        2: Source selected by the INTSRC bit of the OSCTUNE register, see text.
;							;        3: Default output frequency of HFINTOSC on Reset.
	bcf	OSCCON, SCS1			; primary clock set by CONFIG1H <----------- set to 00 for PLL
	bcf	OSCCON, SCS0


; osccon2
;							; bit 7-3 Unimplemented: Read as 

;							; bit 2   PRI_SD: Primary Oscillator Drive Circuit shutdown bit
;							;        1 = Oscillator drive circuit on
;							;        0 = Oscillator drive circuit off (zero power)
;	bsf		OSCCON2, PRI_SD		; default

;							; bit 1   HFIOFL: HFINTOSC Frequency Locked bit
;							;        1 = HFINTOSC is in lock
;							;        0 = HFINTOSC has not yet locked
; status	

;							; bit 0   LFIOFS: LFINTOSC Frequency Stable bit
;							;        1 = LFINTOSC is stable
;							;        0 = LFINTOSC is not stable
; status	



; osctune
;							; bit 7   INTSRC: Internal Oscillator Low-Frequency Source Select bit
;							;        1 = 31.25 kHz device clock derived from 16 MHz HFINTOSC source (divide-by-512 enabled)
;							;        0 = 31 kHz device clock derived directly from LFINTOSC internal oscillator
	bcf	OSCTUNE, INTSRC			; internal osc

;							; bit 6   PLLEN: Frequency Multiplier PLL bit
;							;        1 = PLL enabled (for HFINTOSC 8 MHz only)
;							;        0 = PLL disabled
#ifdef USE_PLL
	bsf	OSCTUNE, PLLEN			; PLL enabled
#else
	bcf	OSCTUNE, PLLEN			; PLL disabled
#endif ; ! USE_PLL
;							; bit 5-0 TUN<5:0>: Frequency Tuning bits
;							;        011111 = Maximum frequency
;							;        011110 =
;							;         ···
;							;        000001 =
;							;        000000 = Oscillator module is running at the factory calibrated frequency.
;							;        111111 =
;							;         ···
;							;        100000 = Minimum frequency
	bcf		OSCTUNE, 5		
	bcf		OSCTUNE, 4		
	bcf		OSCTUNE, 3			; factory preset	
	bcf		OSCTUNE, 2		
	bcf		OSCTUNE, 1		
	bcf		OSCTUNE, 0		


; digital I/O defines

; i2c related	
#define SCL_OUT					LATB,6
#define SDA_IN					PORTC,2
#define SDA_PULLDOWN			LATB,4

; ports init

	clrf	PORTA
	clrf	LATA

	clrf	PORTB
	clrf	LATB

	clrf 	PORTC
	clrf	LATC


; I/O:
; pin  1	Vdd
; pin  2							RA5			i	carrier detect
; pin  3							RA4			o	clock out		
; pin  4	Vpp						RA3			i  	100k to ground
; pin  5							RC5 	    o	
; pin  6							RC4		    o	
; pin  7							RC3			o
; pin  8							RC6		    o 	
; pin  9							RC7		    o	
; pin 10	serial out				RB7			o	serial out
; pin 11							RB6			o	SCL_OUT
; pin 12	serial in				RB5			i	serial in
; pin 13							RB4			o	SDA_PULLDOWN
; pin 14							RC2			i	SDA_IN
; pin 15							RC1			i	//battery voltage	
; pin 16							RC0			o	
; pin 17							RA2			i	RX data
; pin 18	PGC						RA1			o	
; pin 19	PGD						RA0			o	
; pin 20	Vss


	bsf		TRISA,	5		; carrier detect
	bcf		TRISA,	4		; 
	bsf		TRISA,	3		; Vpp
	bsf		TRISA,	2		; RX data
	bcf		TRISA,	1		; 
	bcf		TRISA,	0		; 

	bsf		TRISB,	7		; TX serial out
	bcf		TRISB,	6		; SCL_OUT
	bsf		TRISB,	5		; RX serial in
	bcf		TRISB,	4		; SDA_PULLDOWN

	
	bcf		TRISC, 7		; 
	bcf		TRISC, 6		; 
	bcf		TRISC, 5		;
	bcf		TRISC, 4		; 
	bcf		TRISC, 3		; 
	bsf		TRISC, 2		; SDA_IN
	bcf		TRISC, 1		; //battery voltage
	bcf		TRISC, 0		; 


	clrf 	PORTC
	clrf	LATC


; init async serial out 
;12.1.1.6        Asynchronous Transmission Set-up:
;1.  Initialize the SPBRGH, SPBRG register pair and
;    the BRGH and BRG16 bits to achieve the desired
;    baud rate (see Section 12.3 "EUSART Baud
;    Rate Generator (BRG)").
;2.  Enable the asynchronous serial port by clearing
;    the SYNC bit and setting the SPEN bit.
;3.  If 9-bit transmission is desired, set the TX9 con-
;    trol bit. A set ninth data bit will indicate that the 8
;    Least Significant data bits are an address when
;    the receiver is set for address detection.
;4.  Enable the transmission by setting the TXEN
;    control bit. This will cause the TXIF interrupt bit
;    to be set.
;5.  If interrupts are desired, set the TXIE interrupt
;    enable bit. An interrupt will occur immediately
;    provided that the GIE and PEIE bits of the
;    INTCON register are also set.
;6.  If 9-bit transmission is selected, the ninth bit
;    should be loaded into the TX9D data bit.
;7.  Load 8-bit data into the TXREG register. This
;    will start the transmission.

; 8 bit async
; SYNC = 0, BRG16 = 0, BRGH=1, at 8MHz: 19231 Baud, 0.16% for SPBRG=25

;	bcf TRISB, TRISB7			; pin 10 (RB7,TX,CK) output	


;	bcf	BAUDCTL, BRG16
	bsf	BAUDCTL, BRG16

	bsf TXSTA, BRGH
	bcf TXSTA, SYNC
	bsf TXSTA, TXEN

	bsf RCSTA, SPEN


;12.1.2.8        Asynchronous Reception Set-up:
;1.  Initialize the SPBRGH, SPBRG register pair and
;    the BRGH and BRG16 bits to achieve the
;    desired baud rate (see Section 12.3 "EUSART
;    Baud Rate Generator (BRG)").
;2.  Enable the serial port by setting the SPEN bit.
;    The SYNC bit must be clear for asynchronous
;    operation.
;3.  If interrupts are desired, set the RCIE interrupt
;    enable bit and set the GIE and PEIE bits of the
;    INTCON register.
;4.  If 9-bit reception is desired, set the RX9 bit.
;5.  Enable reception by setting the CREN bit.
;6.  The RCIF interrupt flag bit will be set when a
;    character is transferred from the receive shift
;    register to the receive buffer. An interrupt will be
;    generated if the RCIE interrupt enable bit was
;    also set.
;7.  Read the RCSTA register to get the error flags
;    and, if 9-bit data reception is enabled, the ninth
;    data bit.
;8.  Get the received 8 Least Significant data bits
;    from the receive buffer by reading the RCREG
;    register.
;9.  If an overrun occurred, clear the OERR flag by
;    clearing the CREN receiver enable bit.


	bsf	RCSTA, CREN
; SYNC and SPEN already done.	


; select internal reference 1.024 x 4 = 4096 mV
;								; bit 7   FVR1EN: Fixed Voltage Reference 1 Enable bit
;								; 0 = FVR is disabled
;								; 1 = FVR is enabled
	bsf	VREFCON0, FVR1EN		; vref on

;								; bit 6   FVR1ST: Fixed Voltage Reference 1 Stable bit
;								; 0 = FVR is not stable
;								; 1 = FVR is stable

	
;								; bit 5-4 FVR1S<1:0>: Fixed Voltage Reference 1 Voltage Select bits
;								; 00 = Reserved, do not use
;								; 01 = 1.024V (x1)
;								; 10 = 2.048V (x2)
;								; 11 = 4.096V (x4)
	bsf	VREFCON0, FVR1S1		; x4
	bsf	VREFCON0, FVR1S0		

; 								; bit 3-0 Unimplemented: Read as '0'



	clrf	VREFCON1
;								; bit 7   D1EN: DAC 1 Enable bit
;								; 0 = DAC 1 is disabled
;								; 1 = DAC 1 is enabled
;								; bit 6   D1LPS: DAC 1 Low-Power Voltage State Select bit
;								; 0 = VDAC = DAC1 Negative reference source selected
;								; 1 = VDAC = DAC1 Positive reference source selected
;								; bit 5   DAC1OE: DAC 1 Voltage Output Enable bit
;								; 1 = DAC 1 voltage level is also outputed on the RA0/AN0/CVREF/VREF-/C1IN+/INT0/PGD pin
;								; 0 = DAC 1 voltage level is disconnected from RA0/AN0/CVREF/VREF-/C1IN+/INT0/PGD pin pin
;								; bit 4   Unimplemented: Read as 
;								; bit 3-2 D1PSS<1:0>: DAC 1 Positive Source Select bits
;								; 00 = VDD
;								; 01 = VREF+
;								; 10 = FVR output
;								; 11 = Reserved, do not use
;								; bit 1   Unimplemented: Read as 
;								; bit 0   D1NSS: DAC1 Negative Source Select bits
;								; 0 = VSS
;								; 1 = VREF-



	clrf	VREFCON2
;								; bit 7-5      Unimplemented: Read as 
;								; bit 4-0      DAC1R<4:0>: DAC1 Voltage Output Select bits
;								; VOUT = ((VSOURCE+) - (VSOURCE-))*(DAC1R<4:0>/(2^5)) + VSOURCE-
;								; Note 1: The output select bits are always right justified to ensure that any number of bits can be used without affecting the register layout.


; This code block configures the ADC
; for polling,  Vdd reference, Frc clock and AN0 input.
; Conversion start & polling for completion are included.
;
	movlw     B'00100000'	; not used, :32, 
							; bit 7	not used, zero
							; bit 6 clock speed
							; bit 5 clock speed
							; bit 4 clock speed
							; bit 3 not used
							; bit 2 not used
							; bit 1 not used
							; bit 0 not used
	movwf     ADCON1


; select which port c pins are analog inputs
; Analog inputs:	AN0, AN1, AN2, AN3,    AN5,    AN7, AN8, AN9
	bcf		ANSEL, 0			; ASN 0
	bcf		ANSEL, 1			; ASN 1
	bcf		ANSEL, 2			; ASN 2
	bcf		ANSEL, 3			; ASN 3
	bcf		ANSEL, 4			; ASN 4
	bcf		ANSEL, 5			; ASN 5
	bcf		ANSEL, 6			; ASN 6
	bcf		ANSEL, 7			; ASN 7

	bcf		ANSELH, 0			; ASN 8
	bcf		ANSELH, 1			; ASN 9
	bcf		ANSELH, 2			; ASN 10
	bcf		ANSELH, 3			; ASN 11	UART RX
							; only lower 4 bits used	


; timer 0
; timer 0 timeout 25 ms 
	bsf		T0CON, TMR0ON

								; bit 6 T08BIT Timer0 8-bit / 16-bit Control bit
								;	0 = 16 bit	<------------------------------------------
								;	1 = 8 bit
	bcf		T0CON, T08BIT

								; bit 5 T0CS Timer0 Clock Source Select bit
								;	1 = Transition on T0CKI pin
								;	0 = Internal instruction cycle clock (CLKOUT) <------------------------------------------

	bcf		T0CON, T0CS

								; bit 4 T0SE Timer0 Source Edge Select bit,
								;	1 = Increment on high-to-low transition on T0CKI pin
								;	0 = Increment on low-to-high transition on T0CKI pin <------------------------------------------
	bcf		T0CON, T0SE

								; bit 3 PSA Timer0 Prescaler Assignment bit
								;	1 = Timer0 prescaler is NOT assigned. Timer0 clock input bypasses prescaler,
								; 	0 = Timer0 prescaler is assigned.  Timer0 clock input comes from prescaler output. <------------------------------------------
	bcf		 T0CON, PSA

								; bit 2-0 T0PS<2:0>: Timer0 Prescaler Select bits
								;	111 = 1:256 prescale value		<------------------------------------------
								;	110 = 1:128 prescale value
								; 	101 = 1:64 prescale value
								;	100 = 1:32 prescale value
								; 	011 = 1:16 prescale value
								;	010 = 1:8 prescale value
								;	001 = 1:4 prescale value
								;	000 = 1:2 prescale value
	bsf		T0CON, T0PS2
	bsf		T0CON, T0PS1
	bsf		T0CON, T0PS0

	movlw	HIGH	TIMER0_PRELOAD
	movwf	TMR0H

	movlw	LOW		TIMER0_PRELOAD
	movwf	TMR0L


; Timer 1
; timer 1 off
; timer1 alarm tones (up-counter int on overflow)
; f = (16000000 / 8) / (65535 - x)  ->  (65535 - x) * f = (16000000 / 8)  ->  65535 - x = (16000000 / 8) / f  ->  x = 65535 - ( (16000000 / 8)  / f) use wcalc -EE
; T1CON
; TIMER1 CONTROL REGISTER
;     R/W-0           R-0            R/W-0         R/W-0          R/W-0            R/W-0        R/W-0          R/W-0
;     RD16            T1RUN          T1CKPS1       T1CKPS0        T1OSCEN           T1SYNC      TMR1CS         TMR1ON
;     bit 7                                                                                                    bit 0
; Legend:
; R = Readable bit                W = Writable bit             U = Unimplemented bit, read as  -n = Value at POR
;  1 = Bit is set             0 = Bit is cleared           x = Bit is unknown
;
;									bit 7   RD16: 16-bit Read/Write Mode Enable bit
;					            	   	1 = Enables register read/write of Timer1 in one 16-bit operation
;               						0 = Enables register read/write of Timer1 in two 8-bit operations	<------------------------------------------
	bcf		T1CON, RD16

;									bit 6  T1RUN: Timer1 System Clock Status bit
; 						              	1 = Main system clock is derived from Timer1 oscillator
;						              	0 = Main system clock is derived from another source	<------------------------------------------
	bcf		T1CON, T1RUN

;									bit 5-4	T1CKPS<1:0>: Timer1 Input Clock Prescale Select bits
;						                11 = 1:8 Prescale value 			<------------------------------------------
; 						               	10 = 1:4 Prescale value
;						                01 = 1:2 Prescale value
;						                00 = 1:1 Prescale value				
	bsf		T1CON, T1CKPS1
	bsf		T1CON, T1CKPS0


;									bit 3	T1OSCEN: Timer1 Oscillator Enable bit
;						                1 = Timer1 oscillator is enabled
;						                0 = Timer1 oscillator is shut off,  the oscillator inverter and feedback resistor are turned off to eliminate power drain.	<------------------------------------------
	bcf		T1CON, T1OSCEN

;									bit 2	T1SYNC: Timer1 External Clock Input Synchronization Select bit
;									    When TMR1CS = 1:
;							                1 = Do not synchronize external clock input	
;							                0 = Synchronize external clock input
;						                When TMR1CS = 0:
;							                This bit is ignored. Timer1 uses the internal clock when TMR1CS = 0.
	bcf		T1CON,	NOT_T1SYNC


;									bit 1	TMR1CS: Timer1 Clock Source Select bit
;						               	1 = External clock from the T13CKI pin (on the rising edge)
;       						       	0 = Internal clock (FOSC/4)		<------------------------------------------
	bcf		T1CON, TMR1CS

;									bit 0	 TMR1ON: Timer1 On bit
;						                1 = Enables Timer1					<------------------------------------------
; timer1 off untill an alarm detected
	bcf		T1CON, TMR1ON


; set timer 1 reload and clear TMR1IF as on page 88 pdf	
;	movlw	HIGH	TIMER1_PRELOAD_4000Hz
;	movwf	TMR1H
;	movlw	LOW		TIMER1_PRELOAD_4000Hz
;	movwf	TMR1L

; reset timer 1 interrupt flag	
	bcf		PIR1,	TMR1IF






; interrupt enable
; no interrupts
	clrf	PIE1

; WAS timer 1 interrupt enable
; clock off
	bcf		PIE1,	TMR1IE			; timer 1 overflow interrupt enable
;	bsf		PIE1,	CMIE			; comparator interrupt enable
;	bsf		PIE1,	TXIE			; 

;	bsf		IPR1, RCIP				; bit 5 RCIP: EUSART Receive Interrupt Priority bit   1 = High priority 0 = Low priority
;	bsf		IPR1, TMR1IP 			; bit 0 TMR1IP: TMR1 Overflow Interrupt Priority bit   1 = High priority  0 = Low priority


; PIE2: OSFIE C2IE C1IE EEIE -- -- -- --
	bcf	PIE2,	EEIE			; EEPROM write complete interrupt enable
	


main_select_baudrate:
; baudrate selection
; WAS TEST
	movlw	D'6'					; select 57600 Bd 
	call	select_baudrate			; uses W

; flags1 to zero
	clrf	flags1					; also resets SIGNED_FLAG

; flags2							; some flags2 are preserved over reset
	bsf		REPORT_FLAG

; flags4 to zero
	clrf	flags4

; flags5 to zero
	clrf	flags5

; timer 1 not used
	clrf	T1CON

	clrf	IOCB
	clrf	IOCA

; enable gobal and peripheral interrupt
	clrf	INTCON

	bsf		INTCON,	PEIE			; peripheral interrupts enable
	bsf		INTCON,	TMR0IE			; timer 0 overflow interrupt enable
;	bcf		INTCON,	INTE			; GP2/INT interrupt enable
	bcf 	INTCON,	INT0IE
	bcf		INTCON, RABIE			; port A and B interrupt on change
	bsf		INTCON,	GIE				; global interrupt enable


;								; bit 7-1        Unimplemented: Read as 
;								; bit 0          SWDTEN: Software Enable or Disable the Watchdog Timer bit(1)
;								; 1 = WDT is turned on
;								; 0 = WDT is turned off (Reset value)
;								; Note 1: This bit has no effect if the Configuration bit, WDTEN, is enabled.
	bcf		WDTCON, SWDTEN		; disable watchdog for now


	lfsr	FSR1,	DATA_BUFFER_START

	clrf	volts
	clrf	heading_h
	clrf	heading_l
	clrf	latitude_3
	clrf	latitude_2
	clrf	latitude_1
	clrf	latitude_0
	clrf	longitude_3
	clrf	longitude_2
	clrf	longitude_1
	clrf	longitude_0
	clrf	satellites

; select display 0
	clrf	dspmem

main:

; activate watchdog
;	clrwdt

; report any watchdog reset
test_watchdog_reset:
	btfss	WATCHDOG_RESET_FLAG
	bra		end_test_watchdog_reset		

	movlw	'W'
	call	tx_w
	movlw	'a'
	call	tx_w
	movlw	't'
	call	tx_w
	movlw	'c'
	call	tx_w
	movlw	'h'
	call	tx_w
	movlw	'd'
	call	tx_w
	movlw	'o'
	call	tx_w
	movlw	'g'
	call	tx_w
	call	tx_crlf

	bcf		WATCHDOG_RESET_FLAG
end_test_watchdog_reset:

; report any brownout reset
test_brownout_reset:
	btfss	BROWNOUT_RESET_FLAG
	bra		end_test_brownout_reset

	movlw	'B'
	call	tx_w
	movlw	'r'
	call	tx_w
	movlw	'o'
	call	tx_w
	movlw	'w'
	call	tx_w
	movlw	'n'
	call	tx_w
	movlw	'o'
	call	tx_w
	movlw	'u'
	call	tx_w
	movlw	't'
	call	tx_w
	call	tx_crlf

	bcf		BROWNOUT_RESET_FLAG
end_test_brownout_reset:

	call	iicini

	call	saa5281_ini

	call	tcls

	clrf	buffer_counter

; enable serial port
	call	enable_serial_port		; will also send ID

main_loop:
	clrwdt
	bra		main_loop



; ***************** subroutines ********************


print_status:
	call	tx_crlf

; battery voltage
;	call    print_adc

	call	print_battery_voltage

	call    tx_crlf


; acquisition memory
	movlw	'a'
	call	tx_w
	
	movlw	'c'
	call	tx_w
	
	movlw	'q'
	call	tx_w
	
	movlw	'm'
	call	tx_w
	
	movlw	'e'
	call	tx_w
	
	movlw	'm'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w
	
	movlw	' '
	call	tx_w

	movfw	acqmem
	call	print_w_ascii_dec
	call	tx_crlf

; display memory
	movlw	'd'
	call	tx_w
	
	movlw	's'
	call	tx_w
	
	movlw	'p'
	call	tx_w
	
	movlw	'm'
	call	tx_w
	
	movlw	'e'
	call	tx_w
	
	movlw	'm'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w
	
	movlw	' '
	call	tx_w

	movfw	dspmem
	call	print_w_ascii_dec
	call	tx_crlf


; cursor x
	movlw	'c'
	call	tx_w
	
	movlw	'u'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'x'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w
	
	movlw	' '
	call	tx_w

	movfw	xpos
	call	print_w_ascii_dec
	call	tx_crlf


; cursor y
	movlw	'c'
	call	tx_w
	
	movlw	'u'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'y'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w
	
	movlw	' '
	call	tx_w

	movfw	ypos
	call	print_w_ascii_dec
	call	tx_crlf


; double height
	movlw	'd'
	call	tx_w
	
	movlw	'o'
	call	tx_w
	
	movlw	'u'
	call	tx_w
	
	movlw	'b'
	call	tx_w
	
	movlw	'l'
	call	tx_w
	
	movlw	'e'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'h'
	call	tx_w

	movlw	'e'
	call	tx_w
	
	movlw	'i'
	call	tx_w
	
	movlw	'g'
	call	tx_w
	
	movlw	'h'
	call	tx_w
	
	movlw	't'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w

	movlw	' '
	call	tx_w
	
	movlw	'1'
	btfss	DOUBLE_HEIGHT_FLAG
	movlw	'0'

	call	tx_w
	call	tx_crlf
	

; box
	movlw	'b'
	call	tx_w
	
	movlw	'o'
	call	tx_w
	
	movlw	'x'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'1'
	btfss	BOX_FLAG
	movlw	'0'

	call	tx_w
	call	tx_crlf

; short box
	movlw	's'
	call	tx_w

	movlw	'h'
	call	tx_w
	
	movlw	'o'
	call	tx_w
	
	movlw	'r'
	call	tx_w
	
	movlw	't'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'b'
	call	tx_w
	
	movlw	'o'
	call	tx_w
	
	movlw	'x'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'1'
	btfss	SHORT_BOX_FLAG
	movlw	'0'

	call	tx_w
	call	tx_crlf


; in text flag
	movlw	'i'
	call	tx_w
	
	movlw	'n'
	call	tx_w
	
	movlw	't'
	call	tx_w
	
	movlw	'e'
	call	tx_w
	
	movlw	'x'
	call	tx_w
	
	movlw	't'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w

	movlw	' '
	call	tx_w
	
	movlw	'1'
	btfss	IN_TEXT_FLAG
	movlw	'0'

	call	tx_w
	call	tx_crlf

; text length
	movlw	't'
	call	tx_w
	
	movlw	'x'
	call	tx_w
	
	movlw	'_'
	call	tx_w
	
	movlw	'l'
	call	tx_w
	
	movlw	'e'
	call	tx_w
	
	movlw	'n'
	call	tx_w
	
	movlw	' '
	call	tx_w
	
	movlw	'='
	call	tx_w
	
	movlw	' '
	call	tx_w

	movfw	txt_len
	call	print_w_ascii_dec
	call	tx_crlf

#ifdef PRINT_TEXT
; text
	movlw	D'0'
	cpfseq	txt_len, 0 
	bra	 	status_have_text
	bra		status_end

status_have_text:
; have some text
	lfsr 	FSR0,	TEXT_START
	movff	txt_len, temp1

status_text_print_loop:
	movfw	POSTINC0
;	movlw	'*'
	call	tx_w
	decfsz	temp1
	bra		status_text_print_loop
#endif ; PRINT_TEXT


#ifdef PRINT_32_TEST
; print 32 test	
	movlw	D'0'
	movwf	REGA3

	movlw	UPPER	D'4097'
	movwf	REGA2

	movlw	HIGH	D'4097'
	movwf	REGA1

	movlw	LOW	D'4097'
	movwf	REGA0

;	bsf		DIVIDE_BY_TEN_FLAG
;	bsf		DIVIDE_BY_HUNDRED_FLAG
	bsf		DIVIDE_BY_THOUSAND_FLAG
;	bsf		DIVIDE_BY_TEN_THOUSAND_FLAG
	call	tprint_32_ascii_dec					; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display at position xpos, ypos, normal height, short box
	bcf		DIVIDE_BY_THOUSAND_FLAG

	call	tx_crlf
#endif ; PRINT_32_TEST

status_end:
	call 	tx_crlf
	
	return



delay10us:						; delays W * 10 us 
	movlw	D'10'				; 10 us
	movwf	delay_count1
delay_loop1:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	decf	delay_count1,f
	btfss	STATUS,	Z		
	bra		delay_loop1		

	return



#define USE_PINT_HEX
#ifdef USE_PINT_HEX
print_w_hex:
	movwf	temp_x2

	rrncf	WREG, W
	rrncf	WREG, W
	rrncf	WREG, W
	rrncf	WREG, W

	andlw	0xf
	call	tx_hex_nibble_in_w

	movfw	temp_x2
	andlw	0xf
	call	tx_hex_nibble_in_w	

	return


tx_hex_nibble_in_w:
	movwf	temp_x
	movlw	D'9'
	cpfsgt	temp_x	
	bra		les_ten
; >  9
	movfw	temp_x
	addlw	D'55'
	bra		tx_w
les_ten:
	movfw	temp_x
#endif ; USE_PRINT_HEX

; send byte in W
tx_digit_in_w:
	addlw	'0'				; zero

tx_w:
	btfsc	REPORT_FLAG
    movwf   TXREG

test_tx_empty:
    btfss   TXSTA,  TRMT 
    bra	    test_tx_empty

	return



print_w_ascii_dec:				; prints register W in ASCII decimal
	bsf		FIRST_ZERO_SUPPRESSED_FLAG

	movwf	temp1

	clrf	count				; number of hundreds found
loop_hundreds:
	movlw	D'100'
	subwf	temp1
	btfss	STATUS,	C			; if no carry flag, no more hundreds, go count tenth		
	bra		count10				; substraction failed
	incf	count	
	bra		loop_hundreds
count10:
	movlw	D'0'
	subwf	count,	W
	btfsc	STATUS,	Z
	bra		suppress_first_zero

	bcf		FIRST_ZERO_SUPPRESSED_FLAG

	movfw	count

	call	tx_digit_in_w			; print hundreds

suppress_first_zero:
	movlw	D'100'				; restore temp1 from one substract to many
	addwf	temp1	

	clrf	count				; number of tenth found
loop_tenth:
	movlw	D'10'
	subwf	temp1
	btfss	STATUS,	C			; if no carry flag no more tenth, only units left	
	bra		count1		
	incf	count
	bra		loop_tenth
count1:
	movlw	D'0'
	subwf	count,	W
	btfss	STATUS,	Z
	bra		print_tenth			; tenth not zero

; tenth zero
; test if zero supression was active in hundreds (first digit)
	btfsc	FIRST_ZERO_SUPPRESSED_FLAG
	bra		print_units			; if first digit was not zero, print this zero	

print_tenth:
	movfw	count
	call	tx_digit_in_w			; print tenth
	
	btfss	DIVIDE_BY_TEN_FLAG
	bra		print_units

	movlw	'.'
	call	tx_w

print_units:
	movlw	D'10'				; restore temp1 from 1 substract to many
	addwf	temp1

; units
	movfw	temp1
	call	tx_digit_in_w			; print units

	return



tx_crlf:
	movlw	D'13'
	call	tx_w				; CR
	movlw	D'10'				; LF
	call	tx_w
	return



; math start

;*** 32 BIT SIGNED SUBTRACT ***
subtract:	; REGA - REGB -> REGA return carry set if overflow
	call	negateb		;Negate REGB
	skpnc
	return			;Overflow

;*** 32 BIT SIGNED ADD ***
add:	; REGA + REGB -> REGA return carry set if overflow
	movf	REGA3,w		;Compare signs
	xorwf	REGB3,w
	movwf	sign

	call	addba		;Add REGB to REGA

	clrc			;Check signs
	movf	REGB3,w		;If signs are same
	xorwf	REGA3,w		;so must result sign
	btfss	sign,7		;else overflow
	addlw	0x80
	return

;*** 32 BIT SIGNED MULTIPLY ***
multiply:	; REGA * REGB -> REGA return carry set if overflow 
	clrf	sign		;Reset sign flag
	call	absa		;Make REGA positive
	skpc
	call	absb		;Make REGB positive
	skpnc
	return			;Overflow

	call	movac		;Move REGA to REGC
	call	clra		;Clear product

	movlw	D'31'		;Loop counter
	movwf	MCOUNT

muloop:
	call	slac		;Shift left product and multiplicand
	
	rlf	REGC3,w		;Test MSB of multiplicand
	skpnc			;If multiplicand bit is a 1 then
	call	addba		;add multiplier to product

	skpc			;Check for overflow
	rlf	REGA3,w
	skpnc
	return

	decfsz	MCOUNT,f	;Next
	bra		muloop

	btfsc	sign,0		;Check result sign
	call	negatea		;Negative
	return


;*** 32 BIT SIGNED DIVIDE ***
divide:		; REGA / REGB -> REGA return carry set if overflow or division by zero
	clrf	sign		;Reset sign flag
	movf	REGB0,w		;Trap division by zero
	iorwf	REGB1,w
	iorwf	REGB2,w
	iorwf	REGB3,w
	sublw	0
	skpc
	call	absa		;Make dividend (REGA) positive
	skpc
	call	absb		;Make divisor (REGB) positive
	skpnc
	return			;Overflow

	clrf	REGC0		;Clear remainder
	clrf	REGC1
	clrf	REGC2
	clrf	REGC3
	call	slac		;Purge sign bit

	movlw	D'31'		;Loop counter
	movwf	MCOUNT

dvloop:
	call	slac		;Shift dividend (REGA) msb into remainder (REGC)

	movf	REGB3,w		;Test if remainder (REGC) >= divisor (REGB)
	subwf	REGC3,w
	skpz
	bra		dtstgt
	movf	REGB2,w
	subwf	REGC2,w
	skpz
	bra		dtstgt
	movf	REGB1,w
	subwf	REGC1,w
	skpz
	bra		dtstgt
	movf	REGB0,w
	subwf	REGC0,w
dtstgt:
	skpc			;Carry set if remainder >= divisor
	bra		dremlt

	movf	REGB0,w		;Subtract divisor (REGB) from remainder (REGC)
	subwf	REGC0,f
	movf	REGB1,w
	skpc
	incfsz	REGB1,w
	subwf	REGC1,f
	movf	REGB2,w
	skpc
	incfsz	REGB2,w
	subwf	REGC2,f
	movf	REGB3,w
	skpc
	incfsz	REGB3,w
	subwf	REGC3,f
	clrc
	bsf	REGA0,0		;Set quotient bit

dremlt:
	decfsz	MCOUNT,f	;Next
	bra		dvloop

	btfsc	sign,0		;Check result sign
	call	negatea		;Negative
	return

;*** ROUND RESULT OF DIVISION TO NEAREST INTEGER ***

round:
	clrf	sign		;Reset sign flag
	call	absa		;Make positive
	clrc
	call	slc		;Multiply remainder by 2
	movf	REGB3,w		;Test if remainder (REGC) >= divisor (REGB)
	subwf	REGC3,w
	skpz
	bra		rtstgt
	movf	REGB2,w
	subwf	REGC2,w
	skpz
	bra		dtstgt
	movf	REGB1,w
	subwf	REGC1,w
	skpz
	bra		rtstgt
	movf	REGB0,w
	subwf	REGC0,w
rtstgt:
	skpc			;Carry set if remainder >= divisor
	bra		rremlt
	incfsz	REGA0,f		;Add 1 to quotient
	bra		rremlt
	incfsz	REGA1,f
	bra		rremlt
	incfsz	REGA2,f
	bra		rremlt
	incf	REGA3,f
	skpnz
	return			;Overflow,return carry set
rremlt:
	btfsc	sign,0		;Restore sign
	call	negatea
	return

#ifdef OLD_CODE
;*** 32 BIT SQUARE ROOT ***
;sqrt(REGA) -> REGA
;Return carry set if negative
sqrt:
	rlf	REGA3,w		;Trap negative values
	skpnc
	return

	call	movac		;Move REGA to REGC
	call	clrba		;Clear remainder (REGB) and root (REGA)

	movlw	D'16'		;Loop counter
	movwf	MCOUNT

sqloop:
	rlf	REGC0,f		;Shift two msb's
	rlf	REGC1,f		;into remainder
	rlf	REGC2,f
	rlf	REGC3,f
	rlf	REGB0,f
	rlf	REGB1,f
	rlf	REGB2,f
	rlf	REGC0,f
	rlf	REGC1,f
	rlf	REGC2,f
	rlf	REGC3,f
	rlf	REGB0,f
	rlf	REGB1,f
	rlf	REGB2,f

	setc			;Add 1 to root
	rlf	REGA0,f		;Align root
	rlf	REGA1,f
	rlf	REGA2,f

	movf	REGA2,w		;Test if remdr (REGB) >= root (REGA)
	subwf	REGB2,w
	skpz
	bra		ststgt
	movf	REGA1,w
	subwf	REGB1,w
	skpz
	bra		ststgt
	movf	REGA0,w
	subwf	REGB0,w
ststgt:
	skpc			;Carry set if remdr >= root
	bra		sremlt

	movf	REGA0,w		;Subtract root (REGA) from remdr (REGB)
	subwf	REGB0,f
	movf	REGA1,w
	skpc
	incfsz	REGA1,w
	subwf	REGB1,f
	movf	REGA2,w
	skpc
	incfsz	REGA2,w
	subwf	REGB2,f
	bsf	REGA0,1		;Set current root bit

sremlt:
	bcf	REGA0,0		;Clear test bit
	decfsz	MCOUNT,f	;Next
	bra		sqloop

	clrc
	rrf	REGA2,f		;Adjust root alignment
	rrf	REGA1,f
	rrf	REGA0,f
	return
#endif ; OLD_CODE


;UTILITY ROUTINES


;Add REGB to REGA (Unsigned)
;Used by add, multiply,
addba:
	movf	REGB0,w		;Add lo byte
	addwf	REGA0,f

	movf	REGB1,w		;Add mid-lo byte
	skpnc			;No carry_in, so just add
	incfsz	REGB1,w		;Add carry_in to REGB
	addwf	REGA1,f		;Add and propagate carry_out

	movf	REGB2,w		;Add mid-hi byte
	skpnc
	incfsz	REGB2,w
	addwf	REGA2,f

	movf	REGB3,w		;Add hi byte
	skpnc
	incfsz	REGB3,w
	addwf	REGA3,f
	return


;Move REGA to REGC
;Used by multiply, sqrt
movac:
	movf	REGA0,w
	movwf	REGC0
	movf	REGA1,w
	movwf	REGC1
	movf	REGA2,w
	movwf	REGC2
	movf	REGA3,w
	movwf	REGC3
	return


;Clear REGB and REGA
;Used by sqrt
clrba:
	clrf	REGB0
	clrf	REGB1
	clrf	REGB2
	clrf	REGB3

;Clear REGA
;Used by multiply, sqrt
clra:
	clrf	REGA0
	clrf	REGA1
	clrf	REGA2
	clrf	REGA3
	return


;Check sign of REGA and convert negative to positive
;Used by multiply, divide, bin2dec, round
absa:
	rlf	REGA3,w
	skpc
	return			;Positive

;Negate REGA
;Used by absa, multiply, divide, bin2dec, dec2bin, round
negatea:
	movf	REGA3,w		;Save sign in w
	andlw	0x80

	comf	REGA0,f		;2's complement
	comf	REGA1,f
	comf	REGA2,f
	comf	REGA3,f
	incfsz	REGA0,f
	bra		nega1
	incfsz	REGA1,f
	bra		nega1
	incfsz	REGA2,f
	bra		nega1
	incf	REGA3,f
nega1:
	incf	sign,f		;flip sign flag
	addwf	REGA3,w		;Return carry set if -2147483648
	return


;Check sign of REGB and convert negative to positive
;Used by multiply, divide
absb:
	rlf	REGB3,w
	skpc
	return			;Positive

;Negate REGB
;Used by absb, subtract, multiply, divide
negateb:
	movf	REGB3,w		;Save sign in w
	andlw	0x80

	comf	REGB0,f		;2's complement
	comf	REGB1,f
	comf	REGB2,f
	comf	REGB3,f
	incfsz	REGB0,f
	bra		negb1
	incfsz	REGB1,f
	bra		negb1
	incfsz	REGB2,f
	bra		negb1
	incf	REGB3,f
negb1:
	incf	sign,f		;flip sign flag
	addwf	REGB3,w		;Return carry set if -2147483648
	return


;Shift left REGA and REGC
;Used by multiply, divide, round
slac:
	rlf	REGA0,f
	rlf	REGA1,f
	rlf	REGA2,f
	rlf	REGA3,f
slc:
	rlf	REGC0,f
	rlf	REGC1,f
	rlf	REGC2,f
	rlf	REGC3,f
	return

; math end


disable_serial_port:
	bcf RCSTA, SPEN
	
	bcf TXSTA, TXEN

	bcf PIE1,   RCIE            ; USART receive interrupt enable

	return


enable_serial_port:
	bsf RCSTA, SPEN
	
	bsf TXSTA, TXEN

	bsf PIE1,   RCIE            ; USART receive interrupt enable

; confirm to user by sending ID
	call	print_id

	return


int_8:
; interrupt entry point
; do interrupt processing here

	save_w_stat					; save W and status

	clrwdt						; print for example help continously will not allow main_loop  to clear teh watchdog time.
; what interrupt?


; test for timer0 interrupt, RX data timeout
	btfss	INTCON, TMR0IF
	bra		end_test_timer0_interrupt
; timer0 interrupt, timout no incoming data for 20 ms

; WAS 
;	movlw	'T'
;	call	tx_w
;	call	tx_crlf

; reset the data pointer to the start of the data buffer
; FSR1 is pointer in buffer
	lfsr	FSR1, DATA_BUFFER_START

#ifdef DEBUG_BUFFER
; print whole buffer
	lfsr	FSR0, DATA_BUFFER_START

; beware, do not use decfsz, loop will do 0xff steps if buffer_counter is zero, so prevent that
pri_loop:
	movlw	D'0'
	cpfseq	buffer_counter
	bra		show_buffer_byte
	bra		set_buffer_pointer_zero	

show_buffer_byte:
	movfw	POSTINC0
	call	print_w_hex
	movlw	' '
	call	tx_w
	decf	buffer_counter
	bra		pri_loop		
	call	tx_crlf
#endif ; DEBUG_BUFFER

set_buffer_pointer_zero:
	clrf	buffer_counter

	bcf		INTCON, TMR0IF

; disable further timer0 interrupts, only RX characters will enable timer0 again
	bcf		INTCON, TMR0IE          ; timer 0 overflow interrupt disable

; process the data acquired in the buffer
	bra		process_buffer

end_test_timer0_interrupt:



; test_serial_port_interrupt
	btfss	PIR1,	RCIF				; test if serial port interrupt
	bra		not_serial_port_interrupt

; serial port interrupt
	bcf PIE1,	RCIE 

; test framing error
	btfss	RCSTA,	FERR
	bra		test_overrun
; framing error
; ignore character, read next one

; From datasheet page 185:	Framing error (can be updated by reading RCREG register and receive next valid byte)	
; What they mean is read RCREG!!

#ifdef REPORT_COM_ERRORS
	movlw	'F'
	call	tx_w
	movlw	'E'
	call	tx_w
	call	tx_crlf
endif REPORT_COM_ERRORS

	movfw	RCREG

; need this too, not in datahsheet!
	bcf		RCSTA, CREN
	bsf		RCSTA, CREN

	bra		int_end_rx

test_overrun:
	btfss	RCSTA,	OERR
	bra		get_rx_char	
; overrun error

#ifdef REPORT_COM_ERRORS
	movlw	'O'
	call	tx_w
	movlw	'E'
	call	tx_w
	call	tx_crlf
#endif REPORT_COM_ERRORS

; either clear framing error,
	bcf		RCSTA, CREN
	bsf		RCSTA, CREN
; or resert UART
;	bcf		RCSTA, SPEN	
;	bsf		RCSTA, SPEN	
	bra		int_end_rx


get_rx_char:
; have serial char in RCREG

; WAS TEST
; echo
;	movfw	RCREG
;	call	tx_w
;	bra		int_end_rx

; store in buffer
; set limit to data block size
	movlw	D'32'
	cpfslt	buffer_counter
; buffer full, ignore byte
	bra		int_end_rx
; store byte in buffer and increment pointer
	movff	RCREG, POSTINC1
	incf	buffer_counter

; reset timer0 the timeout counter
; timer0 interrupt, timout no incoming data for 20 ms
	movlw	HIGH	TIMER0_PRELOAD
	movwf	TMR0H
	movlw	LOW		TIMER0_PRELOAD
	movwf	TMR0L
; enable timer0 timeout counter
	bsf		INTCON, TMR0IE          ; timer 0 overflow interrupt enable

; loop for next character
	bra		int_end_rx


process_buffer:	
; if FF_FLAG is set, test next byte



; if FF_FLAG and one of the following byte sequences:
; FF 09   43            09  00 00  01 00  47 01  51									5C		00 00 roll  01 00 pitch  01 47 yaw  51 battery  5C checksum
; FF 09   43            09  C1 FF  E8 FF  03 00  51									31

; FF 0B	  41            09  C5 83 BD 1F 98 6D 56 03 05	 				            09		C5 83 BD 1F latitude	98 6D 56 03 longitude	05 number of sats	09 checksum     remote
; FF 0B   42            0B  EC 7C BD 1F AA 70 56 03 09                              FD  	EC 7C BD 1F latitude	AA 70 56 03	longitude	09 number of sats   FD checksum		drone 

; FF 0B   41            09  2F 83 BD 1F EB 6D 56 03 06                              93

; FF 0C	  16 01 09      08  20 20 20 20 30 2E 30 25									1D		airspeed		ASCII	0. 0%
; FF 0C	  16 04 05      08  20 20 20 20 20 5F 30 6D									3D		distance		ASCII	_0m
; FF 0C	  16 00 03      08  20 20 20 5E 30 2E 30 6D									20		altitude		ASCII	^ 0.0 m

; text top or bottom line
; FF 0D	  15 00	00 23   08  20 50 48 4F 54 4F 20 20									52		photo button	ASCII	   P H O T O    

; FF 11	  15 00	00 23   0C  48 35 30 31 53 20 56 31 2E 32 2E 39						59		power up ID		ASCII	H501S V1.2.9
; FF 11   15 00	00 23   0C  20 46 4F 4C 4C 4F 57 20 4F 4E 20 20						2A		follow on		ASCII	FOLLOW ON
; FF 11   15 00	00 25   0C  20 46 4F 4C 4C 4F 57 20 4F 46 46 20						42		follow off		ASCII	FOLLOW OFF
; FF 11   15 00	00 23   0C  20 4C 45 44 20 53 57 49 54 43 48 20	 					45		LED switch		ASCII	LED SWITCH 	cycles, red, green, but as there are 3 states (on flash off) colors are not assigned to 'on' or 'off'
; FF 11   15 00	00 25   0C  20 4C 45 44 20 53 57 49 54 43 48 20	 					..		LED switch		ASCII	LED SWITCH		so sometimes 'on' is red. and sometimes 'on' is green. this is a Hubsan coding error.

; FF 13	  15 00	14 05   0E  20 4D 61 6E 75 61 6C 20 6D 6F 64 65 20 20 				33		status			ASCII	Manual mode
; FF 13   15 00	14 05   0E  20 47 50 53 20 48 6F 6C 64 20 20 20 20 20				41		status			ASCII	GPS Hold
; FF 13   15 00	14 05   OE  20 41 6C 74 69 74 75 64 65 20 48 6F 6C 64 				15		,,				ASCII	Altitude Hold
; FF 13   15 00	14 05   0E  20 46 6F 6C 6C 6F 77 20 4D 6F 64 65 20 20				18		,,				ASCII	Follow Mode
; FF 13   15 00	14 05   0E  20 52 65 74 75 72 6E 20 48 6F 6D 65 20 20 				0F		,,				ASCII	Return Home
; FF 13   15 00	00 23   0E  20 48 45 41 44 4C 45 53 53 20 4F 4E 20 20				38		,,				ASCII	HEADLESS ON
; FF 13   15 00	00 25   0E  20 48 45 41 44 4C 45 53 53 20 4F 46 46 20				50		,,				ASCII	HEADLESS OFF

; FF 14	  15 00	01 09   0F  43 48 45 43 4B 20 47 59 52 4F 20 53 45 4E 53			5C		request			ASCII	CHECK GYRO SENS 	
; FF 15	  15 00	01 09   10  43 61 6C 69 62 20 43 6F 6D 70 61 73 73 20 31 20			09		request			ASCII	Calib Compass 1
; FF 15   15 00	01 09   10  43 61 6C 69 62 20 43 6F 6D 70 61 73 73 20 32 20			0A		request			ASCII	Calib Compass 2
; FF 17   15 00 01 00   12  20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20	17		clear field		ASCII	spaces



; get input from Hubsan H501S remote testpoint AFC_RX character in RCREG

; byte 0 is 0xff flag start data
; byte 1 is length excluding checksum
; checksum is xor sum
; checksum is over all bytes following byte 2
; byte 2
;  0x43 = left side screen
;  0x41 = 3 lines from bottom satellites and position
;  0x17 = text clear line
;  0x16 = right side screen 
;  0x15 = text
;   if text then
;     byte 3 = xpos
;     byte 4 = ypos
;     byte 5 = color?



; test byte 0 in buffer
; test for 0xff flag
; point ot byte 0
	lfsr	FSR0, DATA_BUFFER_START
	movlw	0xff
	cpfseq	POSTINC0
	bra		int_end_rx

have_ff:	
; test for any of 0x09 0x0b 0x0c 0x0d 0x11 0x13 0x14 0x17	

; point to byte 2
	lfsr	FSR0, DATA_BUFFER_START+D'2'

; test byte 2
	movlw	0x43
	cpfseq	INDF0
	bra		test_0x42
; 0x43  roll, pitch, yaw, battery, left side of screen
; FF 09  43 09             00 00 01 00 47 01 51										5C		00 00 roll  01 00 pitch  01 47 yaw  51 battery  5C checksum
; FF 09  43 09             C1 FF E8 FF 03 00 51										31		same, negative numbers
;  0  1   2  3              4  5  6  7  8  9 10                                     11

; do the checksum thing	
	call	do_checksum		; from FSR0,  returns status zero flag set if OK
	btfss	STATUS, Z
	bra		cerror
; checksum OK

; get volts
	lfsr	FSR0, DATA_BUFFER_START+D'10'
	movff	INDF0, volts


#ifdef DEBUG_DECODE
; battery
; report volts
	movfw	volts
	bsf		DIVIDE_BY_TEN_FLAG
	call	print_w_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG
	movlw	'V'
	call	tx_w	
	call	tx_crlf
#endif ; DEBUG_DECODE


#ifdef TO_SAA
; volts to SAA
	movlw	VOLTS_X
	movwf	xpos

	movlw	VOLTS_Y
	movwf	ypos

	clrf	acqmem
	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

	clrf	txt_len
	lfsr	FSR0, TEXT_START
	movfw	volts
	bsf		DIVIDE_BY_TEN_FLAG
	call	tprint_w_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG

	movlw	'V'
	movwf	POSTINC0
	incf	txt_len
#endif ; TO_SAA

	call	tputstr


; yaw = heading
	lfsr	FSR0, DATA_BUFFER_START+D'9'
; LOW Byte first!!!
	movff	POSTDEC0, heading_h
	movff	INDF0, heading_l

#ifdef DEBUG_DECODE
	movlw	'H'
	call	tx_w
	clrf	REGA3
	clrf	REGA2
	movff	heading_h, REGA1
	movff	heading_l, REGA0
; REGA3:REGA0 = heading in degrees 0-359
	call	print_32_ascii_dec
;	movlw	D'176'	; degrees
;	call	tx_w
	call	tx_crlf
#endif ; DEBUG_DECODE

#ifdef TO_SAA
; heading to SAA
	movlw	HEADING_X
	movwf	xpos

	movlw	HEADING_Y
	movwf	ypos

	clrf	acqmem
	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

	clrf	txt_len
	lfsr	FSR0, TEXT_START

	movlw	'H'
	movwf	POSTINC0
	incf	txt_len

	clrf	REGA3
	clrf	REGA2
	movff	heading_h, REGA1
	movff	heading_l, REGA0

	bcf		FIRST_ZERO_SUPPRESSED_FLAG
	call	tprint_div_100

	call	tputstr
#endif ; TO_SAA

	bra		int_end_rx


test_0x42:	; drone
	movlw	0x42
	cpfseq	INDF0
	bra		test_0x41

; do the checksum thing	
	call	do_checksum		; from FSR0,  returns status zero flag set if OK
	btfss	STATUS, Z
	bra		cerror
; checksum OK

; 0x42  number of sats, latitude, longitude, 3 lines from bottom
; FF 0B 42  0B  EC 7C BD 1F AA 70 56 03 09      FD     EC 7C BD 1F latitude    AA 70 56 03 longitude   09 number of sats   FD checksum     drone
;  0  1  2   3   4  5  6  7  8  9 10 11 12      13 

; WAS TEST
;#define SAT_LAT_LON_TEST
#ifdef SAT_LAT_LON_TEST
	lfsr	FSR0, DATA_BUFFER_START+D'4'

	movlw	0xC5
	movwf	POSTINC0
	movlw	0x83
	movwf	POSTINC0
	movlw	0xBD
	movwf	POSTINC0
	movlw	0x1F
	movwf	POSTINC0

	movlw	0x98
	movwf	POSTINC0
	movlw	0x6D
	movwf	POSTINC0
	movlw	0x56
	movwf	POSTINC0
	movlw	0x03
	movwf	POSTINC0

	movlw	0x05
	movwf	INDF0
#endif ; SAT_LAT_LON_TEST
; END TEST

; point to satellites
	lfsr	FSR0, DATA_BUFFER_START+D'12'
; satellites
	movff	POSTDEC0, d_satellites

#ifdef FULL_GPS
; longitude, reverse Endian 0x03566D98 ->   55995800 ->   5.5995800
	movff	POSTDEC0, d_longitude_3
	movff	POSTDEC0, d_longitude_2
	movff	POSTDEC0, d_longitude_1
	movff	POSTDEC0, d_longitude_0

; latitude	reverse Endian 0x1FBD83C5 ->  532513733 ->  53.2513733
	movff	POSTDEC0, d_latitude_3
	movff	POSTDEC0, d_latitude_2
	movff	POSTDEC0, d_latitude_1
	movff	POSTDEC0, d_latitude_0
#endif ; FULL_GPS

#ifdef DEBUG_DECODE
; ID drone
	movlw	'X'
	call	tx_w
	movlw	'4'
	call	tx_w
	movlw	'G'
	call	tx_w
	movlw	'P'
	call	tx_w
	movlw	'S'
	call	tx_w
	movlw	' '
	call	tx_w

; print satellites
	movfw	satellites
	call	print_w_ascii_dec
	movlw	' '
	call	tx_w

#ifdef FULL_GPS
; print latitude
	movlw	'S'
	btfss	REGA3, 7
	movlw	'N'
	call	tx_w

	movff	d_latitude_3, REGA3
	movff	d_latitude_2, REGA2
	movff	d_latitude_1, REGA1
	movff	d_latitude_0, REGA0

	call	absa
	bsf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	call	print_32_ascii_dec_lz
	bcf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	movlw	' '
	call	tx_w
	
; print longitude
	movlw	'W'
	btfss	REGA3, 7
	movlw	'E'
	call	tx_w

	movff	d_longitude_3, REGA3
	movff	d_longitude_2, REGA2
	movff	d_longitude_1, REGA1
	movff	d_longitude_0, REGA0

	call	absa
	bsf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	call	print_32_ascii_dec_lz
	bcf		DIVIDE_BY_HUNDRED_MILLION_FLAG
#endif ; FULL_GPS

	call	tx_crlf	
#endif ; DEBUG_DECODE


#ifdef TO_SAA
	movlw	DRONE_POSITION_X
	movwf	xpos

	movlw	DRONE_POSITION_Y
	movwf	ypos

	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

	lfsr	FSR0, TEXT_START	
	clrf	txt_len

; X4GPS
	movlw	'X'
	movwf	POSTINC0
	incf	txt_len

	movlw	'4'
	movwf	POSTINC0
	incf	txt_len

	movlw	'G'
	movwf	POSTINC0
	incf	txt_len

	movlw	'P'
	movwf	POSTINC0
	incf	txt_len

	movlw	'S'
	movwf	POSTINC0
	incf	txt_len

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movfw	d_satellites
	bcf		FIRST_ZERO_SUPPRESSED_FLAG
	call	tprint_w_ascii_dec_lz

#ifdef FULL_GPS

	movlw	 ' '
	movwf	POSTINC0
	incf	txt_len

	movff	d_latitude_3, REGA3
	movff	d_latitude_2, REGA2
	movff	d_latitude_1, REGA1
	movff	d_latitude_0, REGA0

	movlw	'S'
	btfss	REGA3, 7
	movlw	'N'
	movwf	temp3

	call	absa

; replace first zero that is always there with N or S

	bsf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	bcf		FIRST_ZERO_SUPPRESSED_FLAG
	call	tprint_32_ascii_dec_lz
	bcf		DIVIDE_BY_HUNDRED_MILLION_FLAG

	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0

	movff	temp3, INDF0

	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0


	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movff	d_longitude_3, REGA3
	movff	d_longitude_2, REGA2
	movff	d_longitude_1, REGA1
	movff	d_longitude_0, REGA0

	movlw	'W'
	btfss	REGA3, 7
	movlw	'E'
	movwf	POSTINC0
	incf	txt_len

	call	absa

	bsf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	bcf		FIRST_ZERO_SUPPRESSED_FLAG
	call	tprint_32_ascii_dec_lz
	bcf		DIVIDE_BY_HUNDRED_MILLION_FLAG

#endif ; FULL_GPS
	
	call	tputstr

#endif ; TO_SSA

	bra		int_end_rx


test_0x41:	; remote
	movlw	0x41
	cpfseq	INDF0
	bra		test_0x16

; do the checksum thing	
	call	do_checksum		; from FSR0,  returns status zero flag set if OK
	btfss	STATUS, Z
	bra		cerror
; checksum OK

; 0x41  number of sats, latitude, longitude, 3 lines from bottom
; FF 0B	 41  09  C5 83 BD 1F 98 6D 56 03 05		 09		C5 83 BD 1F latitude	98 6D 56 03 longitude	05 number of sats	09 checksum
;  0  1   2   3   4  5  6  7  8  9 10 11 12      13 

; WAS TEST
;#define SAT_LAT_LON_TEST
#ifdef SAT_LAT_LON_TEST
	lfsr	FSR0, DATA_BUFFER_START+D'4'

	movlw	0xC5
	movwf	POSTINC0
	movlw	0x83
	movwf	POSTINC0
	movlw	0xBD
	movwf	POSTINC0
	movlw	0x1F
	movwf	POSTINC0

	movlw	0x98
	movwf	POSTINC0
	movlw	0x6D
	movwf	POSTINC0
	movlw	0x56
	movwf	POSTINC0
	movlw	0x03
	movwf	POSTINC0

	movlw	0x05
	movwf	INDF0
#endif ; SAT_LAT_LON_TEST
; END TEST

; point to satellites
	lfsr	FSR0, DATA_BUFFER_START+D'12'
; satellites
	movff	POSTDEC0, satellites

#ifdef FULL_GPS
; longitude, reverse Endian 0x03566D98 ->   55995800 ->   5.5995800
	movff	POSTDEC0, longitude_3
	movff	POSTDEC0, longitude_2
	movff	POSTDEC0, longitude_1
	movff	POSTDEC0, longitude_0

; latitude	reverse Endian 0x1FBD83C5 ->  532513733 ->  53.2513733
	movff	POSTDEC0, latitude_3
	movff	POSTDEC0, latitude_2
	movff	POSTDEC0, latitude_1
	movff	POSTDEC0, latitude_0
#endif ; FULL_GPS

#ifdef DEBUG_DECODE
; print latitude
	movlw	'R'
	call	tx_w
	movlw	'M'
	call	tx_w
	movlw	'G'
	call	tx_w
	movlw	'P'
	call	tx_w
	movlw	'S'
	call	tx_w
	movlw	' '
	call	tx_w

; print satellites
	movfw	satellites
	call	print_w_ascii_dec
	movlw	' '
	call	tx_w

#ifdef FULL_GPS
	movlw	'S'
	btfss	REGA3, 7
	movlw	'N'
	call	tx_w

	movff	latitude_3, REGA3
	movff	latitude_2, REGA2
	movff	latitude_1, REGA1
	movff	latitude_0, REGA0

	call	absa
	bsf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	call	print_32_ascii_dec_lz
	bcf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	movlw	' '
	call	tx_w

	
; print longitude
	movlw	'W'
	btfss	REGA3, 7
	movlw	'E'
	call	tx_w

	movff	longitude_3, REGA3
	movff	longitude_2, REGA2
	movff	longitude_1, REGA1
	movff	longitude_0, REGA0

	call	absa
	bsf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	call	print_32_ascii_dec_lz
	bcf		DIVIDE_BY_HUNDRED_MILLION_FLAG

#endif ; FULL_GPS
	call	tx_crlf
#endif ; DEBUG_DECODE


#ifdef TO_SAA
	movlw	REMOTE_POSITION_X
	movwf	xpos

	movlw	REMOTE_POSITION_Y
	movwf	ypos

	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

	lfsr	FSR0, TEXT_START	
	clrf	txt_len
; RMGPS
	movlw	'R'
	movwf	POSTINC0
	incf	txt_len

	movlw	'M'
	movwf	POSTINC0
	incf	txt_len

	movlw	'G'
	movwf	POSTINC0
	incf	txt_len

	movlw	'P'
	movwf	POSTINC0
	incf	txt_len

	movlw	'S'
	movwf	POSTINC0
	incf	txt_len

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movfw	satellites
	bcf		FIRST_ZERO_SUPPRESSED_FLAG
	call	tprint_w_ascii_dec_lz

#ifdef FULL_GPS
	movlw	 ' '
	movwf	POSTINC0
	incf	txt_len

	movff	latitude_3, REGA3
	movff	latitude_2, REGA2
	movff	latitude_1, REGA1
	movff	latitude_0, REGA0

	movlw	'S'
	btfss	REGA3, 7
	movlw	'N'
	movwf	temp3

	call	absa

; replace first zero that is always there with N or S

	bsf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	bcf		FIRST_ZERO_SUPPRESSED_FLAG
	call	tprint_32_ascii_dec_lz
	bcf		DIVIDE_BY_HUNDRED_MILLION_FLAG

	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0
	movfw	POSTDEC0

	movff	temp3, INDF0

	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0
	movfw	POSTINC0


	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movff	longitude_3, REGA3
	movff	longitude_2, REGA2
	movff	longitude_1, REGA1
	movff	longitude_0, REGA0

	movlw	'W'
	btfss	REGA3, 7
	movlw	'E'
	movwf	POSTINC0
	incf	txt_len

	call	absa

	bsf		DIVIDE_BY_HUNDRED_MILLION_FLAG
	bcf		FIRST_ZERO_SUPPRESSED_FLAG
	call	tprint_32_ascii_dec_lz
	bcf		DIVIDE_BY_HUNDRED_MILLION_FLAG
#endif ; FULL_GPS
	
	call	tputstr

#endif ; TO_SSA

	bra		int_end_rx


test_0x16:
	movlw	0x16
	cpfseq	INDF0
	bra		test_0x15
; 0x16 airspeed, distance, altitude right side screen
; FF 0C	 16 01 09      08  20 20 20 20 30 2E 30 25									1D		airspeed		ASCII	0. 0%
; FF 0C	 16 04 05      08  20 20 20 20 20 5F 30 6D									3D		distance		ASCII	_0m
; FF 0C	 16 00 03      08  20 20 20 5E 30 2E 30 6D									20		altitude		ASCII	^ 0.0 m
;  0  1   2  3  4       5   6  7  8  9 10 11 12 13                                  14

; do the checksum thing	
	call	do_checksum		; from FSR0,  returns status zero flag set if OK
	btfss	STATUS, Z
	bra		cerror
; checksum OK

; test if altitude
	lfsr	FSR0, DATA_BUFFER_START+D'3'
	movlw	0x00	; 0x00 is altitude
	cpfseq	INDF0
	bra		test_if_airspeed

#ifdef DEBUG_DECODE
; report altitude
; get length
	lfsr	FSR0, DATA_BUFFER_START+D'5'
	movff	POSTINC0, temp1
altitude_loop:
	movfw	POSTINC0
	call	tx_w
	decfsz	temp1
	bra		altitude_loop
	call	tx_crlf	
#endif ; DEBUG_DECODE

#ifdef TO_SAA
; at buffer + 6 upwards is a '^', but the teletext chip character generator uses an other symbol, so substitude that byte for 'A' for altitude
	lfsr FSR0,  DATA_BUFFER_START+D'6' 

	movlw	D'8'
	movwf	temp3
subs_loop:
	movlw	'^'
	cpfseq	INDF0
	bra		next_byte
; subsitude
	movlw	'A'
	movwf	INDF0		
	bra		subs_loop_done
next_byte:
	movfw	POSTINC0
	decfsz	temp3
	bra		subs_loop
subs_loop_done:

; altitude to SAA
	movlw	ALTITUDE_X
	movwf	xpos

	movlw	ALTITUDE_Y
	movwf	ypos

	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

; to_saa macro pointer
	to_saa	DATA_BUFFER_START+D'5' 	; point to length field just before text to be displayed

#endif ; TO_SAA
	bra		int_end_rx



test_if_airspeed:
	lfsr	FSR0, DATA_BUFFER_START+D'3'
	movlw	0x01	; 0x01 is airspeed
	cpfseq	INDF0
	bra		test_if_distance

#ifdef DEBUG_DECODE
; report airspeed
; get length
	lfsr	FSR0, DATA_BUFFER_START+D'5'
	movff	POSTINC0, temp1
; remove the trailing % sign, replace with m/s later
	decf	temp1
airspeed_loop:
	movfw	POSTINC0
	call	tx_w
	decfsz	temp1
	bra		airspeed_loop
	movlw	'm'
	call	tx_w
; add m/s
	movlw	'/'
	call	tx_w
	movlw	's'
	call	tx_w 
	call	tx_crlf	
#endif ; DEBUG_DECODE

#ifdef TO_SAA
; airspeed to SAA
; replace '%' sign with ' '
;	lfsr	FSR0, DATA_BUFFER_START+D'13'
;	movlw	' '
;	movwf	POSTINC0

; find the first no space character and replace the one before it with 'S' for speed
	lfsr	FSR0, DATA_BUFFER_START+D'6'
	movlw	D'8'
	movwf	temp3
s_loop:
	movlw	' '
	cpfseq	POSTINC0
	bra		not_a_space_s
	decfsz	temp3	
	bra		s_loop
; some error
	bra		int_end_rx
not_a_space_s:
; point to previous
	movfw	POSTDEC0
	movfw	POSTDEC0
	movlw	'S'	; S for speed
	movwf	INDF0

	movlw	AIRSPEED_X
	movwf	xpos

	movlw	AIRSPEED_Y
	movwf	ypos

	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

; to_saa  macro pointer
	to_saa	DATA_BUFFER_START+D'5' 	; point to length field just before text to be displayed
#endif ; TO_SAA

	bra		int_end_rx


test_if_distance:
	lfsr	FSR0, DATA_BUFFER_START+D'3'
	movlw	0x04	; 0x04 is distance
	cpfseq	INDF0
	bra		test_0x15

#ifdef DEBUG_DECODE
; report distance
; get length
	lfsr	FSR0, DATA_BUFFER_START+D'5'
	movff	POSTINC0, temp1
distance_loop:
	movfw	POSTINC0
	call	tx_w
	decfsz	temp1
	bra		distance_loop
	call	tx_crlf	
#endif ; DEBUG_DECODE


#ifdef TO_SAA

; distance to SAA

; at buffer + 6 upwards is a '_', substitude that byte for 'D' for altitude
	lfsr	FSR0, DATA_BUFFER_START+D'6'
	movlw	D'8'
	movwf	temp3
subs_loop_d:
	movlw	'_'
	cpfseq	INDF0
	bra		next_byte_d
; subsitude
	movlw	'D'
	movwf	INDF0		
	bra		subs_loop_d_done
next_byte_d:
	movfw	POSTINC0
	decfsz	temp3
	bra		subs_loop_d
subs_loop_d_done:

	movlw	DISTANCE_X
	movwf	xpos

	movlw	DISTANCE_Y
	movwf	ypos

	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

; to_saa  macro pointer
	to_saa	DATA_BUFFER_START+D'5' 	; point to length field just before text to be displayed
#endif ; TO_SAA

	bra		int_end_rx





test_0x15:
	movlw	0x15
	cpfseq	INDF0
	bra		int_end_rx		; unknown field

; 0x015, text fields, byte 4 set ypos, byte 5 set color, 0 = black
; FF 0D	 15 00	00 23  08  20 50 48 4F 54 4F 20 20									52		photo button	ASCII	P H O T O			top screen, temp, red

; FF 11	 15 00	00 23  0C  48 35 30 31 53 20 56 31 2E 32 2E 39						59		power up ID		ASCII	H501S V1.2.9		top screen, temp, red
; FF 11  15 00	00 23  0C  20 46 4F 4C 4C 4F 57 20 4F 4E 20 20						2A		follow on		ASCII	FOLLOW ON			top screen, temp, red
; FF 11  15 00	00 25  0C  20 46 4F 4C 4C 4F 57 20 4F 46 46 20						42		follow off		ASCII	FOLLOW OFF			top screen, temp, green
; FF 11  15 00	00 23  0C  20 4C 45 44 20 53 57 49 54 43 48 20	 					45		LED switch		ASCII	LED SWITCH			top screen, temp, geen for off, red for on

; FF 13  15 00	00 23  0E  20 48 45 41 44 4C 45 53 53 20 4F 4E 20 20				38		status			ASCII	HEADLESS ON			top screen, temp, red
; FF 13  15 00	00 25  0E  20 48 45 41 44 4C 45 53 53 20 4F 46 46 20				50		status			ASCII	HEADLESS OFF		top screen, temp, green

; FF 14	 15 00	01 09  0F  43 48 45 43 4B 20 47 59 52 4F 20 53 45 4E 53				5C		request			ASCII	CHECK GYRO SENS 	top screen
; FF 15	 15 00	01 09  10  43 61 6C 69 62 20 43 6F 6D 70 61 73 73 20 31 20			09		request			ASCII	Calib Compass 1		top screen
; FF 15  15 00  01 09  10  43 61 6C 69 62 20 43 6F 6D 70 61 73 73 20 32	20			0A		request			ASCII	Calib Compass 2 	top screen		
; FF 17  15 00  01 00  12  20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20	17		clear request	ASCII	18 spaces			top screen

; FF 13	 15 00	14 05  0E  20 4D 61 6E 75 61 6C 20 6D 6F 64 65 20 20 				33		status			ASCII	Manual mode			bottom screen, green, 1
; FF 13  15 00	14 05  0E  20 47 50 53 20 48 6F 6C 64 20 20 20 20 20				41		status			ASCII	GPS Hold			bottom screen, green, 2
; FF 13  15 00	14 05  OE  20 41 6C 74 69 74 75 64 65 20 48 6F 6C 64 				15		status			ASCII	Altitude Hold		bottom screen
; FF 13  15 00	14 05  0E  20 46 6F 6C 6C 6F 77 20 4D 6F 64 65 20 20				18		status			ASCII	Follow Mode			bottom screen
; FF 13  15 00	14 05  0E  20 52 65 74 75 72 6E 20 48 6F 6D 65 20 20 				0F		status			ASCII	Return Home			bottom screen, green, 3

;  0  1   2  3   4  5   6   7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24            21

; calibrate compass line 4

; all top    screens have byte 4	0x00 
; all bottom screens have byte 4	0x14
; clear field has byte 4 0x01


; do the checksum thing	
	call	do_checksum		; from FSR0,  returns status zero flag set if OK
	btfss	STATUS, Z
	bra		cerror
; checksum OK


#ifdef DEBUG_DECODE
; get position
;     byte 4 = ypos
;     byte 5 = xpos
	lfsr	FSR0, DATA_BUFFER_START+D'4'
	movff	POSTINC0, ypos
	movff	INDF0, xpos

	movfw	xpos
	call	print_w_ascii_dec
	movlw ','
	call	tx_w
	movfw	ypos
	call	print_w_ascii_dec
	movlw	':'
	call	tx_w

; get text length
	lfsr	FSR0, DATA_BUFFER_START+D'6'
	movff	POSTINC0, temp1
text_loop:
	movfw	POSTINC0
	call	tx_w
	decfsz	temp1
	bra		text_loop
	call	tx_crlf	
#endif ; DEBUG_DECODE


#ifdef TO_SAA
; text to SAA

; test byte 4 for which ypos
	lfsr	FSR0, DATA_BUFFER_START+D'4'
	movlw	0x00
	cpfseq	INDF0
	bra		test_0x01

; is 0x00
; FF 0D	 15 00	00 23  08  20 50 48 4F 54 4F 20 20									52		photo button	ASCII	P H O T O			top screen, temp, red
; FF 11	 15 00	00 23  0C  48 35 30 31 53 20 56 31 2E 32 2E 39						59		power up ID		ASCII	H501S V1.2.9		top screen, temp, red
; FF 11  15 00	00 23  0C  20 46 4F 4C 4C 4F 57 20 4F 4E 20 20						2A		follow on		ASCII	FOLLOW ON			top screen, temp, red
; FF 11  15 00	00 25  0C  20 46 4F 4C 4C 4F 57 20 4F 46 46 20						42		follow off		ASCII	FOLLOW OFF			top screen, temp, green
; FF 11  15 00	00 23  0C  20 4C 45 44 20 53 57 49 54 43 48 20	 					45		LED switch		ASCII	LED SWITCH			top screen, temp, geen for off, red for on
; FF 13  15 00	00 23  0E  20 48 45 41 44 4C 45 53 53 20 4F 4E 20 20				38		status			ASCII	HEADLESS ON			top screen, temp, red
; FF 13  15 00	00 25  0E  20 48 45 41 44 4C 45 53 53 20 4F 46 46 20				50		status			ASCII	HEADLESS OFF		top screen, temp, green
	movlw	BUTTON_X
	movwf	xpos

	movlw	BUTTON_Y
	movwf	ypos
; clear line	
	movlw	D'20'
	call	clear_line		; xpos, ypos, length in W
	bra		tsend_text


test_0x01:
	movlw	0x01
	cpfseq	INDF0
	bra		test_0x14

; is 0x01
; FF 14	 15 00	01 09  0F  43 48 45 43 4B 20 47 59 52 4F 20 53 45 4E 53				5C		request			ASCII	CHECK GYRO SENS 	top screen
; FF 15	 15 00	01 09  10  43 61 6C 69 62 20 43 6F 6D 70 61 73 73 20 31 20			09		request			ASCII	Calib Compass 1		top screen
; FF 15  15 00  01 09  10  43 61 6C 69 62 20 43 6F 6D 70 61 73 73 20 32	20			0A		request			ASCII	Calib Compass 2 	top screen		
; FF 17  15 00  01 00  12  20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20	17		clear request	ASCII	18 spaces			top screen
	movlw	REQUEST_X
	movwf	xpos

	movlw	REQUEST_Y
	movwf	ypos
	bra		tsend_text


test_0x14:
	movlw	0x14
	cpfseq	INDF0
; unknown code
	bra		int_end_rx

; is 0x14
; FF 13	 15 00	14 05  0E  20 4D 61 6E 75 61 6C 20 6D 6F 64 65 20 20 				33		status			ASCII	Manual mode			bottom screen, green, 1
; FF 13  15 00	14 05  0E  20 47 50 53 20 48 6F 6C 64 20 20 20 20 20				41		status			ASCII	GPS Hold			bottom screen, green, 2
; FF 13  15 00	14 05  OE  20 41 6C 74 69 74 75 64 65 20 48 6F 6C 64 				15		status			ASCII	Altitude Hold		bottom screen
; FF 13  15 00	14 05  0E  20 46 6F 6C 6C 6F 77 20 4D 6F 64 65 20 20				18		status			ASCII	Follow Mode			bottom screen
; FF 13  15 00	14 05  0E  20 52 65 74 75 72 6E 20 48 6F 6D 65 20 20 				0F		status			ASCII	Return Home			bottom screen, green, 3
	movlw	STATUS_X
	movwf	xpos

	movlw	STATUS_Y
	movwf	ypos
;	bra		tsend_text

tsend_text:
; to_saa  macro pointer
	to_saa	DATA_BUFFER_START+D'6' 	; point to length field just before text to be displayed
#endif ; TO_SAA

	bra		int_end_rx



;#define TO_OLD
#ifdef TO_SAA_OLD
; to screen

 ; WAS TEST
	clrf	xpos
	clrf	ypos

	clrf	acqmem
	call	tputcu		; cursor  to dspmem, acqmem, xpos, ypos

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	'A'
	movwf	POSTINC0
	incf	txt_len

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	clrf	REGA3
	clrf	REGA2
;	movff	current_altitude_decimeters_h, REGA1
;	movff	current_altitude_decimeters_l, REGA0

	bcf		DIVIDE_BY_TEN_THOUSAND_FLAG
	bcf		DIVIDE_BY_THOUSAND_FLAG
	bcf		DIVIDE_BY_HUNDRED_FLAG
	bsf		DIVIDE_BY_TEN_FLAG
	bsf		FIRST_ZERO_SUPPRESSED_FLAG
	call	tprint_32_ascii_dec					; prints 32 bit value in registers REGA3:REGA0, in ASCII decimal to teletext display at cursor, increments txt_len
												; uses DIVIDE_BY_TEN_THOUSAND_FLAG, DIVIDE_BY_THOUSAND_FLAG, DIVIDE_BY_HUNDRED_FLAG, DIVIDE_BY_TEN_FLAG, FIRST_ZERO_SUPPRESSED_FLAG
	bcf		DIVIDE_BY_TEN_FLAG	
	bcf		FIRST_ZERO_SUPPRESSED_FLAG

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len	

	call	tputstr			; text string at TEXT_START to dspmem at cursor position, length txt_len

#endif ; TO_SAA_OLD


cerror:
#ifdef REPORT_CHECKSUM_ERRORS
	lfsr	FSR0, DATA_BUFFER_START+D'1'
	movfw	INDF0
	call	print_w_hex
	movlw	' '
	call	tx_w
	movlw	'C'
	call	tx_w
	movlw	'E'
	call	tx_w
	call	tx_crlf
#endif ; REPORT_CHECKSUM_ERRORS

	bra		int_end_rx



; end of interrupt routines
command_end:
; reset the value, so we can continue for other numerical commands.
;	clrf	value3
;	clrf	value2
;	clrf	value1
;	clrf	value0
;	clrf	digit_cnt
	bra		int_end_rx

int_end_rx:
	bcf		PIR1,	RCIF
	bsf		PIE1,	RCIE 
	bra		int_done

not_serial_port_interrupt:

; clear any other interrupt flags

;	OLD IF COMPARATOR movf	CMCON,	W			; read CMCON to end mismatch because of comparator output change
	bcf INTCON,	INT0IE
	bcf	INTCON,	INT0IF			; GP2/INT
	bcf	INTCON,	RABIF			; port change interrupt flag bit

; PIR1   --   ADIF RCIF TXIF  SSPIF CCP1IF TMR2IF TMR1IF
	bcf	PIR1,	ADIF
	bcf	PIR1,	RCIF
	bcf	PIR1,	TXIF
	bcf	PIR1,	SSPIF
	bcf	PIR1,	CCP1IF
	bcf	PIR1,	TMR2IF
;	bcf	PIR1,	TMR1IF

; PIR2  OSFIF C2IF C1IF EEIF   --     --     --     --
	bcf	PIR2,	OSCFIF
	bcf	PIR2,	C2IF
	bcf	PIR2,	C1IF
	bcf	PIR2,	EEIF

int_done:
	restore_w_stat				; get back W and status
	retfie




print_id:

	movlw	UPPER	id_text
	movwf	temp_u

	movlw	HIGH	id_text
	movwf	temp_h

	movlw	LOW		id_text
	movwf	temp_l

	bra		text_pri



text_pri:
;	bsf		EECON1,	EEPGD			; EEPGD: Flash Program or Data EEPR, 1 = Access Flash program memory, 0 = Access data EEPROM memory
;	bcf		EECON1, CFGS			; CFGS: Flash Program/Data EEPROM or Configuration Select bit, 1 = Access Configuration registers, 0 = Access Flash program or data EEPROM memory
;									; FREE: Flash Row (Block) Erase Enable bit, 1 = Erase the program memory block addressed by TBLPTR on the next WR command (cleared by completion of erase operation), 0 = Perform write-only
;									; WREN: Flash Program/Data EEPROM Write Enable bit, 1 = Allows write cycles to Flash program/data EEPROM, 0 = Inhibits write cycles to Flash program/data EEPROM

	movf	temp_u, W				; Load TBLPTR with the base
	movwf	TBLPTRU					; address of the word

	movf	temp_h, W
	movwf	TBLPTRH

	movf	temp_l, W
	movwf   TBLPTRL

; print until zero byte found.
text_pri_loop:
	TBLRD*+							; read into TABLAT and increment
	movf    TABLAT, W				; get data

; test for end of string
	xorlw	0
	andlw	D'127'
	btfsc	STATUS,	Z
	return

	call	tx_w

	bra		text_pri_loop





id_text:
; Set version here
	DA "Panteltje H501S_HUD-0.1 for 18F14K22.\r\n\0"



select_baudrate:
	movwf	temp3
	call	disable_serial_port

; compare 
	movlw	D'1'
	subwf	temp3, W
	btfsc	STATUS, Z
	bra		set_115200bd

	movlw	D'2'
	subwf	temp3, W
	btfsc	STATUS, Z
	bra		set_19200bd

	movlw	D'3'
	subwf	temp3, W
	btfsc	STATUS, Z
	bra		set_9600bd

	movlw	D'4'
	subwf	temp3, W
	btfsc	STATUS, Z
	bra		set_4800bd

	movlw	D'5'
	subwf	temp3, W
	btfsc	STATUS, Z
	bra		set_1200bd

	movlw	D'6'
	subwf	temp3, W
	btfsc	STATUS, Z
	bra		set_57600bd

; if illegal value use default
;	bra		select_baudrate_end
	bra		set_115200bd
	

set_115200bd:
	movlw	D'1'
	movwf	BRG16

	movlw	HIGH	D'137'
	movwf	SPBRGH

	movlw	LOW		D'137'
	movwf	SPBRG

	bra		select_baudrate_end

set_19200bd:
;	bcf     TXSTA, SYNC
	movlw	D'1'
	movwf	BRG16

	movlw	HIGH	D'832'
	movwf	SPBRGH
	
	movlw	LOW		D'832'
	movwf	SPBRG

	bra		select_baudrate_end

set_9600bd:
	movlw	HIGH	D'1658'
	movwf	SPBRGH

	movlw	LOW		D'1658'
	movwf	SPBRG

	bra		select_baudrate_end

set_4800bd:
	movlw	HIGH	D'3316'
	movwf	SPBRGH

	movlw	LOW		D'3316'
	movwf	SPBRG

	bra		select_baudrate_end

set_1200bd: ; SPBRGH:SPBRG=9999 BRG16=1 for 48 MHz  -> 9999 * 64/48 = 13332
	movlw	D'1'
	movwf	BRG16

	movlw	HIGH	D'13332'
	movwf	SPBRGH

	movlw	LOW		D'13332'
	movwf	SPBRG
	bra		select_baudrate_end

set_57600bd:
	movlw	D'1'
	movwf	BRG16

; see 18F1XK22_datasheet_41365C.pdf page 190
; (64 / 48) * 207 = 276
	movlw	HIGH	D'276'
	movwf	SPBRGH
	
	movlw	LOW		D'276'
	movwf	SPBRG

;	movlw	HIGH	D'274'
;	movwf	SPBRGH
	
;	movlw	LOW		D'274'
;	movwf	SPBRG

select_baudrate_end:
	call	enable_serial_port

	return


; EEPROM
; WARNING this will reset all EEPROM vars to zero!!!!!!
;	org	0xf00000
; 9=minutes, 10=hours, 11=days
;	DE	0,0,0,0,0,0,0,0,0,8,1,0	



print_32_ascii_dec_signed  ; value in REG3:REGA0
; test if negative

	btfss	REGA1, 7
	bra		print_16_ascii_dec_signed_positive
; negative
	comf	REGA3	
	comf	REGA2
	comf	REGA1	
	comf	REGA0	

	incf	REGA0

	movlw	'-'
	call	tx_w
	bra		print_16_ascii_dec_signed_result

print_16_ascii_dec_signed_positive:
	movlw	'+'
	call	tx_w

print_16_ascii_dec_signed_result:
	call	print_32_ascii_dec

	return



print_32_ascii_dec:					; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal via RS232.
	bsf		FIRST_ZERO_SUPPRESSED_FLAG

print_32_ascii_dec_lz:

; Max 4 G
; divide by 1G = 1000 000 000 = 0x 3B 9A CA 00 
;	clrf	temp
	movlw	0x3B
	movwf	REGC3

	movlw	0x9A
	movwf	REGC2

	movlw	0xCA
	movwf	REGC1

	movlw	0x00
	movwf	REGC0

	call	l32_print_nibble

; divide by 100M = 100 000 000 = 0x 05 F5 E1 00 
	movlw	0x05
	movwf	REGC3

	movlw	0xF5
	movwf	REGC2

	movlw	0xE1
	movwf	REGC1

	movlw	0x00
	movwf	REGC0

	call	l32_print_nibble

; divide by 10M = 10 000 000 = 0x 98 96 80 
	movlw	0x00
	movwf	REGC3

	movlw	0x98
	movwf	REGC2

	movlw	0x96
	movwf	REGC1

	movlw	0x80
	movwf	REGC0

	call	l32_print_nibble
; test if divide by one million dot print needed
	btfss	DIVIDE_BY_HUNDRED_MILLION_FLAG
	bra		print_div_1000000
; print dot
	movlw	'.'
	call	tx_w

print_div_1000000:
; divide by 1M = 1 000 000 = 0x 00 0F 42 40
	movlw	0x00
	movwf	REGC3

	movlw	0x0F
	movwf	REGC2

	movlw	0x42
	movwf	REGC1

	movlw	0x40
	movwf	REGC0

	call	l32_print_nibble

; test if divide by one million dot print needed
	btfss	DIVIDE_BY_ONE_MILLION_FLAG
	bra		print_div_100000
; print dot
	movlw	'.'
	call	tx_w

print_div_100000:
; divide by 100k = 100 000 = 0x 00 01 86 a0
	movlw	0x00
	movwf	REGC3

	movlw	0x01
	movwf	REGC2

	movlw	0x86
	movwf	REGC1

	movlw	0xA0
	movwf	REGC0

	call	l32_print_nibble

; test if divide by hundred thousand dot print needed
	btfss	DIVIDE_BY_HUNDRED_THOUSAND_FLAG
	bra		print_div_10000
; print dot
	movlw	'.'
	call	tx_w

print_div_10000:
; divide by 10k = 10 000
	clrf	REGC3

	movlw	UPPER	D'10000'
	movwf	REGC2

	movlw	HIGH	D'10000'
	movwf	REGC1

	movlw	LOW		D'10000'
	movwf	REGC0

	call	l32_print_nibble

; test if divide by ten thousand dot print needed
	btfss	DIVIDE_BY_TEN_THOUSAND_FLAG
	bra		print_div_1000
; print dot
	movlw	'.'
	call	tx_w

print_div_1000:
; divide by 1k = 1000
	clrf	REGC3

	movlw	UPPER		D'1000'
	movwf	REGC2

	movlw	HIGH		D'1000'
	movwf	REGC1

	movlw	LOW			D'1000'
	movwf	REGC0

	call	l32_print_nibble

; test if divide by thousand dot print needed
	btfss	DIVIDE_BY_THOUSAND_FLAG
	bra		print_div_100
; print dot
	movlw	'.'
	call	tx_w
print_div_100:
; divide by 100
	clrf	REGC3
	clrf	REGC2
	clrf	REGC1

	movlw	D'100'
	movwf	REGC0

	call	l32_print_nibble
; test if divide by hundred dot print needed
	btfss	DIVIDE_BY_HUNDRED_FLAG
	bra		print_div_10
; print dot
	movlw	'.'
	call	tx_w
print_div_10:
; divide by 10
	clrf	REGC3
	clrf	REGC2
	clrf	REGC1

	movlw	D'10'
	movwf	REGC0

	call	l32_print_nibble

; test if divide by ten dot print needed
	btfss	DIVIDE_BY_TEN_FLAG
	bra		print_units_32
; print dot
	movlw	'.'
	call	tx_w

print_units_32:
; print units
	movfw	REGA0
; always print last digit, even if al zeros
	call	l32_print_lpd1

	return


l32_print_nibble:
	clrf	temp4
l32_loop_lpd:
	movfw	REGC3
	movwf	REGB3

	movfw	REGC2
	movwf	REGB2

	movfw	REGC1
	movwf	REGB1

	movfw	REGC0
	movwf	REGB0

; save value before subtract
	movfw	REGA3
	movwf	AARGB7

	movfw	REGA2
	movwf	AARGB6

	movfw	REGA1
	movwf	AARGB5

	movfw	REGA0
	movwf	AARGB4

	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag

; test sign bit, skip if not negative
	btfsc	REGA3, D'7'

	bra		l32_print_lpd

	incf	temp4
	bra		l32_loop_lpd

l32_print_lpd:
	movfw	temp4
; test if zero surpresson still active
	btfss	FIRST_ZERO_SUPPRESSED_FLAG
	bra		l32_print_lpd1
; test if zero
	xorlw	D'0'
	btfsc	STATUS, Z
	bra		l32_print_none
	movfw	temp4

l32_print_lpd1:
	call	tx_digit_in_w

; clear the zero suppression flag
	bcf		FIRST_ZERO_SUPPRESSED_FLAG

l32_print_none:
	movfw	AARGB7
	movwf	REGA3

	movfw	AARGB6
	movwf	REGA2

	movfw	AARGB5
	movwf	REGA1

	movfw	AARGB4
	movwf	REGA0

	return






; start software i2c routines

sdah:
; sda to tristate	
	bsf		SDA_PULLDOWN
	call	iicdly
	return


sdal:
	bcf		SDA_PULLDOWN
	call	iicdly
	return


sclh:
	bsf		SCL_OUT
	call	iicdly
	return


scll:
	bcf		SCL_OUT
	call	iicdly
	return


sdain:							; returns data in W, 0 or 1
	clrf	WREG
	btfsc	SDA_IN
	incf	WREG	
	return


iicini:
	call	sdah
	call	sclh
	return


iicstart:
; A high-to-low transition of the SDA line while the clock (SCL) is high, determines a Start condition. All commands must be preceded by a Start condition.
	call	sclh
	call	sdal
	call	scll
	return


iicstop:
; A low-to-high transition of the SDA line, while the clock (SCL) is high, determines a Stop condition. All operations must end with a Stop condition.
	call	sdal
	call	sclh
	call	sdah
	return


iicdly:								; iic bus delay pullup RC time!
;	movlw	D'25'
	movlw	D'12'
	movwf	iic_delay_cnt
iicdly_loop:
	decfsz	iic_delay_cnt
	bra		iicdly_loop

	clrf	WREG	
	incf	WREG
	return



sndbyte:							; transmits byte in W, scl must be low at start, scl is low at end, returns W=0 for error, W=1 for ack received 
	movwf	iic_temp
	movlw	D'8'				; 8 bits
	movwf	iic_loop_cnt
sndbyte_loop:
	btfss	iic_temp, 7			; high bit first
	bra		sndbyte_l
; sndbyte_h
	call	sdah
	bra		sndbyte_clock
sndbyte_l:
	call	sdal

sndbyte_clock:
	call	sclh
	call	scll
	rlncf	iic_temp			; look one bit lower
	decfsz	iic_loop_cnt
	bra		sndbyte_loop		; get next bit

; read ack bit
	call	sdah

	call	sclh		

	call	sdain				; no ack slave is W=1 = error
	movwf	iic_temp			; data in iic_temp bit 0

	call	scll
		
	btfss	iic_temp, 0			; get data
	bra		sndbyte_ok

sndbyte_err:
; in case called with verify waiting for ack we do no want any error messages
	btfsc	NO_SND_BYTE_ERROR_MSG_FLAG
	bra		sndbyte_return_err

; I2C ack error
; alert main board
	movlw	SIGNAL_BELL
	call	tx_w

	movlw	'I'
	call	tx_w

	movlw	'I'
	call	tx_w

	movlw	'C'
	call	tx_w

	movlw	' '
	call	tx_w

	movlw	's'
	call	tx_w

	movlw	'n'
	call	tx_w

	movlw	'd'
	call	tx_w

	movlw	'b'
	call	tx_w

	movlw	'y'
	call	tx_w

	movlw	't'
	call	tx_w

	movlw	'e'
	call	tx_w

	movlw	' '
	call	tx_w

	movlw	'f'
	call	tx_w

	movlw	'a'
	call	tx_w

	movlw	'i'
	call	tx_w

	movlw	'l'
	call	tx_w

	movlw	'e'
	call	tx_w

	movlw	'd'
	call	tx_w

	call 	tx_crlf

sndbyte_return_err:
; return error
	clrf	WREG
	return	

sndbyte_ok:
	call	scll

; return OK
	bsf		WREG, 1
	return




rcvbyte:						; returns received byte in W, LAST_BYTE_FLAG must be clear, and set on the last byte
	call	sdah			; for read
	call	scll

	clrf	iic_temp		; data goes here

	movlw	D'8'			;set for 8 bits
	movwf	iic_loop_cnt

;	clrf	iic_temp
rcvbyte_loop:
	rlncf	iic_temp		; shift bit to the left, this first so we do not shift the end result 

	call	sclh

	call	sdain			; get bit
	btfss	WREG, 0
	bra		rcvbyte_have_0
rcvbyte_have_1:
	bsf		iic_temp, 0
	bra		rcv_byte_have_bit

rcvbyte_have_0:
	bcf		iic_temp, 0

rcv_byte_have_bit:
	call 	scll

; test if more bits
	decfsz	iic_loop_cnt	
	bra		rcvbyte_loop	; loop if more bits

; send ack bit if not last byte
; if last byte set sdah, else set sdal
	btfsc	LAST_BYTE_FLAG
	bra		rcvbyte_do_h

; not last byte
	call	sdal
	bra		rcvbyte_done	

rcvbyte_do_h:
; last byte
	call	sdah

rcvbyte_done:
	call	sclh
	call	scll

	call	sdah

; return with data in W
	movfw	iic_temp
	return




iicsnd: ; iic_device_address, iic_byte_cnt, address FSR0 in RAM
	call	iicstart

	movfw	iic_device_address
	call	sndbyte
	btfss	WREG, 0
	bra		iicsnd_error_return

; loop counter in iic_byte_cnt, destroyed
iicsnd_loop:
	movfw	POSTINC0
	call	sndbyte
	btfss	WREG, 0
	bra		iicsnd_error_return

	decfsz	iic_byte_cnt
	bra		iicsnd_loop

	call	iicstop
	movlw	D'1'
	return ; OK

iicsnd_error_return:
	call	iicstop
	clrf	WREG
	return ; error



iicrcv:	; iic_device_address, iic_byte_cnt, address FSR0 in RAM
	call	iicstart

	movfw	iic_device_address
	call	sndbyte
	btfss	WREG, 0
	bra		iicrcv_error_return

	call	sdah ; for read

; if only one byte bra	 read only last byte
	movlw	D'1'
	subwf	iic_byte_cnt, W
	btfsc	STATUS, Z
	bra		iicrcv_read_last_byte

; loop counter in iic_byte_cnt, destroyed

; read all but last byte
	decf	iic_byte_cnt
	bcf		LAST_BYTE_FLAG
iicrcv_loop:
	call	rcvbyte	; data in W

	movwf	POSTINC0

	decfsz	iic_byte_cnt
	bra		iicrcv_loop	

iicrcv_read_last_byte:
; read last byte
	bsf		LAST_BYTE_FLAG

	call	rcvbyte	; data in W

	movwf	INDF0

	call	iicstop
	movlw	D'1'	
	return ; OK

iicrcv_error_return:
	call	iicstop
	clrf	WREG
	return ; error



saa5281_ini:
	movlw	SAA5281_ADDRESS
	movwf	iic_device_address

; R0	
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'0'				; R0
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[0] = 0;					; advanced control
;//cctreg[0] |= 128;			; X24 POS (auto display FASTTEXT prompt)
;//cctreg[0] |= 64;				; free run PLL
;//cctreg[0] |= 32;				; auto !ODD/EVEN if 1 forces ODD/EVEN low if  pic, if disbale odd/even = 0
;//cctreg[0] |= 16;				; disable HDR roll (green rolling header)*/
;//cctreg[0] |= 8;				; _
;//cctreg[0] |= 4;				; disable !ODD/EVEN (forces ODD/EVEN output low)
;cctreg[0] |= 2;				; VCR mode (short time constant PLL)
;//cctreg[0] |= 1;				; !R11/R11b select
	movlw	D'2'					; FAST mode
	movwf	POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM

; R1
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'1'				; R1
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[1] = 0;					; mode
;//cctreg[1] |= 128;			; VCS to SCS (enables display messages with 60 Hz input signal)
;//cctreg[1] |= 64;				; !7+p/ 8bit (7 bits with parity check or 8 bits)
;cctreg[1] |= 32;				; ACQ !ON/OFF
;//cctreg[1] |= 16;				; EXT packet enable (allocates 2KB memory per chapter)
;//cctreg[1] |= 8;				; !DEW/FULL FIELD ( field flyback of full channel mode)
;//cctreg[1] |= 4;				; TCS ON FFB MODE (text composite sync or direct sync select)
;//cctreg[1] |= 2;				; T1 interlace/non interlace 312/313 line ctr;*/
;//cctreg[1] |= 1;				; T0
	movlw	D'32'					; ACQ OFF
	movwf	POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM


; R2
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'2'				; R2	
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[2] = 0;					; page request address
;//cctreg[2] |= 128;			; HAM CHECK
;//cctreg[2] |= 64;				; BANK SELECT A2
;//cctreg[2] |= 32;				; ACQ CIRCUIT A1
;//cctreg[2] |= 16;				; ACQ CUIRCUIT A0
;//cctreg[2] |= 8;				; TB
;//cctreg[2] |= 4;				; START COLUMN SC2
;//cctreg[2] |= 2;				; START COLUMN SC1
;//cctreg[2] |= 1;				; START COLUMN SC0
	movlw	D'0'					; no page request
	movwf	POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM


; R3
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'3'				; R3
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[3] = 0;					; page request data (lowest 5 bits)
	movlw	D'0'					; no page request data
	movwf	POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM


; R4
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'4'				; R4
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[4] = 0;					; display chapter (lowest 3 bits)
	movlw	D'0'					; display 0
	movwf	POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM


; R5
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'5'				; R5
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[5] = 0;					; normal display control 
;//cctreg[5] |= 128;			; BKGND background out
;//cctreg[5] |= 64;				; BKGND background in
;//cctreg[5] |= 32;				; COR out (contrast reduction)
;//cctreg[5] |= 16;				; COR in
;cctreg[5] |= 8;				; TEXT out
;cctreg[5] |= 4;				; TEXT in box
;cctreg[5] |= 2;				; PON out (picture on) 
;//cctreg[5] |= 1;				; PON in
	movlw	D'14'					; text out box, text in box, picture on
	movwf	POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM


; R6
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'6'				; R6
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[6] = 0;					; newsflash display control
;//cctreg[6] |= 128;			; BKGND out
;//cctreg[6] |= 64;				; BKGND in
;//cctreg[6] |= 32;				; COR out
;//cctreg[6] |= 16;				; COR in
;cctreg[6] |= 8;				; TEXT out
;cctreg[6] |= 4;				; TEXT in
;cctreg[6] |= 2;				; PON out
;//cctreg[6] |= 1;				; PON in (picture on)
	movlw	D'14'					; text out box, text in box, picture on
	movwf	POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM


; R7
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'7'				; R7
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[7] = 0;					; display mode, does not auto increment	
;//cctreg[7] |= 128;			; row 25 displayed above or below main text
;//cctreg[7] |= 64;				; cursor on
;//cctreg[7] |= 32;				; reveal
;//cctreg[7] |= 16;				; bottom half if double height = 1
;//cctreg[7] |= 8;				; double height
;cctreg[7] |= 4;				; box 24
;cctreg[7] |= 2;				; box 1-23
;cctreg[7] |= 1;				; box 0
	movlw	D'135'					; row 25 bottom, box 0-23
	movwf	POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM

	return			; end saa5281_ini 



tputcu:								; cursor to dspmem, xpos, ypos, acqmem
	movlw	SAA5281_ADDRESS
	movwf	iic_device_address

	lfsr	FSR0, COMMAND_START

	movlw	D'8'				; R8
	movwf	POSTINC0

;	movff	dspmem, POSTINC0
	movff	acqmem, POSTINC0
	
	movff	ypos, POSTINC0

	movff	xpos, POSTINC0
	
	movlw	D'4'
	movwf	iic_byte_cnt

	lfsr	FSR0, COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM

	return




tcls:					; clear display memory in W, cursor y to x=0, y=0
;; cursor to zero
;	movlw	D'0'
;	movwf	xpos
;	movwf	ypos
;	movwf	dspmem

	call	tputcu	

	movlw	SAA5281_ADDRESS
	movwf	iic_device_address

	lfsr	FSR0, COMMAND_START

	movlw	D'8'				; R8	
	movwf	POSTINC0

;	movff	dspmem, WREG			; bit 2:0 dspmem
	movff	acqmem, WREG			; bit 2:0 dspmem
	bsf		WREG, 3					; bit 3 clear memory
	movwf	POSTINC0			; 8 + m	

	movlw	D'2'
	movwf	iic_byte_cnt

	lfsr	FSR0, COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM

	return



tputstr:	; text string at TEXT_START to dspmem at cursor position, length txt_len
	movlw	SAA5281_ADDRESS
	movwf	iic_device_address

	lfsr	FSR0, TEXT_COMMAND_START	

; select R11
	movlw	D'11'				; R11
	movwf	POSTINC0

; send text string, now pointing to TEXT_START
	movff	txt_len, iic_byte_cnt	; length of text data
	incf	iic_byte_cnt			; plus R11 select byte

	lfsr	FSR0, TEXT_COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM

	return



seldisp:						; select display memory dspmem
	movlw	D'2'
	movwf	iic_byte_cnt
	
	lfsr 	FSR0,	COMMAND_START
	
	movlw	D'4'				; R4
	movwf	POSTINC0 			; INDF0 with post increment

;cctreg[4] = 0;					; display chapter (lowest 3 bits)
	movff	dspmem, POSTINC0

	lfsr 	FSR0,	COMMAND_START
	call	iicsnd 				; iic_device_address, iic_byte_cnt, data at address FSR0 in RAM

	return


tprint_32_ascii_dec:	; prints 32 bit value in registers REGA3:REGA0, increments txt_len,  in ASCII decimal to teletext display at cursor
						; uses DIVIDE_BY_TEN_THOUSAND_FLAG, DIVIDE_BY_THOUSAND_FLAG, DIVIDE_BY_HUNDRED_FLAG, DIVIDE_BY_TEN_FLAG, FIRST_ZERO_SUPPRESSED_FLAG
	bsf		FIRST_ZERO_SUPPRESSED_FLAG

tprint_32_ascii_dec_lz:

; Max 4 G
; divide by 1G = 1000 000 000 = 0x 3B 9A CA 00 
;	clrf	temp
	movlw	0x3B
	movwf	REGC3

	movlw	0x9A
	movwf	REGC2

	movlw	0xCA
	movwf	REGC1

	movlw	0x00
	movwf	REGC0

	call	tl32_print_nibble

; divide by 100M = 100 000 000 = 0x 05 F5 E1 00 
	movlw	0x05
	movwf	REGC3

	movlw	0xF5
	movwf	REGC2

	movlw	0xE1
	movwf	REGC1

	movlw	0x00
	movwf	REGC0

	call	tl32_print_nibble

; divide by 10M = 10 000 000 = 0x 98 96 80 
	movlw	0x00
	movwf	REGC3

	movlw	0x98
	movwf	REGC2

	movlw	0x96
	movwf	REGC1

	movlw	0x80
	movwf	REGC0

	call	tl32_print_nibble
	
; test if divide by one million dot print needed
	btfss	DIVIDE_BY_HUNDRED_MILLION_FLAG
	bra		tprint_div_1000000
; print dot
	movlw	'.'
	movwf	POSTINC0
	incf	txt_len

tprint_div_1000000:
; divide by 1M = 1 000 000 = 0x 00 0F 42 40
	movlw	0x00
	movwf	REGC3

	movlw	0x0F
	movwf	REGC2

	movlw	0x42
	movwf	REGC1

	movlw	0x40
	movwf	REGC0

	call	tl32_print_nibble

; test if divide by one million dot print needed
	btfss	DIVIDE_BY_ONE_MILLION_FLAG
	bra		tprint_div_100000
; print dot
	movlw	'.'
	movwf	POSTINC0
	incf	txt_len

tprint_div_100000:
; divide by 100k = 100 000 = 0x 00 01 86 a0
	movlw	0x00
	movwf	REGC3

	movlw	0x01
	movwf	REGC2

	movlw	0x86
	movwf	REGC1

	movlw	0xA0
	movwf	REGC0

	call	tl32_print_nibble

; test if divide by hundred thousand dot print needed
	btfss	DIVIDE_BY_HUNDRED_THOUSAND_FLAG
	bra		tprint_div_10000
; print dot
	movlw	'.'
	movwf	POSTINC0
	incf	txt_len

tprint_div_10000:
; divide by 10k = 10 000
	clrf	REGC3

	movlw	UPPER	D'10000'
	movwf	REGC2

	movlw	HIGH	D'10000'
	movwf	REGC1

	movlw	LOW		D'10000'
	movwf	REGC0

	call	tl32_print_nibble

; test if divide by ten thousand dot print needed
	btfss	DIVIDE_BY_TEN_THOUSAND_FLAG
	bra		tprint_div_1000
; print dot
	movlw '.'
	movwf	POSTINC0
	incf	txt_len

tprint_div_1000:
; divide by 1k = 1000
	clrf	REGC3

	movlw	UPPER		D'1000'
	movwf	REGC2

	movlw	HIGH		D'1000'
	movwf	REGC1

	movlw	LOW			D'1000'
	movwf	REGC0

	call	tl32_print_nibble

; test if divide by thousand dot print needed
	btfss	DIVIDE_BY_THOUSAND_FLAG
	bra		tprint_div_100
; print dot
	movlw '.'
	movwf	POSTINC0
	incf	txt_len

tprint_div_100:
; divide by 100
	clrf	REGC3
	clrf	REGC2
	clrf	REGC1

	movlw	D'100'
	movwf	REGC0

	call	tl32_print_nibble

; test if divide by hundred dot print needed
	btfss	DIVIDE_BY_HUNDRED_FLAG
	bra		tprint_div_10
; print dot
	movlw '.'
	movwf	POSTINC0
	incf	txt_len

tprint_div_10:
; divide by 10
	clrf	REGC3
	clrf	REGC2
	clrf	REGC1

	movlw	D'10'
	movwf	REGC0

	call	tl32_print_nibble

; test if divide by ten dot print needed
	btfss	DIVIDE_BY_TEN_FLAG
	bra		tprint_units
; print dot
	movlw '.'
	movwf	POSTINC0
	incf	txt_len
tprint_units:

; print units
	movfw	REGA0
; always print last digit, even if al zeros
	call	tl32_print_lpd1

	return

tl32_print_nibble:
	clrf	temp4
tl32_loop_lpd:
	movfw	REGC3
	movwf	REGB3

	movfw	REGC2
	movwf	REGB2

	movfw	REGC1
	movwf	REGB1

	movfw	REGC0
	movwf	REGB0

; save value before subtract
	movfw	REGA3
	movwf	AARGB7

	movfw	REGA2
	movwf	AARGB6

	movfw	REGA1
	movwf	AARGB5

	movfw	REGA0
	movwf	AARGB4

	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag

; test sign bit, skip if not negative
	btfsc	REGA3, D'7'

	bra		tl32_print_lpd

	incf	temp4
	bra		tl32_loop_lpd

tl32_print_lpd:
	movfw	temp4
; test if zero surpression still active
	btfss	FIRST_ZERO_SUPPRESSED_FLAG
	bra		tl32_print_lpd1
; test if zero
	xorlw	D'0'
	btfsc	STATUS, Z
	bra		tl32_print_none
	movfw	temp4

tl32_print_lpd1:
; byte in W to teletext display memory at cursor
ttx_digit_in_w:
	addlw	'0'				; zero

; store text at TEXT_START and up
	movwf	POSTINC0
	incf	txt_len

; clear the zero suppression flag
	bcf		FIRST_ZERO_SUPPRESSED_FLAG

tl32_print_none:
	movfw	AARGB7
	movwf	REGA3

	movfw	AARGB6
	movwf	REGA2

	movfw	AARGB5
	movwf	REGA1

	movfw	AARGB4
	movwf	REGA0

; to teletext display

	return




select_adc:									; select and initialize analog channel W
; TRISC or other ports must be set for input if used for ADC
; ANSEL and ANSELH must have bits set for analog input

; select an analog channel for the ADC
; set bit<5-2> in ADCON0 for the selected input number
; save W
	movwf	temp3
; keep ADC on between channel and mode changes, so leave bit 1 and 0
	movlw	B'00000011'
	andwf	ADCON0
; get back W
	movfw	temp3
; max channels
	andlw	D'15'
; shift to the left to bit 5:2
	rlncf	WREG
	rlncf	WREG
; combine with DO/DONE and ADON 
	iorwf	ADCON0

; ADCON0:
;								; bit 7-6 Unimplemented: Read as 


;								; bit 5-2 CHS<3:0>: Analog Channel Select bits
;								;        0000 = AN0
;								;        0001 = AN1
;								;        0010 = AN2
;								;        0011 = AN3
;								;        0100 = AN4
;								;        0101 = AN5
;								;        0110 = AN6
;								;        0111 = AN7
;								;        1000 = AN8
;								;        1001 = AN9
;								;        1010 = AN10
;								;        1011 = AN11
;								;        1100 = Reserved
;								;        1101 = Reserved
;								;        1110 = DAC
;								;        1111 = FVR
;;	clrf	ADCON0
;;	movwf	ADCON0

;								; bit 1   GO/DONE: A/D Conversion Status bit
;								;        1 = A/D conversion cycle in progress. Setting this bit starts an A/D conversion cycle.
;								;            This bit is automatically cleared by hardware when the A/D conversion has completed.
;								;        0 = A/D conversion completed/not in progress


;								; bit 0   ADON: ADC Enable bit
;								;        1 = ADC is enabled
;								;        0 = ADC is disabled and consumes no operating current



; ADCON1
;								; bit 7-4 Unimplemented: Read as 
;								;								; bit 3-2 PVCFG<1:0>: Positive Voltage Reference select bit
;								;								; 00 = Positive voltage reference supplied internally by VDD.
;								;								; 01 = Positive voltage reference supplied externally through VREF+ pin.
;								;								; 10 = Positive voltage reference supplied internally through FVR.
;								;								; 11 = Reserved.
	bcf		ADCON1,	PVCFG1		; select Vdd as Positive voltage reference
	bcf		ADCON1, PVCFG0

;								; bit 1-0 NVCFG<1:0>: Negative Voltage Reference select bit
;								; 00 = Negative voltage reference supplied internally by VSS.
;								; 01 = Negative voltage reference supplied externally through VREF- pin.
;								; 10 = Reserved.
;								; 11 = Reserved.
	bcf		ADCON1,	NVCFG1			; select Vss as negative reference	
	bcf		ADCON1,	NVCFG0	



	bcf		ADCON2,	ADFM		; bit 7        ADFM: A/D Conversion Result Format Select bit, 1= Right justified,  0 = Left justified
;								; bit 6  Unimplemented: Read as 
										; left justified, use high byte only

;								; bit 5-3 ACQT<2:0>: A/D Acquisition Time Select bits. Acquisition time is the duration that the A/D charge
;								; holding capacitor remains connected to A/D channel from the instant the GO/DONE bit is set until conversions begins.
;								; 000 = 0(1), 001 = 2 TAD, 010 = 4 TAD, 011 = 6 TAD, 100 = 8 TAD, 101 = 12 TAD, 110 = 16 TAD, 111 = 20 TAD

	bsf		ADCON2, 5						; 100 = 8 TAD	SLOW
	bcf		ADCON2, 4
	bcf		ADCON2, 3	; WAS 1

;								; bit 2-0      ADCS<2:0>: A/D Conversion Clock Select bits
;								; 000 = FOSC/2, 001 = FOSC/8, 010 = FOSC/32,  011 = FRC(1) (clock derived from a dedicated internal oscillator = 600 kHz nominal)
;								; 100 = FOSC/4, 101 = FOSC/16, 110 = FOSC/64, 111 = FRC(1) (clock derived from a dedicated internal oscillator = 600 kHz nominal)
								; Note 1: When the A/D clock source is selected as FRC then the start of conversion is delayed by one instruction  cycle after the GO/DONE bit is set to allow the SLEEP instruction to be executed.
; YYY
;	bcf		ADCON2, 2
;	bcf 	ADCON2, 1				; / 2
;	bcf		ADCON2, 0

;	bsf		ADCON2, 2
;	bcf 	ADCON2, 1				; / 4
;	bcf		ADCON2, 0

;	bcf		ADCON2, 2
;	bcf 	ADCON2, 1				; / 8
;	bsf		ADCON2, 0

select_adc_low_speed:			; xy mode, low speed
; low speed
	bcf		ADCON2, 2				; / 32
	bsf 	ADCON2, 1
	bcf		ADCON2, 0

select_adc_end:
	
; ADC on
	bsf     ADCON0, ADON

	return


print_adc:
; ADC result is left justified, shift right 6 x       1111 1111 1100 0000

	movlw	B'11000000'
	andwf	ADRESL

	movfw	ADRESH
	call	print_w_ascii_dec

	movlw	' '
	call	tx_w

	movlw	'H'
	call	tx_w

	movlw	 ' '
	call	tx_w

	movlw	 ' '
	call	tx_w

	movfw	ADRESL
	call	print_w_ascii_dec

	movlw	' '
	call	tx_w

	movlw	'L'
	call	tx_w

	movlw	'\t'
	call	tx_w

	call	tx_crlf

; shift rigt left justified ADC result
	bcf		STATUS, C	
	rrcf	ADRESH				; bit 0 ADRESH to carry
	rrcf	ADRESL				; carry to bit 7 ADRESL
; >> 1	/ 2

	bcf		STATUS, C	
	rrcf	ADRESH
	rrcf	ADRESL
; >> 2	/ 4

	bcf		STATUS, C	
	rrcf	ADRESH
	rrcf	ADRESL
; >> 3	/ 8

	bcf		STATUS, C	
	rrcf	ADRESH
	rrcf	ADRESL
; >> 4	/ 16

	bcf		STATUS, C	
	rrcf	ADRESH
	rrcf	ADRESL
; >> 5	/ 32

	bcf		STATUS, C	
	rrcf	ADRESH
	rrcf	ADRESL
; >> 6	/ 64

	clrf	REGA3
	clrf	REGA2
	movff	ADRESH, REGA1
	movff	ADRESL, REGA0
; REGA3:REGA0 = ADC steps

	call	print_32_ascii_dec					; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display at position xpos, ypos, normal height, short box

	movlw	' '
	call	tx_w

	movlw	's'
	call	tx_w
	
	movlw	't'
	call	tx_w

	movlw	'e'
	call	tx_w
	
	movlw	'p'
	call	tx_w

	movlw	's'
	call	tx_w

	return


print_battery_voltage:
	movlw	'b'
	call	tx_w

	movlw	'a'
	call	tx_w

	movlw	't'
	call	tx_w

	movlw	' '
	call	tx_w

	movlw	'='
	call	tx_w

	movlw	' '
	call	tx_w

; select ADC channel 5 battery voltage
	movlw	D'5'
	call	select_adc

; read ADC AN1 ch 1 y input
	bsf		ADCON0, GO			; Start conversion			bit	1
	btfsc	ADCON0, GO			; Is conversion done?
	bra		$-1					; No, test again

; calculate voltage

;	mV = (steps * 4 * VREF) / 1023, but will do (steps * 4 * VREF) / (10230 * 64), because left justified, and will put point at hundreds 
;	put dot at 3rd digit


	clrf	REGA3
	clrf	REGA2
	movff	ADRESH, REGA1
	movff	ADRESL, REGA0
;  REGA3:REGA0 = ADC steps

	clrf	REGB3

	clrf	REGB2

	movlw	HIGH (VREF*4)
	movwf	REGB1

	movlw	LOW (VREF*4)
	movwf	REGB0
; REGB3:REGB0 = VREF * 4, the 4 comes from the 4x resistor divider at the input

    call    multiply                                    ; 32 bit signed multiply  REGA * REGB -> REGA  return carry set if overflow
; REGA3:REGA0 = vsteps *  VREF * 4

	clrf 	REGB3

	movlw	UPPER	(D'10230'*D'64') 					; 64 becausewe areleft  justified by 6 shifts, 2^6 = 64, in ADC result 
	movwf	REGB2

	movlw	HIGH	(D'10230'*D'64')
	movwf	REGB1

	movlw	LOW		(D'10230'*D'64')
	movwf	REGB0
; REGB3:REGB0 = 10230

	call	divide										; REGA / REGB -> REGA return carry set if overflow or division by zero
; REGA3:REGA0 = (vsteps *  VREF * 4) / 10230		


; put point in right place
	bsf		DIVIDE_BY_HUNDRED_FLAG
	call	print_32_ascii_dec					; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display at position xpos, ypos, normal height, short box
	bcf		DIVIDE_BY_HUNDRED_FLAG

	movlw	' '
	call	tx_w

	movlw	'V'
	call	tx_w

	return


#ifdef OLD_CODE
time_to_display:
	movff	xpos, temp_xpos
	movff	ypos, temp_ypos
	movff	acqmem, temp_acqmem

	movlw	D'7'
	movwf	xpos

	movlw	D'23'
	movwf	ypos

	clrf	acqmem

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len
; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; hour
	movfw	hours
	call	tprint_w_ascii_dec
	
; separator
	movlw	':'
	btfss	USE_FLASHING_CLOCK_FLAG
	bra		ttd_separator
; flash ':', on on even seconds, else space
	btfss	seconds, 0
	movlw	' '
ttd_separator:
	movwf	POSTINC0
	incf	txt_len	

; minute
	movfw	minutes
	call	tprint_w_ascii_dec

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len	

	call	tputstr

	movff	temp_acqmem, acqmem
	movff	temp_xpos, xpos
	movff	temp_ypos, ypos
	return
#endif ; OLD_CODE



tprint_w_ascii_dec:				; First use tputcu, set LFSR0, set txt_len,  prints register W in ASCII decimal to current  LFSR, increments txt_len, to display data call tputstr

	bsf		FIRST_ZERO_SUPPRESSED_FLAG

tprint_w_ascii_dec_lz:
	movwf	temp1

	clrf	count				; number of hundreds found
t_loop_hundreds:
	movlw	D'100'
	subwf	temp1
	btfss	STATUS,	C			; if no carry flag, no more hundreds, go count tenth		
	bra		t_count10				; substraction failed
	incf	count	
	bra		t_loop_hundreds
t_count10:
	movlw	D'0'
	subwf	count,	W
	btfsc	STATUS,	Z
	bra		t_suppress_first_zero

	bcf		FIRST_ZERO_SUPPRESSED_FLAG

	movfw	count

	call	t_tx_digit_in_w			; print hundreds

t_suppress_first_zero:
	movlw	D'100'				; restore temp1 from one substract to many
	addwf	temp1	

	clrf	count				; number of tenth found
t_loop_tenth:
	movlw	D'10'
	subwf	temp1
	btfss	STATUS,	C			; if no carry flag no more tenth, only units left	
	bra		t_count1		
	incf	count
	bra		t_loop_tenth
t_count1:
	movlw	D'0'
	subwf	count,	W
	btfss	STATUS,	Z
	bra		t_print_tenth			; tenth not zero

; tenth zero
; test if zero supression was active in hundreds (first digit)
	btfsc	FIRST_ZERO_SUPPRESSED_FLAG
	bra		t_print_units			; if first digit was not zero, print this zero	

t_print_tenth:
	movfw	count
	call	t_tx_digit_in_w			; print tenth

	btfss	DIVIDE_BY_TEN_FLAG
	bra		t_print_units
; print the dot
	movlw	'.'
	movwf	POSTINC0
	incf	txt_len

t_print_units:
	movlw	D'10'				; restore temp1 from 1 substract to many
	addwf	temp1

; units
	movfw	temp1
	call	t_tx_digit_in_w			; print units

	return


t_tx_digit_in_w:
	addlw	'0'				; zero

t_tx_w:
	movwf	POSTINC0
	incf	txt_len

	return		; end subroutine tprint_w_ascii_dec




#ifdef OLD_CODE
data_to_saa_mem:				; data in variables to SAA5281 acq memory 0 for display

; battery_voltage (= fuel)
	movlw	BATTERY_VOLTAGE_POS_X
	movwf	xpos

	movlw	BATTERY_VOLTAGE_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	peek	battery_voltage, WREG

	bsf		DIVIDE_BY_TEN_FLAG
	call	tprint_w_ascii_dec			; uses DIVIDE_BY_TEN_FLAG, first use tputcu, set LFSR0, set txt_len,  prints register W in ASCII decimal to current LFSR, increments txt_len, to display data call tputstr
	bcf		DIVIDE_BY_TEN_FLAG

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	'V'
	movwf	POSTINC0
	incf	txt_len

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; heading GPS
	movlw	GPS_HEADING_POS_X
	movwf	xpos

	movlw	GPS_HEADING_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

;	movlw	DOUBLE_HEIGHT
;	movwf	POSTINC0
;	incf	txt_len

	clrf	REGA3
	clrf	REGA2
	peek	heading_degrees_h, REGA1
	peek	heading_degrees_l, REGA0
	
	call	tprint_32_ascii_dec			; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; magnetic_heading_h
; magnetic_heading_l
	movlw	MAGNETIC_HEADING_POS_X
	movwf	xpos

	movlw	MAGNETIC_HEADING_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

;	movlw	DOUBLE_HEIGHT
;	movwf	POSTINC0
;	incf	txt_len

	clrf	REGA3
	clrf	REGA2
	peek	magnetic_heading_h, REGA1
	peek	magnetic_heading_l, REGA0
	
	call	tprint_32_ascii_dec			; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; latitude_degrees
	movlw	GPS_LATITUDE_POS_X
	movwf	xpos

	movlw	GPS_LATITUDE_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	peek	latitude_degrees, WREG
	call	tprint_w_ascii_dec			; uses DIVIDE_BY_TEN_FLAG, first use tputcu, set LFSR0, set txt_len,  prints register W in ASCII decimal to current LFSR, increments txt_len, to display data call tputstr

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

; latitude_minutes_u
; latitude_minutes_h
; latitude_minutes_l	
	clrf	REGA3
	peek	latitude_minutes_u, REGA2
	peek	latitude_minutes_h, REGA1
	peek	latitude_minutes_l, REGA0
	bsf		DIVIDE_BY_TEN_THOUSAND_FLAG	
	call	tprint_32_ascii_dec
	bcf		DIVIDE_BY_TEN_THOUSAND_FLAG	
	
	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

; North south
	peek	telemetry_flags, REGC0
	movlw	'N'
	btfss	REGC0, NORTH
	movlw	'S'	

	movwf	POSTINC0
	incf	txt_len

	call	tputstr
	

; longitude_degrees_h
; longitude_degrees_l
	movlw	GPS_LONGITUDE_POS_X
	movwf	xpos

	movlw	GPS_LONGITUDE_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	clrf	REGA3
	clrf	REGA2
	peek	longitude_degrees_h, REGA1
	peek	longitude_degrees_l, REGA0
	
	call	tprint_32_ascii_dec			; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

; longitude_minutes_u
; longitude_minutes_h
; longitude_minutes_l
	clrf	REGA3
	peek	longitude_minutes_u, REGA2
	peek	longitude_minutes_h, REGA1
	peek	longitude_minutes_l, REGA0
	bsf		DIVIDE_BY_TEN_THOUSAND_FLAG	
	call	tprint_32_ascii_dec					; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal
	bcf		DIVIDE_BY_TEN_THOUSAND_FLAG	

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

; North south
	peek	telemetry_flags, REGC0
	movlw	'E'
	btfss	REGC0, EAST
	movlw	'W'	

	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; gps_altitude_h
; gps_altitude_l
	movlw	GPS_ALTITUDE_POS_X
	movwf	xpos

	movlw	GPS_ALTITUDE_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	clrf	REGA3
	clrf	REGA2
	peek	altitude_h, REGA1
	peek	altitude_l, REGA0
	
	call	tprint_32_ascii_dec			; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; pressure_altitude_h
; pressure_altitude_l

	movlw	PRESSURE_ALTITUDE_POS_X
	movwf	xpos

	movlw	PRESSURE_ALTITUDE_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	clrf	REGA3
	clrf	REGA2
	peek	pressure_altitude_h, REGA1
	peek	pressure_altitude_l, REGA0
	
	call	tprint_32_ascii_dec			; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; speed_kmh
	movlw	GPS_SPEED_KMH_POS_X
	movwf	xpos

	movlw	GPS_SPEED_KMH_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	peek	speed_kmh, WREG
	call	tprint_w_ascii_dec			; uses DIVIDE_BY_TEN_FLAG, first use tputcu, set LFSR0, set txt_len,  prints register W in ASCII decimal to current LFSR, increments txt_len, to display data call tputstr

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; acceleration_x
	movlw	ACCELERATION_X_POS_X
	movwf	xpos

	movlw	ACCELERATION_X_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; test if  negative, if so  print '-' sign convert to pos, else '+' sign
	clrf	REGA3
	clrf	REGA2
	peek	acceleration_x_h, REGA1
	peek	acceleration_x_l, REGA0
; REGA3:REGA0 acceleration_x_h:acceleration_x_l

; test sign bit
	btfss	REGA1, 7
	bra	 acc_x_pos
; acc_x negative		value = 65536 - value
	
	movff	REGA3, REGB3
	movff	REGA2, REGB2
	movff	REGA1, REGB1
	movff	REGA0, REGB0
; REGB3:REGB0 = acc x signed

	clrf	REGA3
	movlw	UPPER	D'65536'
	movwf	REGA2
	movlw	HIGH	D'65536'
	movwf	REGA1
	movlw	LOW		D'65536'
	movwf	REGA0
; REGAR3:REGA0 = 65536

; REGB3:REGB0 = acceleration_x
	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = 65536 - acceleration_x_h:acceleration_x_l

	movlw	'-'
	movwf	POSTINC0
	incf	txt_len
	bra		acc_x_pri

acc_x_pos:
	movlw	'+'
	movwf	POSTINC0
	incf	txt_len

acc_x_pri:
	bsf		DIVIDE_BY_TEN_FLAG
	call	tprint_32_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	'g'
	movwf	POSTINC0
	incf	txt_len

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; acceleration_y
	movlw	ACCELERATION_Y_POS_X
	movwf	xpos

	movlw	ACCELERATION_Y_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; test if  negative, if so  print '-' sign convert to pos, else '+' sign
	clrf	REGA3
	clrf	REGA2
	peek	acceleration_y_h, REGA1
	peek	acceleration_y_l, REGA0
; REGA3:REGA0 acceleration_y_h:acceleration_y_l

; test sign bit
	btfss	REGA1, 7
	bra	 acc_y_pos
; acc_y negative		value = 65536 - value
	
	movff	REGA3, REGB3
	movff	REGA2, REGB2
	movff	REGA1, REGB1
	movff	REGA0, REGB0
; REGB3:REGB0 = acc y signed

	clrf	REGA3
	movlw	UPPER	D'65536'
	movwf	REGA2
	movlw	HIGH	D'65536'
	movwf	REGA1
	movlw	LOW		D'65536'
	movwf	REGA0
; REGAR3:REGA0 = 65536

; REGB3:REGB0 = acceleration_y
	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = 65536 - acceleration_y_h:acceleration_y_l

	movlw	'-'
	movwf	POSTINC0
	incf	txt_len
	bra		acc_y_pri

acc_y_pos:
	movlw	'+'
	movwf	POSTINC0
	incf	txt_len

acc_y_pri:
	bsf		DIVIDE_BY_TEN_FLAG
	call	tprint_32_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	'g'
	movwf	POSTINC0
	incf	txt_len

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; acceleration_z
	movlw	ACCELERATION_Z_POS_X
	movwf	xpos

	movlw	ACCELERATION_Z_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; test if  negative, if so  print '-' sign convert to pos, else '+' sign
	clrf	REGA3
	clrf	REGA2
	peek	acceleration_z_h, REGA1
	peek	acceleration_z_l, REGA0
; REGA3:REGA0 acceleration_z_h:acceleration_z_l

; test sign bit
	btfss	REGA1, 7
	bra	 acc_z_pos
; acc_z negative		value = 65536 - value
	
	movff	REGA3, REGB3
	movff	REGA2, REGB2
	movff	REGA1, REGB1
	movff	REGA0, REGB0
; REGB3:REGB0 = acc x signed

	clrf	REGA3
	movlw	UPPER	D'65536'
	movwf	REGA2
	movlw	HIGH	D'65536'
	movwf	REGA1
	movlw	LOW		D'65536'
	movwf	REGA0
; REGAR3:REGA0 = 65536

; REGB3:REGB0 = acceleration_z
	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = 65536 - acceleration_z_h:acceleration_z_l

	movlw	'-'
	movwf	POSTINC0
	incf	txt_len
	bra		acc_z_pri

acc_z_pos:
	movlw	'+'
	movwf	POSTINC0
	incf	txt_len

acc_z_pri:
	bsf		DIVIDE_BY_TEN_FLAG
	call	tprint_32_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	'g'
	movwf	POSTINC0
	incf	txt_len

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; rotation_x
	movlw	ROTATION_X_POS_X
	movwf	xpos

	movlw	ROTATION_X_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; test if  negative, if so  print '-' sign convert to pos, else '+' sign
	clrf	REGA3
	clrf	REGA2
	peek	rotation_x_h, REGA1
	peek	rotation_x_l, REGA0
; REGA3:REGA0 = rotation_x_h:rotation_x_l

; test sign bit
	btfss	REGA1, 7
	bra	 rot_x_pos
; rot_x negative		value = 65536 - value
	
	movff	REGA3, REGB3
	movff	REGA2, REGB2
	movff	REGA1, REGB1
	movff	REGA0, REGB0
; REGB3:REGB0 = rotation_x signed

	clrf	REGA3
	movlw	UPPER	D'65536'
	movwf	REGA2
	movlw	HIGH	D'65536'
	movwf	REGA1
	movlw	LOW		D'65536'
	movwf	REGA0
; REGAR3:REGA0 = 65536

; REGB3:REGB0 = 
	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = 65536 - rotation_x_h:rotation_x_l

	movlw	'-'
	movwf	POSTINC0
	incf	txt_len
	bra		rot_x_pri

rot_x_pos:
	movlw	'+'
	movwf	POSTINC0
	incf	txt_len

rot_x_pri:
	bsf		DIVIDE_BY_TEN_FLAG
	call	tprint_32_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	'd'
	movwf	POSTINC0
	incf	txt_len

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; rotation_y 
	movlw	ROTATION_Y_POS_X
	movwf	xpos

	movlw	ROTATION_Y_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; test if  negative, if so  print '-' sign convert to pos, else '+' sign
	clrf	REGA3
	clrf	REGA2
	peek	rotation_y_h, REGA1
	peek	rotation_y_l, REGA0
; REGA3:REGA0 = rotation_y_h:rotation_y_l

; test sign bit
	btfss	REGA1, 7
	bra	 rot_y_pos
; rot_y negative		value = 65536 - value
	
	movff	REGA3, REGB3
	movff	REGA2, REGB2
	movff	REGA1, REGB1
	movff	REGA0, REGB0
; REGB3:REGB0 = rotation_y signed

	clrf	REGA3
	movlw	UPPER	D'65536'
	movwf	REGA2
	movlw	HIGH	D'65536'
	movwf	REGA1
	movlw	LOW		D'65536'
	movwf	REGA0
; REGAR3:REGA0 = 65536

; REGB3:REGB0 = 
	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = 65536 - rotation_y_h:rotation_y_l

	movlw	'-'
	movwf	POSTINC0
	incf	txt_len
	bra		rot_y_pri

rot_y_pos:
	movlw	'+'
	movwf	POSTINC0
	incf	txt_len

rot_y_pri:
	bsf		DIVIDE_BY_TEN_FLAG
	call	tprint_32_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	'd'
	movwf	POSTINC0
	incf	txt_len

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; rotation_z
	movlw	ROTATION_Z_POS_X
	movwf	xpos

	movlw	ROTATION_Z_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; test if  negative, if so  print '-' sign convert to pos, else '+' sign
	clrf	REGA3
	clrf	REGA2
	peek	rotation_z_h, REGA1
	peek	rotation_z_l, REGA0
; REGA3:REGA0 = rotation_z_h:rotation_z_l

; test sign bit
	btfss	REGA1, 7
	bra	 rot_z_pos
; rot_z negative		value = 65536 - value
	
	movff	REGA3, REGB3
	movff	REGA2, REGB2
	movff	REGA1, REGB1
	movff	REGA0, REGB0
; REGB3:REGB0 = rotation_z signed

	clrf	REGA3
	movlw	UPPER	D'65536'
	movwf	REGA2
	movlw	HIGH	D'65536'
	movwf	REGA1
	movlw	LOW		D'65536'
	movwf	REGA0
; REGAR3:REGA0 = 65536

; REGB3:REGB0 = 
	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = 65536 - rotation_z_h:rotation_z_l

	movlw	'-'
	movwf	POSTINC0
	incf	txt_len
	bra		rot_z_pri

rot_z_pos:
	movlw	'+'
	movwf	POSTINC0
	incf	txt_len

rot_z_pri:
	bsf		DIVIDE_BY_TEN_FLAG
	call	tprint_32_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	'd'
	movwf	POSTINC0
	incf	txt_len

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; gps_time
	movlw	GPS_TIME_POS_X
	movwf	xpos

	movlw	GPS_TIME_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	peek	hours, WREG
	bcf		DIVIDE_BY_TEN_FLAG
	call	tprint_w_ascii_dec			; uses DIVIDE_BY_TEN_FLAG, first use tputcu, set LFSR0, set txt_len,  prints register W in ASCII decimal to current LFSR, increments txt_len, to display data call tputstr

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	':'
	movwf	POSTINC0
	incf	txt_len

; minutes
	peek	minutes, WREG
	bcf		DIVIDE_BY_TEN_FLAG
	call	tprint_w_ascii_dec			; uses DIVIDE_BY_TEN_FLAG, first use tputcu, set LFSR0, set txt_len,  prints register W in ASCII decimal to current LFSR, increments txt_len, to display data call tputstr

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; temperature
	movlw	TEMPERATURE_POS_X
	movwf	xpos

	movlw	TEMPERATURE_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; test if  negative, if so  print '-' sign convert to pos, else '+' sign
	peek 	temperature, WREG
	btfss	WREG, 7
	bra	 tempe_p_pos
; tempe_p negative		value = 256 - value
	clrf	REGB3
	clrf	REGB2
	clrf	REGB1
	movff	REGA0, REGB0
; REGB3:REGB0 = temperature

	clrf	REGA3
	clrf	REGA2
	movlw	HIGH	D'256'
	movwf	REGA1
	movlw	LOW		D'256'
	movwf	REGA0
; REGAR3:REGA0 = 256

	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = 256 - temperature

	movlw	'-'
	movwf	POSTINC0
	incf	txt_len

	movfw	REGA0
	bra		tempe_p_pri
	
tempe_p_pos:
	movlw	'+'
	movwf	POSTINC0
	incf	txt_len
	peek	temperature, WREG
tempe_p_pri:
	bcf		DIVIDE_BY_TEN_FLAG
	call	tprint_w_ascii_dec			; uses DIVIDE_BY_TEN_FLAG, first use tputcu, set LFSR0, set txt_len,  prints register W in ASCII decimal to current LFSR, increments txt_len, to display data call tputstr

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

;	movlw	0xb0					; degrees symbol in X, but not in SAA? and no space to print it
;	movwf	POSTINC0
;	incf	txt_len

	movlw	'C'
	movwf	POSTINC0
	incf	txt_len

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; print status flags
; GEAR_DOWN
; AUTO_PILOT_ON
; RX_OK
; TARGET_LOCK
; NORTH
; EAST
	movlw	STATUS_POS_X
	movwf	xpos

	movlw	STATUS_POS_Y
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len
; gear
	peek	telemetry_flags, REGC0

	movlw	'G'
	btfss	REGC0, GEAR_DOWN
	movlw	' '

	movwf	POSTINC0
	incf	txt_len

; auto pllot
	movlw	'A'
	btfss	REGC0, AUTO_PILOT_ON
	movlw	' '

	movwf	POSTINC0
	incf	txt_len

; Rx status
	movlw	'R'
	btfss	REGC0, RX_OK
	movlw	' '
	
	movwf	POSTINC0
	incf	txt_len

; target lock
	movlw	'T'
	btfss	REGC0, TARGET_LOCK
	movlw	' '

	movwf	POSTINC0
	incf	txt_len


	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	call	tputstr


; print local battery voltage

; select ADC channel 5 battery voltage
	movlw	D'5'
	call	select_adc

; read ADC AN1 ch 1 y input
	bsf		ADCON0, GO			; Start conversion			bit	1
	btfsc	ADCON0, GO			; Is conversion done?
	bra		$-1					; No, test again

; calculate voltage
;	mV = (steps * 4 * VREF) / 1023, but will do (steps * 4 * VREF) / (10230 * 64), becauseleft justified, and will put point at hundreds 
;	put dot at 3rd digit

	clrf	REGA3
	clrf	REGA2
	movff	ADRESH, REGA1
	movff	ADRESL, REGA0
;  REGA3:REGA0 = ADC steps

	clrf	REGB3

	clrf	REGB2

	movlw	HIGH (VREF*4)
	movwf	REGB1

	movlw	LOW (VREF*4)
	movwf	REGB0
; REGB3:REGB0 = VREF * 4, the 4 comes from the 4x resistor divider at the input

    call    multiply                                    ; 32 bit signed multiply  REGA * REGB -> REGA  return carry set if overflow
; REGA3:REGA0 = vsteps *  VREF * 4

	clrf 	REGB3

	movlw	UPPER	(D'10230'*D'64') 					; 64 becausewe areleft  justified by 6 shifts, 2^6 = 64, in ADC result 
	movwf	REGB2

	movlw	HIGH	(D'10230'*D'64')
	movwf	REGB1

	movlw	LOW		(D'10230'*D'64')
	movwf	REGB0
; REGB3:REGB0 = 10230

	call	divide										; REGA / REGB -> REGA return carry set if overflow or division by zero
; REGA3:REGA0 = (vsteps *  VREF * 4) / 10230		

	movlw	D'0'
	movwf	xpos

	movlw	D'23'
	movwf	ypos

	call	tputcu

	clrf	txt_len
	lfsr	FSR0,	TEXT_START

	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

; SAA needs 2 x for box start!
	movlw	START_BOX
	movwf	POSTINC0
	incf	txt_len

	bsf		DIVIDE_BY_HUNDRED_FLAG
	call	tprint_32_ascii_dec			; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal to teletext display at position xpos, ypos, uses BOX_FLAG, SHORT_BOX_FLAG, DOUB:E_HEIGHT_FLAG, default normal height, no box
	bcf		DIVIDE_BY_HUNDRED_FLAG

	movlw	END_BOX
	movwf	POSTINC0
	incf	txt_len

	movlw	' '
	movwf	POSTINC0
	incf	txt_len

	movlw	'V'
	movwf	POSTINC0
	incf	txt_len

	call	tputstr



	return
#endif ; OLD_CODE




do_checksum:				; from FSR0,  returns status zero flag set if OK
	lfsr	FSR0, DATA_BUFFER_START+D'1'
; length from byte 1
	movff	POSTINC0, temp1
; first data byte next byte
	clrf	checksum
c_loop:
	movfw	POSTINC0
	xorwf	checksum
	decfsz	temp1	
	bra		c_loop
; checksum next byte
; compare
	movfw	INDF0
	subwf	checksum

	return			; end subroutine checksum



clear_line:			; xpos, ypos, length in W
; make a text consisting of spaces
	lfsr	FSR0, TEXT_START
; set length field
	movwf	POSTINC0
; fill with spaces
	movwf	temp3
	movlw	' '
clr_line_loop:
	movwf	POSTINC0
	decfsz	temp3
	bra		clr_line_loop
; to_saa  macro pointer
	to_saa	TEXT_START

	return			; end subrooutine clear_line


 	end


