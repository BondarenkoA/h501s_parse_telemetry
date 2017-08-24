
; Panteltje (c) gpss Copyright Jan Panteltje 2016-always

; 32 bit signed integer math copyright PETER HEMSLEY
; the rest is copyright Jan Panteltje
; and released under the GPL. GPL violaters will be procecuted. If you use
; this code, you are required to release SOURCE CODE.

; The SDcard routines are based on NXP Semiconductors application note AN10406 Accessing SD/MMC card using SPI on LPC2000, in C,
; I rewrote those in asm with some changes.

; This program is free softlware; you can redistribute it and/or modify
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


; What it does:
; This reads fake NMEA GPS messages from SDcard, and sends it to the serial port at 57600 Bd
; Only 2 GB sdcard is supported.
; No filesystem, data must be in correct format on /dev/sdX
;
; Ech of the below blocks is send every 200 ms, emulating a 5 Hz GPS module
;
; $GPRMC,101752.60,A,5315.07684,N,00535.97300,E,0.267,,051016,,,D*72M             <- position GPS
; $GPGGA,101752.60,5315.07684,N,00535.97300,E,2,07,1.28,-6.6,M,45.7,M,,0000*7FM   <- position GPS
; $GPGSV,4,1,15,01,02,248,,07,12,289,18,08,51,293,27,10,57,128,29*79M
; $GPGSV,4,2,15,11,15,266,,13,04,007,,15,11,035,,16,36,190,28*7EM
; $GPGSV,4,3,15,18,45,070,30,20,00,036,,21,19,072,19,26,11,176,25*74M
; $GPGSV,4,4,15,27,85,160,19,30,12,317,,33,26,206,*4FM
;
; The system is connected to the Hubsan H501S quadcopter remote's GPS input,
; so that in follow-me mode the quadcopter follows the 3D position specified in the SDcard sectors.
;
; The program name 'gpss' stands for GPS spoofer.




; This code is assembled with gpasm in Linux.
; This code likely will not assemble without changes in Microchip mplab.


;                           **** set TABS to 4 to read any of this ****



; CHANGES:
;
; 0.1:
; This is a first release, test code
;
; 0.2:
; Enabled save_wstat and restore_w_stat macros as ther IS something in main
; Using output_mode now, cycles through modes
;
; 0.3:
; Changed everything
; added altitude control from remote AFC_RX testpoint, drives throttle, 2 extra wires.
; added a NPN PNP buffer to drive the throttle.
;
; 0.4:
; v_speed_limit_dm_per_400_ms now used in interrupt, was fixed to D'1'
; changed the code DAC control nwo in main
; added WAIT_UPDATING_ALTITUDE_FLAG to prevent interrupt messing up altitude vars.
; changed LED functions:
;  red green flashing is wait to reach altitude specified (looping sector),
;  red only flashing is moving to next waypoint
;  green only on steady is normal GPS.
; no v speed limit, use DAC_MAX_VALUE (up limit) and DAC_MIN_VALUE (down limit).
; test for v speed limit now in main (set by int), when v speed limit is reached then control is released, and DAC set to midrange, as if in correct altitude, sequence incr / decr DAC restarts 
;
; 0.5:
; work on drop command via LED switch
;



; This source file was assembled with gpasm-0.13.5 beta.


; uncomment this to see many variables if not in test mdoe
;#define DEBUG

; Uncomment this for serial input from user (DAC test), h for help, else from remote AFC_RX
;#define TEST_MODE


; DAC
; Measured:
; pots run on 3.3 V, measured slider to top and bottiom in circuit 3k3 O km
; >= 2.0 V on left stick up gets into liftoff, motor speed then seems to hold at new level
; lowering voltage below 1.66 decreases motor speed again, forces landing.
; 1.65 = 3.3V / 2
;
; WAS needs testing, 'up' behaviour less threshold than down?
;
; Need 1.65 V (half of 3.3 V) have a reference of 4.096 V, so (1.65 / 4.096) * 32 = 12.890625
;#define DAC_MID_RANGE	D'13'

; or perhaps 1.65 + 0.7 = 2.3 V   ( (1.65 + 0.7) / 4.096) * 32  = 18.359375
#define DAC_MID_RANGE   D'18'

; Need 3.3 V + 0.7 V   ( (3.3 + 0.7) / 4.096) * 32  = 31.250000, the 0.7 for the VBE drop in te hNPN buffer
#define DAC_MAX_RANGE	D'31'

; 0V on DAC gives 0.7 V out on the buffer
#define DAC_MIN_RANGE	D'0'

		

; maxmimum vertical speed decimeter per 400 ms
#define MAX_V_SPEED_DM_PER_400ms		4


; how many blocks of data per second GPS simulator sends, 6 NMEA lines per block, as shown above
#define	DEFAULT_GPS_RATE	5

; time button must be pressed to switch to card output from GPS (normally GPS passthrough), in 200 ms steps, so 5 is 1 second
#define DEBOUNCE_TIME	5

; define bits of command_flags
#define LED_SWITCH_FLAG		0

; MMC the number of 512 bytes sectors, this is (2^32 / 4) * 2  = 2,147,483,648 =  0x80 00 00 00  for a 1GB card, sorry to big for gpasm?, hardoced look or '0x80000000' in the code.
; Note I am using 32 bits math so 4 GB cards is the maximum anyways.
#define MMC_MEMORY_LIMIT				0x80000000	

; how long to wait after a MMC sector write for card signalling ready.
#define MMC_WRITE_TIMEOUT				D'4095'

; how long to wait for a response from the MMC (SDcard) before deciding there is no card or no valid card.
#define MMC_RESPONSE_TIMEOUT			D'4095'
;#define MMC_RESPONSE_TIMEOUT			D'65534'

; default clock speed (1 second tick) if no GPS signal received.
; timer0 clock = (64000000 / 4) / 256 = 62500
; for a 1 second tick we need to preset timer 1 to 65535 - 62500 = 3035
; for 200 mS = 3035 / 5 = 607, 1/5 count down time 65535 - 607 = 64928

; reload for 256 prescaler =  65535 - ( (16000000 / 256)  / f) = 53035 for 5 hz
; for 5 Hz = 53035 
#define TIMER0_PRELOAD					D'53035'

; not used
#define TIMER1_PRELOAD					D'53035'

; software loop time between I/O bits on SPI bus
#define SPI_DELAY						D'5'


; For speed, either 16 MHz without PLL 19200 Bd, or 64 MHz with PLL 115200 Bd
#define USE_PLL


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
;	CONFIG	WDTEN = OFF			; WDT is controlled by SWDTEN bit of the WDTCON register
	CONFIG	WDTEN = ON			; WDT is always enabled. SWDTEN bit has no effect.
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
;	CONFIG	WDTPS = 1024		; 1:1024			x 4 ms = about 4 seconds.
;	CONFIG	WDTPS = 2048		; 1:2048
	CONFIG	WDTPS = 4096		; 1:4096
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



; RS232 57600 Bd 1 start bit, 8 data bits, 1 stop bit, type h for help.


; for commands with numeric arguments.
#define COMMAND_OFF										D'0'
#define COMMAND_GET_NUMBER								D'1'
#define COMMAND_PRINT_STATUS							D'2'
#define COMMAND_DUMP_DATABSE							D'3'
#define COMMAND_WRITE_DATA_RECORD						D'4'
#define COMMAND_READ_DATA_RECORD						D'5'
#define COMMAND_SET_DATA_RECORD_POINTER					D'6'
#define COMMAND_SET_GPS_RATE							D'7'
#define COMMAND_SET_HOURS								D'8'
#define COMMAND_SET_MINUTES								D'9'
#define COMMAND_PRINT_HELP								D'10'
#define COMMAND_SET_DAC									D'11'
#define COMMAND_DISABLE_DAC								D'12'
;#define 


; RS232 
#define SIGNAL_BELL									D'7'


; variables
	CBLOCK 0x00

	temp1							; print_w_ascii_dec						
	flags1							; cleared on reset	
	count						
	temp_u							; text_pri
	temp_h						
	temp_l						
	temp4							; print_32_ascii_dec
	command						
	digit_in					
	digit_cnt					
	value3
	value2
	value1
	value0					
	flags2							; some of these flags are left over reset
	AARGB4							; used in print 16 in debug, do not use for anything else.
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
	flags3					; saved in EEPROM
	flags4
	flags5
	temp3					; select baudrate
	temp_3					; print_adc
	temp_2
	temp_1
	temp_0
	temp_x					; hex printing
	temp_x2					; hex printing only
	delay_count1
	spi_byte
	spi_bit_count
	spi_in
	spi_out
	spi_byte_count	
	data_record_number3
	data_record_number2
	data_record_number1
	data_record_number0
	debounce_counter
	timer0_reload_h
	timer0_reload_l
	gps_rate
	temp_s
	temp_w
	command_flags
	altitude_mode
	requested_altitude_decimeters_h
	requested_altitude_decimeters_l	
	current_altitude_decimeters_h
	current_altitude_decimeters_l
	checksum
	i_counter
	after_dot_count
	previous_current_altitude_decimeters_h
	previous_current_altitude_decimeters_l
	iAARGB4							; used in print 16 in debug, do not use for anything else.
	iAARGB3
	iAARGB7
	iAARGB6
	iAARGB5
	isign
	iREGA3
	iREGA2
	iREGA1
	iREGA0
	iREGB3
	iREGB2
	iREGB1
	iREGB0
	iREGC3
	iREGC2
	iREGC1
	iREGC0
	iMCOUNT
	iflags4
	vertical_speed_dm_per_400ms_h
	vertical_speed_dm_per_400ms_l
	v_speed_limit_dm_per_400_ms
	pa_error
; 94

; 96 is overwritten!

	ENDC


; data segment
#define SPI_RESULT_ADDRESS					0x100


; variables kept in RAM accessed by 'poke' and 'peek':



;#define	VARS2_ADDRESS						0x190

;#define free								VARS2_ADDRESS+D'0'
;#define
;#define
;#define


; define  flags1
#define END_OF_DATA_FLAG					flags1,0	; main
#define DISABLE_ERROR3_REPORTING_FLAG		flags1,1	; main
#define MMC_INIT_OK_FLAG 					flags1,2	; main
#define TIME_FLAG							flags1,3	; int to main exchange
#define UP_FLAG								flags1,4	; main
#define ECHO_GPS_FLAG						flags1,5	; main
#define WAIT_UPDATING_ALTITUDE_FLAG			flags1,6	; int to main when setting current_altitude_decimeters_h:current_altitude_decimeters_l
#define NEGATIVE_ALTITUDE_FLAG				flags1,7	; int to main

; define flags2 some flags 2 are preserved over warm start
#define WATCHDOG_RESET_FLAG					flags2,0	; main
#define BROWNOUT_RESET_FLAG					flags2,1	; main
				
; define flags3, flags3 is saved in EEPROM


; define flags4 these need to be preserved in interrupt
#define FIRST_ZERO_SUPPRESSED_FLAG		  	flags4,0	; int and main printing
#define DIVIDE_BY_TEN_FLAG					flags4,1	; int and main printing
#define	DIVIDE_BY_HUNDRED_FLAG				flags4,2	; int and main printing
#define	DIVIDE_BY_THOUSAND_FLAG				flags4,3	; int and main printing
#define	DIVIDE_BY_TEN_THOUSAND_FLAG			flags4,4	; int and main printing
;#define HAVE_DROPPED_FLAG					flags4,5	; main

; define flags5
#define NEGATIVE_INPUT_SIGN_FLAG			flags5,0	; int number processing
#define HAVE_DOT_FLAG						flags5,1	; int number processing	
#define VERTICAL_SPEED_LIMIT_FLAG			flags5,2	; int


print macro address
	movlw	UPPER	address
	movwf	temp_u

	movlw	HIGH	address
	movwf	temp_h

	movlw	LOW		address
	movwf	temp_l
	
	call	text_pri

	endm




;div32_by_2 macro
; clr carry 
; rotate right REGB1 into carry
; rotate right REGB0 into carry  
; rotate right REGC1 into carry  
; rotate right REGC0 into carry  
;	bcf		STATUS, C
;	rrcf	REGB1
;	rrcf	REGB0
;	rrcf	REGC1
;	rrcf	REGC0
;	endm

movfw macro source
	movf	source, W
	endm

; macros to save and restore W and status register in interrupt.
save_w_stat macro
	movwf   temp_w
	swapf   STATUS,W
	clrf	STATUS			; extra force bank 0 clears IRP, RP1, RP0
	movwf   temp_s
	movff	AARGB4, iAARGB4
	movff	AARGB3, iAARGB3
	movff	AARGB7, iAARGB7
	movff	AARGB6, iAARGB6
	movff	AARGB5, iAARGB5
	movff	sign, isign
	movff	REGA3, iREGA3
	movff	REGA2, iREGA2
	movff	REGA1, iREGA1
	movff	REGA0, iREGA0
	movff	REGB3, iREGB3 
	movff	REGB2, iREGB2
	movff	REGB1, iREGB1
	movff	REGB0, iREGB0
	movff	REGC3, iREGC3
	movff	REGC2, iREGC2
	movff	REGC1, iREGC1
	movff	REGC0, iREGC0
	movff	MCOUNT, iMCOUNT
	movff	flags4, iflags4
	endm

restore_w_stat macro
	swapf   temp_s,W
	movwf   STATUS
	swapf   temp_w,F
	swapf   temp_w,W
	movff	iAARGB4, AARGB4
	movff	iAARGB3, AARGB3
	movff	iAARGB7, AARGB7
	movff	iAARGB6, AARGB6
	movff	iAARGB5, AARGB5
	movff	isign, sign
	movff	iREGA3, REGA3
	movff	iREGA2, REGA2
	movff	iREGA1, REGA1
	movff	iREGA0, REGA0
	movff	iREGB3, REGB3 
	movff	iREGB2, REGB2
	movff	iREGB1, REGB1
	movff	iREGB0, REGB0
	movff	iREGC3, REGC3
	movff	iREGC2, REGC2
	movff	iREGC1, REGC1
	movff	iREGC0, REGC0
	movff	iMCOUNT, MCOUNT
	movff	iflags4, flags4
	endm


#ifdef OLD_CODE
poke macro register, address
	lfsr	FSR1, address
	movff	register, INDF1
	endm

peek macro address, register
	lfsr	FSR1, address
	movff	INDF1, register
	endm
#endif ; OLD_CODE

; code start
code_start:
	org	0
	bra		reset_entry

; no priorty interrups used, only one interrupt vertor, no interrupt interrupts an other one, and nothing in the main loop.
; first interrupt vector
	org	8
	goto	int_8
;	retfie

	org 0x18
;	goto	int_18
	retfie


; main program start
reset_entry:
; set internal OSC speed, the output on pin 3 (OSC2), if enabled, is this clock / 4. 

; no priority interrupts
	bcf	RCON, IPEN

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
#endif
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

; SPI SDcard related
#define SPI_SI								LATC,2
#define SPI_SO								PORTC,1 				
#define SPI_NOT_CS							LATC,0
#define SPI_SCK								LATB,4

; switches
#define NOT_MMC_PRESENT_SWITCH				PORTA,2	
#define NOT_MODE_CHANGE_BUTTON				PORTB,6

; LEDs
#define RED_LED								LATA,4
#define GREEN_LED							LATA,5

; analog switch control GPS lopp through versus GPS from PIC TX
#define CARD_OUT_SELECT						LATC,7

; drop solenoid on remote LED swicth
#define NOT_REMOTE_LED_SWITCH				LATC,5


; ports init

	clrf	PORTA
	clrf	LATA

	clrf	PORTB
	clrf	LATB

	clrf 	PORTC
	clrf	LATC


; I/O:
; pin  1	Vdd
; pin  2							RA5			o	RED_LED
; pin  3							RA4			o	GREEN_LED
; pin  4	digital in 2 / Vpp		RA3			o	
; pin  5							RC5 	    o	NOT_REMOTE_LED_SWITCH
; pin  6							RC4		    o		
; pin  7							RC3			o	
; pin  8							RC6		    o	
; pin  9							RC7		    o	CARD_OUT_SELECT
; pin 10	TX data serial out		RB7			o	TXD
; pin 11							RB6			i	NOT_MODE_CHANGE_BUTTON
; pin 12	RX serial in			RB5			i	RXD
; pin 13							RB4			o	SPI_SCK
; pin 14							RC2			o	SPI_SI
; pin 15							RC1			i	SPI_SO
; pin 16							RC0			o	SPI_NOT_CS
; pin 17							RA2			i	NOT_MMC_PRESENT_SWITCH
; pin 18	PGC						RA1			o	
; pin 19	PGD	CVREF (DAC)			RA0			i	throttle	default tristate,  VREFCON1, DAC1OE overrides that
; pin 20	Vss


	bcf		TRISA,	5		; RED_LED
	bcf		TRISA,	4		; GREEN_LED
	bcf		TRISA,	3		; Vpp
	bsf		TRISA,	2		; NOT_MMC_PRESENT_SWITCH
	bcf		TRISA,	1		; 
	bsf		TRISA,	0		; throttle default tristate

	bsf		TRISB,	7		; TX serial out
	bsf		TRISB,	6		; NOT_MODE_CHANGE_BUTTON
	bsf		TRISB,	5		; RX serial in
	bcf		TRISB,	4		; SPI_SCK
	
	bcf		TRISC, 7		; CARD_OUT_SELECT
	bcf		TRISC, 6		; 
	bcf		TRISC, 5		; NOT_REMOTE_LED_SWITCH
	bcf		TRISC, 4		; 
	bcf		TRISC, 3		; 
	bcf		TRISC, 2		; SPI_SI
	bsf		TRISC, 1		; SPI_SO
	bcf		TRISC, 0		; SPI_NOT_CS


	clrf 	PORTC
	clrf	LATC

	bcf		WPUA, WPUA2


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
;	bcf		TRISB, TRISB7			; pin 10 (RB7,TX,CK) output	


; baudrate pre select 
	bsf		BAUDCTL, BRG16
	bsf		TXSTA, BRGH
	bcf		TXSTA, SYNC


; load settings
	call	load_settings			; sets flags3, data_record_number3:data_record_number0, gps_rate

	call	calculate_timer0_reload            ; sets timer0_reload_h:timer0_reload_l from gps_speed


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
;								; 	0 = FVR is disabled	
;								; 	1 = FVR is enabled											<------------------------------------------
	bsf	VREFCON0, FVR1EN		; vref on

;								; bit 6   FVR1ST: Fixed Voltage Reference 1 Stable bit
;								; 	0 = FVR is not stable
;								; 	1 = FVR is stable

	
;								; bit 5-4 FVR1S<1:0>: Fixed Voltage Reference 1 Voltage Select bits
;								; 	00 = Reserved, do not use
;								; 	01 = 1.024V (x1)
;								; 	10 = 2.048V (x2)											<------------------------------------------
;								; 	11 = 4.096V (x4)
	bsf	VREFCON0, FVR1S1		; x4
	bsf	VREFCON0, FVR1S0		

; 								; bit 3-0 Unimplemented: Read as '0'



	clrf	VREFCON1
;								; bit 7   D1EN: DAC 1 Enable bit
;								; 	0 = DAC 1 is disabled										
;								; 	1 = DAC 1 is enabled										<------------------------------------------									 
	bsf		VREFCON1, D1EN

;								; bit 6   D1LPS: DAC 1 Low-Power Voltage State Select bit
;								; 	0 = VDAC = DAC1 Negative reference source selected
;								; 	1 = VDAC = DAC1 Positive reference source selected

;								; bit 5   DAC1OE: DAC 1 Voltage Output Enable bit
;								; 	1 = DAC 1 voltage level is also outputed on the RA0/AN0/CVREF/VREF-/C1IN+/INT0/PGD pin
;								; 	0 = DAC 1 voltage level is disconnected from RA0/AN0/CVREF/VREF-/C1IN+/INT0/PGD pin		<------------------------------------------ will be enabled, TRSA0 is input so default pin 19 is tristate
	bsf		VREFCON1, DAC1OE

;								; bit 4   Unimplemented: Read as 

;								; bit 3-2 D1PSS<1:0>: DAC 1 Positive Source Select bits
;								; 	00 = VDD
;								; 	01 = VREF+		
;								; 	10 = FVR output												<------------------------------------------
;								; 	11 = Reserved, do not use
	bsf		VREFCON1, D1PSS1
	bcf		VREFCON1, D1PSS0

;								; bit 1   Unimplemented: Read as 

;								; bit 0   D1NSS: DAC1 Negative Source Select bits
;								; 	0 = VSS														<------------------------------------------
;								; 	1 = VREF-
	bcf		VREFCON1, D1NSS


; analog out
;	clrf	VREFCON2
;								; bit 7-5      Unimplemented: Read as 

;								; bit 4-0      DAC1R<4:0>: DAC1 Voltage Output Select bits
;								; 	VOUT = ((VSOURCE+) - (VSOURCE-))*(DAC1R<4:0>/(2^5)) + VSOURCE-
;								; 	Note 1: The output select bits are always right justified to ensure that any number of bits can be used without affecting the register layout.

	movlw	DAC_MID_RANGE
	movwf	VREFCON2


; This code block configures the ADC for polling, Vdd reference, Frc clock and AN0 input.
; Conversion start & polling for completion are included.

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
	bcf	ANSEL, 0			; ASN 0
	bcf	ANSEL, 1			; ASN 1
	bcf	ANSEL, 2			; ASN 2
	bcf	ANSEL, 3			; ASN 3
	bsf	ANSEL, 4			; ASN 4
	bcf	ANSEL, 5			; ASN 5
	bcf	ANSEL, 6			; ASN 6
	bcf	ANSEL, 7			; ASN 7

	bcf	ANSELH, 0			; ASN 8
	bcf	ANSELH, 1			; ASN 9
	bcf	ANSELH, 2			; ASN 10
	bcf	ANSELH, 3			; ASN 11	UART RX
							; only lower 4 bits used	

; reload for 256 prescaler =  65535 - ( (16000000 / 256)  / f) = 53035 for 5 hz
; timer0
; default 0xff
; WAS TEST								; bit 7 TMR0ON timer0 on / off
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

; this was set by load_settings
	movff	timer0_reload_h, TMR0H	
	movff	timer0_reload_l, TMR0L

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
;	movwf	timer1_reload_h
;	movlw	LOW		TIMER1_PRELOAD_4000Hz
;	movwf	TMR1L
;	movwf	timer1_reload_l

; reset timer 1 interrupt flag	
	bcf		PIR1,	TMR1IF



; configure comparators

; comparator 1 init for GM pulse sense

; CM1CON0
;	 R/W-0  R-0  R/W-0 R/W-0 R/W-0 R/W-0 R/W-0 R/W-0
;	 C1ON  C1OUT C1OE  C1POL C1SP   C1R  C1CH1 C1CH0
; bit 7											  bit 0


;							bit 7		C1ON: Comparator C1 Enable bit
;								1 = Comparator C1 is enabled
;								0 = Comparator C1 is disabled
	bcf		CM1CON0,  C1ON

;							bit 6		C1OUT: Comparator C1 Output bit
;								If C1POL = 1 (inverted polarity):
;									C1OUT = 0 when C1VIN+ > C1VIN-
;									C1OUT = 1 when C1VIN+ < C1VIN-
;								If C1POL = 0 (non-inverted polarity):
;									C1OUT = 1 when C1VIN+ > C1VIN-
;									C1OUT = 0 when C1VIN+ < C1VIN-
; read only

;							bit 5		C1OE: Comparator C1 Output Enable bit
;								1 = C1OUT is present on the C1OUT pin(1)
;								0 = C1OUT is internal only							<------------------------------------------
	bcf		CM1CON0, C1OE

;							bit 4		C1POL: Comparator C1 Output Polarity Select bit
;								1 = C1OUT logic is inverted
;								0 = C1OUT logic is not inverted						<------------------------------------------
	bcf		CM1CON0, C1POL

;							bit 3		C1SP: Comparator C1 Speed/Power Select bit
;								1 = C1 operates in normal power, higher speed mode	<------------------------------------------
;								0 = C1 operates in low-power, low-speed mode
	bsf		CM1CON0, C1SP

;							bit 2		C1R: Comparator C1 Reference Select bit (non-inverting input)
;								1 = C1VIN+ connects to C1VREF output				<------------------------------------------
;								0 = C1VIN+ connects to C12IN+ pin
	bsf 	CM1CON0, C1R

;							bit 1-0	 C1CH<1:0>: Comparator C1 Channel Select bit
;								00 = C12IN0- pin of C1 connects to C1VIN-
;								01 = C12IN1- pin of C1 connects to C1VIN-			<------------------------------------------
;								10 = C12IN2- pin of C1 connects to C1VIN-
;								11 = C12IN3- pin of C1 connects to C1VIN-
	bcf		CM1CON0, C1CH1
	bsf		CM1CON0, C1CH0

; Note 1: Comparator output requires the following three conditions: C1OE = 1, C1ON = 1 and corresponding port TRIS bit = 0.


; There is no CM1CON1



; comparator 2 voltage sense for +500 V
; comparator 2 init for auto shutdown and auto restart.

; CM2CON0
;	 R/W-0  R-0  R/W-0 R/W-0 R/W-0 R/W-0 R/W-0 R/W-0
;	 C2ON  C2OUT C2OE  C2POL C2SP   C2R  C2CH1 C2CH0
; bit 7											  bit 0

;						bit 7		C2ON: Comparator C2 Enable bit
;							1 = Comparator C2 is enabled
;							0 = Comparator C2 is disabled
	bcf		CM2CON0, C2ON 

;						bit 6		C2OUT: Comparator C2 Output bit
;							If C2POL = 1 (inverted polarity):
;								C2OUT = 0 when C2VIN+ > C2VIN-
;								C2OUT = 1 when C2VIN+ < C2VIN-
;							If C2POL = 0 (non-inverted polarity):
;								C2OUT = 1 when C2VIN+ > C2VIN-
;								C2OUT = 0 when C2VIN+ < C2VIN-
; read only

;						bit 5		C2OE: Comparator C2 Output Enable bit
;							1 = C2OUT is present on C2OUT pin(1) 
;							0 = C2OUT is internal only									<------------------------------------------
	bcf		CM2CON0, C2OE 


;						bit 4		C2POL: Comparator C2 Output Polarity Select bit
;							1 = C2OUT logic is inverted									<------------------------------------------
;							0 = C2OUT logic is not inverted
	bsf		CM2CON0, C2POL


;						bit 3		C2SP: Comparator C2 Speed/Power Select bit
;							1 = C2 operates in normal power, higher speed mode			<------------------------------------------
;							0 = C2 operates in low-power, low-speed mode
	bsf		CM2CON0, C2SP


;						bit 2		C2R: Comparator C2 Reference Select bits (non-inverting input)
;							1 = C2VIN+ connects to C2VREF								<------------------------------------------
;							0 = C2VIN+ connects to C2IN+ pin
	bsf		CM2CON0, C2R


;						bit 1-0	 C2CH<1:0>: Comparator C2 Channel Select bits
;							00 = C12IN0- pin of C2 connects to C2VIN-
;							01 = C12IN1- pin of C2 connects to C2VIN-
;							10 = C12IN2- pin of C2 connects to C2VIN-					<------------------------------------------
;							11 = C12IN3- pin of C2 connects to C2VIN-
	bsf		CM2CON0, C2CH1
	bcf		CM2CON0, C2CH0

; Note 1: Comparator output requires the following three conditions: C2OE = 1, C2ON = 1 and corresponding port TRIS bit = 0.


; comparator 1 and comparator2 reference and sync select
; CM2CON1
;	  R-0   R-0   R/W-0  R/W-0 R/W-0 R/W-0  R/W-0  R/W-0
;   MC1OUT MC2OUT C1RSEL C2RSEL C1HYS C2HYS C1SYNC C2SYNC
; bit 7												  bit 0

;  Defaults zero
;						bit 7 MC1OUT: Mirror Copy of C1OUT bit

;						bit 6 MC2OUT: Mirror Copy of C2OUT bit

;						bit 5 C1RSEL: Comparator C1 Reference Select bit
;							1 = FVR routed to C1VREF input
;							0 = CVREF routed to C1VREF input						<------------------------------------------
	bcf		CM2CON1, C1RSEL

;						bit 4 C2RSEL: Comparator C2 Reference Select bit
;							1 = FVR routed to C2VREF input							
;							0 = CVREF routed to C2VREF input						<------------------------------------------
	bcf		CM2CON1, C2RSEL		; set by DAC

;						bit 3 C1HYS: Comparator C1 Hysteresis Enable bit
;							1 = Comparator C1 hysteresis enabled
;							0 = Comparator C1 hysteresis disabled					<------------------------------------------
	bcf		CM2CON1, C1HYS

;						bit 2 C2HYS: Comparator C2 Hysteresis Enable bit
;							1 = Comparator C2 hysteresis enabled
;							0 = Comparator C2 hysteresis disabled					<------------------------------------------
	bcf		CM2CON1, C2HYS

;						bit 1 C1SYNC: C1 Output Synchronous Mode bit
;							1 = C1 output is synchronous to rising edge to TMR1 clock
;							0 = C1 output is asynchronous							<------------------------------------------
	bcf		CM2CON1, C1SYNC


;						bit 0 C2SYNC: C2 Output Synchronous Mode bit
;							1 = C2 output is synchronous to rising edge to TMR1 clock
;							0 = C2 output is asynchronous							<------------------------------------------
	bcf		CM2CON1, C2SYNC



; interrupt enable
	clrf	PIE1
;;  bsf     PIE1, RCIE		; USART receive interrupt enable
;   bsf     PIE1, ADIE		; AD converter interrupt enable
; WAS TEST
;	bsf     PIE1, TMR1IE	; timer 1 overflow interrupt enable
;   bsf     PIE1, TXIE
;   bsf     IPR1, RCIP		; bit 5 RCIP: EUSART Receive Interrupt Priority bit   1 = High priority 0 = Low priority
;   bsf     IPR1, TMR1IP	; bit 0 TMR1IP: TMR1 Overflow Interrupt Priority bit   1 = High priority  0 = Low priority


; PIE2: OSFIE C2IE C1IE EEIE -- -- -- --
	bcf	PIE2,	EEIE			; EEPROM write complete interrupt enable
	bcf	PIE2,	C1IE			; comparator1  interrupt enable

; flags1 to zero
	clrf	flags1

; flags2							; some flags2 are preserved over reset

; flags3 are saved in EEPROM

; flags4 to zero
	clrf	flags4

; flags5 to zero
	clrf	flags5

; numeric input to zero
	clrf	value3
	clrf	value2
	clrf	value1
	clrf	value0
	bcf		NEGATIVE_INPUT_SIGN_FLAG

; debounce
	clrf	debounce_counter

	movlw	MAX_V_SPEED_DM_PER_400ms
	movwf	v_speed_limit_dm_per_400_ms

; commands
	clrf	command_flags


main:

#ifdef OLD_CODE
; report any watchdog reset
test_watchdog_reset:
	btfss	WATCHDOG_RESET_FLAG
	bra		end_test_watchdog_reset		
	print	text_watchdog
	bcf		WATCHDOG_RESET_FLAG
end_test_watchdog_reset:

; report any brownout reset
test_brownout_reset:
	btfss	BROWNOUT_RESET_FLAG
	bra		end_test_brownout_reset
	print	text_brownout
	bcf		BROWNOUT_RESET_FLAG
end_test_brownout_reset:
#endif OLD_CODE

	call	spi_ini

; if no card no init
	btfss	NOT_MMC_PRESENT_SWITCH
	call	mmc_init



; baudrate selection			; enables serial port
	movlw	D'6'					; select 57600 Bd
	call	select_baudrate			; uses W


; read card sector 0, this seems to give error E 3 if first read, but that now is not reported becaue serial port is not enabled yet
; BUG FIX
; As this first read of sector 0 gives error E3, and no data output, preventing error message to serial link interfering with Hubsan H501S remote GPS
	bsf		DISABLE_ERROR3_REPORTING_FLAG
	clrf	REGA3
	clrf	REGA2
	clrf	REGA1
	clrf	REGA0
	call	mmc_read_sector
	bcf		DISABLE_ERROR3_REPORTING_FLAG


; enable gobal and peripheral interrupt
	clrf	INTCON
	bsf		INTCON,	PEIE			; peripheral interrupts enable
	bsf		INTCON, TMR0IE			; timer0

	bsf		INTCON,	GIE				; global interrupt enable

;	bsf		INTCON, RABIE			; 
;	bsf		IOCB, IOCB6				; enable RB6 mode change button interrupt on change
;	bsf		IOCA, IOCA2				; RA2 mode change card reader card detect

	
; CVref out RA0 to tristate
; disconnect DAC from pin 19 CVref
	bcf     VREFCON1, DAC1OE

; set DAC voltage to mid range
	movlw	DAC_MID_RANGE
	movwf	VREFCON2

#ifdef OLD_CODE
; say Panteltje
	movlw	'P'
	call	tx_w

	movlw	'a'
	call	tx_w

	movlw	'n'
	call	tx_w

	movlw	't'
	call	tx_w

	movlw	'e'
	call	tx_w

	movlw	'l'
	call	tx_w

	movlw	't'
	call	tx_w

	movlw	'j'
	call	tx_w

	movlw	'e'
	call	tx_w

	movlw	' '
	call	tx_w

	movlw	'g'
	call	tx_w

	movlw	'p'
	call	tx_w

	movlw	's'
	call	tx_w

	movlw	's'
	call	tx_w

	movlw	'-'
	call	tx_w

	movlw	'0'
	call	tx_w

	movlw	'.'
	call	tx_w

	movlw	'1'
	call	tx_w

	call	tx_crlf
#endif ; OLD_CODE

; init altitude AFC_RX related
	clrf	current_altitude_decimeters_h
	clrf	current_altitude_decimeters_l
	clrf	previous_current_altitude_decimeters_h
	clrf	previous_current_altitude_decimeters_l
	clrf	requested_altitude_decimeters_h
	clrf	requested_altitude_decimeters_l
	clrf	altitude_mode
	clrf	checksum

; by default echo GPS
	bsf		ECHO_GPS_FLAG

; release LED switch
	bsf		NOT_REMOTE_LED_SWITCH

main_loop:
; activate watchdog
	clrwdt

#ifdef MATH_TEST1
	clrf	REGA3
	clrf	REGA2
	movlw	HIGH	D'1234'
	movwf	REGA1
	movlw	LOW		D'1234'
	movwf	REGA0

	call	negatea

	movfw	REGA3
	call	print_w_hex
	movlw	' '
	call	tx_w

	movfw	REGA2
	call	print_w_hex
	movlw	' '
	call	tx_w

	movfw	REGA1
	call	print_w_hex
	movlw	' '
	call	tx_w

	movfw	REGA0
	call	print_w_hex

	call	tx_crlf
	bra		main_loop
#endif ; MATH_TEST1
	


#ifdef DEBUG
; WAS TEST
;	bra		main_loop

;	movlw	'M'
;	call	tx_w
;	call	tx_crlf
#endif ; DEBUG


main_test_card_present:
	btfss	NOT_MMC_PRESENT_SWITCH
	bra		main_end_test_card_present
; no card
	bcf		CARD_OUT_SELECT
; Basically if no card release control to user
; release throttle, DAC no longer to pin 19, pin 19 is default tristate (input)
	bcf		VREFCON1, DAC1OE	
; DAC midrange 
	movlw	DAC_MID_RANGE
	movwf	VREFCON2
	print	text_no_mmc_present
	bsf		GREEN_LED
	bcf		RED_LED
; releasse LED switch
	bsf		NOT_REMOTE_LED_SWITCH
	bra		main_loop
main_end_test_card_present:


main_test_output_gps:
	btfss	ECHO_GPS_FLAG
	bra		main_output_card
; mode output GPS
	bcf		CARD_OUT_SELECT
	bcf		RED_LED
	bsf		GREEN_LED
; release throttle, DAC no longer to pin 19, pin 19 is default tristate (input)
	bcf		VREFCON1, DAC1OE	
; DAC midrange 
	movlw	DAC_MID_RANGE
	movwf	VREFCON2
; reset data_record_number to zero
	clrf	data_record_number3
	clrf	data_record_number2
	clrf	data_record_number1
	clrf	data_record_number0
	bcf		END_OF_DATA_FLAG
	bra		main_loop

main_output_card:
; VREFCON1, DAC1OE unchanged!
	bsf		CARD_OUT_SELECT
; look for end of data in card
	btfss	END_OF_DATA_FLAG
	bra		test_time	
; end of data, reset output_mode to GPS
	bsf		ECHO_GPS_FLAG
	bra		main_loop

test_time:
; test for 200 ms timer flag if 5 Hz
	btfss	TIME_FLAG
	bra		main_loop

; WAS TEST
;	movlw	'D'
;	call	tx_w
;	movlw	' '
;	call	tx_w
;	movfw	VREFCON2
;	call	print_w_ascii_dec
;	call	tx_crlf


; time to output data

; toggle red LED
	btg		RED_LED

main_read_sector:
	movff	data_record_number3, REGA3
	movff	data_record_number2, REGA2
	movff	data_record_number1, REGA1
	movff	data_record_number0, REGA0
	call	mmc_read_sector                                    ; reads a 512 Byte block from the MMC, sector number in REGA3:REGA0, output to serial link, sets END_OF_DATA_FLAG and returns if any zero byte found

; while we are not at requested altitude, adjust throttle and repeat same sector.
; compare requested_altitude_decimeters_h:requested_altitude_decimeters_l  to  current_altitude_decimeters_h:current_altitude_decimeters_l

	bcf		TIME_FLAG

#ifdef DEBUG
; WAS TEST
; report requested altitude and command_flags
	call	tx_crlf
	movlw	'R'
	call	tx_w
	movlw	'A'
	call	tx_w
	movlw	' '
	call	tx_w
 	clrf	REGA3
	clrf	REGA2
	movff	requested_altitude_decimeters_h, REGA1
	movff	requested_altitude_decimeters_l, REGA0
	bsf		DIVIDE_BY_TEN_FLAG
	call	print_32_ascii_dec
	bcf		DIVIDE_BY_TEN_FLAG
	call	tx_crlf
	
	movlw	'C'
	call	tx_w
	movlw	'O'
	call	tx_w
	movlw	' ' 
	call	tx_w
	movfw	command_flags
	call	print_w_ascii_dec	
	call	tx_crlf
	call	tx_crlf
#endif ; DEBUG



; test if LED switch needs activated or released, we are here every 200 mS, (5 Hz GPS)
	btfss	command_flags, LED_SWITCH_FLAG	; flag is set in sectors when LED switch needs to be pushed down
	bra		release_led_switch
push_led_switch:
	bcf		NOT_REMOTE_LED_SWITCH
	bra		end_test_led_switch_command

release_led_switch:
	bsf		NOT_REMOTE_LED_SWITCH
end_test_led_switch_command:



; if abs(requested_altitude_decimeters_h:requested_altitude_decimeters_l  to  current_altitude_decimeters_h:current_altitude_decimeters_l ) then goto main_read_sector
	clrf	REGA3
	clrf	REGA2
	movff	requested_altitude_decimeters_h, REGA1
	movff	requested_altitude_decimeters_l, REGA0
; REGA3:REGA0 = requested_altitude_decimeters_h:requested_altitude_decimeters_l

	clrf	REGB3
	clrf	REGB2
; signal to interrupt routine not to change these bytes one at the time 
	bsf		WAIT_UPDATING_ALTITUDE_FLAG
	movff	current_altitude_decimeters_h, REGB1
	movff	current_altitude_decimeters_l, REGB0
	bcf		WAIT_UPDATING_ALTITUDE_FLAG
; REGB3:REGB0 = current_altitude_decimeters_h:current_altitude_decimeters_l

	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = requested_altitude_decimeters_h:requested_altitude_decimeters_l - current_altitude_decimeters_h:current_altitude_decimeters_l  = correction factor
; this is positive if too low, then go up
; this is negative if too high, then go down

;  remember sign bit in UP_FLAG


main_positive_alitude:
	bsf		UP_FLAG
	btfsc	REGA3, 7
	bcf		UP_FLAG

; if negative altitude things reverse, yes GPS gives negative altitude sometimes
	btfss	NEGATIVE_ALTITUDE_FLAG
	bra		main_positive_altitude
; how to handle GPS negative altitude, well what do you say, we go into hold tristate DAC out pin 19. and hold position sector 
	movlw	D'16'
	movwf	VREFCON2
; DAC out pin 19 tristate, HOLD 
	bcf		VREFCON1, DAC1OE	
	bra	main_loop
main_positive_altitude:


; get ABS difference in REGA3:REGA0
	call	absa			; checks sign of REGA and convert negative to positive
; REGA3:REGA0 = abs(requested_altitude_decimeters_h:requested_altitude_decimeters_l - current_altitude_decimeters_h:current_altitude_decimeters_l)

	movff	REGA0, pa_error

; check if altitude error < 1.5 m
	clrf	REGB3
	clrf	REGB2
	clrf	REGB1
	movlw	D'15'	; 15 dm, 1.5 m
	movwf	REGB0
; REGB3:REGB0 = 15

	call    subtract            ; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = abs(requested_altitude_decimeters_h:requested_altitude_decimeters_l - current_altitude_decimeters_h:current_altitude_decimeters_l) - 15

; REGA is SIGNED!

; test if in proportional range
; this is negative for abs altitude error < 15
	btfss	REGA3, 7
	bra		altitude_error_out_of_proportional_range


; WAS TEST
;	movff	pa_error, ttemp
;	call	tx_crlf
;	movfw	ttemp
;	call	print_w_ascii_dec
;	movlw	' '
;	call	tx_w
;	movlw	'1'
;	btfss	UP_FLAG
;	movlw	'0'
;	call	tx_w
;	call	tx_crlf
;	movff	ttemp, REGA0


; To create a proportional band
;	if the error is smaller than 15
;		if going up
;			add the difference to the DAC + 16 (makes range 16 to 31)
;		else
;			sutract the difference from the DAC + 16 (makes range 16 to 1) 
;   else
;		if going up
;			set DAC to maximum (<= 31)
;		else
;			set DAC to minimum (>= 0)


; note now always DAC out pin 19 enabled!!, throttle on remote control can still override due to series resistors.


; test if up or down required
	btfss	UP_FLAG
	bra		proportional_down


; proportional up
; add REGA0 (for sure < 16) to DAC 16
	movlw	D'16'
	addwf	pa_error
	movff	pa_error, VREFCON2
; DAC to pin 19
	bsf		VREFCON1, DAC1OE	
; do not wait till altitude reached, close enough, 1.6 meter either way, correct while moving to next sector / point
	bra		main_test_end_of_data
;	bra		main_loop

proportional_down:
; subract REGA0 (for sure < 16) from DAC 16	
	movff	pa_error, WREG
	sublw	D'16'
	movwf	VREFCON2
; DAC to pin 19
	bsf		VREFCON1, DAC1OE	
; do not wait till altitude reached, close enough, 1.6 meter either way, correct while moving to next sector / point
	bra		main_test_end_of_data
;	bra		main_loop


altitude_error_out_of_proportional_range:
; test if need to go up or down
; flashing red and green indicates trying to get to specified altitude
	btg		GREEN_LED

	btfss	UP_FLAG
	bra		max_down
; max up
; set DAC to DAC_MAX_RANGE
	movlw	DAC_MAX_RANGE
	movwf	VREFCON2
; DAC to pin 19
	bsf		VREFCON1, DAC1OE	
; wait till altitude reached
	bra		main_loop

max_down:
; set DAC to DAC_MIN_RANGE
	movlw	DAC_MIN_RANGE
    movwf	VREFCON2
; DAC to pin 19
	bsf		VREFCON1, DAC1OE	
; wait till altitude reached
	bra		main_loop	


main_test_end_of_data:
	btfss	END_OF_DATA_FLAG
	bra		increment_data_record_number
; all GPS data send from card, go back to normal gps mode, with END_OF_DATA_FLAG set
	bsf		ECHO_GPS_FLAG
	bra		main_loop

increment_data_record_number:
; 32 bit increment
	incf    data_record_number0, F
	btfsc   STATUS ,Z
	incf    data_record_number1, F
	btfsc   STATUS, Z
	incf    data_record_number2, F
	btfsc   STATUS, Z
	incf    data_record_number3, F

	bra		main_loop





; ***************** subroutines ********************


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
;	call	print_id

	return



select_baudrate:
	movwf	temp3
	call	disable_serial_port

; compare 
	movlw	D'1'
	subwf	temp3, W
	btfsc	STATUS, Z
	goto	set_115200bd

	movlw	D'2'
	subwf	temp3, W
	btfsc	STATUS, Z
	goto	set_19200bd

	movlw	D'3'
	subwf	temp3, W
	btfsc	STATUS, Z
	goto	set_9600bd

	movlw	D'4'
	subwf	temp3, W
	btfsc	STATUS, Z
	goto	set_4800bd

	movlw	D'5'
	subwf	temp3, W
	btfsc	STATUS, Z
	goto	set_1200bd

	movlw	D'6'
	subwf	temp3, W
	btfsc	STATUS, Z
	goto	set_57600bd

; if illegal value use default
;	goto	select_baudrate_end
	goto	set_115200bd
	

set_115200bd:
	movlw	D'1'
	movwf	BRG16

	movlw	HIGH	D'137'
	movwf	SPBRGH

	movlw	LOW		D'137'
	movwf	SPBRG

	goto	select_baudrate_end

set_19200bd:
;	bcf     TXSTA, SYNC
	movlw	D'1'
	movwf	BRG16

	movlw	HIGH	D'832'
	movwf	SPBRGH
	
	movlw	LOW		D'832'
	movwf	SPBRG

	goto	select_baudrate_end

set_9600bd:
	movlw	HIGH	D'1658'
	movwf	SPBRGH

	movlw	LOW		D'1658'
	movwf	SPBRG

	goto	select_baudrate_end

set_4800bd:
	movlw	HIGH	D'3316'
	movwf	SPBRGH

	movlw	LOW		D'3316'
	movwf	SPBRG

	goto	select_baudrate_end

set_1200bd: ; SPBRGH:SPBRG=9999 BRG16=1 for 48 MHz  -> 9999 * 64/48 = 13332
	movlw	D'1'
	movwf	BRG16

	movlw	HIGH	D'13332'
	movwf	SPBRGH

	movlw	LOW		D'13332'
	movwf	SPBRG
	goto	select_baudrate_end

set_57600bd:
	movlw	D'1'
	movwf	BRG16

; see 18F1XK22_datasheet_41365C.pdf page 190
; (64 / 48) * 207 = 276
	movlw	HIGH	D'276'
	movwf	SPBRGH
	
	movlw	LOW		D'276'
	movwf	SPBRG

select_baudrate_end:
	call	enable_serial_port


	return



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



print_status:
	call	tx_crlf

; data record pointer
	print	text_record_number
	movff	data_record_number3, REGA3
	movff	data_record_number2, REGA2
	movff	data_record_number1, REGA1
	movff	data_record_number0, REGA0
	call	print_32_ascii_dec
	call	tx_crlf

; MMC present
	btfss	NOT_MMC_PRESENT_SWITCH
	bra		say_have_card
; no card
	print	text_no_mmc_present
	bra		say_card_done
say_have_card:
	print 	text_mmc_present
say_card_done:

; MMC init
	btfss	MMC_INIT_OK_FLAG
	bra		say_mmc_init_failed
; init ok
	print	text_mmc_init_ok	
	bra		say_mmc_init_end
say_mmc_init_failed:
	print	text_mmc_init_failed
say_mmc_init_end:

; GPS rate
	print	text_gps_rate
	movfw	gps_rate
	call	print_w_ascii_dec
	movlw	' '
	call	tx_w
	movlw	'H'
	call	tx_w
	movlw	'z'
	call	tx_w

	call	tx_crlf

; DAC
	print	text_dac
	movfw	VREFCON2
	call	print_w_ascii_dec
	movlw	' '
	call	tx_w
; pin state. 1 if DAC to pin 19, 0 if tristate, RA0 is then configured as input, setting VREFCON1, DAC1OE overrides that
	movlw	'1'	
	btfss		VREFCON1, DAC1OE
	movlw	'0'
	call	tx_w
	call	tx_crlf

; memory range
;	btfss	MEMORY_FULL_FLAG
;	bra		print_status_end_24LC1025_full_test
;	print	text_memory_full
;	call	tx_crlf
;print_status_end_24LC1025_full_test:

	return											; end subroutine print_status





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


eeprom_write:						; data in EEDATA, address in EEADR
; save value of data written
	movff	EEDATA, temp4
	clrf	EEADRH

	bcf		EECON1, EEPGD			; Point to DATA memory
	bcf		EECON1, CFGS			; Access EEPROM
	bsf		EECON1, WREN			; Enable writes
;	bcf		INTCON, GIE				; Disable Interrupts

	movlw	0x55
	movwf	EECON2					; Write 55h

	movlw	0xAA
	movwf	EECON2					; Write 0AAh

	bsf		EECON1, WR				; Set WR bit to begin write
;	bsf		INTCON, GIE				; Enable Interrupts

; wait until WR bit zero to indicate write complete
eeprom_poll_write_complete_loop:
    btfsc   EECON1, WR
    bra	    eeprom_poll_write_complete_loop

	bcf		EECON1, WREN				; Disable writes on write complete (EEIF set)
	
; eeprom_verify
	call	eeprom_read			; address in EEADR, returns data in EEDATA
	movfw	EEDATA
	cpfseq	temp4	
	bra		eeprom_write_error

; return OK
	return

eeprom_write_error:

; eeprom write result
	print	text_eeprom_write_result
; address=nnn	
	print	text_address
	movfw	EEADR
	call	print_w_ascii_dec
	movlw	' '
	call	tx_w
; original=nnn
	print	text_original
	movfw	temp4
	call	print_w_ascii_dec
	movlw	' '
; read=nnn
	call	tx_w
	movlw	'r'
	call	tx_w
	movlw	'e'
	call	tx_w
	movlw	'a'
	call	tx_w
	movlw	'd'
	call	tx_w
	movlw	'='
	call	tx_w
; read eeprom	
	movfw	EEDATA
	call	print_w_ascii_dec	
	call	tx_crlf
	return


eeprom_read:					; address in EEADR, returns data in EEDATA
	bcf		EECON1, EEPGD 		; Point to DATA memory
	bcf		EECON1, CFGS  		; Access EEPROM
	bsf		EECON1, RD    		; EEPROM Read
	return


save_settings:
;	movlw   'S'
;	call    tx_w
;	call    tx_crlf

	movlw	D'0'
	movwf	EEADR
    movff   flags3, EEDATA
    call    eeprom_write    

	movlw	D'1'
	movwf	EEADR
    movff   data_record_number3, EEDATA
    call    eeprom_write    

	movlw	D'2'
	movwf	EEADR
    movff   data_record_number2, EEDATA
    call    eeprom_write    

	movlw	D'3'
	movwf	EEADR
    movff   data_record_number1, EEDATA
    call    eeprom_write    

	movlw	D'4'
	movwf	EEADR
    movff   data_record_number0, EEDATA
    call    eeprom_write    

	movlw	D'5'
	movwf	EEADR
	movff	gps_rate, EEDATA
	call	eeprom_write

; mark EEPROM writtem
	movlw	D'123'
	movwf	EEDATA
	movlw	D'255'
	movwf	EEADR
	call	eeprom_write

 	return					; 



load_settings:
; this will load settings from EEPROM.
; To see if any value was actually programmed in EEPROM, first EEPROM address 255 is read.
; if EEPROM address 255 reads 255, then nothing was programmed, defaults are then written to EEPROM,
; and the value 123 is written to EEPROM address 255 as a marker that it is programmed. 

; test if EEPROM was programmed
	movlw	D'255'
	movwf	EEADR
	call	eeprom_read					; address in EEADR, returns data in EEDATA
	movlw	D'123'
	cpfseq	EEDATA
	bra		eemprom_program_defaults
	bra		eeprom_is_programmed	

eemprom_program_defaults:
; program default values in eeprom

; set defaults
	clrf	flags3

	clrf	data_record_number3
	clrf	data_record_number2
	clrf	data_record_number1
	clrf	data_record_number0


	movlw	DEFAULT_GPS_RATE
	movwf	gps_rate

	call	save_settings

	return


eeprom_is_programmed:

;	movlw	'P'
;	call	tx_w
;	call	tx_crlf

	movlw	D'0'
	movwf	EEADR
	call	eeprom_read					; address in EEADR, returns data in EEDATA
	movff	EEDATA, flags3

	movlw	D'1'
	movwf	EEADR
	call	eeprom_read					; address in EEADR, returns data in EEDATA
	movff	EEDATA, data_record_number3

	movlw	D'2'
	movwf	EEADR
	call	eeprom_read					; address in EEADR, returns data in EEDATA
	movff	EEDATA, data_record_number2

	movlw	D'3'
	movwf	EEADR
	call	eeprom_read					; address in EEADR, returns data in EEDATA
	movff	EEDATA, data_record_number1

	movlw	D'4'
	movwf	EEADR
	call	eeprom_read					; address in EEADR, returns data in EEDATA
	movff	EEDATA, data_record_number0

	movlw	D'5'
	movwf	EEADR
	call	eeprom_read					; address in EEADR, returns data in EEDATA
	movff	EEDATA, gps_rate

	return



; math start

; *** 32 BIT SIGNED SUBTRACT ***
subtract:			; REGA - REGB -> REGA return carry set if overflow
	call	negateb		; Negate REGB
	skpnc
	return			; Overflow


; *** 32 BIT SIGNED ADD ***
add:				; REGA + REGB -> REGA return carry set if overflow
	movf	REGA3,w		; Compare signs
	xorwf	REGB3,w
	movwf	sign

	call	addba		; Add REGB to REGA

	clrc				; Check signs
	movf	REGB3,w		; If signs are same
	xorwf	REGA3,w		; so must result sign
	btfss	sign,7		; else overflow
	addlw	0x80
	return


; *** 32 BIT SIGNED MULTIPLY ***
multiply:			; REGA * REGB -> REGA return carry set if overflow 
	clrf	sign		; Reset sign flag
	call	absa		; Make REGA positive
	skpc
	call	absb		; Make REGB positive
	skpnc
	return				; Overflow

	call	movac		; Move REGA to REGC
	call	clra		; Clear product

	movlw	D'31'		; Loop counter
	movwf	MCOUNT

muloop:
	call	slac		; Shift left product and multiplicand
	
	rlcf	REGC3,w		; Test MSB of multiplicand
	skpnc				; If multiplicand bit is a 1 then
	call	addba		; add multiplier to product

	skpc				; Check for overflow
	rlcf	REGA3,w
	skpnc
	return

	decfsz	MCOUNT,f	; Next
	bra		muloop

	btfsc	sign,0		; Check result sign
	call	negatea		; Negative
	return


; *** 32 BIT SIGNED DIVIDE ***
divide:				; REGA / REGB -> REGA return carry set if overflow or division by zero
	clrf	sign		; Reset sign flag
	movf	REGB0,w		; Trap division by zero
	iorwf	REGB1,w
	iorwf	REGB2,w
	iorwf	REGB3,w
	sublw	0
	skpc
	call	absa		; Make dividend (REGA) positive
	skpc
	call	absb		; Make divisor (REGB) positive
	skpnc
	return			; Overflow

	clrf	REGC0		; Clear remainder
	clrf	REGC1
	clrf	REGC2
	clrf	REGC3
	call	slac		; Purge sign bit

	movlw	D'31'		; Loop counter
	movwf	MCOUNT

dvloop:
	call	slac		; Shift dividend (REGA) msb into remainder (REGC)

	movf	REGB3,w		; Test if remainder (REGC) >= divisor (REGB)
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
	skpc				; Carry set if remainder >= divisor
	bra		dremlt

	movf	REGB0,w		; Subtract divisor (REGB) from remainder (REGC)
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
	bsf	REGA0,0			; Set quotient bit

dremlt:
	decfsz	MCOUNT,f	; Next
	bra		dvloop

	btfsc	sign,0		; Check result sign
	call	negatea		; Negative
	return

; *** ROUND RESULT OF DIVISION TO NEAREST INTEGER ***

round:
	clrf	sign		; Reset sign flag
	call	absa		; Make positive
	clrc
	call	slc			; Multiply remainder by 2
	movf	REGB3,w		; Test if remainder (REGC) >= divisor (REGB)
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
	skpc				; Carry set if remainder >= divisor
	bra		rremlt
	incfsz	REGA0,f		; Add 1 to quotient
	bra		rremlt
	incfsz	REGA1,f
	bra		rremlt
	incfsz	REGA2,f
	bra		rremlt
	incf	REGA3,f
	skpnz
	return			; Overflow, return carry set
rremlt:
	btfsc	sign,0		; Restore sign
	call	negatea
	return


#ifdef USE_SQRT
; *** 32 BIT SQUARE ROOT ***
sqrt:					; sqrt(REGA) -> REGA  Return carry set if negative
	rlf	REGA3,w			; Trap negative values
	skpnc
	return

	call	movac		; Move REGA to REGC
	call	clrba		; Clear remainder (REGB) and root (REGA)

	movlw	D'16'		; Loop counter
	movwf	MCOUNT

sqloop:
	rlf	REGC0,f			; Shift two msb's
	rlf	REGC1,f			; into remainder
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

	setc				; Add 1 to root
	rlf	REGA0,f			; Align root
	rlf	REGA1,f
	rlf	REGA2,f

	movf	REGA2,w		; Test if remdr (REGB) >= root (REGA)
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
	skpc				; Carry set if remdr >= root
	bra		sremlt

	movf	REGA0,w		; Subtract root (REGA) from remdr (REGB)
	subwf	REGB0,f
	movf	REGA1,w
	skpc
	incfsz	REGA1,w
	subwf	REGB1,f
	movf	REGA2,w
	skpc
	incfsz	REGA2,w
	subwf	REGB2,f
	bsf	REGA0,1			; Set current root bit

sremlt:
	bcf	REGA0,0			; Clear test bit
	decfsz	MCOUNT,f	; Next
	bra		sqloop

	clrc
	rrf	REGA2,f			; Adjust root alignment
	rrf	REGA1,f
	rrf	REGA0,f
	return
#endif ; USE_SQRT


; MATH UTILITY ROUTINES

; Add REGB to REGA (Unsigned)
; Used by add, multiply,
addba:
	movf	REGB0,w		; Add lo byte
	addwf	REGA0,f

	movf	REGB1,w		; Add mid-lo byte
	skpnc				; No carry_in, so just add
	incfsz	REGB1,w		; Add carry_in to REGB
	addwf	REGA1,f		; Add and propagate carry_out

	movf	REGB2,w		; Add mid-hi byte
	skpnc
	incfsz	REGB2,w
	addwf	REGA2,f

	movf	REGB3,w		; Add hi byte
	skpnc
	incfsz	REGB3,w
	addwf	REGA3,f
	return


; Move REGA to REGC
; Used by multiply, sqrt
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


; Clear REGB and REGA
; Used by sqrt
clrba:
	clrf	REGB0
	clrf	REGB1
	clrf	REGB2
	clrf	REGB3

; Clear REGA
; Used by multiply, sqrt
clra:
	clrf	REGA0
	clrf	REGA1
	clrf	REGA2
	clrf	REGA3
	return


; Check sign of REGA and convert negative to positive
; Used by multiply, divide, bin2dec, round
absa:
	rlcf	REGA3,w
	skpc
	return			; Positive

; Negate REGA
; Used by absa, multiply, divide, bin2dec, dec2bin, round
negatea:
	movf	REGA3,w		; Save sign in w
	andlw	0x80

	comf	REGA0,f		; 2's complement
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
	incf	sign,f		; flip sign flag
	addwf	REGA3,w		; Return carry set if -2147483648
	return


; Check sign of REGB and convert negative to positive
; Used by multiply, divide
absb:
	rlcf	REGB3,w
	skpc
	return			; Positive

; Negate REGB
; Used by absb, subtract, multiply, divide
negateb:
	movf	REGB3,w		; Save sign in w
	andlw	0x80

	comf	REGB0,f		; 2's complement
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
	incf	sign,f		; flip sign flag
	addwf	REGB3,w		; Return carry set if -2147483648
	return


; Shift left REGA and REGC
; Used by multiply, divide, round
slac:
	rlcf	REGA0,f
	rlcf	REGA1,f
	rlcf	REGA2,f
	rlcf	REGA3,f
slc:
	rlcf	REGC0,f
	rlcf	REGC1,f
	rlcf	REGC2,f
	rlcf	REGC3,f
	return

; math end




int_8:
; interrupt entry point
; do interrupt processing here

; save resgisters used in main
	save_w_stat					; save W and status

;	movlw 	'I'
;	call	tx_w

;	clrwdt						; print for example help continously will not allow main_loop  to clear the watchdog timer.
; what interrupt?


#ifdef OLD_CODE
; test for port a b interrupt
	btfss	INTCON, RABIE
	bra		end_test_port_a_b_interrupt

; prevent multiple interrupts due to switch bounce
	 bcf     INTCON, RABIE

; WAS TEST
;	movlw	'A'
;	call	tx_w

; restart GPS from card output
;	clrf	data_record_number3
;	clrf	data_record_number2
;	clrf	data_record_number1
;	clrf	data_record_number0	
;	bcf		END_OF_DATA_FLAG


#ifdef OLD_CODE
; test for card removed
	btfss	NOT_MMC_PRESENT_SWITCH
	bra		card_inserted
; card removed
	movlw	'r'
	call	tx_w

;	bcf		READ_FROM_CARD_FLAG
;	bcf		RED_LED
;	bsf		GREEN_LED
;	bcf		CARD_OUT_SELECT
	bra		end_test_port_a_b_interrupt1

card_inserted:
	movlw	'i'
	call	tx_w

	call	spi_ini
	call	mmc_init

	clrf	REGA3
	clrf	REGA2
	clrf	REGA1
	clrf	REGA0
	call	mmc_read_sector
#endif ; OLD_CODE

end_test_port_a_b_interrupt1:
	bcf		INTCON, RABIF

; allow port AB interrupts again
	bsf     INTCON, RABIE

end_test_port_a_b_interrupt:
#endif ; OLD_CODE



; test for timer0 interrupt, clock tick
	btfss	INTCON, TMR0IF
	bra		end_test_timer0_interrupt
; timer0 interrupt

; WAS TEST
;	movlw	'0'
;	call	tx_w

; 200 ms
	movff	timer0_reload_h, TMR0H
	movff	timer0_reload_l, TMR0L

; test if mode change button prressed
; only on button down
	btfss	NOT_MODE_CHANGE_BUTTON
	bra		button_is_pressed
; button is not pressed
	clrf	debounce_counter
	bra		in_5	

button_is_pressed:
; start debounce test
	incf	debounce_counter
	movlw	DEBOUNCE_TIME
	subwf	debounce_counter, W
	btfss	STATUS, Z
	bra		in_5
; debounce count 5
	clrf	debounce_counter
; flip the GPS output control
;	btg		ECHO_GPS_FLAG
	bcf		ECHO_GPS_FLAG
	bra		in_5
; end debounce test

in_5:
; WAS TEST
;	movfw	output_mode
;	call	print_w_ascii_dec
;	call	tx_crlf

; signal to main
	bsf		TIME_FLAG
	bcf		INTCON, TMR0IF
end_test_timer0_interrupt:


; test for serial port interrupt
	btfss	PIR1,	RCIF				; test if serial port interrupt
	goto	int_end

	bcf		PIR1,   RCIF

; test framing error
	btfss	RCSTA,	FERR
	bra		test_overrun
; framing error
; discard data (datasheet page 185)
	movfw	RCREG
	goto		int_end_rx

test_overrun:
	btfss	RCSTA,	OERR
	bra		get_rx_char
; overrun error
; flip CREN (datasheet page 185)
	bcf		RCSTA,	CREN
	bsf		RCSTA,	CREN

	goto		int_end_rx						; discard character


get_rx_char:
	bcf 	PIE1,   RCIE            ; USART receive interrupt disable


#ifndef TEST_MODE

; get input from Hubsan H501S remote testpoint AFC_RX character in RCREG

;		from find_altitude.c by me
; /*
; FF 0C
; 16 00 03
; 08 // field length
; 20 20 20 5E 30 2E 30 6D
;          ^  0  .  0  m      altitude m
; 20   // checksum
;          
; looking for FF 0C 16 00 03
; */
; 
; checksum = 0;
; i = 0;
; altitude_mode = 0;
; while(1)
; 	{
; 	c = fgetc(fptr);
; 
; 	if(c == EOF)
; 		{
; 		break;
; 		}
; 
; 	if(altitude_mode < 2)
; 		{
; 		checksum = 0;
; 		}
; 	else if(altitude_mode < 6)
; 		{
; 		checksum ^= c;
; 		}
; 
; 	if(altitude_mode == 6)
; 		{
; //fprintf(stderr, "c=%02x\n", c);		
; 
; 		if(c == checksum)
; 			{
; 			altitude = atof(temp);
; 			fprintf(stdout, "altitude=%.1f\n", altitude);
; 			}
; 		else
; 			{
; 			fprintf(stderr, "CHECKSUM error\n");
; 			}
; 
; 		altitude_mode = 0;
; //exit(1);
; 		}
; 
; 	if(i)
; 		{
; 		if( isdigit(c) || (c == '.') || (c == '-') )
; 			{
; 			temp[j] = c;
; 			j++;
; 			}
; 		i--;
; 		if(i == 0)
; 			{
; 			temp[j] = 0;
; //			fprintf(stderr, "temp=%s\n", temp);
; 
; 			altitude_mode = 6;
; 			}
; 
; 		continue;
; 		}		
; 
; 	if(altitude_mode == 5)
; 		{
; 		i = c;
; 		j = 0;
; 		continue;
; 		}
; 
; 	if(altitude_mode == 4)
; 		{
; 		if(c == 0x03) altitude_mode++;
; 		else altitude_mode = 0;
; 		}
; 	if(altitude_mode == 3)
; 		{
; 		if(c == 0x00) altitude_mode++;
; 		else altitude_mode = 0;
; 		}
; 	if(altitude_mode == 2)
; 		{
; 		if(c == 0x16) altitude_mode++;
; 		else altitude_mode = 0;
; 		}
; 	if(altitude_mode == 1)
; 		{
; 		if(c == 0x0c) altitude_mode++;
; 		else altitude_mode = 0;
; 		}
; 
; 	if(c == 0xff)
; 		{
; 		altitude_mode++;
; 		}
; 

; get current_altitude_decimeter_h:current_altitude_decimeter_l


; WAS TEST
; echo
;	movfw	RCREG
;	call	tx_w
;	bra		int_end_rx

;	movfw	altitude_mode
;	call	print_w_ascii_dec
;	call	tx_crlf

	movlw	D'2'
	cpfslt	altitude_mode
	bra		test_less_than_6
; < 2
	clrf	checksum
	bra		checksum_done
test_less_than_6:
	movlw	D'6'
	cpfslt	altitude_mode
	bra		checksum_done
; < 6
	movfw	RCREG
	xorwf	checksum
checksum_done:


; test for mode 6
	movlw	D'6'
	cpfseq	altitude_mode
	bra		test_if_i_counter
; mode 6
; clear mode
	clrf	altitude_mode
; check checksum	
	movfw	checksum
	cpfseq	RCREG
	bra		checksum_error

; WAS TEST
; checksum OK	
;	movlw	'O'
;	call	tx_w
;	movlw	'K'
;	call	tx_w
;	movlw	' '
;	call	tx_w

;	movlw	'-'
;	btfsc	NEGATIVE_INPUT_SIGN_FLAG
;	call	tx_w

; test for multiply by 10 needed if dot was found
	btfsc	HAVE_DOT_FLAG
	bra		no_dot_found

; no dot found, meters only, multiply by 10	
	movff	value3, REGA3
	movff	value2,	REGA2
	movff	value1,	REGA1
	movff	value0, REGA0
; REGA3:REGA0 = value3:value0

	clrf	REGB3
	clrf	REGB2
	clrf	REGB1
	movlw	D'10'
	movwf	REGB0
; REGB3:REGB0 = 10

    call    multiply                                    ; 32 bit signed multiply  REGA * REGB -> REGA  return carry set if overflow
; REGA3:REGA0 = value * 10

	movff	REGA3, value3
	movff	REGA2, value2
	movff	REGA1, value1
	movff	REGA0, value0
; value3:value0 = 10 * 

no_dot_found:
; prevent main from using only high or low byte if interrupt happens when moving registers for calculation
wait_updating_altitude:
	btfsc	WAIT_UPDATING_ALTITUDE_FLAG
	bra		wait_updating_altitude

	movff	value1, current_altitude_decimeters_h
	movff	value0, current_altitude_decimeters_l
	bcf		WAIT_UPDATING_ALTITUDE_FLAG

#ifdef DEBUG
; WAS TEST
	call	tx_crlf
	clrf	REGA3
	clrf	REGA2
	movff	current_altitude_decimeters_h, REGA1
	movff	current_altitude_decimeters_l, REGA0
	call	print_32_ascii_dec
	movlw	' '
	call	tx_w
	movlw	'd'
	call	tx_w
	movlw	'm'
	call	tx_w
	call	tx_crlf
#endif ; DEBUG


; at this point we have the current altitude in dm as integer from the AFC_RX testpoint on the remote, so as displayed on the remote as float in meters, so our altitude resolution is 1 dm (10 cm).
; we are here (measured) 2.5 times per second, or every 400 ms (half the GPS 5 Hz rate)
; 
; from the change in altitude over 400 ms we can calculate the vertical speed
;   and then if main sets the elevation_mode to ELEVATION_MODE_UP or ELEVATION_MODE_DOWN we can enable the DAC output pin,
;  and increase DAC if ELEVATION_MODE_UP and decrease DAC if ELEVATION_MODE_DOWN.
;   until the vertical speed matches the maximum vertical speed limit specified by us (maybe later get it from card).
; if vertical speed exceeds the specified limit than we decrease the DAC if going up too fast, or increase the DAC if going down too fast.
; if main sets elevation_mode to ELEVATION_MODE_HOLD then we set the DAC to 16 (mid range) and tri-state it.

; calculate vertical speed

; calculate difference
	clrf	REGA3
	clrf	REGA2
	movff	current_altitude_decimeters_h, REGA1	
	movff	current_altitude_decimeters_l, REGA0	
; REGA3:REGA0 = current_altitude_decimeters_h:current_altitude_decimeters_l

	clrf	REGB3
	clrf	REGB2
	movff	previous_current_altitude_decimeters_h, REGB1
	movff	previous_current_altitude_decimeters_l, REGB0
; REGB3:REGB0 = previous_current_altitude_decimeters_h:previous_current_altitude_decimeters_l

	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = current_altitude_decimeters_h:current_altitude_decimeters_l - previous_current_altitude_decimeters_h:previous_current_altitude_decimeters_l 	this positive if going up

; vspeed, signed! positive if moving up, negative if moving down in RA3:RA0
	movff	REGA1, vertical_speed_dm_per_400ms_h
	movff	REGA0, vertical_speed_dm_per_400ms_l
; vertical_speed_dm_per_400ms_h:vertical_speed_dm_per_400ms_l = vertical speed in dm per 400 ms, signed, negative for going down

#ifdef DEBUG
; WAS TEST
; report vertical_speed_dm_per_400ms_h:vertical_speed_dm_per_400ms_l
	movlw	'V'
	call	tx_w
	movlw	'S'
	call	tx_w
	movlw	' '
	call	tx_w
	call	print_32_ascii_dec_signed
	movlw	' '
	call	tx_w
	movlw	'd'
	call	tx_w
	movlw	'm'
	call	tx_w
	movlw	'/'
	call	tx_w
	movlw	'4'
	call	tx_w
	movlw	'0'
	call	tx_w
	movlw	'0'
	call	tx_w
	movlw	' '
	call	tx_w
	movlw	'm'
	call	tx_w
	movlw	's'
	call	tx_w
	call tx_crlf
#endif ; DEBUG


; test if vertical speed limit reached
; get absolute value
	clrf	REGA3
	clrf	REGA2
	movff	vertical_speed_dm_per_400ms_h, REGA1
	movff	vertical_speed_dm_per_400ms_l, REGA0
; REGA3:REGA0 = vertical_speed_dm_per_400ms_h:vertical_speed_dm_per_400ms_l

	call	absa			; checks sign of REGA and convert negative to positive
; REGA3:REGA0 = abs(vertical_speed_dm_per_400ms_h:vertical_speed_dm_per_400ms_l)

; compare to limit 1 dm / 400 ms
	clrf	REGB3
	clrf	REGB2
	clrf	REGB1
	movff	v_speed_limit_dm_per_400_ms, REGB0
	call	subtract			; 32 bit subtract REGA - REGB -> REGA, return carry set if overflow, test bit 7 REGA3 for sign flag
; REGA3:REGA0 = abs(vertical_speed_dm_per_400ms_h:vertical_speed_dm_per_400ms_l) - v_speed_limit_dm_per_400_ms

; this is negative if within speed limit
;	bcf		VERTICAL_SPEED_LIMIT_FLAG
;	btfss	REGA3, 7
;	bsf		VERTICAL_SPEED_LIMIT_FLAG

	btfss	REGA3, 7
	bra		set_v_speed_limit_flag
; not v speed limit
	bcf		VERTICAL_SPEED_LIMIT_FLAG
	bra		set_v_speed_limit_flag_done
set_v_speed_limit_flag:
	bsf		VERTICAL_SPEED_LIMIT_FLAG
set_v_speed_limit_flag_done:

#ifdef DEBUG
; WAS TEST
	call	tx_crlf
	movlw	'V'
	call	tx_w
	movlw	'L'
	call	tx_w
	movlw	'='
	call	tx_w
	movlw	'1'
	btfss	VERTICAL_SPEED_LIMIT_FLAG
	movlw	'0'
	call	tx_w
	call	tx_crlf
#endif ; DEBUG



#ifdef OLD_CODE
; TEST reset DAC if too fast, this will cause a 'hold' and start up down incr / decr from 16 again
;	movlw	DAC_MID_RANGE
;	btfsc	VERTICAL_SPEED_LIMIT_FLAG
;	movwf	VREFCON2

; end test for vertical speed limit


; test for elevation mode set by main
	movlw	ELEVATION_MODE_HOLD
	subwf	elevation_mode, W
	btfsc	STATUS, Z
	bra		i_test_elevation_hold

	movlw	ELEVATION_MODE_DOWN
	subwf	elevation_mode, W
	btfsc	STATUS, Z
	bra		i_test_elevation_down

	movlw	ELEVATION_MODE_UP
	subwf	elevation_mode, W
	btfsc	STATUS, Z
;	bra		i_test_elevation_up


i_test_elevation_up:
; test for max positive vertical speed 
#ifdef DEBUG
; WAS TEST
	call	tx_crlf
	movlw	'E'
	call	tx_w
	movlw	'M'
	call	tx_w
	movlw	' '
	call	tx_w
	movlw	'U'
	call	tx_w
	movlw	'P'
	call	tx_w
	call	tx_crlf
#endif ; DEBUG

; WAS V peed limiting disabled

; test if max v speed reached
;	btfsc	VERTICAL_SPEED_LIMIT_FLAG
;	bra		decrement_dac

	bra		increment_dac	


i_test_elevation_down:
; test for max negative vertical speed
#ifdef DEBUG
; WAS TEST
	call	tx_crlf
	movlw	'E'
	call	tx_w
	movlw	'M'
	call	tx_w
	movlw	' '
	call	tx_w
	movlw	'D'
	call	tx_w
	movlw	'O'
	call	tx_w
	movlw	'W'
	call	tx_w
	movlw	'N'
	call	tx_w
	call	tx_crlf
#endif ; DEBUG

; WAS V speed limiting disabled
; test if max v speed reached
;	btfsc	VERTICAL_SPEED_LIMIT_FLAG
;	bra		increment_dac

	bra		decrement_dac


i_test_elevation_hold:	
#ifdef DEBUG
; WAS TEST
	call	tx_crlf
	movlw	'E'
	call	tx_w
	movlw	'M'
	call	tx_w
	movlw	' '
	call	tx_w
	movlw	'H'
	call	tx_w
	movlw	'O'
	call	tx_w
	movlw	'L'
	call	tx_w
	movlw	'D'
	call	tx_w
	call	tx_crlf
#endif ; DEBUG

	bra		hold_dac


increment_dac:
; range check, increment if < DAC_MAX_RANGE
	movlw	DAC_MAX_RANGE
	subwf	VREFCON2, W
	btfss	STATUS, Z
	incf	VREFCON2
; enable DAC
	bsf		VREFCON1, DAC1OE
	bra		update_previous_altitude	


decrement_dac:
; range check decrement if > 0
	tstfsz	VREFCON2
	decf	VREFCON2
; enable DAC
	bsf		VREFCON1, DAC1OE
	bra		update_previous_altitude

hold_dac:
; DAC tristate
	bcf		VREFCON1, DAC1OE
; DAC midrange
	movlw	DAC_MID_RANGE
	movwf	VREFCON2
;	bra		update_previous_altitude

update_previous_altitude:
#ifdef DEBUG
; WAS TEST
; report DAC
	movlw	'D'
	call	tx_w
	movlw	'A'
	call	tx_w
	movlw	'C'
	call	tx_w
	movlw	' '
	call	tx_w
	movfw	VREFCON2
	call	print_w_ascii_dec
	call	tx_crlf
#endif ; DEBUG
#endif ; OLD_CODE


	movff	current_altitude_decimeters_h, previous_current_altitude_decimeters_h
	movff	current_altitude_decimeters_l, previous_current_altitude_decimeters_l
	bra		int_end_rx	

checksum_error:
#ifdef DEBUG
; WAS TEST
	movlw	'C'
	call	tx_w
	movlw	'E'
	call	tx_w
	movlw	' '
	call	tx_w
	movfw	checksum
	call	print_w_ascii_dec
	call	tx_crlf
#endif ; DEBUG

	bra		int_end_rx

test_if_i_counter:
	movlw	D'0'
	tstfsz	i_counter
	bra		have_data	
	bra		test_for_mode_5
have_data:
; test if digits

; WAS TEST
;	movfw	RCREG
;	call	tx_w

	call	field_to_decimal		; data in RCREG to value3:value0, value3:value0 and NEGATIVE_ALTITUDE_FLAG must be cleared when starting parsing field,
									; sets NEGATIVE_ALTITUDE_FLAG if negative sign encountered anywhere in the field
									; if end result has not HAVE_DOT_FLAG set, it is in meters, multiply by 10 to get dm
									; if HAVE_DOT_FLAG is set, do not multiply at all.
	decf	i_counter
; test for i_counter 0
	movlw	D'0'
	tstfsz	i_counter
	bra		int_end_rx
; i_counter is zero
	incf	altitude_mode

; WAS TEST
;	call	tx_crlf

	bra		int_end_rx
	

test_for_mode_5:
	movlw	D'5'
	cpfseq	altitude_mode
	bra		test_for_mode_4
; mode 5
	movff	RCREG, i_counter
;	clrf	j_counter
	
	bcf		NEGATIVE_ALTITUDE_FLAG
	bcf		HAVE_DOT_FLAG
	clrf	value3
	clrf	value2
	clrf	value1
	clrf	value0

	bra		int_end_rx

test_for_mode_4:
	movlw	D'4'
	cpfseq	altitude_mode
	bra		test_for_mode_3
; mode 4
	movlw	0x03
	cpfseq	RCREG
 	bra		clear_altitude_mode
	incf	altitude_mode
	bra		int_end_rx
	
test_for_mode_3:
	movlw	D'3'
	cpfseq	altitude_mode
	bra		test_for_mode_2
; mode 3
	movlw	0x00
	cpfseq	RCREG
 	bra		clear_altitude_mode
	incf	altitude_mode
	bra		int_end_rx

test_for_mode_2:	
	movlw	D'2'
	cpfseq	altitude_mode
	bra		test_for_mode_1
; mode 2
	movlw	0x16
	cpfseq	RCREG
 	bra		clear_altitude_mode
	incf	altitude_mode
	bra		int_end_rx

test_for_mode_1:	
	movlw	D'1'
	cpfseq	altitude_mode
	bra		test_for_mode_0
; mode 1
	movlw	0x0c
	cpfseq	RCREG
 	bra		clear_altitude_mode
	incf	altitude_mode
	bra		int_end_rx

test_for_mode_0:
	movlw	0xff
	cpfseq	RCREG	
	bra		int_end_rx			
	incf	altitude_mode
	bra		int_end_rx

clear_altitude_mode:
	clrf	altitude_mode
	bra		int_end_rx

; process any normal commands


; ignore input if not in echo mode
;	bra		int_end_rx

#else	; TEST_MODE

; LF
	movlw	D'10'
	subwf	RCREG,	W
	btfsc	STATUS, Z
  	bra		process_digits_cr

; CR
	movlw	D'13'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	bra		process_digits_cr

	movlw	'D'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	bra		set_dac_command

	movlw	'd'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	bra		disable_dac_command

	movlw	'h'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	bra		print_help_command

	movlw	'r'
	subwf	RCREG, W
	btfsc	STATUS, Z
	bra		read_data_record_command

	movlw	'S'
	subwf	RCREG, W
	btfsc	STATUS, Z
	bra		set_gps_rate_command

	movlw	'U'
	subwf	RCREG, W
	btfsc	STATUS, Z
	bra		set_data_record_pointer_command

	movlw	'v'
	subwf	RCREG, W
	btfsc	STATUS, Z
	bra		print_status_command

	movlw   'X'
	subwf   RCREG, W
	btfsc   STATUS, Z
	bra     dump_database_command



process_digits_cr:
; digits
	
; CR
	movlw	D'13'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	bra		cr_command

	movlw	D'10'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	bra		cr_command

	movlw	'0'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'1'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'2'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'3'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'4'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'5'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'6'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'7'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'8'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'9'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	movlw	'-'
	subwf	RCREG,	W
	btfsc	STATUS, Z
	goto	process_digits_command

	goto	int_end_rx


cr_command:
; If zero digits were entered, do nothing
	movlw	COMMAND_OFF
	subwf	digit_cnt, W
	btfsc	STATUS,	Z						; if zero digits do nothing
	goto	int_end_rx				
	
	movlw	COMMAND_SET_DATA_RECORD_POINTER
	subwf	command, W
	btfsc	 STATUS, Z
	bra		end_set_data_record_pointer_command

	movlw	COMMAND_READ_DATA_RECORD
	subwf	command, W
	btfsc	STATUS, Z
	bra		end_read_data_record_command

	movlw	COMMAND_SET_GPS_RATE
	subwf	command, W
	btfsc	STATUS, Z
	bra		end_set_gps_rate_command

	movlw	COMMAND_SET_DAC
	subwf	command, W
	btfsc	STATUS, Z
	bra		end_set_dac_command


	bra		int_end_rx


; these commands have no numeric argument

disable_dac_command:
	bcf		VREFCON1, DAC1OE
	movlw	COMMAND_DISABLE_DAC
	movwf	command
	bra		int_end_clr

print_status_command:
	movlw	COMMAND_PRINT_STATUS
	movwf	command
	call	print_status
	bra		int_end_clr

print_help_command:
	movlw	COMMAND_PRINT_HELP
	movwf	command
	print	text_help_menu
	bra		int_end_clr

dump_database_command:
	movlw	COMMAND_DUMP_DATABSE
	movwf	command
; test if card present
	btfss	NOT_MMC_PRESENT_SWITCH
	bra		dump_database_command_have_card
; no card
	print	text_no_mmc_present
	bra		int_end_clr
dump_database_command_have_card
	btfss	MMC_INIT_OK_FLAG
	call	mmc_init									; inits MMC card, sets MMC_INIT_OK_FLAG if card init OK
	call	dump_database
	bra		int_end_clr


; these commands have a numeric argument

set_dac_command:
	movlw	COMMAND_SET_DAC
	movwf	command
	bra		command_end

set_gps_rate_command:
	movlw	COMMAND_SET_GPS_RATE
	movwf	command
	bra		command_end

set_data_record_pointer_command:
	movlw	COMMAND_SET_DATA_RECORD_POINTER
	movwf	command
	bra		command_end

read_data_record_command:
	movlw	COMMAND_READ_DATA_RECORD
	movwf	command
	bra		command_end


; final processing of commands with numeric arguments, we have the number in 'value' now.

end_set_dac_command:
	movff	value0, VREFCON2
	bsf		VREFCON1, DAC1OE
	bra		int_end_clr

end_set_gps_rate_command:
	movff	value0, gps_rate
	call	calculate_timer0_reload            ; sets timer0_reload_h:timer0_reload_l from gps_speed
	call	save_settings
	bra		int_end_clr

end_read_data_record_command:
; test if card present
	btfss	NOT_MMC_PRESENT_SWITCH
	bra		end_read_data_record_command_have_card
; no card
	print	text_no_mmc_present
	bra		int_end_clr
end_read_data_record_command_have_card:
	btfss	MMC_INIT_OK_FLAG
	call	mmc_init									; inits MMC card, sets MMC_INIT_OK_FLAG if card init OK

	movff	value3,	REGA3
	movff	value2, REGA2
	movff	value1, REGA1
	movff	value0, REGA0
	call	mmc_read_sector                                    ; reads a 512 Byte block from the MMC, sector number in REGA3:REGA0, output to serial link, sets END_OF_DATA_FLAG and returns if any zero byte found
	bra		int_end_clr


end_set_data_record_pointer_command:
	movff	value3, data_record_number3
	movff	value2, data_record_number2
	movff	value1, data_record_number1
	movff	value0, data_record_number0
	bcf		END_OF_DATA_FLAG
	bra		int_end_clr


process_digits_command:
	movlw	'-'
	cpfseq	RCREG	
	bra		process_digits_command_not_minus_sign
; negative sign
	bsf		NEGATIVE_INPUT_SIGN_FLAG
	bra		int_end_rx

process_digits_command_not_minus_sign:
    movlw   D'48'
    subwf   RCREG, W           ; digit now in W

    movwf   digit_in

; value3:value0 *= 10
; value3:value0 += RCREG

	movff	value3, REGA3
    movff	value2, REGA2
    movff   value1, REGA1  
    movff   value0, REGA0  
; REGA = value3:value0

    clrf    REGB3
    clrf    REGB2
    clrf    REGB1
    movlw   D'10'
    movwf   REGB0
; REGB = 10
    
    call    multiply                                    ; 32 bit signed multiply  REGA * REGB -> REGA  return carry set if overflow
; REGA = value3:value0 * 10   

    clrf    REGB3
    clrf    REGB2
    clrf    REGB1
    movff   digit_in, REGB0   
; REGB = digit_in   

    call    add                                         ; 32 bit signed add  REGA + REGB -> REGA  return carry set if overflow
; REGA = (value2:value0 * 10) + digit_in

	movff	REGA3, value3
	movff	REGA2, value2
    movff	REGA1, value1
    movff	REGA0, value0

    incf    digit_cnt

    bra	    int_end_rx
#endif ; TEST_MODE


; end of interrupt routines
command_end:
; reset the value, so we can continue for other numerical commands.
	clrf	value3
	clrf	value2
	clrf	value1
	clrf	value0
	clrf	digit_cnt
	bcf		NEGATIVE_INPUT_SIGN_FLAG
	bra		int_end_rx


int_end_clr:					; for non numeric commands, no digits expected.
	clrf	command

int_end_rx:
	bcf	PIR1,	RCIF
	bsf PIE1,   RCIE            ; USART receive interrupt enable

	bra		int_done


int_end:
;	bcf		INTCON,	RABIE
;	bcf		INTCON,	TMR0IF			; timer 0 overflow interrupt flag
	bcf		INTCON,	INT0IF			; GP2/INT
	bcf		INTCON,	RABIF			; port change interrupt flag bit

; PIR1   --   ADIF RCIF TXIF  SSPIF CCP1IF TMR2IF TMR1IF
	bcf		PIR1,	ADIF
;	bcf		PIR1,	RCIF
	bcf		PIR1,	TXIF
	bcf		PIR1,	SSPIF
	bcf		PIR1,	CCP1IF
	bcf		PIR1,	TMR2IF
	bcf		PIR1,	TMR1IF

; PIR2  OSFIF C2IF C1IF EEIF   --     --     --     --
	bcf	PIR2,	OSCFIF
	bcf	PIR2,	C2IF
	bcf	PIR2,	C1IF
	bcf	PIR2,	EEIF

int_done:
	restore_w_stat				; get back W and status
	retfie





print_32_ascii_dec:					; prints 32 bit value in registers REGA3:REGA0 in ASCII decimal via RS232.
	bsf		FIRST_ZERO_SUPPRESSED_FLAG

; Max 4 G
; divide by 1G = 1000 000 000 = 0x 3B 9A CA 00 
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



#ifdef OLD_CODE
write_data_record:					; record to MMC if present, else to 24LC1025 EEPROM, at [sector] address data_record_pointer_h:data_record_pointer_l. increments data_record_pointer_h:data_record_pointer_l, called 1 x per minute
;	movlw	'L'
;	call	tx_w
;	call	tx_crlf

; was GPS switch on, if not clear GPS_POSITION_VALID_FLAG
	btfss	GPS_INPUT_SWITCH
	bcf		GPS_POSITION_VALID_FLAG

; data we have

; so, if using binary records
; we have 1024 128 byte pages, 4 records fit into one page, this makes fot 4 * 1024 records = 4096 records, ( (128 * 1024) / 32 )
; at 1 record per minute 4096 minutes, or 68.2 hours, or 2.8 days.


; data to RAM
	lfsr 	FSR0, SPI_RESULT_ADDRESS
; 0
	movff	flags3								, POSTINC0
; 1
	movff	flags4								, POSTINC0
; 2
	peek	counts_per_minute_h					, POSTINC0
; 3
	peek	counts_per_minute_l					, POSTINC0
; 4
	movff	hours								, POSTINC0
; 5
	movff	minutes 							, POSTINC0
; 6
	movff	latitude_degrees					, POSTINC0
; 7
	movff	latitude_minutes_u					, POSTINC0
; 8
	movff	latitude_minutes_h	 				, POSTINC0
; 9
	movff	latitude_minutes_l	 				, POSTINC0
; 10
	movff	longitude_degrees					, POSTINC0
; 11
	movff	longitude_minutes_u					, POSTINC0
; 12
	movff	longitude_minutes_h					, POSTINC0
; 13
	movff	longitude_minutes_l					, POSTINC0
; 14
	movff	gps_altitude_h						, POSTINC0
; 15
	movff	gps_altitude_l						, POSTINC0
; 16
	peek	gm_voltage_h						, POSTINC0
; 17
	peek	gm_voltage_l						, POSTINC0
; 18
	peek	battery_voltage_h					, POSTINC0
; 19
	peek	battery_voltage_l					, POSTINC0
; 20
	peek	cpm_alarm_level_h					, POSTINC0
; 21
	peek	cpm_alarm_level_l					, POSTINC0
; 22
	peek	accumulated3						, POSTINC0
; 23
	peek	accumulated2						, POSTINC0
; 24
	peek	accumulated1						, POSTINC0
; 25
	peek	accumulated0						, POSTINC0
; 26
	peek	accumulated_alarm_level3			, POSTINC0
; 27
	peek	accumulated_alarm_level2			, POSTINC0
; 28
	peek	accumulated_alarm_level1			, POSTINC0
; 29
	peek	accumulated_alarm_level0			, POSTINC0
; 30
	movff	gps_heading_h						, POSTINC0
; 31
	movff	gps_heading_l						, POSTINC0
; 32
	movff	gps_speed_kmh_h						, POSTINC0
; 33
	movff	gps_speed_kmh_l						, POSTINC0
; 34
	movff	years								, POSTINC0
; 35
	movff	months								, POSTINC0
; 36
	movff	days								, POSTINC0
; 37
	peek 	clock_alarm_hours					, POSTINC0
; 38
	peek	clock_alarm_minutes					, POSTINC0
; 39
	peek	time_zone_offset					, POSTINC0

; add date here that you want logged,
; maximum 63
	
; test if MMC in slot
	btfss	NOT_MMC_PRESENT_SWITCH
; try to init MMC
	call	mmc_init												; inits MMC card, sets MMC_INIT_OK_FLAG if card init OK

; get record number to REGA3:REGA0
	peek	data_record_number3, REGA3
	peek	data_record_number2, REGA2
	peek	data_record_number1, REGA1
	peek	data_record_number0, REGA0

; test if MMC present
;	btfsc	NOT_MMC_PRESENT_SWITCH
;	bra		write_data_record_use_24LC1025

	btfss	MMC_INIT_OK_FLAG


	bra		write_data_record_use_24LC1025

; if a MMC was detected record to it
	call	mmc_write_sector					; sector number in REGA3:REGA0
	bra		write_data_record_increment_record_number

write_data_record_use_24LC1025:

	call	lc1025_write_record					; write one RECORD_SIZE record number in REGA3:REGA0, from SPI_RESULT_ADDRESS to 24LC1025, chip is 128 k x 8, 1024 pages

write_data_record_increment_record_number:
; increment record number
	peek	data_record_number3, REGA3
	peek	data_record_number2, REGA2
	peek	data_record_number1, REGA1
	peek	data_record_number0, REGA0

; 32 bit increment
	incf    REGA0, F
	btfsc   STATUS ,Z
	incf    REGA1, F
	btfsc   STATUS, Z
	incf    REGA2, F
	btfsc   STATUS, Z
	incf    REGA3, F

	movff	REGA3, data_record_number3
	movff	REGA2, data_record_number2
	movff	REGA1, data_record_number1
	movff	REGA0, data_record_number0

;	btfss	NOT_MMC_PRESENT_SWITCH
	btfss	MMC_INIT_OK_FLAG
	bra		test_24LC1025_memory_limit

test_MMC_memory_limit:
	bcf		MEMORY_FULL_FLAG
; test if REGA3 0x80
	movlw	0x80
	subwf	REGA3, W
	btfsc	STATUS, Z
; was 0x80, memory is full for 1 GB card
	bsf		MEMORY_FULL_FLAG
	bra		write_data_record_save_record_number


test_24LC1025_memory_limit:
; if 24LC1025 EEPROM used, check if it is full, it only has space for 4096 32 bits entries
	clrf	REGB3
	movlw	UPPER	D'2048'		; D'131072'/RECORD_SIZE
	movwf	REGB2
	movlw	HIGH	D'2048'		; D'131072'/RECORD_SIZE
	movwf	REGB1
	movlw	LOW		D'2048'		; D'131072'/RECORD_SIZE
	movwf	REGB0
; REGB3:REGB0 = 4096

	call	subtract		; REGA - REGB -> REGA
; REGA3:REGA0 = data_record_number3:data_record_number0 - 4096

; if negative then there is still space
	bcf		MEMORY_FULL_FLAG
	btfss	REGA3, 7
	bsf		MEMORY_FULL_FLAG

write_data_record_save_record_number:
; record number to eeprom	
	movlw	D'2'
	movwf	EEADR
    peek	data_record_number3, EEDATA
    call    eeprom_write    

	incf	EEADR
    peek	data_record_number2, EEDATA
    call    eeprom_write    

	incf	EEADR
    peek	data_record_number1, EEDATA
    call    eeprom_write    

	incf	EEADR
    peek	data_record_number0, EEDATA
    call    eeprom_write    

	return										; end subroutine write_data_record
#endif ; OLD_CODE




dump_database:									; read data records from MMC starinting at data_record_number3:data_record_number0

; done on calling in int_8
;	btfss	MMC_INIT_OK_FLAG
;	call	mmc_init										; inits MMC card, sets MMC_INIT_OK_FLAG if card init OK

; start at data_record_number3:data_record_number0
	movff	data_record_number3, temp_3
	movff	data_record_number2, temp_2
	movff	data_record_number1, temp_1
	movff	data_record_number0, temp_0

dump_database_loop:

; data to REGA3:REGA0
	movff	temp_3, REGA3
	movff	temp_2, REGA2
	movff	temp_1, REGA1
	movff	temp_0, REGA0

; read sector	
	bcf		END_OF_DATA_FLAG
	call	mmc_read_sector                                    ; reads a 512 Byte block from the MMC, sector number in REGA3:REGA0, output to serial link, sets END_OF_DATA_FLAG and returns if any zero byte found

; test for last record
	btfss	END_OF_DATA_FLAG
	bra		dump_database_more_data

; printing all records at 57600 Bd  will take long enough to trigger the watchdog, so reset it after each record.
	clrwdt

; report zero found in sector
	print	text_end_of_data
	return 		; end dump database

dump_database_more_data:
; increment sector number

; 32 bit increment
	incf	temp_0, f
	btfsc	STATUS ,Z
	incf	temp_1, f
	btfsc	STATUS, Z
	incf	temp_2, f
	btfsc	STATUS, Z
	incf	temp_3, f

	bra		dump_database_loop
; end subroutine dump_database




; start SPI routines for MMC

mmc_spi_cs_h:
	bsf		SPI_NOT_CS
	return											; end subroutine mmc_spi_cs_h


mmc_spi_cs_l:
	bcf		SPI_NOT_CS	
	return											; end subroutine mmc_spi_cs_l


spi_sdi_h:
	bsf		SPI_SI
	return											; end subroutine spi_sdi_h


spi_sdi_l:
	bcf		SPI_SI
	return											; end subroutine spi_sdi_l


spi_clk_h:
	bsf		SPI_SCK
	return											; end subroutine spi_clk_h


spi_clk_l:
	bcf		SPI_SCK
	return											; end subroutine spi_clk_l


spi_sdo_in:											; returns data in WREG0
	bsf		WREG, 0
	btfss	SPI_SO
	bcf		WREG, 0
	return											; end subroutine spi_sdo_in


spi_delay:							; as fast as possible = no delay
	movlw	SPI_DELAY
spi_delay_loop:
	nop
	decfsz	WREG
	bra		spi_delay_loop
	return							; end subroutine spi_delay


spi_ini:							; ini spi bus
; !CS high
	call	mmc_spi_cs_h
; clock low
	call	spi_clk_l
; data low	
	call	spi_sdi_l
	return							; end subroutine spi_ini


; Card Registers
; The SD card has six registers and SD Status information: CID, OCR, CSD, RCA, DSR, SCR.
; The detail definition as follow table:
; Resister   Bit Width  Description
; Name
;   CID         128    Card Identification information
;   OCR          32    Operation Conditions Registers
;   CSD         128    Card specific information
;   SCR          64    SD Memory Card's Special features
;   RCA          16    Relative Card Address
;   DSR          16    Driver Stage Register

; CID Register	The Card Identification (CID) register is 128-bit width. It contains the card identification information used during the Card Identification phase.
; OCR Register	This 32-bit register describes operating voltage range and status bit in the power supply.
; CSD Register	The Card-Specific Data (CID) register provides information on how to access the cards contents. Some field of this register can be writable by PROGRAM_CSD(CMD27). This register is 128bit width
; SCR Register  The SD Card Configuration (SCR) register provides information on SD Memory Card's special features. This register is 64bit width.
; RCA Register	The writable 16bit relative card address register carries the card address in SD Card mode.
; DSR Register	The Driver Stage Register (DSR) register provides an optional function for output driver condition. This register is 16bit width.

; All the SD Card commands are 6 bytes long and transmitted MSB first.
;             Byte 1                           Bytes 2--5             Byte 6
;         7   6   5               0            31                     0  7      0
;         0   1        Command                 Command Argument       CRC     1


; Commands used, see NXP app note AN10406:
;
; CMD0	GO_IDLE_STATE		reset the card to idle state
; CMD1	SEND_OP_COND,		ask the card in idle state to send their operation conditions contents in the response on the MISO line. Any negative response indicates the media card cannot be initialized correctly.
; CMD16	SET_BLOCKLEN		set the block length (in bytes) for all the following block commands, both read and write.
; CMD17	READ_SINGLE_BLOCK	read a block of data,  its size is determined by the SET_BLOCKLEN command.
; CMD24	WRITE_BLOCK			write a block of data, its size is determined by the SET_BLOCKLEN command.





mmc_spi_io:											; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movwf	spi_byte

; in byte zero
	clrf	spi_out

; bit loop
	movlw	D'8'
	movwf	spi_bit_count
mmc_spi_io_bit_loop:

; clock low
;;	call	spi_clk_l

; test SDO
	btfss	spi_byte, 7
	bra		mmc_spi_io_send_zero
; send one
	call	spi_sdi_h
	bra		mmc_spi_io_send_done

mmc_spi_io_send_zero:
; send zero
	call	spi_sdi_l	

mmc_spi_io_send_done:
	call	spi_delay

; next bit position input
	rlncf	spi_byte

; clock high, card samples data
	call	spi_clk_h
	call	spi_delay

; read card SDO
	call	spi_sdo_in								; data in WREG0		
	bsf		spi_out, 7
	btfss	WREG, 0
	bcf		spi_out, 7

; clock low
	call	spi_clk_l

; next bit position output	
	rlncf	spi_out

; loop if more bits
	decfsz	spi_bit_count
	bra		mmc_spi_io_bit_loop

; result to WREG
	movfw	spi_out

; result in WREG and spi_out
	return											; end subroutine mmc_spi_io


	

mmc_error:
	movwf	spi_byte

	call	tx_crlf
	movlw	'E'
	call	tx_w
	movlw	' '
	call	tx_w

	movfw	spi_byte
	call	print_w_ascii_dec
	call	tx_crlf

	call	mmc_spi_cs_h
	movlw	0xff
	call	mmc_spi_io

;	goto	reset_entry

	return											; end subroutine mmc_error




mmc_response:												; repeatedly reads the MMC until we get the response as in WREG, if so return 0, else return 1 timeout

; save test byte
	movwf	spi_in

; loop conter in REGA3:REGA2
; zero loop counter
	clrf	REGA3
	clrf	REGA2

mmc_response_loop1:
	clrwdt

; send 0xff
	movlw	0xff
	call	mmc_spi_io 	; result in W

;	call	print_w_hex
;	movlw	' '
;	call	tx_w

	movfw	spi_out

; compare to test byte
	subwf	spi_in, W
	btfss	STATUS, Z
	bra		mmc_response_test_next

; return OK
	retlw	0x00

mmc_response_test_next:

; increment loop counter
    incf    REGA3, f        
    incfsz  REGA2, f   
    decf    REGA3, f

; test if 
	movlw	HIGH	MMC_RESPONSE_TIMEOUT
	subwf	REGA3, W
	btfss	STATUS, Z
	bra		mmc_response_loop1
; REGA3 zero
	movlw	LOW		MMC_RESPONSE_TIMEOUT
	subwf	REGA2, W
	btfss	STATUS, Z
	bra		mmc_response_loop1

; timeout

mmc_response_errror:
; return error
	retlw	0x01											; end subroutine mmc_response



mmc_init:											; inits MMC card, sets MMC_INIT_OK_FLAG if card init OK
;	movlw	'M'
;	call	tx_w
;	call	tx_crlf

	call	mmc_spi_cs_h

; initialise the MMC card into SPI mode by sending 80 clks on 

	movlw	D'10'
	movwf	spi_byte_count
mmc_init_l1:

	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	decfsz	spi_byte_count
	bra		mmc_init_l1

	call	mmc_spi_cs_l

; send CMD0 (RESET or GO_IDLE_STATE) command, all the arguments are 0x00 for the reset command, precalculated checksum

	movlw	0x40
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	clrf	WREG
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	clrf	WREG
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	clrf	WREG
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	clrf	WREG
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movlw	0x95
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

; if = 1 then there was a timeout waiting for 0x01 from the MMC 
	movlw	D'1'
	call	mmc_response							; repeatedly reads the MMC until we get the response as in WREG, if so return 0, else return 1 timeout

;	call	print_w_ascii_dec
;	call	tx_crlf

; send some dummy clocks after GO_IDLE_STATE 
	call	mmc_spi_cs_h

	movlw	0xff
	call	mmc_spi_io

	call	mmc_spi_cs_l

; must keep sending command until zero response ia back
	movlw	0xff
	movwf	spi_byte_count

mmc_init_l2:
; send command
	movlw	0x41
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movlw	0x00
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movlw	0x00
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movlw	0x00
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movlw	0x00
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; checksum is no longer required but we always send 0xff
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

; test for response 0x00
	clrf	WREG
	call	mmc_response							; repeatedly reads the MMC until we get the response as in WREG, if so return 0, else return 1 timeout

	btfss	WREG, 0
	bra		mmc_init_p1

; decrement loop counter
	decfsz	spi_byte_count
	bra		mmc_init_l2

; timeout loop
	bcf		MMC_INIT_OK_FLAG

; say error 1
	movlw	D'1'
	bra		mmc_error

mmc_init_p1:

; send some dummy clocks after SEND_OP_COND 
	call	mmc_spi_cs_h
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	call	mmc_spi_cs_l

; send MMC CMD16 (SET_BLOCKLEN) to set the block length
	movlw	0x50
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; 4  bytes from here is the block length, SB is first
; 00 00 00 10 set to 16 bytes
; 00 00 02 00 set to 512 bytes <------
	clrf	WREG
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	clrf	WREG
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; high block length bits - 512 bytes 
	movlw	0x02					
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; low block length bits 
	clrf	WREG
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; checksum not
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

; test for OK
	movlw	0x00
	call	mmc_response							; repeatedly reads the MMC until we get the response as in WREG, if so return 0, else return 1 timeout
	btfss	WREG, 0
	bra		mmc_init_p2	
	movlw	D'2'
	bra		mmc_error

mmc_init_p2:
	call	mmc_spi_cs_h

;#define SAY_INIT_OK
#ifdef SAY_INIT_OK
; say MMC card init OK
	movlw	'M'
	call	tx_w
	movlw	'M'
	call	tx_w
	movlw	'C'
	call	tx_w
	movlw	' '
	call	tx_w
	movlw	'i'
	call	tx_w
	movlw	'n'
	call	tx_w
	movlw	'i'
	call	tx_w
	movlw	't'
	call	tx_w
	movlw	' '
	call	tx_w
	movlw	'O'
	call	tx_w
	movlw	'K'
	call	tx_w

	call	tx_crlf
#endif ; SAY_INIT_OK

	bsf		MMC_INIT_OK_FLAG

	return											; end subroutine mmc_init



mmc_read_sector:									; reads a 512 Byte block from the MMC, sector number in REGA3:REGA0, output to serial link, sets END_OF_DATA_FLAG and returns if any zero byte found

;	btfss	MMC_INIT_OK_FLAG
;	call	mmc_init									; inits MMC card, sets MMC_INIT_OK_FLAG if card init OK

; then call SPI_Receive() to read the data block back followed by the checksum.

; calculate address from sector number
; address is 512 * sector number
	clrf	REGB3
	movlw	UPPER 	D'512'
	movwf	REGB2
	movlw	HIGH	D'512'
	movwf	REGB1
	movlw	LOW		D'512'
	movwf	REGB0
; REGB3:REGB0 = 512		

	call	multiply
; REGA3:REGA0 = sector number * 512

; enable card
	call	mmc_spi_cs_l

; send MMC CMD17 (READ_SINGLE_BLOCK) to read the data from MMC card
	movlw	0x51
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; 32 bit byte address, high byte firsts, mustbe aligned on 512 block size (sector size). 
	movfw	REGA3
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movfw	REGA2
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movfw	REGA1
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movfw	REGA0
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; checksum not
	movlw	0xff	
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

; test for resposnse 0 is OK
	clrf	WREG
	call	mmc_response							; repeatedly reads the MMC until we get the response as in WREG, if so return 0, else return 1 timeout

	btfss	WREG, 0
	bra		mmc_read_sector_p1

	btfsc	DISABLE_ERROR3_REPORTING_FLAG	
	return

;;	call	mmc_init
	movlw	D'3'
	bra		mmc_error

mmc_read_sector_p1:

; wait for data token 

	movlw	0xfe
	call	mmc_response							; repeatedly reads the MMC until we get the response as in WREG, if so return 0, else return 1 timeout
; WAS TEST DISABLE ERROR 4
;;	btfss	WREG, 0
	bra		mmc_read_sector_p2
	movlw	D'4'
	bra		mmc_error

mmc_read_sector_p2:

; no zeros yet
	bcf		END_OF_DATA_FLAG

; get the block of data based on the length
; REGA1:REGA0 is loop counter
	clrf	REGA1
	clrf	REGA0
mmc_read_sector_loop1:
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

;	call	print_w_hex
;	movlw	' '
;	call	tx_w

	
; BUG fix, sometimes read errors cause sectors to be filled with 0xff 
; test if byte 0 of sector is 0xff, this signals a sector with read error
	tstfsz	REGA1
	bra		read_sector_end_test_for_0xff
	tstfsz	REGA0
	bra		read_sector_end_test_for_0xff
	tstfsz	WREG
	bra		read_sector_end_test_for_0xff
	bsf		END_OF_DATA_FLAG	
read_sector_end_test_for_0xff:

; test if byte 0 of sector is zero, such an empty sector is EOF
	movfw	spi_out
	tstfsz	REGA1
	bra		read_sector_end_test_for_eof
	tstfsz	REGA0
	bra		read_sector_end_test_for_eof
	tstfsz	WREG
	bra		read_sector_end_test_for_eof
	bsf		END_OF_DATA_FLAG	
read_sector_end_test_for_eof:

; byte 509:510 = requested_altitude_decimeter_h:requested_altitude_decimeter_l
; byte 511 = command_flags;
; get those, no serial output

; test for altitude_h in byte 509
	movfw	spi_out
	movlw	HIGH	D'509'
	subwf	REGA1, W
	btfss	STATUS, Z
	bra		read_sector_end_test_for_altitude_h
	movlw	LOW 	D'509'
	subwf	REGA0, W
	btfss	STATUS, Z
	bra		read_sector_end_test_for_altitude_h
	movff	spi_out,  requested_altitude_decimeters_h
	bra		read_sector_no_output
read_sector_end_test_for_altitude_h:

; test for altitude_l in byte 510
	movfw	spi_out
	movlw	HIGH	D'510'
	subwf	REGA1, W
	btfss	STATUS, Z
	bra		read_sector_end_test_for_altitude_l
	movlw	LOW 	D'510'
	subwf	REGA0, W
	btfss	STATUS, Z
	bra		read_sector_end_test_for_altitude_l
	movff	spi_out,  requested_altitude_decimeters_l
	bra		read_sector_no_output
read_sector_end_test_for_altitude_l:

; test for command_flags in byte 511
	movfw	spi_out
	movlw	HIGH	D'511'
	subwf	REGA1, W
	btfss	STATUS, Z
	bra		read_sector_end_test_for_command_flags
	movlw	LOW 	D'511'
	subwf	REGA0, W
	btfss	STATUS, Z
	bra		read_sector_end_test_for_command_flags
	movff	spi_out,  command_flags
	bra		read_sector_no_output
read_sector_end_test_for_command_flags:

; test if byte is zero, there are no zero bytes in NMEA GPS messages
	movfw	spi_out
	tstfsz	WREG
	bra		read_sector_end_test_for_zero_byte
	bra		read_sector_no_output
read_sector_end_test_for_zero_byte:

read_sector_serial_output:
; data to serial out
	movfw	spi_out
	call	tx_w

read_sector_no_output:
; increment loop counter
    incf    REGA1, f
    incfsz  REGA0, f   
    decf    REGA1, f

; test all bytes in sector done
	movlw   HIGH    D'512'
    subwf   REGA1, W
    btfss   STATUS, Z
    bra	    mmc_read_sector_loop1
; REGA3 zero
	movlw   LOW 	D'512'
    subwf   REGA0, W
    btfss   STATUS, Z
    bra		mmc_read_sector_loop1

; read CRC bytes and ignore CRC
; CRC high byte
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; CRC low byte
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

	call	mmc_spi_cs_h

; some clocks
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

	return						; end subroutine mmc_read_sector



mmc_wait_for_write_finish:							; wait a long time testing for 0x00, if found returns WREG = 0, if timeout WREG = 1

; The delay is set to maximum considering the longest data block length to handle
	clrf	REGA1
	clrf	REGA0
mmc_wait_for_write_finish_loop1:

	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

; if not zero ready
	movlw	0x00
	subwf	spi_out
	btfsc	STATUS, Z
	bra		mmc_wait_for_write_finish_try_again

;  not zero, ready
; return OK
	retlw	0x00

mmc_wait_for_write_finish_try_again:

; increment loop counter
    incf    REGA1, f  
    incfsz  REGA0, f
    decf    REGA1, f  

; test if 
    movlw   HIGH    MMC_WRITE_TIMEOUT
    subwf   REGA1, W    
    btfss   STATUS, Z
    bra		mmc_wait_for_write_finish_loop1
; REGA3 zero
    movlw   LOW     MMC_WRITE_TIMEOUT
    subwf   REGA0, W
    btfss   STATUS, Z
    bra		mmc_wait_for_write_finish_loop1

; timeout
; return error
	retlw	0xff
													; end subroutine mmc_wait_for_write_finish


#ifdef OLD_CODE
mmc_write_sector:									; write a 512 byte sector, sector address in REGA3:REGA0
;	movlw	'W'
;	call	tx_w
;	call	tx_crlf

	btfsc	MEMORY_FULL_FLAG
	return

	clrf	REGB3
	movlw	UPPER 	D'512'
	movwf	REGB2
	movlw	HIGH	D'512'
	movwf	REGB1
	movlw	LOW		D'512'
	movwf	REGB0
; REGB3:REGB0 = 512		

	call	multiply
; REGA3:REGA0 = sector number * 512

; enable card
	call	mmc_spi_cs_l

; send mmc CMD24 (WRITE_SINGLE_BLOCK) to write the data to MMC card
	movlw	0x58
	call	mmc_spi_io
; 32 bit byte address high byte first
	movfw	REGA3
	call	mmc_spi_io	
	movfw	REGA2
	call	mmc_spi_io	
	movfw	REGA1
	call	mmc_spi_io	
	movfw	REGA0
	call	mmc_spi_io	
; checksum not
	movlw	0xff
	call	mmc_spi_io

; wait for response 0x00 is OK
	clrf	WREG
	call	mmc_response							; repeatedly reads the MMC until we get the response as in WREG, if so return 0, else return 1 timeout
	btfss	WREG, 0
	bra		mmc_write_sector_p1
	movlw	D'5'
	bra		mmc_error

mmc_write_sector_p1:

; set bit 0 to 0 which indicates the beginning of the data block 
	movlw	0xfe
	call	mmc_spi_io

; point ot where data is, 512 bytes
	lfsr 	FSR0, SPI_RESULT_ADDRESS
	
; write the block of data based on the length
; REGA1:REGA0 is loop counter
    clrf    REGA1
    clrf    REGA0
mmc_write_sector_loop1

; get data
	movfw	POSTINC0
; WAS TEST 1, 2, 3, 4 etc
;	movfw	REGA0

; send data
	call	mmc_spi_io

; increment loop counter
	incf    REGA1, f
	incfsz  REGA0, f
	decf    REGA1, f

; test if 512
	movlw   HIGH    D'512'
	subwf   REGA1, W 
	btfss   STATUS, Z
	bra		mmc_write_sector_loop1
; REGA3 zero
	movlw   LOW     D'512'
	subwf   REGA0, W
	btfss   STATUS, Z
	bra		mmc_write_sector_loop1

; Send dummy checksum 
; when the last check sum is sent, the response should come back immediately.
; So, check the SPI FIFO MISO and make sure the status return 0xX5, the bit 3 through 0 should be 0x05
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte
; mask out lower nibble
	andlw	0x0f
; test for 0x05 
	sublw	0x05
	btfsc	STATUS, Z
	bra		mmc_write_sector_p2
; was not 0x05, write failed
	movlw	D'6'
	bra		mmc_error

mmc_write_sector_p2:

; if the status is already zero, the write hasn't finished yet and card is busy
	call	mmc_wait_for_write_finish				; wait a long tine testing for 0x00, if found returns WREG = 0, if timeout WREG = 1
	btfss	WREG, 0
	bra		mmc_write_sector_ok
; write timeout	
	movlw	D'7'
	bra		mmc_error

mmc_write_sector_ok:
	call	mmc_spi_cs_h	
; some clocks
	movlw	0xff
	call	mmc_spi_io								; sends data in W, returns byte read in WREG and spi_out, uses spi_byte

; disable card
	call	mmc_spi_cs_h

	return											; end subroutine write_mmc_sector

; end SPI routines for MMC

#endif ; OLD_CODE



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


calculate_timer0_reload:			; sets timer0_reload_h:timer0_reload_l from gps_speed
; reload for 256 prescaler =  65535 - ( (16000000 / 256)  / f) = 53035 for 5 Hz
; for 5 Hz = 53035
; 16000000 / 256 = 62500

	clrf 	REGA3
	clrf	REGA2
	movlw	HIGH	D'62500'
	movwf	REGA1
	movlw	LOW		D'62500'
	movwf	REGA0
; REGA3:REGA0 = 62500

	clrf	REGB3
	clrf	REGB2
	clrf	REGB1
	movff	gps_rate, REGB0
; REGB3:REGB0 = gps_rate

	call	divide             ; REGA / REGB -> REGA return carry set if overflow or division by zero
; REGA3:REGA0 = 62500 / gps_rate

	movff	REGA3, REGB3
	movff	REGA2, REGB2
	movff	REGA1, REGB1
	movff	REGA0, REGB0
; REGB3:REGB0 = 62500 / gps_rate

	clrf	REGA3
	clrf	REGA2
	movlw	0xff
	movwf	REGA1
	movwf	REGA0
; REGA3:REGA0 = 65535

	call	subtract			; REGA - REGB -> REGA return carry set if overflow
; REGA3:REGA0 = 65535 - (62500 / gps_rate)	

	movff	REGA1, timer0_reload_h
	movff	REGA0, timer0_reload_l

	return		; end subroutine calcluate_timer0_reload


field_to_decimal:		; data in RCREG to value3:value0, value3:value0 and NEGATIVE_ALTITUDE_FLAG must be cleared when starting parsing field,
						; sets NEGATIVE_altitude_FLAG if negative sign encountered anywhere in the field
						; if end result has not HAVE_DOT_FLAG set, it is in meters, multiply by 10 to get dm
						; if HAVE_DOT_FLAG is set, do not miltiply at all.
; only allow '-' and digits 0 through 9 	

; WAS
; to be done: we only want ONE digit after the dot, not fall in the trap of 0.34 m

; test if '-'
	movlw	'-'
	cpfseq	RCREG	
	bra		field_to_decimal_not_minus_sign
; negative sign
	bsf		NEGATIVE_ALTITUDE_FLAG
	return
field_to_decimal_not_minus_sign:

; test if '.' 
	movlw 	'.'
	cpfseq	RCREG
	bra		not_a_dot
; dot
	bsf		HAVE_DOT_FLAG
	clrf	after_dot_count
	return

not_a_dot:
; test if < '0'
	movlw 	D'47'
	cpfsgt	RCREG	; skip if RCREG > 47
	return

; test if > '9'
	movlw	D'58'
	cpfslt	RCREG	; skip if RCREG < 58
	return

; if a '.' was detected allow only one more digit
	btfss	HAVE_DOT_FLAG	
	bra		process_digit
; test if second digit
	movlw	D'1'
	cpfslt	after_dot_count	
; ignore any second digit after dot
	return

process_digit:
; convert ASCII digit to decimal
    movlw   D'48'
    subwf   RCREG, W           ; digit now in W

    movwf   digit_in

; value3:value0 *= 10
; value3:value0 += RCREG

	movff	value3, REGA3
    movff	value2, REGA2
    movff   value1, REGA1  
    movff   value0, REGA0  
; REGA = value3:value0

    clrf    REGB3
    clrf    REGB2
    clrf    REGB1
    movlw   D'10'
    movwf   REGB0
; REGB = 10
    
    call    multiply                                    ; 32 bit signed multiply  REGA * REGB -> REGA  return carry set if overflow
; REGA = value3:value0 * 10   

    clrf    REGB3
    clrf    REGB2
    clrf    REGB1
    movff   digit_in, REGB0   
; REGB = digit_in   

    call    add                                         ; 32 bit signed add  REGA + REGB -> REGA  return carry set if overflow
; REGA = (value2:value0 * 10) + digit_in

	movff	REGA3, value3
	movff	REGA2, value2
    movff	REGA1, value1
    movff	REGA0, value0

	incf	after_dot_count

	return		; end subroutine digits_to_altitude 


text_brownout:
   DA "brownout\n\r\0"

text_watchdog:
	DA "watchdog\n\r\0"

text_record_number:
	DA "record number \0"

text_eeprom_write_result:
	DA "EEPROM write result \0"

text_address:
	DA "address \0"

text_original:
	DA "original \0"

text_ready:
	DA "Ready\n\r\0"

text_no_mmc_present:
	DA "no sdcard present\n\r\0"

text_mmc_present:
	DA "sdcard present\n\r\0"

text_mmc_init_ok:
	DA "MMC init OK\n\r\0"

text_mmc_init_failed:
	DA "MMC init falied\n\r\0"

text_gps_rate:
	DA "GPS rate \0"

text_end_of_data:
	DA "End of data\n\r\0"

text_dac:
	DA "DAC \0"


;text_:
;	DA "\n\r\0"


;text_:
;	DA "\n\r\0"

text_help_menu:
; DO NOT FOLD THIS LINE!!!!!
; help menu, unfortunately gputils (gpasm) does not understand '\' at the end of a line for continuation, like in C.
; \n will empty buffer in Linux / Unix, do not use \n\r at the end, \0 writes zero and is string termination, so always end with \r\n\0. 
	DA	"\n\rPanteltje (c) gpss-0.5\n\r\n\r      Commands in test mode (#define TEST_MODE in asm)\n\rD int         set DAC  range 0-31, outout to pin 19\n\rd             pin 19 tristate\n\rh             help (this)\n\rO             DAC to output pin 19\n\ro             pin 19 tristate, default\n\rr int         read data record command\n\rS int         GPS output rate in Hz, default 5 Hz\n\rU int         set data record pointer command\n\rv             print status\n\rX             dump database command\n\r\0"


; EEPROM
; WARNING this will reset all EEPROM vars to zero!!!!!!
;	org	0xf00000



  	end
 
 
