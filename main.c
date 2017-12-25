/**
 * Used:
 * 06-imp-demo-FITkit3-src/Sources/main.c
 * Laboratorni uloha c. 1 z predmetu IMP
 * KSDK demo app - rtc_func/drivers/fsl_rtc.c
 **/

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "MK60D10.h"

/*
 * Macros
 */
#define	DELAY_OSC_STAB 0x600000
#define SECONDS_IN_A_DAY (86400U)
#define SECONDS_IN_AN_HOUR (3600U)
#define SECONDS_IN_A_MINUTE (60U)
#define HOURS_IN_A_DAY (24U)
#define MINUTES_IN_AN_HOUR (60U)

/*
 * Global variables
 */
char g_StrMenu[] =
    "\r\n"
    "Please choose the sub demo to run:\r\n"
    "1) Get current date time.\r\n"
    "2) Set current date time.\r\n"
    "3) Alarm trigger show.\r\n"
    "4) Second interrupt show (demo for 7s).\r\n";

/*
 * Types
 */
/* Structure used for storing hours, minutes and seconds from the time of the day */
typedef struct _rtc_time
{
    uint8_t hour;   // Range 0-23
    uint8_t minute; // Range 0-59
    uint8_t second; // Range 0-59
} dayTime;

/*
 * Prototypes
 */
void delay(long long bound);
void MCUInit(void);
void PinInit(void);
void UARTInit(UART_Type *base);
void RTCInit(void);
void resetTime(dayTime *rtc_time);
void SendCh(UART_Type *base, char ch);
char ReceiveCh(UART_Type *base);
void SendStr(UART_Type *base, char *s);
void beep(void);
void RTCGetTime(dayTime *time);
void RTCSetTime(dayTime *time);
void secondsToDayTime(uint32_t seconds, dayTime *time);
uint32_t dayTimeToSeconds(dayTime *time);
void dayTimeToStr(dayTime *dTime, char *strTime);
bool strToDayTime(dayTime *dTime, char *strTime);

/*
 * Main
 */
int main(void) {
	dayTime time;
	char recv_str[5], sendMsg[1024], buf[1024] = "";
	char c;
	uint8_t index;
	UART_Type *UARTChannel = UART5;

	MCUInit();
	PinInit();
	UARTInit(UARTChannel);
	RTCInit();

  while(1) {
	beep();
	SendStr(UARTChannel, g_StrMenu);
	SendStr(UARTChannel, "\r\nSelect:\r\n");
	index = ReceiveCh(UARTChannel);
	SendCh(UARTChannel, index);		// Link echo
	SendStr(UARTChannel, "\r\n");

	switch (index) {
		case '1':
			RTCGetTime(&time);
			dayTimeToStr(&time, buf);
			SendStr(UARTChannel, buf);
			SendStr(UARTChannel, "\r\n");
			break;
		case '2':
			SendStr(UARTChannel, "Write day time in format like: \"10:10:10\"\r\n");
			for (int i = 0; i < 8; i++) {
				c = ReceiveCh(UARTChannel);
				SendCh(UARTChannel, c);		// Link echo
				buf[i] = c;
			}
			SendStr(UARTChannel, "\r\n");
			if (strToDayTime(&time, buf)) {
				RTCSetTime(&time);
				RTCGetTime(&time);
				dayTimeToStr(&time, buf);
				SendStr(UARTChannel, buf);
				SendStr(UARTChannel, "\r\n");
			}
			else {
				SendStr(UARTChannel, "Invalid input format\r\n");
			}
			break;
		case '3':
			break;
		case '4':
			SendStr(UARTChannel, "Cas je:\r\n");
			for (long long i = 0; i < 6000; i++) {
				RTCGetTime(&time);
				dayTimeToStr(&time, buf);
				SendStr(UARTChannel, buf);
			}
			SendStr(UARTChannel, "\r\n");
			break;
		default:
			break;
	}
  }
}

/*
 * Delay function
 */
void delay(long long bound) {
  long long i;
  for(i = 0; i < bound; i++);
}

/*
 * MCU initialization - basic clock settings, watchdog turn off
 */
void MCUInit(void) {
	//MCG->C1 &= ~MCG_C1_CLKS_MASK;							// FLL for MCGCLKOUT
	//MCG->C1 |= MCG_C1_IREFS_MASK;							// Slow intern. ref. CLK for FLL
	MCG->C4 |= (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x01));	// DCO 48 MHz
	//MCG->C6 = 0x0000;										// FLL selected
	//SIM->CLKDIV1 &= ~SIM_CLKDIV1_OUTDIV1_MASK;			// Core clock divided by 1
	//SIM->CLKDIV1 &= ~SIM_CLKDIV1_OUTDIV2_MASK;			// Bus clock divided by 1
	WDOG->STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;				// Disable watchdog
}

/*
 * Pins initialization, enable clocks
 */
void PinInit(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  		// Enable clock for PORTA
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;  		// Enable clock for PORTB
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;  		// Enable clock for PORTE
	SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;			// Enable clock for UART5
	SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;			// Enable access to RTC

	PORTE->PCR[8] = PORT_PCR_MUX(0x03); 		// UART5_TX
	PORTE->PCR[9] = PORT_PCR_MUX(0x03); 		// UART5_RX

	PORTB->PCR[5] = PORT_PCR_MUX(0x01); 		// MCU_LED0 D9
	PORTB->PCR[4] = PORT_PCR_MUX(0x01); 		// MCU_LED1 D10
	PORTB->PCR[3] = PORT_PCR_MUX(0x01); 		// MCU_LED2 D11

	PORTA->PCR[4] = PORT_PCR_MUX(0x01); 		// Beeper (PTA4)

	PTA->PDDR = GPIO_PDDR_PDD(0x0010); 			// PTA4 as output
	PTB->PDDR = GPIO_PDDR_PDD(0x0038); 			// PTB3/4/5 as output
	//PTB->PDOR |= GPIO_PDOR_PDO(0x0038);			// Turn off the LEDs
}

/*
 * UART initialization settings
 */
void UARTInit(UART_Type *base) {
	base->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);	// Transmitter and receiver turn off

	base->BDH = 0x00;
	base->BDL = 0x1A;						// Baud rate 115 200 Bd
	base->C4 = 0x01;						// Baud rate fine adjust 1/32, match address mode disabled
	//base->C1 = 0x00;						// 8 data bitov, bez parity
	//base->C2 = (0 | UART_C2_TCIE_MASK);	// transmission complete interrupt enable
	//NVIC_EnableIRQ(UART0_RX_TX_IRQn);		// Enable UART0 receive/transmit interrupt
	//base->C3 = 0x00;
	//base->MA1 = 0x00;						// no match address (mode disabled in C4)
	//base->MA2 = 0x00;						// no match address (mode disabled in C4)
	//base->S1 |= 0x1F;						// clear IDLE, OR, NF, FE, PF
	//base->S2 |= 0xC0;						// clear LBKDIF, RXEDGIF

	base->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);	// Transmitter and receiver turn on
}

/*
 * Real Time Clock initialization settings
 */
void RTCInit(void) {
	RTC->CR |= RTC_CR_SWR_MASK;		// reset all RTC registers
	RTC->CR &= ~RTC_CR_SWR_MASK;	// clear SWR
	RTC->TCR = 0x0000;				// clear compensation interval/time
	RTC->TSR = 0x0000;				// clear TOF and TIF
	RTC->CR |= RTC_CR_OSCE_MASK;	// enable oscillator
	delay(DELAY_OSC_STAB);			// wait for the oscillator to stabilize
	RTC->SR |= RTC_SR_TCE_MASK;		// enable counter
	RTC->TAR = 0x0000;				// clear TAF
}

/*
 * Fill the dayTime structure by the converted value of RTC counter
 */
void RTCGetTime(dayTime *time)
{
    uint32_t seconds = 0;
    seconds = RTC->TSR;
    secondsToDayTime(seconds, time);
}

/*
 * Fill the RTC counter by the converted value from dayTime structure
 */
void RTCSetTime(dayTime *time)
{
    uint32_t seconds;
    seconds = dayTimeToSeconds(time);
    RTC->SR &= ~RTC_SR_TCE_MASK;		// Disable counter
    RTC->TSR = seconds;					// Write seconds value to the register
    RTC->SR |= RTC_SR_TCE_MASK;			// Enable counter
}

/*
 * Convert seconds from RTC counter to dayTime
 */
void secondsToDayTime(uint32_t seconds, dayTime *time)
{
    uint32_t secondsRemaining = seconds;

    /* Relativize seconds with respect to max number of seconds in a day*/
    secondsRemaining = secondsRemaining % SECONDS_IN_A_DAY;

    /* Compute hours, minutes and seconds  */
    time->hour = secondsRemaining / SECONDS_IN_AN_HOUR;
    secondsRemaining = secondsRemaining % SECONDS_IN_AN_HOUR;
    time->minute = secondsRemaining / SECONDS_IN_A_MINUTE;
    time->second = secondsRemaining % SECONDS_IN_A_MINUTE;
}

/*
 * Convert dayTime values to single value of seconds
 */
uint32_t dayTimeToSeconds(dayTime *time) {
	uint32_t seconds = time->hour * SECONDS_IN_AN_HOUR;
	seconds += time->minute * SECONDS_IN_A_MINUTE;
	seconds += time->second;
	return seconds;
}

/*
 * Convert dayTime values to string like "\r00:00:00"
 */
void dayTimeToStr(dayTime *dTime, char *strTime) {
	strTime[0] = '\r';
	strTime[1] = ((dTime->hour / 10) + 0x30);
	strTime[2] = ((dTime->hour % 10) + 0x30);
	strTime[3] = ':';
	strTime[4] = ((dTime->minute / 10) + 0x30);
	strTime[5] = ((dTime->minute % 10) + 0x30);
	strTime[6] = ':';
	strTime[7] = ((dTime->second / 10) + 0x30);
	strTime[8] = ((dTime->second % 10) + 0x30);
}

/*
 * Convert string like "00:00:00" to dayTime values
 */
bool strToDayTime(dayTime *dTime, char *strTime) {
	bool succ = false;
	uint32_t result;
	uint16_t hour, minute, second;

	result = sscanf(strTime, "%02hd:%02hd:%02hd", &hour, &minute, &second);
	/* The return value must be 3U - 3 input items successfully matched */
	if (result != 3U) {
		succ = false;
	}
	else {
		/* Check the limits */
		if ((hour >= HOURS_IN_A_DAY) || (minute >= MINUTES_IN_AN_HOUR) ||
			(second >= SECONDS_IN_AN_HOUR)) {
			succ = false;
		}
		else {
			dTime->hour = (uint8_t)hour;
			dTime->minute = (uint8_t)minute;
			dTime->second = (uint8_t)second;
			succ = true;
		}
	}
	return succ;
}

/*
 * Reset time
 */
void resetTime(dayTime *rtc_time) {
	rtc_time->hour = 0U;
	rtc_time->minute = 0U;
	rtc_time->second = 0U;
}

/*
 * Transmit one character through UART
 */
void SendCh(UART_Type *base, char ch)  {
    while (!(base->S1 & UART_S1_TDRE_MASK) && !(base->S1 & UART_S1_TC_MASK));
    base->D = ch;
}

/*
 * Receive one character through UART
 */
char ReceiveCh(UART_Type *base) {
	while (!(base->S1 & UART_S1_RDRF_MASK));
	return base->D;
}

/*
 * Transmit string terminated by 0
 */
void SendStr(UART_Type *base, char *s)  {
	int i = 0;
	while (s[i] != 0) {
		SendCh(base, s[i++]);
	}
}

/*
 * Beep from the beeper on PTA4
 */
void beep(void) {
	int q;
	for (q = 0; q < 500; q++) {
    	PTA->PDOR = GPIO_PDOR_PDO(0x0010);
    	delay(500);
    	PTA->PDOR = GPIO_PDOR_PDO(0x0000);
    	delay(500);
    }
}
