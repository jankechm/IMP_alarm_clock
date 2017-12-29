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
#define DEF_BEEP_CYCLES (500U)
#define DEF_BEEP_HALF_PERIOD (500U)
#define	DELAY_OSC_STAB 0x600000
#define	BUF_SIZE 1024
#define SECONDS_IN_A_DAY (86400U)
#define SECONDS_IN_AN_HOUR (3600U)
#define SECONDS_IN_A_MINUTE (60U)
#define HOURS_IN_A_DAY (24U)
#define MINUTES_IN_AN_HOUR (60U)
#define LED_D9  0x20				// PORT B, bit 5
#define LED_D10 0x10				// PORT B, bit 4
#define LED_D11 0x8					// PORT B, bit 3
#define LED_D12 0x4					// PORT B, bit 2
#define ALL_MCU_LEDS 0x003C			// PORT B, bits 2-5
#define BTN_SW6 0x800 				// Port E, bit 11

/*
 * Types
 */
/* Structure used for storing hours, minutes and seconds from the time of the day */
typedef struct _rtc_time {
    uint8_t hour;   // Range 0-23
    uint8_t minute; // Range 0-59
    uint8_t second; // Range 0-59
} dayTime;

/* Structure used for storing alarm settings */
typedef struct _alarmOptions {
	bool standBy;			// is or is not in standBy mode
	uint8_t light;			// light signalization
	uint8_t sound;			// sound signalization
	uint8_t againCount;		// number of attempts to wake up
	dayTime againInterval;	// time after which the next attempt to wake up triggers
	dayTime time;			// alarm time
} alarmOptions;

/*
 * Global variables
 */
UART_Type *g_UARTChannel = UART5;
alarmOptions g_alarmOps = {false, 1, 1, 2, {0, 10, 0}, {0}};
volatile bool g_alarmRings = false;
volatile uint8_t g_alarmAttempts = 0;
static char g_StrMenu[] =
    "\r\n"
    "Terminal options:\r\n"
	"1: Show current time, self updating.\r\n"
    "2: Set current day-time.\r\n"
    "3: Set alarm time.\r\n"
	"4: Get alarm time.\r\n"
	"l: Choose the alarm light signalization.\r\n"
	"s: Choose the alarm sound signalization.\r\n"
	"i: Set the interval between attempts to wake up.\r\n"
	"a: Set the number of attempts to wake up.\r\n"
	"h: Show this help message.\r\n"
	"\r\n"
	"HW options:\r\n"
	"Button SW6: When alarm triggers, turn it off.\r\n"
	"            Toggle alarm stand-by mode on/off.\r\n";

/*
 * Prototypes
 */
void delay(long long bound);
void MCUInit(void);
void PinInit(void);
void UARTInit(UART_Type *base);
void RTCInit(void);
void PITInit(void);
void PIT0Init(void);
void PIT1Init(void);
void PIT2Init(void);
void SendCh(UART_Type *base, char ch);
char ReceiveCh(UART_Type *base);
void SendStr(UART_Type *base, char *s);
void beep(uint32_t cycles, uint32_t halfPeriod);
void RTCGetTime(dayTime *time);
void RTCSetTime(dayTime *time);
void RTCResetTime(void);
void stopAlarmSignalization(void);
void RTCAlarmStandByOn(void);
void RTCAlarmStandByOff(void);
void RTCActualizeAlarm(void);
void secondsToDayTime(uint32_t seconds, dayTime *time);
uint32_t dayTimeToSeconds(dayTime *time);
void dayTimeToStr(dayTime *dTime, char *strTime);
bool strToDayTime(dayTime *dTime, char *strTime);
void lightSignalize1(void);
void lightSignalize2(void);
void lightSignalize3(void);
void soundSignalize1(void);
void soundSignalize2(void);
void soundSignalize3(void);

/*
 * Main
 */
int main(void) {
	dayTime time;
	char c, buf[BUF_SIZE] = "";
	uint8_t opt, sigOpt;

	MCUInit();
	PinInit();
	UARTInit(g_UARTChannel);
	RTCInit();
	PITInit();
	SendStr(g_UARTChannel, g_StrMenu);

  while(1) {
	beep(DEF_BEEP_CYCLES, DEF_BEEP_HALF_PERIOD);
	SendStr(g_UARTChannel, "\r\nSelect option:\r\n");
	opt = ReceiveCh(g_UARTChannel);
	SendCh(g_UARTChannel, opt);		// Link echo
	SendStr(g_UARTChannel, "\r\n");

	switch (opt) {
		case '1':		// Getting time by RTC seconds interrupt
			RTC->IER |= RTC_IER_TSIE_MASK;		// enable RTC seconds interrupt
			opt = ReceiveCh(g_UARTChannel);
			RTC->IER &= ~RTC_IER_TSIE_MASK;		// disable seconds interrupt
			SendStr(g_UARTChannel, "\r\n");
			SendCh(g_UARTChannel, opt);
			SendStr(g_UARTChannel, "\r\n");
			break;
		case '2':		// Set time
			SendStr(g_UARTChannel, "Set time\r\n");
			SendStr(g_UARTChannel, "Write day time in format: \"hh:mm:ss\"\r\n");
			for (int i = 0; i < 8; i++) {
				c = ReceiveCh(g_UARTChannel);
				SendCh(g_UARTChannel, c);		// Link echo
				buf[i] = c;
			}
			SendStr(g_UARTChannel, "\r\n");
			if (strToDayTime(&time, buf)) {
				RTCSetTime(&time);
				SendStr(g_UARTChannel, "OK\r\n");
			}
			else {
				SendStr(g_UARTChannel, "Invalid input format\r\n");
			}
			break;
		case '3':		// Set alarm
			SendStr(g_UARTChannel, "Set alarm\r\n");
			SendStr(g_UARTChannel, "Write alarm time in format: \"hh:mm:ss\"\r\n");
			for (int i = 0; i < 8; i++) {
				c = ReceiveCh(g_UARTChannel);
				SendCh(g_UARTChannel, c);		// Link echo
				buf[i] = c;
			}
			SendStr(g_UARTChannel, "\r\n");
			if (strToDayTime(&time, buf)) {
				g_alarmOps.time = time;
				RTCAlarmStandByOn();
				SendStr(g_UARTChannel, "OK\r\n");
			}
			else {
				SendStr(g_UARTChannel, "Invalid input format\r\n");
			}
			break;

		case '4':		// Get alarm time
			dayTimeToStr(&(g_alarmOps.time), buf);
			SendStr(g_UARTChannel, buf);
			SendStr(g_UARTChannel, "\r\n");
			break;
		case 'l':
			SendStr(g_UARTChannel, "Choose the alarm light signalization 0-3 (0 = no light)\r\n");
			sigOpt = ReceiveCh(g_UARTChannel);
			SendCh(g_UARTChannel, sigOpt);		// Link echo
			SendStr(g_UARTChannel, "\r\n");
			switch (sigOpt) {
				case '0':
					g_alarmOps.light = 0;
					break;
				case '1':
					g_alarmOps.light = 1;
					break;
				case '2':
					g_alarmOps.light = 2;
					break;
				case '3':
					g_alarmOps.light = 3;
					break;
				default:
					SendStr(g_UARTChannel, "Bad option\r\n");
					break;
			}
			break;
		case 's':
			SendStr(g_UARTChannel, "Choose the alarm sound signalization 0-3 (0 = no sound)\r\n");
			sigOpt = ReceiveCh(g_UARTChannel);
			SendCh(g_UARTChannel, sigOpt);		// Link echo
			SendStr(g_UARTChannel, "\r\n");
			switch (sigOpt) {
				case '0':
					g_alarmOps.sound = 0;
					break;
				case '1':
					g_alarmOps.sound = 1;
					break;
				case '2':
					g_alarmOps.sound = 2;
					break;
				case '3':
					g_alarmOps.sound = 3;
					break;
				default:
					SendStr(g_UARTChannel, "Bad option\r\n");
					break;
			}
			break;
		case 'i':
			SendStr(g_UARTChannel, "Set the interval, in which the alarm "
					"will be triggered again, if it was not turned off.\r\n"
					"Write the time interval in format: \"hh:mm:ss\"\r\n");
			for (int i = 0; i < 8; i++) {
				c = ReceiveCh(g_UARTChannel);
				SendCh(g_UARTChannel, c);		// Link echo
				buf[i] = c;
			}
			SendStr(g_UARTChannel, "\r\n");
			if (strToDayTime(&(g_alarmOps.againInterval), buf)) {
				SendStr(g_UARTChannel, "OK\r\n");
			}
			else {
				SendStr(g_UARTChannel, "Invalid input format\r\n");
			}
			break;
		case 'a':
			SendStr(g_UARTChannel, "Set the number of attempts to wake up (0-5)\r\n");
			c = ReceiveCh(g_UARTChannel);
			SendCh(g_UARTChannel, c);		// Link echo
			SendStr(g_UARTChannel, "\r\n");
			if (c >= '0' && c <= '5') {
				g_alarmOps.againCount = c - 0x30;
			}
			else {
				SendStr(g_UARTChannel, "Bad option\r\n");
			}
			break;
		case 'h':
			SendStr(g_UARTChannel, g_StrMenu);
			break;
		default:
			SendStr(g_UARTChannel, "Bad option\r\n");
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
	MCG->C4 |= (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x01));	// DCO 48 MHz
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
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;			// Enable clock for PIT
	SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;			// Enable access to RTC

	PORTE->PCR[8] = PORT_PCR_MUX(0x03); 		// UART5_TX
	PORTE->PCR[9] = PORT_PCR_MUX(0x03); 		// UART5_RX

	PORTB->PCR[5] = PORT_PCR_MUX(0x01); 		// MCU_LED0 D9
	PORTB->PCR[4] = PORT_PCR_MUX(0x01); 		// MCU_LED1 D10
	PORTB->PCR[3] = PORT_PCR_MUX(0x01); 		// MCU_LED2 D11
	PORTB->PCR[2] = PORT_PCR_MUX(0x01); 		// MCU_LED3 D12

	PORTE->PCR[11] |= PORT_PCR_ISF_MASK;		// Clear interrupt flag
	PORTE->PCR[11] |= PORT_PCR_IRQC(0x0A);		// Interrupt on falling edge
	PORTE->PCR[11] |= PORT_PCR_MUX(0x01);		// MCU_BUTTON4 SW6
	PORTE->PCR[11] |= PORT_PCR_PE_MASK;			// Pull resistor enable
	PORTE->PCR[11] |= PORT_PCR_PS_MASK;			// Pull up resistor select

	PORTA->PCR[4] = PORT_PCR_MUX(0x01); 		// Beeper (PTA4)

	PTA->PDDR = GPIO_PDDR_PDD(0x0010); 			// PTA4 as output
	PTB->PDDR = GPIO_PDDR_PDD(ALL_MCU_LEDS); 	// PTB2/3/4/5 as output
	PTB->PDOR |= GPIO_PDOR_PDO(ALL_MCU_LEDS);	// Turn off the LEDs

	NVIC_ClearPendingIRQ(PORTE_IRQn); 			// clear pending interrupts of PORTE
	NVIC_EnableIRQ(PORTE_IRQn);					// enable PORTE interrupt
}

/*
 * UART initialization settings
 */
void UARTInit(UART_Type *base) {
	base->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);	// Transmitter and receiver turn off
	base->BDH = 0x00;
	base->BDL = 0x1A;	// Baud rate 115 200 Bd
	base->C4 = 0x01;	// Baud rate fine adjust 1/32, match address mode disabled
	base->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);	// Transmitter and receiver turn on
}

/*
 * Real Time Clock initialization settings
 */
void RTCInit(void) {
	RTC->CR |= RTC_CR_SWR_MASK;				// reset all RTC registers
	RTC->CR &= ~RTC_CR_SWR_MASK;			// clear SWR
	RTC->TCR = 0x0000;						// clear compensation interval/time
	RTC->TSR = 0x0001;						// clear TOF and TIF flags
	RTC->CR |= RTC_CR_OSCE_MASK;			// enable oscillator
	delay(DELAY_OSC_STAB);					// wait for the oscillator to stabilize
	RTC->SR |= RTC_SR_TCE_MASK;				// enable counter
	RTC->TAR = 0x0000;						// clear TAF flag
	RTC->IER = 0x0000;						// disable all RTC interrupts
	NVIC_ClearPendingIRQ(RTC_IRQn); 		// clear pending RTC interrupts
	NVIC_ClearPendingIRQ(RTC_Seconds_IRQn); // clear pending RTC_Seconds interrupts
	NVIC_EnableIRQ(RTC_IRQn);				// enable RTC interrupt
	NVIC_EnableIRQ(RTC_Seconds_IRQn);		// enable RTC_Seconds interrupt
}

/*
 * Periodic interrupt timer initialization
 */
void PITInit() {
	PIT->MCR = 0x00;	// enable PIT module
	PIT0Init();			// initialize PIT0
	PIT1Init();			// initialize PIT1
	PIT2Init();			// initialize PIT2
}

/*
 * PIT0 initialization settings
 */
void PIT0Init(void) {
	PIT->CHANNEL[0].LDVAL = 0xB71AFF;				// 250 ms period
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;	// timer interrupt enable
	PIT->CHANNEL[0].TFLG = 0x01;					// clear interrupt flag
	NVIC_ClearPendingIRQ(PIT0_IRQn); 				// clear pending interrupts
	NVIC_EnableIRQ(PIT0_IRQn);						// enable PIT0 interrupt
}

/*
 * PIT1 initialization settings
 */
void PIT1Init(void) {
	PIT->CHANNEL[1].LDVAL = 0x89543FF;				// 3000 ms period
	PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TIE_MASK;	// timer interrupt enable
	PIT->CHANNEL[1].TFLG = 0x01;					// clear interrupt flag
	NVIC_ClearPendingIRQ(PIT1_IRQn); 				// clear pending interrupts
	NVIC_EnableIRQ(PIT1_IRQn);						// enable PIT1 interrupt
}

/*
 * PIT2 initialization settings
 */
void PIT2Init(void) {
	PIT->CHANNEL[2].LDVAL = 0x1406F3FF;				// 7000 ms period
	PIT->CHANNEL[2].TCTRL |= PIT_TCTRL_TIE_MASK;	// timer interrupt enable
	PIT->CHANNEL[2].TFLG = 0x01;					// clear interrupt flag
	NVIC_ClearPendingIRQ(PIT2_IRQn); 				// clear pending interrupts
	NVIC_EnableIRQ(PIT2_IRQn);						// enable PIT2 interrupt
}

/*
 * Fill the dayTime structure by the converted value from RTC seconds register
 */
void RTCGetTime(dayTime *time)
{
    uint32_t seconds = RTC->TSR;
    secondsToDayTime(seconds, time);
}

/*
 * Fill the RTC seconds register by the converted value from dayTime structure
 */
void RTCSetTime(dayTime *time)
{
    uint32_t seconds = dayTimeToSeconds(time);	// Get seconds from dayTime
    RTC->SR &= ~RTC_SR_TCE_MASK;				// Disable counter
    RTC->TSR = seconds;							// Write seconds value to the register
    RTC->SR |= RTC_SR_TCE_MASK;					// Enable counter
    RTCActualizeAlarm();						// Actualize RTC alarm register
}

/*
 * Reset the RTC seconds register
 */
void RTCResetTime(void) {
	RTC->SR &= ~RTC_SR_TCE_MASK;		// Disable counter
	RTC->TSR = 0U;						// Reset seconds register
	RTC->SR |= RTC_SR_TCE_MASK;			// Enable counter
}

/*
 * Stop the light and sound alarm signalization
 */
void stopAlarmSignalization(void) {
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;	// PIT0 disable
	PIT->CHANNEL[0].TFLG = 0x01;					// clear PIT0 interrupt flag
	PTB->PDOR |= ALL_MCU_LEDS;						// Turn off all MCU LEDS
	PIT->CHANNEL[1].TCTRL &= ~PIT_TCTRL_TEN_MASK;	// PIT1 disable
	PIT->CHANNEL[1].TFLG = 0x01;					// clear PIT1 interrupt flag
	PTA->PDOR = GPIO_PDOR_PDO(0x0000);				// Turn off beeper
	g_alarmRings = false;
}

/*
 * Turn on the alarm stand-by mode
 */
void RTCAlarmStandByOn(void) {
	g_alarmAttempts = 0;						// Reset attempts to wake up
	g_alarmOps.standBy = true;
	RTCActualizeAlarm();						// Actualize RTC alarm register
	SendStr(g_UARTChannel, "\r\nAlarm in stand-by mode.\r\n");
	RTC->IER |= RTC_IER_TAIE_MASK;				// Enable alarm interrupt
	PTB->PCOR = LED_D12;						// Turn on D12 - alarm stand-by indicator
}

/*
 * Turn off the alarm stand-by mode
 */
void RTCAlarmStandByOff(void) {
	g_alarmAttempts = 0;						// Reset attempts to wake up
	g_alarmOps.standBy = false;
	SendStr(g_UARTChannel, "\r\nAlarm not in stand-by mode.\r\n");
	RTC->IER &= ~RTC_IER_TAIE_MASK;				// Disable alarm interrupt
	PTB->PSOR = LED_D12;						// Turn off D12 - alarm stand-by indicator
}

/*
 * Fill the RTC alarm register by the converted value from dayTime structure
 * considering the absolute value of seconds in TSR and TAR registers
 */
void RTCActualizeAlarm(void) {
	uint32_t currSeconds = RTC->TSR;
	uint32_t alarmSeconds = dayTimeToSeconds(&(g_alarmOps.time));

	/* Elapsed days since RTC counter starts counting from 0 */
	uint32_t daysElapsed = currSeconds / SECONDS_IN_A_DAY;
	/* Make an absolute value of alarmSeconds (include elapsed days) */
	alarmSeconds += daysElapsed * SECONDS_IN_A_DAY;
	/* If the alarm is for another day, increment it by 1 day*/
	if (alarmSeconds < currSeconds) {
		alarmSeconds += SECONDS_IN_A_DAY;
	}
	RTC->TAR = alarmSeconds;					// Fill the alarm register
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
 * Override the RTC IRQ handler.
 */
void RTC_IRQHandler(void)
{
    dayTime time;

    /* Check the alarm flag */
	if (RTC->SR & RTC_SR_TAF_MASK) {
    	/* Clear alarm flag */
    	RTC->TAR = 0U;
    	/* Activate alarm signalizing */
		SendStr(g_UARTChannel, "\r\nAlarm!\r\n");
		PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;	// PIT0 enable
		PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK;	// PIT1 enable
		PIT->CHANNEL[2].TCTRL |= PIT_TCTRL_TEN_MASK;	// PIT2 enable
		g_alarmRings = true;
    }
}

void RTC_Seconds_IRQHandler(void)
{
	dayTime time;
	char buf[] = "\r00:00:00";

	RTCGetTime(&time);
	dayTimeToStr(&time, buf);
	SendStr(g_UARTChannel, buf);
}

/*
 * Override the PIT0 IRQ handler.
 */
void PIT0_IRQHandler(void) {
	switch (g_alarmOps.light) {
		case 0:
			PTB->PDOR |= ALL_MCU_LEDS;						// Turn off all MCU LEDS
			break;
		case 1:
			lightSignalize1();
			break;
		case 2:
			lightSignalize2();
			break;
		case 3:
			lightSignalize3();
			break;
		default:
			lightSignalize1();
			break;
	}
	PIT->CHANNEL[0].TFLG = 0x01;			// clear interrupt flag
}

/*
 * Override the PIT1 IRQ handler.
 */
void PIT1_IRQHandler(void) {
	switch (g_alarmOps.sound) {
		case 0:
			PTA->PDOR = GPIO_PDOR_PDO(0x0000);	// Turn off beeper
			break;
		case 1:
			soundSignalize1();
			break;
		case 2:
			soundSignalize2();
			break;
		case 3:
			soundSignalize3();
			break;
		default:
			soundSignalize1();
			break;
	}
	PIT->CHANNEL[1].TFLG = 0x01;			// clear interrupt flag
}

/*
 * Override the PIT2 IRQ handler.
 */
void PIT2_IRQHandler(void) {
	uint32_t currSeconds = RTC->TSR;

	stopAlarmSignalization();						// Turn off alarm light and sound signalization
	if (g_alarmAttempts < g_alarmOps.againCount) {
		RTC->TAR = currSeconds + dayTimeToSeconds(&(g_alarmOps.againInterval));	// Schedule next alarm triggering
		PTB->PCOR = LED_D12;						// Turn on D12 - alarm stand-by indicator
		g_alarmAttempts++;
	}
	else {
		RTCAlarmStandByOff();		// Turn off alarm stand-by mode
	}
	PIT->CHANNEL[2].TCTRL &= ~PIT_TCTRL_TEN_MASK;	// PIT2 disable
	PIT->CHANNEL[2].TFLG = 0x01;					// clear interrupt flag
}

/*
 * Override the PORTE IRQ handler.
 */
void PORTE_IRQHandler(void) {
	/* Check if the interrupt comes from SW6 */
	if (PORTE->ISFR & BTN_SW6) {
		/* Toggle alarm stand-by mode on/off */
		if (g_alarmOps.standBy) {
			stopAlarmSignalization();	// Turn off alarm light and sound signalization
			RTCAlarmStandByOff();		// Turn off alarm stand-by mode
			PIT->CHANNEL[2].TCTRL &= ~PIT_TCTRL_TEN_MASK;	// PIT2 disable
			PIT->CHANNEL[2].TFLG = 0x01;					// clear interrupt flag
		}
		else {
			RTCAlarmStandByOn();		// Turn on alarm stand-by mode
		}
		beep(DEF_BEEP_CYCLES, DEF_BEEP_HALF_PERIOD);
		PORTE->PCR[11] |= PORT_PCR_ISF_MASK;	// Clear interrupt flag
	}
}

/*
 * Start the light signalization 1 witch MCU LEDs
 */
void lightSignalize1(void) {
	switch (PTB->PDOR) {
		case ALL_MCU_LEDS:
			PTB->PTOR = LED_D9;					// turn on D9
			break;
		case ALL_MCU_LEDS & ~LED_D9:
			PTB->PTOR = (LED_D9 | LED_D10); 	// turn off D9, turn on D10
			break;
		case ALL_MCU_LEDS & ~LED_D10:
			PTB->PTOR = (LED_D10 | LED_D11); 	// turn off D10, turn on D11
			break;
		case ALL_MCU_LEDS & ~LED_D11:
			PTB->PTOR = (LED_D11 | LED_D12); 	// turn off D11, turn on D12
			break;
		case ALL_MCU_LEDS & ~LED_D12:
			PTB->PTOR = (LED_D12 | LED_D9);		// turn off D12, turn on D9
			break;
		default:
			PTB->PDOR |= ALL_MCU_LEDS;			// turn off all MCU LEDs
			break;
	}
}

/*
 * Start the light signalization 2 witch MCU LEDs
 */
void lightSignalize2(void) {
	switch (PTB->PDOR) {
		case ALL_MCU_LEDS:
			PTB->PTOR = LED_D12;				// turn on D12
			break;
		case ALL_MCU_LEDS & ~LED_D12:
			PTB->PTOR = (LED_D12 | LED_D11); 	// turn off D12, turn on D11
			break;
		case ALL_MCU_LEDS & ~LED_D11:
			PTB->PTOR = (LED_D11 | LED_D10); 	// turn off D11, turn on D10
			break;
		case ALL_MCU_LEDS & ~LED_D10:
			PTB->PTOR = (LED_D10 | LED_D9); 	// turn off D10, turn on D9
			break;
		case ALL_MCU_LEDS & ~LED_D9:
			PTB->PTOR = (LED_D9 | LED_D12); 	// turn off D9, turn on D12
			break;
		default:
			PTB->PDOR |= ALL_MCU_LEDS;			// turn off all MCU LEDs
			break;
	}
}

/*
 * Start the light signalization 3 witch MCU LEDs
 */
void lightSignalize3(void) {
	switch (PTB->PDOR) {
		case ALL_MCU_LEDS:
			PTB->PTOR = (LED_D10 | LED_D11);	// turn on D10, D11
			break;
		case ALL_MCU_LEDS & ~(LED_D10 | LED_D11):
			PTB->PTOR = ALL_MCU_LEDS; 			// turn off D10, D11; turn on D9, D12
			break;
		case ALL_MCU_LEDS & ~(LED_D9 | LED_D12):
			PTB->PTOR = (LED_D9 | LED_D12); 	// turn off D9, D12
			break;
		default:
			PTB->PDOR |= ALL_MCU_LEDS;			// turn off all MCU LEDs
			break;
	}
}

/*
 * Start the sound signalization 1 witch the MCU beeper
 */
void soundSignalize1(void) {
	beep(500U, 500U);
	beep(781U, 400U);
	beep(1388U, 300U);
}

/*
 * Start the sound signalization 2 witch the MCU beeper
 */
void soundSignalize2(void) {
	beep(1388U, 300U);
	beep(781U, 400U);
	beep(500U, 500U);
}

/*
 * Start the sound signalization 3 witch the MCU beeper
 */
void soundSignalize3(void) {
	beep(781U, 400U);
	beep(1388U, 300U);
	beep(781U, 400U);
	beep(500U, 500U);
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
void beep(uint32_t cycles, uint32_t halfPeriod) {
	uint32_t q;
	for (q = 0; q < cycles; q++) {
		PTA->PDOR = GPIO_PDOR_PDO(0x0010);
		delay(halfPeriod);
		PTA->PDOR = GPIO_PDOR_PDO(0x0000);
		delay(halfPeriod);
	}
}
