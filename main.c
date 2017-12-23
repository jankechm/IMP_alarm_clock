/**
 * Pouzite:
 * 06-imp-demo-FITkit3-src/Sources/main.c
 * Laboratorni uloha c. 1 z predmetu IMP
 **/

#include <string.h>
#include "MK60D10.h"

/*
 * Makra
 */
/* Cas pre stabilizaciu oscilatora */
#define	DELAY_OSC_STAB 0x600000

/*
 * Typy
 */
/* Struktura pouzita na ukladanie casu */
typedef struct _rtc_time
{
    uint8_t hour;   // Rozsah 0-23
    uint8_t minute; // Rozsah 0-59
    uint8_t second; // Rozsah 0-59
} *rtc_time_t;

/*
 * Prototypy
 */
void delay(long long bound);
void MCUInit(void);
void PinInit(void);
void UARTInit(UART_Type *base);
void RTCInit(void);
void resetTime(rtc_time_t rtc_time);
void SendCh(UART_Type *base, char ch);
char ReceiveCh(UART_Type *base);
void SendStr(UART_Type *base, char *s);
void beep(void);

/*
 * Main
 */
int main(void) {
	rtc_time_t time;
	char recv_str[5], sendMsg[1024];
	char c;
	int n;

	MCUInit();
	PinInit();
	UARTInit(UART5);
	//RTCInit();

  while(1) {
	beep();
	SendStr(UART5, "\r\n");
	SendStr(UART5, "Prompt");
	SendStr(UART5, "\r\n");

	for(n = 0; n < 4; n++) {
	  c = ReceiveCh(UART5);
	  SendCh(UART5, c);		// Prijaty znak se hned vysle - echo linky
	  recv_str[n] = c;	// Postupne se uklada do pole
	  beep();
	}
	//beep();
	SendStr(UART5, "\r\n");
	strcpy(sendMsg, "Poslal si: ");
	strcat(sendMsg, recv_str);
	SendStr(UART5, sendMsg);
  }
}

/*
 * Funkcia oneskorenia
 */
void delay(long long bound) {
  long long i;
  for(i = 0; i < bound; i++);
}

/*
 * Inicializacia MCU - zakladne nastavenie hodin, vypnutie watchdogu
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
 * Inicializacia pinov, povolenie hodin
 */
void PinInit(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  		// Povolit hodiny pre PORTA
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;  		// Povolit hodiny pre PORTB
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;  		// Povolit hodiny pre PORTE
	//SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;			// Povolit hodiny pre UART0
	SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;			// Povolit hodiny pre UART5
	//SIM->SOPT5 = 0x00;							// UART0/1 receive/transmit data src on RX/TX pin

	//PORTA->PCR[1] = (0 | PORT_PCR_MUX(0x02)); 		// UART0_RX
	//PORTA->PCR[2] = (0 | PORT_PCR_MUX(0x02)); 		// UART0_TX
	PORTE->PCR[8] = (0 | PORT_PCR_MUX(0x03)); 	// UART5_TX
	PORTE->PCR[9] = (0 | PORT_PCR_MUX(0x03)); 	// UART5_RX

	PORTB->PCR[5] = PORT_PCR_MUX(0x01); 		// MCU_LED0 D9
	PORTB->PCR[4] = PORT_PCR_MUX(0x01); 		// MCU_LED1 D10
	PORTB->PCR[3] = PORT_PCR_MUX(0x01); 		// MCU_LED2 D11

	PORTA->PCR[4] = (0 | PORT_PCR_MUX(0x01)); 	// Beeper (PTA4)

	PTA->PDDR = GPIO_PDDR_PDD(0x0010); 			// PTA4 pin bude vystupny
	PTB->PDDR = GPIO_PDDR_PDD(0x0038); 			// PTB3/4/5 bude vystupny
	//GPIOE->PDDR = GPIO_PDDR_PDD(0x0100);		// PTE8 bude vystupny
	//PTB->PDOR |= GPIO_PDOR_PDO(0x0038);			// Zhasnut LED-ky
}

void UARTInit(UART_Type *base) {
	base->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);		// Vypnut vysielac a prijimac

	base->BDH = 0x00;
	base->BDL = 0x1A;						// Baud rate 115 200 Bd
	base->C4 = 0x01;						// Baud rate fine adjust 1/32, match address mode disabled
	base->C1 = 0x00;						// 8 data bitov, bez parity
	//base->C2 = (0 | UART_C2_TCIE_MASK);	// transmission complete interrupt enable
	//NVIC_EnableIRQ(UART0_RX_TX_IRQn);		// Enable UART0 receive/transmit interrupt
	base->C3 = 0x00;
	base->MA1 = 0x00;						// no match address (mode disabled in C4)
	base->MA2 = 0x00;						// no match address (mode disabled in C4)
	//base->S1 |= 0x1F;						// clear IDLE, OR, NF, FE, PF
	base->S2 |= 0xC0;						//clear LBKDIF, RXEDGIF

	base->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);	// Zapnut vysielac a prijimac
}

/*
 * Inicializacia Real Time Counter
 */
void RTCInit(void) {
	//RTC->CR |= RTC_CR_SWR_MASK;			//reset vsetkych RTC registrov
	RTC->CR = RTC_CR_SWR(0);				//vynulovanie SWR
	RTC->TCR = RTC_TCR_CIR(0);				//clear compensation interval
	RTC->TCR = RTC_TCR_TCR(0);				//clear compensation time
	RTC->CR |= RTC_CR_OSCE_MASK;			//enable oscillator
	delay(DELAY_OSC_STAB);					//cakanie na stabilizaciu oscilatora
	RTC->SR |= RTC_SR_TCE_MASK;				//enable counter
}

/*
 * Resetovanie casu
 */
void resetTime(rtc_time_t rtc_time) {
	rtc_time->hour = 0U;
	rtc_time->minute = 0U;
	rtc_time->second = 0U;
}

/*
 * Vysielanie jedneho znaku cez UART
 */
void SendCh(UART_Type *base, char ch)  {
    while (!(base->S1 & UART_S1_TDRE_MASK) && !(base->S1 & UART_S1_TC_MASK));
    base->D = ch;
}

/*
 * Prijatie jedneho znaku cez UART
 */
char ReceiveCh(UART_Type *base) {
	while (!(base->S1 & UART_S1_RDRF_MASK));
	return base->D;
}

/*
 * Vysielanie retazca ukonceneho 0
 */
void SendStr(UART_Type *base, char *s)  {
	int i = 0;
	while (s[i] != 0) {
		SendCh(base, s[i++]);
	}
}

/*
 * Pipnutie cez bzuciak na PTA4
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
