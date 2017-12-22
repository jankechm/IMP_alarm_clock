/**
 * Pouzite:
 * 06-imp-demo-FITkit3-src/Sources/main.c
 * Laboratorni uloha c. 1 z predmetu IMP
 **/

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK60D10.h"

/* Inicializacia MCU - zakladne nastavenie hodin, vypnutie watchdogu */
void MCUInit(void) {
	MCG_C4 |= ( MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x01) );
	SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(0x00);
	WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;
}

/* Inicializacia pinov pre vysielanie a prijem cez UART*/
void PinInit(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  		// Povolit hodiny pre PORTA
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;  		// Povolit hodiny pre PORTB
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;			// Povolit hodiny pre UART0

	PORTA->PCR[1] = PORT_PCR_MUX(0x02); 		// UART0_RX
	PORTA->PCR[2] = PORT_PCR_MUX(0x02); 		// UART0_TX

	PORTB->PCR[5] = PORT_PCR_MUX(0x01); 		// MCU_LED0 D9
	PORTB->PCR[4] = PORT_PCR_MUX(0x01); 		// MCU_LED1 D10
	PORTB->PCR[3] = PORT_PCR_MUX(0x01); 		// MCU_LED2 D11

	PORTA->PCR[4] = PORT_PCR_MUX(0x01);  		// Beeper (PTA4)

	PTA->PDDR = GPIO_PDDR_PDD(0x0010); 			// PTA4 pin bude vystupny
	PTB->PDDR = GPIO_PDDR_PDD(0x0034); 			// PTB3/4/5 bude vystupny
}

void UART0Init(void) {
	UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);		// Vypnut vysielac a prijimac

	UART0->BDH = 0x00;
	UART0->BDL = 0x1A;						// Baud rate 115 200 Bd
	UART0->C4 = 0x01;						// Baud rate fine adjust 1/32, match address mode disabled
	UART0->C1 = 0x00;						// 8 data bitov, bez parity
	UART0->C2 = (0 | UART_C2_TCIE_MASK);	// transmission complete interrupt enable
	UART0->C3 = 0x00;
	UART0->MA1 = 0x00;						// no match address (mode disabled in C4)
	UART0->MA2 = 0x00;						// no match address (mode disabled in C4)
	//UART0->S1 |= 0x1F;						// clear IDLE, OR, NF, FE, PF
	UART0->S2 |= 0xC0;						//clear LBKDIF, RXEDGIF

	UART0->C2 |= ( UART0_C2_TE_MASK | UART0_C2_RE_MASK );	// Zapnut vysielac a prijimac
}

//UART0_RX_TX_IRQn

/*!
 * @brief Application entry point.
 */
int main(void) {
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  /* Add your code here */

  for(;;) { /* Infinite loop to avoid leaving the main function */
    __asm("NOP"); /* something to use as a breakpoint stop while looping */
  }
}
