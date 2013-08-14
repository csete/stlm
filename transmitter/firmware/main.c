/********************************************************************************
*																				*
*	Copenhagen Suborbitals														*
*																				*
*	2.3 GHz Telemetry Transmitter												*
*																				*
*	Version:	1.1.1															*
*	Supports hardware version 1.0 and 1.1										*
*																				*
********************************************************************************/

#define HWVERSION	11		/* Alias for 1.1 */


#include <xc.h>
#include <stdint.h>
#include <string.h>

/* Setup clock speed. */
#pragma config FNOSC = FRCPLL
#pragma config ICS = PGD2				/* Debugger is connected to PGC2/PGD2 pins. */
#pragma config FWDTEN = OFF				/* Disable watchdog. */
#pragma config OSCIOFNC = ON			/* Use Pin-10 as regular output. */

/* IO Pin layout. */

#if HWVERSION == 10
#define LED_1				LATBbits.LATB15
#define LED_2				LATBbits.LATB14
#define LED_1_TRIS			TRISBbits.TRISB15
#define LED_2_TRIS			TRISBbits.TRISB14

#define ADF_CS				LATAbits.LATA1		/* RA1  <= CS pin for the ADF. */
#define ADF_CS_TRIS			TRISAbits.TRISA1	/* --///-- */
#define ADF_SCK				PORTBbits.RB1		/* RP1/RB1 */
#define ADF_SCK_TRIS		TRISBbits.TRISB1	/* --///-- */
#define ADF_SDO_TRIS		TRISBbits.TRISB0	/* RP0/RB0 */
#define ADF_SDI				PORTBbits.RB2		/* RP2/RB2 */
#define ADF_SDI_TRIS		TRISBbits.TRISB2	/* --///-- */
#define ADF_SPORT_CLK_TRIS	TRISBbits.TRISB3	/* RP3/RB3 */
#define ADF_SPORT_SDO_TRIS	TRISBbits.TRISB4	/* RP4/RB4 */
#define ADF_SPORT_SDI_TRIS	TRISBbits.TRISB5	/* RP5/RB5 */

#define RX1_IN				PORTBbits.RB9
#define RX2_IN				PORTBbits.RB13

#elif HWVERSION == 11
#define LED_1				LATAbits.LATA2
#define LED_2				LATAbits.LATA3
#define LED_1_TRIS			TRISAbits.TRISA2
#define LED_2_TRIS			TRISAbits.TRISA3

#define PA_EN				LATAbits.LATA4
#define PA_EN_TRIS			TRISAbits.TRISA4

#define ADF_CS				LATAbits.LATA1		/* RA1  <= CS pin for the ADF. */
#define ADF_CS_TRIS			TRISAbits.TRISA1	/* --///-- */
#define ADF_SCK				PORTBbits.RB1		/* RP1/RB1 */
#define ADF_SCK_TRIS		TRISBbits.TRISB1	/* --///-- */
#define ADF_SDO_TRIS		TRISBbits.TRISB0	/* RP0/RB0 */
#define ADF_SDI				PORTBbits.RB2		/* RP2/RB2 */
#define ADF_SDI_TRIS		TRISBbits.TRISB2	/* --///-- */
#define ADF_SPORT_CLK_TRIS	TRISBbits.TRISB3	/* RP3/RB3 */
#define ADF_SPORT_SDO_TRIS	TRISBbits.TRISB4	/* RP4/RB4 */
#define ADF_SPORT_SDI_TRIS	TRISBbits.TRISB5	/* RP5/RB5 */

#define RX1_IN				PORTBbits.RB7
#define RX2_IN				PORTBbits.RB13

#else
#error "Unknown hardware version."
#endif


/* Define the clocking scalers. */
#define C_FOSC 			7370000		/* Frequency of internal FRC oscillator. */

#if 0
#define C_FRCDIV		1			/* No additional scaling of FRC. */
#define C_FRCDIV_		0

#define C_PLLPRE		2			/* PLL Prescaler: 1:2 */
#define C_PLLPRE_		0

#define C_PLLDIV		43			/* PLL loopback divider 43. */
#define C_PLLDIV_		C_PLLDIV-2

#define C_PLLPOST		2			/* PLL Postscaler: 1:2 */
#define C_PLLPOST_		0

#define C_DOZE			1			/* DOZE prescaler: 1:1 */
#define C_DOZE_			0
#else
#define C_FRCDIV		1			/* No additional scaling of FRC. */
#define C_FRCDIV_		0

#define C_PLLPRE		7			/* PLL Prescaler: 1:7 */
#define C_PLLPRE_		(C_PLLPRE-2)

#define C_PLLDIV		152			/* PLL loopback divider 152. */
#define C_PLLDIV_		(C_PLLDIV-2)

#define C_PLLPOST		2			/* PLL Postscaler: 1:2 */
#define C_PLLPOST_		0

#define C_DOZE			1			/* DOZE prescaler: 1:1 */
#define C_DOZE_			0
#endif

#define C_FCY (C_FOSC * C_PLLDIV / (C_FRCDIV * C_PLLPRE * C_PLLPOST * 2))



/* Global variables. */
uint32_t		uptime = 0;			/* Uptime counter - in 1/10 seconds intervals. */
unsigned char	update_flag = 0;	/* Timer update flag - used for slow updates in the main loop. */

unsigned int	debug;
unsigned char	pa_level;
unsigned char	pa_enable = 0;		/* By default enable the PA module. */
unsigned int	id_no = 0;
unsigned int	rx1_idle;
unsigned int	rx2_idle;


/****************************************************************
*																*
*	TIMER1 services - used for time keeping.					*
*																*
****************************************************************/

static void init_timer1 (void)
{
	T1CONbits.TON = 0;			/* Timer disabled. */
	T1CONbits.TCS = 0;			/* Use Internal clock (Fcy). */
	T1CONbits.TGATE = 0;		/* Disable gated Timer mode. */
	T1CONbits.TCKPS = 3;		/* Use 1:256 prescaler. */
	T1CONbits.TSIDL = 0;		/* Continue in Idle mode. */
	TMR1 = 0x00;				/* Clear the timer register. */
	PR1 = C_FCY / (256 * 10);	/* Set the period register to 10 periods / second. */

	IPC0bits.T1IP = 0x01;		/* Priority level 1. */
	IFS0bits.T1IF = 0;			/* Clear any pending interrupts. */
	IEC0bits.T1IE = 1;			/* Enable interrupts. */

	T1CONbits.TON = 1;			/* Start the timer. */
}

void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt (void)
{
	uptime++;					/* Increment the uptime counter. */
	update_flag = 1;			/* Set the update flag so the main loop can do maintenance. */

	IFS0bits.T1IF = 0;			/* Clear the interrupt flag again. */
}



/****************************************************************
*																*
*	SPI1 services - used for commanding the ADF.				*
*																*
****************************************************************/

/* @brief  Initialize SPI1 for commanding the ADF
 */
static void init_spi1 (void)
{
	unsigned char	dummy;

	SPI1STATbits.SPIEN = 0;		/* Disable the SPI1 module. */
	SPI1STATbits.SPISIDL = 0;	/* Continue when in idle mode. */
	SPI1STATbits.SPIROV = 0;	/* No receiver overflow yet. */

	SPI1CON1bits.MODE16 = 0;	/* 8-bit mode. */
	SPI1CON1bits.MSTEN = 1;		/* SPI1 operates in master mode (drives the clock). */
	SPI1CON1bits.DISSCK = 0;	/* SPI clock is enabled. */
	SPI1CON1bits.DISSDO = 0;	/* SDO pin controlled by module. */
	SPI1CON1bits.SMP = 0;		/* Input data sampled at middle of data output time. */
	SPI1CON1bits.CKP = 0;		/* Polarity: low is inactive. */
	SPI1CON1bits.CKE = 1;		/* Output change when clock goes from idle to active. */
	SPI1CON1bits.SSEN = 0;		/* SS is controlled by software. */
	SPI1CON1bits.SPRE = 0;		/* Secondary prescaler 8:1 */
	SPI1CON1bits.PPRE = 0;		/* Primary prescaler 64:1 */

	SPI1CON2bits.FRMEN = 0;		/* No framed mode (done in software). */

	ADF_CS = 1;					/* Set the CS pin high (inactive). */
	ADF_CS_TRIS = 0;			/* Enable the CS pin as an output. */

	ADF_SCK_TRIS = 0;			/* Enable the SCK pin as an output. */
	ADF_SDO_TRIS = 0;			/* Enable the SDO pin as an output. */
	ADF_SDI_TRIS = 1;			/* Enable the SDI pin as an input. */

	IPC2bits.SPI1IP = 0x00;		/* Priviledge level 0 => disabled. */
	IEC0bits.SPI1IE = 0;		/* Disable interrupt here too. */

	SPI1BUF = 0xFF;				/* Load a NOP byte into the tx register. */
	SPI1STATbits.SPIEN = 1;		/* Enable the SPI1 module. */

	dummy = SPI1BUF;			/* Flush the SPI1 buffers. */
}


/** @brief  Read the status word from the tranceiver chip.
 *
 * @return: the status word from the chip.
 */
unsigned char adf_read_status (void)
{
	unsigned char	ret;

	/* Select the chip by lowering the CS pin. */
	ADF_CS = 0;

	/* Then wait for the SPI ready status bit to go high. */
	Nop ();
	Nop ();
	while (ADF_SDI == 0);

	/* Send a SPI_NOP byte. */
	SPI1BUF = 0xFF;

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Send one more SPI_NOP byte. */
	SPI1BUF = 0xFF;

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Wait for the clock bit to drop low again. */
	while (ADF_SCK == 1);

	/* Unselect the transceifer chip by raising the CS pin. */
	ADF_CS = 1;	

	return ret;			/* Return the status word. */
}


/** @brief  Wait for the command queue in the ADF to be ready.
 *
 * @return: The status word from the ADF.
 */
unsigned char adf_wait_ready (void)
{
	unsigned char	ret;

	do {
		ret = adf_read_status ();
	} while ((ret & 0x20) == 0);
	return ret;
}


/** @brief  Send a command byte to the tranceiver.
 *			Terminates the SPI transaction after one byte.
 *
 * @param[in]  cmd		Command byte.
 * @param[in]  wait		If nonzero wait for the command queue to be ready.
 * @return: The status word from the tranceiver chip.
 */
unsigned char adf_send_command (unsigned char cmd, unsigned char wait)
{
	unsigned char	ret;

	if (wait) {
		adf_wait_ready ();

		Nop ();		/* Take a short nap (so the CS pulse is visible on the scope). */
		Nop ();
		Nop ();
		Nop ();
		Nop ();
		Nop ();
		Nop ();
		Nop ();
		Nop ();
		Nop ();
	}

	/* Select the chip by lowering the CS pin. */
	ADF_CS = 0;

	/* Then wait for the SPI ready status bit to go high. */
	Nop ();
	Nop ();
	while (ADF_SDI == 0);

	/* Send the command byte. */
	SPI1BUF = cmd;

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Wait for the clock bit to drop low again. */
	while (ADF_SCK == 1);

	/* Unselect the transceifer chip by raising the CS pin. */
	ADF_CS = 1;	

	return ret;			/* Return the status word. */
}


/** @brief  Write a byte to the memory space of the ADF chip.
 *			Terminates the SPI transaction afterwards.
 *
 * @param[in]  addr		Address of the memory location.
 * @param[in]  data		Data byte to write.
 * @return: The status word from the ADF chip.
 */
unsigned char adf_write_byte (unsigned int addr, unsigned char data)
{
	unsigned char	ret;

	/* Select the chip by lowering the CS pin. */
	ADF_CS = 0;

	/* Then wait for the SPI ready status bit to go high. */
	Nop ();
	Nop ();
	while (ADF_SDI == 0);

	/* Store the top byte of the address with bit 3 set as command. */
	SPI1BUF = 0x08 | (addr >> 8);

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Store the lower byte of the address. */
	SPI1BUF = addr & 0xFF;

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Store the data byte. */
	SPI1BUF = data;

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Wait for the clock bit to drop low again. */
	while (ADF_SCK == 1);

	/* Unselect the transceifer chip by raising the CS pin. */
	ADF_CS = 1;	

	return ret;			/* Return the status word. */
}


/** @brief  Read a byte from the memory space of the ADF chip.
 *			Terminates the SPI transaction afterwards.
 *
 * @param[in]  addr		Address of the byte to fetch.
 * @return: The requested byte.
 */
static unsigned char adf_read_byte (unsigned int addr)
{
	unsigned char	ret;

	/* Select the chip by lowering the CS pin. */
	ADF_CS = 0;

	/* Then wait for the SPI ready status bit to go high. */
	Nop ();
	Nop ();
	while (ADF_SDI == 0);

	/* Store the top byte of the address with bit 3+4+5 set as command. */
	SPI1BUF = 0x38 | (addr >> 8);

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Store the lower byte of the address. */
	SPI1BUF = addr & 0xFF;

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Store a NOP byte (filler so the interface can prepare the output byte). */
	SPI1BUF = 0xFF;

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Store a NOP byte (while retrieving the data byte). */
	SPI1BUF = 0xFF;

	/* Wait for the BufferFull bit to be set (end of transmission). */
	while (SPI1STATbits.SPIRBF == 0);
	ret = SPI1BUF;

	/* Wait for the clock bit to drop low again. */
	while (ADF_SCK == 1);

	/* Unselect the transceifer chip by raising the CS pin. */
	ADF_CS = 1;

	return ret;			/* Return the status word. */
}



/****************************************************************
*																*
*	Packet transmitter services.								*
*																*
****************************************************************/

/** @brief  Transmit buffer header definition.
 */
struct tx_header_block {
	struct {
		uint8_t		pending;		/* Handshake flag. True until transmitted. */
		uint16_t	length;			/* Length of packet incl header. */
	} control;

	struct {
		uint8_t		preamble [4];	/* Preamble bytes. */
		uint8_t		sync [4];		/* Sync bytes. */
	} data;
} __attribute__ ((packed));

/** @brief  Housekeeping telemetry block.
 */
struct tx_housekeeping_block {
	struct tx_header_block	header;

	/* Payload data. */
	uint8_t		payload [100];		/* Only short frames. */
} __attribute__ ((packed));


/** @brief  TX payload data block.
 */
struct tx_payload_block {
	struct tx_header_block	header;

	/* Payload data. */
	uint8_t		payload [600];
} __attribute__ ((packed));


/* Create a pair of tx_filler_block in the DMA memory space. */
struct tx_housekeeping_block tx_housekeeping_block [2]  __attribute__((space(dma)));
struct tx_housekeeping_block *current_tx_filler_block;

/* Create a pair of tx_payload_block buffers in the DMA memory space. */
struct tx_payload_block tx_payload_block [2]  __attribute__((space(dma)));

/* Create a dummy word in the DMA memory space for keeping the receiver happy. */
uint16_t	rx_dummy __attribute__((space(dma)));

/* Define hack to figure out the offset of the DMA memory space. */
#define DMA_BASE (((unsigned int)&rx_dummy) - __builtin_dmaoffset (&rx_dummy))

/* Holder of the next tx sequence value. */
uint8_t		tx_next_sequence_no;

/* Transmitter FIFO struct. */
struct tx_fifo {
	struct tx_header_block	*header;
};

/* Transmitter FIFO variables. */
#define TX_FIFO_LEN		8
uint8_t					tx_fifo_len = 0;
struct tx_fifo			tx_fifo [TX_FIFO_LEN];
struct tx_header_block	*tx_running = NULL;	/* Pointer to the currently transmitting block. */



/** @brief  Add a block to the tx_fifo.
 *
 * @param[in]  header	Pointer to the block header element.
 */
static void tx_fifo_add (struct tx_header_block *header)
{
	if (tx_fifo_len >= TX_FIFO_LEN) {
		return;		/* FIFO is full. Ignore the packet. */
	}

	header->control.pending = 1;			/* Set the pending flag. */
	tx_fifo [tx_fifo_len].header = header;
	tx_fifo_len++;
}



/****************************************************************
*																*
*	SPI2+DMA services - used for transferring data via the ADF.	*
*																*
****************************************************************/

/* @brief  Initialize SPI2 for data transfer via the ADF
 */
static void init_spi2_dma (void)
{
	unsigned char	dummy;

	SPI2STATbits.SPIEN = 0;		/* Disable the SPI2 module. */
	SPI2STATbits.SPISIDL = 0;	/* Continue when in idle mode. */
	SPI2STATbits.SPIROV = 0;	/* No receiver overflow yet. */

	SPI2CON1bits.MODE16 = 0;	/* 8-bit mode. */
	SPI2CON1bits.MSTEN = 0;		/* SPI2 operates in slave mode (follows the clock). */
	SPI2CON1bits.DISSCK = 0;	/* SPI clock is enabled. */
	SPI2CON1bits.DISSDO = 0;	/* SDO pin controlled by module. */
	SPI2CON1bits.SMP = 0;		/* Input data sampled at middle of data output time. */
	SPI2CON1bits.CKP = 1;		/* Polarity: high is idle. */
	SPI2CON1bits.CKE = 0;		/* Output change when clock goes from idle to active. */
	SPI2CON1bits.SSEN = 0;		/* SS is controlled by software. */
	SPI2CON1bits.SPRE = 0;		/* Secondary prescaler N/A */
	SPI2CON1bits.PPRE = 0;		/* Primary prescaler N/A */

	SPI2CON2bits.FRMEN = 0;		/* No framed mode (done in software). */

	ADF_SPORT_CLK_TRIS = 1;		/* Enable the SCK pin as an input. */
	ADF_SPORT_SDO_TRIS = 0;		/* Enable the SDO pin as an output. */
	ADF_SPORT_SDI_TRIS = 1;		/* Enable the SDI pin as an input. */

	IPC8bits.SPI2IP = 0x00;		/* Priviledge level 0 => disabled. */
	IFS2bits.SPI2IF = 0;		/* Clear any pending interrupts. */
	IEC2bits.SPI2IE = 0;		/* Disable interrupt here too. */

	SPI2BUF = 0x01;				/* Load a preamble byte into the tx register. */

	/* Setup DMA channels 0+1 for keeping SPI2 busy. */
	DMACS0 = 0;					/* Clear the DMA Controller Status register. */

	DMA0CONbits.CHEN  = 0;		/* Disable the DMA channel.*/
	DMA0CONbits.SIZE  = 1;		/* 8-bit mode. */
	DMA0CONbits.DIR   = 1;		/* Transfer from RAM to peripheral. */
	DMA0CONbits.HALF  = 0;		/* Interrupt when all data have been moved. */
	DMA0CONbits.NULLW = 0;		/* Normal operation. */
	DMA0CONbits.AMODE = 0;		/* Register Indirect with Post-Increment. */
	DMA0CONbits.MODE  = 1;		/* One-Shot without Ping-Pong. */
//	DMA0STA = __builtin_dmaoffset (&tx_filler_block);
	DMA0PAD = (volatile unsigned int) &SPI2BUF;
//	DMA0CNT = sizeof (tx_filler_block) - 1;
	DMA0REQ = 0x0021;			/* DMA trigger on SPI2 Transfer Done. */
	IPC1bits.DMA0IP = 0x03;		/* Priviledge level 3. */
	IFS0bits.DMA0IF = 0;		/* Clear any pending interrupts. */
	IEC0bits.DMA0IE = 1;

	DMA1CONbits.CHEN  = 0;		/* Disable the DMA channel.*/
	DMA1CONbits.SIZE  = 1;		/* 8-bit mode. */
	DMA1CONbits.DIR   = 0;		/* Transfer from peripheral to RAM. */
	DMA1CONbits.HALF  = 0;		/* Interrupt when all data have been moved. */
	DMA1CONbits.NULLW = 0;		/* Normal operation. */
	DMA1CONbits.AMODE = 1;		/* Register Indirect fixed address. */
	DMA1CONbits.MODE  = 0;		/* Continous without Ping-Pong. */
	DMA1STA = __builtin_dmaoffset (&rx_dummy);
	DMA1PAD = (volatile unsigned int) &SPI2BUF;
	DMA1CNT = 0;				/* Receive as much as possible. */
	DMA1REQ = 0x0021;			/* DMA trigger on SPI2 Transfer Done. */
	IPC3bits.DMA1IP = 0x03;		/* Priviledge level 3. */
	IFS0bits.DMA1IF = 0;		/* Clear any pending interrupts. */
	IEC0bits.DMA1IE = 0;		/* Disable interrupts from this channel (not needed). */

	DMA1CONbits.CHEN = 1;		/* Enable DMA1. */
	SPI2STATbits.SPIEN = 1;		/* Enable the SPI2 module. */

	IFS0bits.DMA0IF = 1;		/* Set the DMA interrupt flag to start the statemachine. */
								/* It will enable the DMA0 channel. */

	dummy = SPI2BUF;			/* Flush the SPI2 buffers. */
}

/** @brief  SPI2 interrupt handler.
 */
void __attribute__((__interrupt__,__auto_psv__)) _SPI2Interrupt (void)
{
	/* This ISR is not used as the SPI2 is serviced by the DMA0+1 */
	IFS2bits.SPI2IF = 0;		/* Clear any pending interrupts. */
}


/** @brief  DMA0 interrupt handler.
 */
void __attribute__((__interrupt__,__auto_psv__)) _DMA0Interrupt (void)
{
	/* Restart the DMA with a new block. */

	/* Mark the previous block done. */
	if (tx_running != NULL) {
		tx_running->control.pending = 0;
	}

	if (tx_fifo_len > 0) {
		unsigned int	x;

		// DMA0STA = __builtin_dmaoffset (&tx_fifo [0].header->data);
		DMA0STA = ((unsigned int)&tx_fifo [0].header->data) - DMA_BASE;
		DMA0CNT = tx_fifo [0].header->control.length;
		SPI2BUF = 0x55;				/* Preload a preamble byte into the SPI2 buffer. */
		DMA0CONbits.CHEN = 1;		/* Enable DMA0. */

		/* Remember which block is being transmitted now. */
		tx_running = tx_fifo [0].header;

		/* Shift the FIFO down one notch. */
		for (x = 0; x < TX_FIFO_LEN - 1; x++) {
			tx_fifo [x] = tx_fifo [x + 1];
		}
		tx_fifo_len--;
	} else {
		/* FIFO is empty. Retransmit the last tx_filler_block. */

		// DMA0STA = __builtin_dmaoffset (&current_tx_filler_block->header.data);
		DMA0STA = ((unsigned int)&current_tx_filler_block->header.data) - DMA_BASE;
		DMA0CNT = current_tx_filler_block->header.control.length;
		SPI2BUF = 0x55;				/* Preload a preamble byte into the SPI2 buffer. */
		DMA0CONbits.CHEN = 1;		/* Enable DMA0. */

		tx_running = NULL;			/* No need to clear its pending bit. */
	}

	IFS0bits.DMA0IF = 0;		/* Clear any pending interrupts. */
}


/** @brief  DMA1 interrupt handler.
 */
void __attribute__((__interrupt__,__auto_psv__)) _DMA1Interrupt (void)
{
	/* This ISR is not used as DMA1 is set to run continuously. */

	/* Restart the rx_dummy operation. */
	DMA1STA = __builtin_dmaoffset (&rx_dummy);
	DMA1CNT = 0;
	DMA1CONbits.CHEN = 1;		/* Enable DMA1. */
	IFS0bits.DMA1IF = 0;		/* Clear any pending interrupts. */
}



/****************************************************************
*																*
*	ADF services - used for handling the ADF.					*
*																*
****************************************************************/

/** @brief  Initializes the ADF tranceiver chip.
 *
 * @param[in]  Frequency divisor value.
 * @param[in]  Bitrate/100 Hz value.
 * @param[in]  Deviation/100 Hz value.
 */
static void adf_init (unsigned long			divisor,
					  unsigned int			bitrate,		/* Divided by 100. */
					  unsigned int			deviation)		/* Divided by 10K. */
{
	/* Reset the chip. */
	adf_send_command (0xC8, 0);

	/* Set 0x13E, rc_cfg to the SPORT mode. */
	adf_write_byte (0x13E, 0x03);

	/* Set 0x30E/0x30F: dr0/dr1 to the data rate. */
	adf_write_byte (0x30E, (bitrate >> 8));
	adf_write_byte (0x30F, (bitrate & 0xFF));

	if (bitrate == 20000) {
		adf_write_byte (0x304, 50);		/* 2 Mbit/s => swing 1000 KHz. */
		adf_write_byte (0x306, 3);		/* Enable GFSK and preemphasis. */
	} else if (bitrate == 10000) {
		adf_write_byte (0x304, 25);		/* 1 Mbit/s => swing 500 KHz. */
		adf_write_byte (0x306, 3);		/* Enable GFSK and preemphasis. */
	} else if (bitrate == 5000) {
		adf_write_byte (0x304, 25);		/* 500 Kbit/s => swing 500 KHz. */
		adf_write_byte (0x306, 3);		/* Enable GFSK and preemphasis. */
	} else if (bitrate == 2500) {
		adf_write_byte (0x304, 13);		/* 250 Kbit/s => swing 260 KHz. */
		adf_write_byte (0x306, 2);		/* Enable GFSK, Disable emphasis. */
	} else {
		adf_write_byte (0x304, 6);		/* less => swing 60 KHz. */
		adf_write_byte (0x305, 55);		/* less => DM_CFG0 = 55. */
		adf_write_byte (0x306, 0);		/* Disable GFSK, Disable emphasis. */
	}

	adf_write_byte (0x335, 0x28);			/* Synth locking time. */

	/* Set 0x32C: gp_cfg to sample SPORT TX data on rising clock edge. */
	adf_write_byte (0x32C, 0x01);

	adf_write_byte (0x389, 0x17);			/* IIRF_CFG=25. */
	adf_write_byte (0x38B, 0x08);			/* DM_CFG1=8. */
	adf_write_byte (0x39B, 0x16);			/* LNA2. 555 kHz BW. */
	adf_write_byte (0x3B4, 0x80);			/* AGC Stuff. */
	adf_write_byte (0x3B6, 0x37);			/* AGC_CFG2=55 - default for GFSK. */
	adf_write_byte (0x3B7, 0x2A);			/* AGC_CFG3=42 - default for GFSK. */
	adf_write_byte (0x3B8, 0x1D);			/* AGC_CFG4=29 - default for GFSK. */
	adf_write_byte (0x3B2, 0x34);			/* AGC_CFG1. */
	adf_write_byte (0x3BA, 0x24);			/* AGC_CFG6 - GFSK defaults. */
	adf_write_byte (0x3BC, 0x7B);			/* AGC_CFG7. */
	adf_write_byte (0x3BF, 0x00);			/* OCL_CFG0=0 - GFSK mode. */
	adf_write_byte (0x3CB, 0xFF);			/* IRQ_SRC0: Clear all pending interrupts. */
	adf_write_byte (0x3CC, 0xFF);			/* IRQ_SRC1: Clear all pending interrupts. */
	adf_write_byte (0x3C7, 0x00);			/* IRQ1_EN0: Disable all interrupts. */
	adf_write_byte (0x3C8, 0x00);			/* IRQ1_EN1: Disable all interrupts. */
	adf_write_byte (0x3C9, 0x00);			/* IRQ2_EN0: Disable all interrupts. */
	adf_write_byte (0x3CA, 0x00);			/* IRQ2_EN1: Disable all interrupts. */
	adf_write_byte (0x3CB, 0xFF);			/* IRQ_SRC0: Clear all pending interrupts. */
	adf_write_byte (0x3CC, 0xFF);			/* IRQ_SRC1: Clear all pending interrupts. */
	adf_write_byte (0x3C4, 0x07);			/* OCL_CFG4=7 - GFSK default. */
	adf_write_byte (0x3D2, 0x1A);			/* OCL_BW0=1A - GFSK default. */
	adf_write_byte (0x3D3, 0x19);			/* OCL_BW1=19 - GFSK default. */
	adf_write_byte (0x3D4, 0x1E);			/* OCL_BW2=1E - GFSK default. */
	adf_write_byte (0x3D5, 0x1E);			/* OCL_BW3=1E - GFSK default. */
	adf_write_byte (0x3D6, 0x1E);			/* OCL_BW4=1E - GFSK default. */
	adf_write_byte (0x3D7, 0x00);			/* OCL_BWS=00 - GFSK default. */
	adf_write_byte (0x3E0, 0xF0);			/* OCL_CFG13=0xF0. */

// 0x3E3			GPIO and SPI pin slewrate and output drive.

	/* Set the carrier frequency into 0x300..0x302. */
	if (divisor == 0) {
		divisor = 0x0003A9E4;	/* 2401 MHz. */
	}
	adf_write_byte (0x300, (divisor & 0xFF));
	adf_write_byte (0x301, (divisor >> 8));
	adf_write_byte (0x302, (divisor >> 16));

	adf_write_byte (0x3F0, 0x20);		/* Normal mode. */

//	adf_write_byte (0x3AA, 0xF1);				/* Set PA power. */
	adf_write_byte (0x3AA, pa_level);			/* Set PA power. */
	/* 0xF1 => -27,50 dBm incl PA -16,17 dBm */
	/* 0xE1 => -29,17 dBm incl PA -17.67 dBm */
	/* 0xA1 => -36,67 dBm incl PA -23.83 dBm */
	/* 0x61 => -44.67 dBm */
	/* 0x31 => -56.00 dBm */

// 0x36E			PA Bias level.
// 0x3A7			PA Ramp rate - default looks good.
// 0x3A8			PA bridge bias level.

	/* Go into idle mode. */
	adf_send_command (0xB2, 1);
}


/** @brief  Switch the ADF to ON state.
 */
static void adf_cmd_phy_on (void)
{
	adf_send_command (0xB3, 1);
}


/** @brief  Switch the ADF to RX state.
 */
static void adf_cmd_phy_rx (void)
{
	adf_send_command (0xB4, 1);
}


/** @brief  Switch the ADF to TX state.
 */
static void adf_cmd_phy_tx (void)
{
	adf_send_command (0xB5, 1);
}


/** @brief  Switch the ADF to SLEEP state.
 */
static void adf_cmd_phy_sleep (void)
{
	adf_send_command (0xB1, 1);
}


/** @brief  Set the PA level for the ADF chip.
 *
 * @param[in]  level		PA level. [3..15].
 */
static void adf_set_pa_level (unsigned char level)
{
	adf_write_byte (0x3AA, (level & 0x0f) << 4);
}


/** @brief  Initialize the AD1 converter.
 */
static void init_adc (void)
{
	/* Enable the AD converter but only use pin 2 (AN0) for analog. */
	AD1CON1 = 0;				/* Disable AD1. */
	AD1CON1bits.ADON = 1;		/* Enable it. */
	AD1CON1bits.AD12B = 1;		/* Enable 12-bit operation. */
	AD1CON1bits.SSRC = 0;		/* Manual trigger of conversion. */
	AD1CON1bits.ASAM = 1;		/* Auto sample. */
	AD1CON2 = 0;				/* AVdd and AVss as reference voltages. */
	AD1CON3 = 0;
	AD1CON3bits.SAMC = 31;		/* Max sample time of 31*Tad. */
	AD1CON3bits.ADCS = 63;		/* Tad = Tcy * 64. */
	AD1CON4 = 0;
	AD1CSSL = 0;				/* No scanning. */

#if HWVERSION == 11
	AD1CHS0 = 9;				/* Sample AN9 with Vref-(AVss) as negative. */
	AD1PCFGL = 0xFFFF;			/* Disable all but AN9. */
	AD1PCFGLbits.PCFG9 = 0;
#elif HWVERSION == 10
	AD1CHS0 = 0;				/* Sample AN0 with Vref-(AVss) as negative. */
	AD1PCFGL = 0xFFFF;			/* Disable all but AN9. */
	AD1PCFGLbits.PCFG0 = 0;
#endif

	AD1CON1bits.ADON = 1;		/* Enable AD1. */

	/* Dont enable AD converter interrupts as it is used in polled mode. */
}


/****************************************************************
*																*
*	Viterbi encoder functions.									*
*																*
****************************************************************/

#define USE_TRELLIS 1

#if USE_TRELLIS
#include "trellis-tab.h"
#endif

inline uint16_t map_trellis (uint16_t val)
{
		if (val >= 0x2000) {
			return trellis_table_hi [val & 0x1FFF];
		} else {
			return trellis_table_lo [val];
		}
}


/** @brief  Build a tx packet based on a input feed.
 *
 * @param[io]  Pointer to the header block to be used.
 * @param[in]  Payload ID value
 * @param[in]  Payload length.
 * @param[in]  Payload fetch byte function.
 */
static void prepare_packet (struct tx_header_block 	*h,
                            uint8_t					id,
                            int						length,
                            uint8_t (*get_byte) ())
{

#if USE_TRELLIS
	register uint16_t	*bp;
	unsigned int		b;
	unsigned int		x;

	/* Initialize the CRC engine. */
	CRCCON = 0b0000000000001111;	/*
	             0					CSIDL Don't stop in idle mode.
	                      0			CRCGO = 0 Enable shifting.
	                       1111		PLEN = 15+1 Length of polynomium. */

	CRCXOR = 0x1020;				/* x^16 + x^12 + x^5 + 1 pattern. */
	CRCWDAT = 0;					/* Clear the shift register. */
	CRCCONbits.CRCGO = 1;			/* Enable the shifter. */

	/* Setup control information. */
	h->control.pending = 0;						/* Clear the pending flag. */
	h->control.length = sizeof (h->data) +		/* Fixed header. */
						2 +						/* Trellis coded length byte. */
						2 +						/* Trellis coded inverted length byte. */
						2 +						/* Trellis coded CRC byte 1. */
						2 +						/* Trellis coded CRC byte 2. */
						2 +						/* Trellis coded ID byte. */
                        length * 2 +			/* Trellis coded data. */
						2;						/* Trellis flush block. */

	/* Fill up the preamble. */
	for (x = 0; x < sizeof (h->data.preamble); x++) {
		h->data.preamble [x] = 0x55;
	}

	/* Add the sync flag. */
	h->data.sync [0] = 0x37;	/* Sync byte 0. */
	h->data.sync [1] = 0x4F;	/* Sync byte 1. */
	h->data.sync [2] = 0xE2;	/* Sync byte 2. */
	h->data.sync [3] = 0xDA;	/* Sync byte 3. */

	/* Start adding trellis coded data. */
	bp = (uint16_t *)(((uint8_t *)h) + sizeof (*h));
	b = 0;								/* Clear the state register. */

	b = (b << 8) | length;				/* Length byte. */
	CRCDAT = b & 0xFF;
	*(bp++) = map_trellis (b);
	b = (b << 8) | (length ^ 0xFF);		/* Inverted Length byte. */
	CRCDAT = b & 0xFF;
	*(bp++) = map_trellis (b);

	b = (b << 8) | id;					/* ID byte. */
	CRCDAT = b & 0xFF;
	*(bp++) = map_trellis (b);

	/* Add payload data. */
	x = length;
	while (x) {
		b = (b << 8) | get_byte ();
		CRCDAT = b & 0xFF;
		*(bp++) = map_trellis (b);
		x--;
	}

	if (! CRCCONbits.CRCMPT) {
		/* Header + length is uneven. Add one more byte to the CRC engine. */
		CRCDAT = 0;
	}

	/* Wait for the CRC engine to finish. */
	CRCCONbits.CRCGO = 0;
	while (! CRCCONbits.CRCMPT) ;
	x = CRCWDAT;

	b = (b << 8) | (x >> 8);			/* CRC byte 1 (high part). */
	*(bp++) = map_trellis (b);
	b = (b << 8) | (x & 0xFF);			/* CRC byte 2 (low part). */
	*(bp++) = map_trellis (b);

	/* Optionally add some padding bytes if the frame is very short. */

	/* Add one more symbol set to flush the trellis. */
	b = (b << 8);
	*(bp++) = map_trellis (b);

#else
	register uint8_t	*bp;

	/* Setup control information. */
	txp->header.control.pending = 0;			/* Clear the pending flag. */
	txp->header.control.length = sizeof (txp->header.data) +	/* Fixed header. */
								 1 +			/* Raw ID byte. */
                                 length;		/* Raw data. */

	/* Fill up the preamble. */
	for (x = 0; x < sizeof (txp->header.data.preamble); x++) {
		txp->header.data.preamble [x] = 0x55;
	}

	/* Add the sync flag. */
	txp->header.data.sync [0] = 0x37;	/* Sync byte 0. */
	txp->header.data.sync [1] = 0x4F;	/* Sync byte 1. */
	txp->header.data.sync [2] = 0xE2;	/* Sync byte 2. */
	txp->header.data.sync [3] = 0xDA;	/* Sync byte 3. */

	/* Start adding raw data. */
	bp = &txp->payload [0];				/* Point to where the payload area starts. */

	*(bp++) = length;					/* Length byte. */
	*(bp++) = (length ^ 0xFF);			/* Inverted Length byte. */

	*(bp++) = 0x12;						/* CRC byte 1. */
	*(bp++) = 0x48;						/* CRC byte 2. */

	*(bp++) = id;						/* ID byte. */

	/* Add payload data. */
	while (length) {
		*(bp++) = get_byte ();
		length--;
	}
#endif
}



/****************************************************************
*																*
*	Serial port functions.										*
*																*
****************************************************************/


/** @brief  Define a set of variables for a ring buffer.
 * @param[io]  Group prefix.
 */
#define RING_VARS(G,S) \
static uint8_t	G##_ring [S];		/* Actual buffer. */ \
static uint8_t	*G##_ring_out;		/* Pointer to where the oldest packet is stored. */ \
static uint8_t	*G##_ring_in;		/* Pointer where new bytes are stored. */ \
static uint8_t	*G##_ring_in_old;	/* Pointer to the block header of the currently receiving packet. */ \
static uint8_t	G##_ring_packet_len;/* Length of the currently receiving packet. */ \
static uint16_t	G##_ring_left;		/* Number of bytes available in the ring buffer. */


/** @brief  Initialize the ring buffer variable set.
 * @param[io]  Group prefix.
 */
#define RING_INIT(G) \
	G##_ring [0]    = 0;			/* Clear the length byte of the ring head. */ \
	G##_ring_out    = G##_ring;		/* Set the ring pointers  to the start of the ring. */ \
	G##_ring_in     = G##_ring + 1; \
	G##_ring_in_old = G##_ring; \
	G##_ring_packet_len  = 0; \
	G##_ring_left   = sizeof (G##_ring) - 1;	/* Have used one header byte already. */


/** @brief  Increment a ring buffer pointer with due respect to roll-over.
 * @param[io]  Group prefix.
 * @param[io]  Actual pointer variable.
 */
#define RING_INC(G,P) do { \
	G##_ring_##P++; \
	if (G##_ring_##P >= &G##_ring [sizeof (G##_ring)]) { \
		G##_ring_##P = &G##_ring [0];		/* Wrap around. */ \
	} \
} while (0)


/** @brief  Finalize a packet in the ring buffer.
 * @param[io]  Group prefix.
 */
#define RING_PACKET_FINALIZE(G) do { \
	*(G##_ring_in_old) = G##_ring_packet_len;	/* Updates the header byte. */ \
	G##_ring_packet_len = 0;					/* Clear the collector length counter. */ \
	G##_ring_in_old = G##_ring_in;				/* Prepare for the next packet. */ \
	*(G##_ring_in) = 0;							/* Clear the header byte. */ \
	RING_INC (G, in);							/* Increment the _in pointer. */ \
	G##_ring_left--;							/* Used one more header byte. */ \
} while (0)


/** @brief  Check if the ring buffer is full.
 * @param[io]  Group prefix.
 * @return: false on no, true on yes.
 */
#define RING_FULL(G) (G##_ring_left < 3)


/** @brief  Abort collecting a packet (restores buffer pointers to the end of the previous packet).
 * @param[io]  Group prefix.
 */
#define RING_ABORT(G) do { \
	G##_ring_in = G##_ring_in_old;			/* Restore the _ring_in pointer. */ \
	RING_INC (G, in); \
	G##_ring_left += G##_ring_packet_len;	/* Restore the space in the buffer. */ \
	G##_ring_packet_len = 0;				/* Clear the byte counter. */ \
} while (0)


/** @brief  Add one more byte to a ring buffer instance.
 * @param[io]  Group prefix.
 * @param[in]  New byte.
 */
#define RING_ADD_BYTE(G,b) do { \
	*(G##_ring_in) = b;					/* Store the received byte. */ \
	RING_INC (G, in);					/* Move the pointer ahead. */ \
	G##_ring_left--;					/* One less byte available. */ \
	G##_ring_packet_len++;				/* One more byte in the packet. */ \
} while (0)


#define RING_PACKET_LEN(G) (*(G##_ring_out))


/** @brief  Data buffer ring and support variables for RX1.
 */
RING_VARS (rx1, 1024)
static uint8_t	rx1_last_char;

static void init_rx1 (void)
{
	/* Initialize ring buffer variables. */
	RING_INIT (rx1)

	/* Initialize baud rate generator to 115200. */
	U1BRG = (C_FCY / (16 * 115200)) - 1;

	/* Initialize the port. */
	U1MODE = 0b0000000000000000;	/* Uart1 mode:
	           0						UARTEN = 0		Disabled.
	             0						USIDL = 0		Continue in idle mode.
	              0						IREN = 0		IRDA disabled.
	               0					RTSMP = 0		RTD in Flow control mode.
	                 00					UEN = 0			RX & TX pins only.
	                   0                WAKE = 0
	                    0				LPBACK = 0		Loopback disabled.
	                     0				ABAUD = 0		Auto Baud-Rate disable.
	                      0				URXINV = 0		RX Not inverted. Idle == high.
	                       0			BRGH = 0		Low Speed mode.
	                        00			PDSEL = 0		8-bit no parity mode.
	                          0			STSEL = 0		One stop bit.
									*/

	U1STA  = 0b0000000000000000;	/* Uart1 status:
	           0						UTXISEL1 = 0	Interrupt on any char.
	            0						UTXINV = 0		TX not inverted. Idle == high.
	             0						UTXISEL0 = 0	Interrupt on any char.
	               0					UTXBRK = 0		Not transmitting break.
	                0					UTXEN = 0		Transmitter disabled.
	                 R					UTXBF			Buffer full.
	                  R					TRMT			Transmitter Empty.
	                   00				URXISEL = 0		Interrupt after every char.
	                     0				ADDEN = 0		No Address Detect mode.
	                      R				RIDLE			Receiver Idle status.
	                       R			PERR			Parity error on current char.
	                        R			FERR			Framing error on current char.
	                         0			OERR = 0		No overrun yet.
	                          R			URXDA			Receiver data available.
									*/

	IFS0bits.U1RXIF = 0;		/* Clear any pending interrupts. */
	IEC0bits.U1RXIE = 1;		/* Enable interrupts. */
	IPC2bits.U1RXIP = 2;		/* Interrupt level 2 => better than timer1. */
	IEC0bits.U1TXIE = 0;		/* TX Interrupts are disabled (not used). */

	U1MODEbits.UARTEN = 1;		/* Enable module. */
}


/** @brief  Serial port 1 RX interrupt handler.
 */
void __attribute__((__interrupt__,__no_auto_psv__)) _U1RXInterrupt (void)
{
	static unsigned char	tmp;

	/* Check if there is space in the ring buffer for more chars. */
	if (RING_FULL (rx1)) {
		/* Ups. Out of memory. Discard the packet being received. */
		RING_ABORT (rx1);

		tmp = U1RXREG;			/* Flush the received byte. */
	} else {
		/* Add the received byte to the input buffer. */
		tmp = U1RXREG;
		RING_ADD_BYTE (rx1, tmp);

		/* Check if the packet is to be finalized. */
		if (rx1_ring_packet_len >= 100) {
			RING_PACKET_FINALIZE (rx1);		/* Packet is done. Finalize it. */
		} else if (rx1_ring_packet_len >= 30 && rx1_last_char == 13 && tmp == 10) {
			RING_PACKET_FINALIZE (rx1);		/* Found framing pattern. Finalize it. */
		}
		rx1_last_char = tmp;
	}

	rx1_idle = 0;			/* Clear the idle counter. */
	U1STAbits.OERR = 0;		/* Clear the overrun flag (it might have been set). */
	IFS0bits.U1RXIF = 0;	/* Clear the pending interrupt flag. */
}


/** @brief  Fetch one byte from the RX1 ring system.
 * @return: The byte
 */
static uint8_t rx1_get_byte (void)
{
	register uint8_t tmp = *rx1_ring_out;
	RING_INC (rx1, out);
	rx1_ring_left++;
	return tmp;
}



/** @brief  Data buffer ring and support variables for RX2.
 */
RING_VARS (rx2, 1024)
static uint8_t	rx2_last_char;


static void init_rx2 (void)
{
	/* Initialize ring buffer variables. */
	RING_INIT (rx2)

	/* Initialize baud rate generator to 115200. */
	U2BRG = ((C_FCY / 115200) / 16) - 1;

	/* Initialize the port. */
	U2MODE = 0b0000000000000000;	/* Uart2 mode:
	           0						UARTEN = 0		Disabled.
	             0						USIDL = 0		Continue in idle mode.
	              0						IREN = 0		IRDA disabled.
	               0					RTSMP = 0		RTD in Flow control mode.
	                 00					UEN = 0			RX & TX pins only.
	                   0                WAKE = 0
	                    0				LPBACK = 0		Loopback disabled.
	                     0				ABAUD = 0		Auto Baud-Rate disable.
	                      0				URXINV = 0		RX Not inverted. Idle == high.
	                       0			BRGH = 0		Low Speed mode.
	                        00			PDSEL = 0		8-bit no parity mode.
	                          0			STSEL = 0		One stop bit.
									*/

	U2STA  = 0b0000000000000000;	/* Uart2 status:
	           0						UTXISEL1 = 0	Interrupt on any char.
	            0						UTXINV = 0		TX not inverted. Idle == high.
	             0						UTXISEL0 = 0	Interrupt on any char.
	               0					UTXBRK = 0		Not transmitting break.
	                0					UTXEN = 0		Transmitter disabled.
	                 R					UTXBF			Buffer full.
	                  R					TRMT			Transmitter Empty.
	                   00				URXISEL = 0		Interrupt after every char.
	                     0				ADDEN = 0		No Address Detect mode.
	                      R				RIDLE			Receiver Idle status.
	                       R			PERR			Parity error on current char.
	                        R			FERR			Framing error on current char.
	                         0			OERR = 0		No overrun yet.
	                          R			URXDA			Receiver data available.
									*/

	IFS1bits.U2RXIF = 0;		/* Clear any pending interrupts. */
	IEC1bits.U2RXIE = 1;		/* Enable interrupts. */
	IPC7bits.U2RXIP = 2;		/* Interrupt level 2 => better than timer1. */
	IEC1bits.U2TXIE = 0;		/* TX Interrupts are disabled (not used). */

	U2MODEbits.UARTEN = 1;		/* Enable module. */
}

/** @brief  Serial port 2 RX interrupt handler.
 */
void __attribute__((__interrupt__,__no_auto_psv__)) _U2TXInterrupt (void)
{
	U2TXREG = 0x55;
	IFS1bits.U2TXIF = 0;		/* Clear any pending interrupts. */
}


/** @brief  Serial port 2 RX interrupt handler.
 */
void __attribute__((__interrupt__,__no_auto_psv__)) _U2RXInterrupt (void)
{
	static unsigned char	tmp;

	/* Check if there is space in the ring buffer for more chars. */
	if (RING_FULL (rx2)) {
		/* Ups. Out of memory. Discard the packet being received. */
		RING_ABORT (rx2);

		tmp = U2RXREG;			/* Flush the received byte. */
	} else {
		/* Add the received byte to the input buffer. */
		tmp = U2RXREG;
		RING_ADD_BYTE (rx2, tmp);

		/* Check if the packet is to be finalized. */
		if (rx2_ring_packet_len >= 100) {
			RING_PACKET_FINALIZE (rx2);		/* Packet is done. Finalize it. */
		} else if (rx2_ring_packet_len >= 30 && rx2_last_char == 13 && tmp == 10) {
			RING_PACKET_FINALIZE (rx2);		/* Found framing pattern. Finalize it. */
		}
		rx2_last_char = tmp;
	}

	rx2_idle = 0;			/* Clear the idle counter. */
	U2STAbits.OERR = 0;		/* Clear the overrun flag (it might have been set). */
	IFS1bits.U2RXIF = 0;	/* Clear the pending interrupt flag. */
}

/** @brief  Fetch one byte from the RX2 ring system.
 * @return: The byte
 */
static uint8_t rx2_get_byte (void)
{
	register uint8_t tmp = *rx2_ring_out;
	RING_INC (rx2, out);
	rx2_ring_left++;
	return tmp;
}



/****************************************************************************
*																			*
*	Housekeeping Telemetry stuff.											*
*																			*
****************************************************************************/

struct housekeeping_telemetry_t {
	uint32_t	uptime;
	uint16_t	supply_voltage;
	uint16_t	flags;
};

#define TLM_FLAGS_HIGHPOWER		0x8000
#define TLM_FLAGS_RX1_LEVEL		0x0001
#define TLM_FLAGS_RX1_ACTIVE	0x0002
#define TLM_FLAGS_RX2_LEVEL		0x0004
#define TLM_FLAGS_RX2_ACTIVE	0x0008

static struct housekeeping_telemetry_t housekeeping_telemetry;
static uint8_t *housekeeping_byte_pointer = (void *)&housekeeping_telemetry;


/** @brief  Reset the housekeeping telemetry block byte counter.
 */
static void housekeeping_reset (void)
{
	/* Set the pointer to the start of the housekeeping_telemetry block. */
	housekeeping_byte_pointer = (void *)&housekeeping_telemetry;

	/* Fetch data for the housekeeping_telemetry block. */
	housekeeping_telemetry.uptime = uptime;
	housekeeping_telemetry.supply_voltage = ADC1BUF0;

	housekeeping_telemetry.flags = 0;
	if (pa_enable == 0) {
		housekeeping_telemetry.flags |= TLM_FLAGS_HIGHPOWER;
	}
	if (rx1_idle < 20) {
		housekeeping_telemetry.flags |= TLM_FLAGS_RX1_ACTIVE;
		rx1_idle++;
	}
	if (RX1_IN) {
		housekeeping_telemetry.flags |= TLM_FLAGS_RX1_LEVEL;
	}		
	if (rx2_idle < 20) {
		housekeeping_telemetry.flags |= TLM_FLAGS_RX2_ACTIVE;
		rx2_idle++;
	}
	if (RX2_IN) {
		housekeeping_telemetry.flags |= TLM_FLAGS_RX2_LEVEL;
	}
}


/** @brief  Fetch one byte from the housekeeping telemetry block.
 * @return: The byte.
 */
static uint8_t housekeeping_get_byte (void)
{
	return *(housekeeping_byte_pointer++);
}


int main (void) {
	int				x;
	unsigned int	ad_value;

	/* Configure oscillator. */
	CLKDIVbits.FRCDIV  = C_FRCDIV_;
	CLKDIVbits.PLLPRE  = C_PLLPRE_;
	CLKDIVbits.PLLPOST = C_PLLPOST_;
//	CLKDIVbits.DOZE    = C_DOZE_;
	PLLFBDbits.PLLDIV  = C_PLLDIV_;


	/* Map PPS pins to their propper devices. */
	// Unlock the control registers.

	/* Pin mappings for SPI port 1 connected to the ADF control port. */
	RPOR0bits.RP0R = 0x07;		/* RP0  <= SDO1		ADF MOSI pin. */
	RPOR0bits.RP1R = 0x08;		/* RP1  <= SCK1		ADF SCLK pin.*/
	RPINR20bits.SCK1R = 1;		/* RP1  => SCK1:in	--//-- */
	RPINR20bits.SDI1R = 2;		/* RP2  => SDI1		ADF MISO pin. */

	/* Pin mappings for SPI port 2 connected to the ADF SPORT data port. */
	RPINR22bits.SCK2R = 3;		/* RP3  => SCK2:in	ADF SPORT CLK pin. */
	RPOR2bits.RP4R = 0x0A;		/* RP4  <= SDO2		ADF SPORT DT pin. */
	RPINR22bits.SDI2R = 5;		/* RP5  => SDI2		ADF SPORT DR pin. */

	/* Pin mappings for UART port 1 connected to the world. */
#if HWVERSION == 11
	RPOR3bits.RP6R = 3;			/* RP6  <= TX1		Serial link 1. */
	RPINR18bits.U1RXR = 7;		/* RP7  => RX1		Serial link 1. */
#elif HWVERSION == 10
	RPOR4bits.RP8R = 3;			/* RP8  <= TX1		Serial link 1. */
	RPINR18bits.U1RXR = 9;		/* RP9  => RX1		Serial link 1. */
#endif

	/* Pin mappings for UART port 2 connected to the world. */
	RPOR6bits.RP12R = 5;		/* RP12 <= TX2		Serial link 2. */
	RPINR19bits.U2RXR = 13;		/* RP13 => RX2		Serial link 2. */

	// Lock the PPS control again.

	/* Configure hardware ports. */	
	LED_1_TRIS = 0;		/* Make it an output port. */
	LED_2_TRIS = 0;		/* Make it an output port. */

	/* Read the hardware ID value from the debug pins. */
	id_no = (PORTB >> 10) & 0x03;
	Nop ();
	Nop ();

	init_timer1 ();		/* Configure Timer-1 to produce 10 interrupts per second. */
	init_adc ();		/* Initialize AD converter. */

	/* Init overall interrupts. */
	INTCON1bits.NSTDIS = 0;		/* Nesting is enabled. */
	SRbits.IPL = 0;

#if 0
	/* Read the ADC value a number of times to get the battery voltage. */
	for (x = 0; x < 3; x++) {
		while (!update_flag) {
			Nop ();		/* Wait for the update flag before continuing. */
		}
		update_flag = 0;

		/* Start the ADC for the next conversion. */
		AD1CON1bits.SAMP = 0;

		/* Wait for the ADC sample to be ready. */
		while (AD1CON1bits.DONE == 0) {
			Nop ();
		}
		ad_value = ADC1BUF0;
	}
	if (ad_value >= 0x08BC) {
		pa_enable = 1;			/* Above 12.0 volt. On battery charger. Disable PA. */
	}
#endif

	/* Initialize the tx_filler_block headers. */
	housekeeping_reset ();
	prepare_packet (&tx_housekeeping_block [0].header, id_no, sizeof (housekeeping_telemetry), housekeeping_get_byte);
	housekeeping_reset ();
	prepare_packet (&tx_housekeeping_block [1].header, id_no, sizeof (housekeeping_telemetry), housekeeping_get_byte);
	current_tx_filler_block = &tx_housekeeping_block [0];

	/* Clear the pending flag in the tx_payload_block elements. */
	tx_payload_block [0].header.control.pending = 0;
	tx_payload_block [1].header.control.pending = 0;

	switch (id_no) {
		case 0:	pa_level = 0xB1;
				// pa_enable = 0;	/* PA enable is controlled by battery voltage. */
				break;
		case 1:	pa_level = 0xC1;
				// pa_enable = 0;	/* PA enable is controlled by battery voltage. */
				break;
		case 2:	pa_level = 0xB1;
				pa_enable = 1;			/* Force disable the PA module. */
				break;
		case 3:	pa_level = 0xC1;
				pa_enable = 1;			/* Force disable the PA module. */
				break;
	}
	Nop ();
	Nop ();

	/* Start the ADF transmitter. */
	init_spi1 ();		/* Init SPI1 for talking to the ADF. */

	switch (id_no) {
		case 0:
		case 2:	adf_init (0x00039000, 2500, 0);
				break;
		case 1:
		case 3: adf_init (0x00039000 + (2000000/10000), 2500, 0);
				break;
	}

	/* Initialize the SPI2 and DMA support for transmitting data. */
	init_spi2_dma ();

	/* Initialize the two serial ports. */
	init_rx1 ();
	init_rx2 ();


	/* Turn on the transmitter and let things onfold from here. */
	adf_cmd_phy_on ();
	adf_cmd_phy_tx ();


#if HWVERSION == 11
	/* Enable the PA block. */
	PA_EN = pa_enable;
	PA_EN_TRIS = 0;
#endif


	/* Main loop - waiting for certain events. */
	while (1) {
		struct tx_payload_block *txp_ready = NULL;

		/* Find a empty tx_payload block. */
		if (tx_payload_block [0].header.control.pending == 0) {
			txp_ready = &tx_payload_block [0];
		} else if (tx_payload_block [1].header.control.pending == 0) {
			txp_ready = &tx_payload_block [1];
		}

		if (txp_ready != NULL) {
			/* Found it. Find some data for it. */
			if (RING_PACKET_LEN (rx1) > 0) {
				/* Fetch a packet from rx1 and put it into txp_ready. */
				unsigned int	len;

				LED_2 ^= 1;		/* Toggle debug LED. */

				len = rx1_get_byte ();			/* The header byte is the length. */
				prepare_packet (&txp_ready->header, 0x11, len, rx1_get_byte);

				/* Enqueue this new block so it gets transmitted. */
				tx_fifo_add (&txp_ready->header);
			}
			else if (RING_PACKET_LEN (rx2) > 0) {
				/* Fetch a packet from rx2 and put it into txp. */
				unsigned int	len;

				LED_1 ^= 1;		/* Toggle debug LED. */

				len = rx2_get_byte ();			/* The header byte is the length. */
				prepare_packet (&txp_ready->header, 0x12, len, rx2_get_byte);

				/* Enqueue this new block so it gets transmitted. */
				tx_fifo_add (&txp_ready->header);
			}
		}


		if (update_flag) {
			/* Set by the timer1 interrupt function 10 times per second. */
			update_flag = 0;

			/* Update the tx_filler_block with new data. */
			if (current_tx_filler_block == &tx_housekeeping_block [0]) {
				/* Update block [1] as block [0] is being used. */
				housekeeping_reset ();
				prepare_packet (&tx_housekeeping_block [1].header, id_no, sizeof (housekeeping_telemetry), housekeeping_get_byte);

				/* Enqueue this new block so it gets transmitted. */
				tx_fifo_add (&tx_housekeeping_block [1].header);

				/* It is now the new filler block. */
				current_tx_filler_block = &tx_housekeeping_block [1];
			} else {
				/* Update block [0] as block [1] is being used. */
				housekeeping_reset ();
				prepare_packet (&tx_housekeeping_block [0].header, id_no, sizeof (housekeeping_telemetry), housekeeping_get_byte);

				/* Enqueue this new block so it gets transmitted. */
				tx_fifo_add (&tx_housekeeping_block [0].header);

				/* It is now the new filler block. */
				current_tx_filler_block = &tx_housekeeping_block [0];
			}

			/* Start the ADC for the next conversion. */
			AD1CON1bits.SAMP = 0;
		}

		switch (debug) {
			case 0: break;

			default:
				adf_set_pa_level (debug);
				debug = 0;
				break;
		}
	}
}
