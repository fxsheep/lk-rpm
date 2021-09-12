#include <lk/reg.h>
#include <stdio.h>
#include <lib/cbuf.h>
#include <kernel/thread.h>
#include <dev/uart.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <platform/interrupts.h>
#include <platform/iomap.h>
//#include <platform/irqs.h>
#include <platform/clock.h>
//#include <platform/gpio.h>
#include <platform/uart_dm.h>
#include <platform/sysaccess.h>
#include <arch/ops.h>

#ifndef NULL
#define NULL        0
#endif

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))

static int uart_init_flag = 0;
static spin_lock_t uart_tx_spin_lock = 0;

/* Note:
 * This is a basic implementation of UART_DM protocol. More focus has been
 * given on simplicity than efficiency. Few of the things to be noted are:
 * - RX path may not be suitable for multi-threaded scenaraio because of the
 *   use of static variables. TX path shouldn't have any problem though. If
 *   multi-threaded support is required, a simple data-structure can
 *   be maintained for each thread.
 * - Right now we are using polling method than interrupt based.
 * - We are using legacy UART protocol without Data Mover.
 * - Not all interrupts and error events are handled.
 * - While waiting Watchdog hasn't been taken into consideration.
 */

#define NON_PRINTABLE_ASCII_CHAR      128

static uint8_t pack_chars_into_words(uint8_t *buffer, uint8_t cnt, uint32_t *word)
{
	uint8_t num_chars_writtten = 0;

	*word = 0;

	 for(int j=0; j < cnt; j++)
	 {
		 if (buffer[num_chars_writtten] == '\n')
		 {
			/* replace '\n' by the NON_PRINTABLE_ASCII_CHAR and print '\r'.
			 * While printing the NON_PRINTABLE_ASCII_CHAR, we will print '\n'.
			 * Thus successfully replacing '\n' by '\r' '\n'.
			 */
			*word |= ('\r' & 0xff) << (j * 8);
			buffer[num_chars_writtten] = NON_PRINTABLE_ASCII_CHAR;
		 }
		 else
		 {
			if (buffer[num_chars_writtten] == NON_PRINTABLE_ASCII_CHAR)
			{
				buffer[num_chars_writtten] = '\n';
			}

			 *word |= (buffer[num_chars_writtten] & 0xff) << (j * 8);

			 num_chars_writtten++;
		 }
	 }

	 return num_chars_writtten;
}

/* Static Function Prototype Declarations */
static unsigned int msm_boot_uart_calculate_num_chars_to_write(char *data_in,
							       uint32_t *num_of_chars);
static unsigned int msm_boot_uart_dm_init(uint64_t base);
static unsigned int msm_boot_uart_dm_read(uint64_t base,
	unsigned int *data, int wait);
static unsigned int msm_boot_uart_dm_write(uint64_t base, char *data,
	unsigned int num_of_chars);
static unsigned int msm_boot_uart_dm_init_rx_transfer(uint64_t base);
static unsigned int msm_boot_uart_dm_reset(uint64_t base);

/* Keep track of uart block vs port mapping.
 */
static uint64_t port_lookup[4];

/* Extern functions */
//void udelay(unsigned usecs);

/*
 * Helper function to keep track of Line Feed char "\n" with
 * Carriage Return "\r\n".
 */
static unsigned int
msm_boot_uart_calculate_num_chars_to_write(char *data_in,
				uint32_t *num_of_chars)
{
	uint32_t i = 0, j = 0;

	if ((data_in == NULL)) {
		return MSM_BOOT_UART_DM_E_INVAL;
	}

	for (i = 0, j = 0; i < *num_of_chars; i++, j++) {
		if (data_in[i] == '\n') {
			j++;
		}

	}

	*num_of_chars = j;

	return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*
 * Reset the UART
 */
static unsigned int msm_boot_uart_dm_reset(uint64_t base)
{
	sys_writel(MSM_BOOT_UART_DM_CMD_RESET_RX, MSM_BOOT_UART_DM_CR(base));
	sys_writel(MSM_BOOT_UART_DM_CMD_RESET_TX, MSM_BOOT_UART_DM_CR(base));
	sys_writel(MSM_BOOT_UART_DM_CMD_RESET_ERR_STAT, MSM_BOOT_UART_DM_CR(base));
	sys_writel(MSM_BOOT_UART_DM_CMD_RES_TX_ERR, MSM_BOOT_UART_DM_CR(base));
	sys_writel(MSM_BOOT_UART_DM_CMD_RES_STALE_INT, MSM_BOOT_UART_DM_CR(base));

	return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*
 * Initialize UART_DM - configure clock and required registers.
 */
static unsigned int msm_boot_uart_dm_init(uint64_t uart_dm_base)
{
	/* Configure UART mode registers MR1 and MR2 */
	/* Hardware flow control isn't supported */
	sys_writel(0x0, MSM_BOOT_UART_DM_MR1(uart_dm_base));

	/* 8-N-1 configuration: 8 data bits - No parity - 1 stop bit */
	sys_writel(MSM_BOOT_UART_DM_8_N_1_MODE, MSM_BOOT_UART_DM_MR2(uart_dm_base));

	/* Configure Interrupt Mask register IMR */
	sys_writel(MSM_BOOT_UART_DM_IMR_ENABLED, MSM_BOOT_UART_DM_IMR(uart_dm_base));

	/* Configure Tx and Rx watermarks configuration registers */
	/* TX watermark value is set to 0 - interrupt is generated when
	 * FIFO level is less than or equal to 0 */
	sys_writel(MSM_BOOT_UART_DM_TFW_VALUE, MSM_BOOT_UART_DM_TFWR(uart_dm_base));

	/* RX watermark value */
	sys_writel(MSM_BOOT_UART_DM_RFW_VALUE, MSM_BOOT_UART_DM_RFWR(uart_dm_base));

	/* Configure Interrupt Programming Register */
	/* Set initial Stale timeout value */
	sys_writel(MSM_BOOT_UART_DM_STALE_TIMEOUT_LSB, MSM_BOOT_UART_DM_IPR(uart_dm_base));

	/* Configure IRDA if required */
	/* Disabling IRDA mode */
	sys_writel(0x0, MSM_BOOT_UART_DM_IRDA(uart_dm_base));

	/* Configure and enable sim interface if required */

	/* Configure hunt character value in HCR register */
	/* Keep it in reset state */
	sys_writel(0x0, MSM_BOOT_UART_DM_HCR(uart_dm_base));

	/* Configure Rx FIFO base address */
	/* Both TX/RX shares same SRAM and default is half-n-half.
	 * Sticking with default value now.
	 * As such RAM size is (2^RAM_ADDR_WIDTH, 32-bit entries).
	 * We have found RAM_ADDR_WIDTH = 0x7f */

	/* Issue soft reset command */
	msm_boot_uart_dm_reset(uart_dm_base);

	/* Enable/Disable Rx/Tx DM interfaces */
	/* Data Mover not currently utilized. */
	sys_writel(0x0, MSM_BOOT_UART_DM_DMEN(uart_dm_base));

	/* Enable transmitter and receiver */
	sys_writel(MSM_BOOT_UART_DM_CR_RX_ENABLE, MSM_BOOT_UART_DM_CR(uart_dm_base));
	sys_writel(MSM_BOOT_UART_DM_CR_TX_ENABLE, MSM_BOOT_UART_DM_CR(uart_dm_base));

	/* Initialize Receive Path */
	msm_boot_uart_dm_init_rx_transfer(uart_dm_base);

	return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*
 * Initialize Receive Path
 */
static unsigned int msm_boot_uart_dm_init_rx_transfer(uint64_t uart_dm_base)
{
	sys_writel(MSM_BOOT_UART_DM_GCMD_DIS_STALE_EVT, MSM_BOOT_UART_DM_CR(uart_dm_base));
	sys_writel(MSM_BOOT_UART_DM_CMD_RES_STALE_INT, MSM_BOOT_UART_DM_CR(uart_dm_base));
	sys_writel(MSM_BOOT_UART_DM_DMRX_DEF_VALUE, MSM_BOOT_UART_DM_DMRX(uart_dm_base));
	sys_writel(MSM_BOOT_UART_DM_GCMD_ENA_STALE_EVT, MSM_BOOT_UART_DM_CR(uart_dm_base));

	return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*
 * UART Receive operation
 * Reads a word from the RX FIFO.
 */
static unsigned int
msm_boot_uart_dm_read(uint64_t base, unsigned int *data, int wait)
{
	static int rx_last_snap_count = 0;
	static int rx_chars_read_since_last_xfer = 0;

	if (data == NULL) {
		return MSM_BOOT_UART_DM_E_INVAL;
	}

	/* We will be polling RXRDY status bit */
	while (!(sys_readl(MSM_BOOT_UART_DM_SR(base)) & MSM_BOOT_UART_DM_SR_RXRDY)) {
		/* if this is not a blocking call, we'll just return */
		if (!wait) {
			return MSM_BOOT_UART_DM_E_RX_NOT_READY;
		}
	}

	/* Check for Overrun error. We'll just reset Error Status */
	if (sys_readl(MSM_BOOT_UART_DM_SR(base)) & MSM_BOOT_UART_DM_SR_UART_OVERRUN) {
		sys_writel(MSM_BOOT_UART_DM_CMD_RESET_ERR_STAT, MSM_BOOT_UART_DM_CR(base));
	}

	/* RX FIFO is ready; read a word. */
	*data = sys_readl(MSM_BOOT_UART_DM_RF(base, 0));

	/* increment the total count of chars we've read so far */
	rx_chars_read_since_last_xfer += 4;

	/* Rx transfer ends when one of the conditions is met:
	 * - The number of characters received since the end of the previous
	 *   xfer equals the value written to DMRX at Transfer Initialization
	 * - A stale event occurred
	 */

	/* If RX transfer has not ended yet */
	if (rx_last_snap_count == 0) {
		/* Check if we've received stale event */
		if (sys_readl(MSM_BOOT_UART_DM_MISR(base)) & MSM_BOOT_UART_DM_RXSTALE) {
			/* Send command to reset stale interrupt */
			sys_writel(MSM_BOOT_UART_DM_CMD_RES_STALE_INT, MSM_BOOT_UART_DM_CR(base));
		}

		/* Check if we haven't read more than DMRX value */
		else if ((unsigned int)rx_chars_read_since_last_xfer <
			sys_readl(MSM_BOOT_UART_DM_DMRX(base))) {
			/* We can still continue reading before initializing RX transfer */
			return MSM_BOOT_UART_DM_E_SUCCESS;
		}

		/* If we've reached here it means RX
		 * xfer end conditions been met
		 */

		/* Read UART_DM_RX_TOTAL_SNAP register
		 * to know how many valid chars
		 * we've read so far since last transfer
		 */
		rx_last_snap_count = sys_readl(MSM_BOOT_UART_DM_RX_TOTAL_SNAP(base));

	}

	/* If there are still data left in FIFO we'll read them before
	 * initializing RX Transfer again */
	if ((rx_last_snap_count - rx_chars_read_since_last_xfer) >= 0) {
		return MSM_BOOT_UART_DM_E_SUCCESS;
	}

	msm_boot_uart_dm_init_rx_transfer(base);
	rx_last_snap_count = 0;
	rx_chars_read_since_last_xfer = 0;

	return MSM_BOOT_UART_DM_E_SUCCESS;
}

/*
 * UART transmit operation
 */
static unsigned int
msm_boot_uart_dm_write(uint64_t base, char *data, unsigned int num_of_chars)
{
	unsigned int tx_word_count = 0;
	unsigned int tx_char_left = 0, tx_char = 0;
	unsigned int tx_word = 0;
	int i = 0;
	char *tx_data = NULL;
	uint8_t num_chars_written;
    spin_lock_saved_state_t state;

	if ((data == NULL) || (num_of_chars <= 0)) {
		return MSM_BOOT_UART_DM_E_INVAL;
	}

	msm_boot_uart_calculate_num_chars_to_write(data, &num_of_chars);

	tx_data = data;

	/* Write to NO_CHARS_FOR_TX register number of characters
	 * to be transmitted. However, before writing TX_FIFO must
	 * be empty as indicated by TX_READY interrupt in IMR register
	 */

	/* Check if transmit FIFO is empty.
	 * If not we'll wait for TX_READY interrupt. */
	if (!(sys_readl(MSM_BOOT_UART_DM_SR(base)) & MSM_BOOT_UART_DM_SR_TXEMT)) {
		while (!(sys_readl(MSM_BOOT_UART_DM_ISR(base)) & MSM_BOOT_UART_DM_TX_READY));
	}

	//We need to make sure the DM_NO_CHARS_FOR_TX&DM_TF are are programmed atmoically.
    spin_lock_irqsave(&uart_tx_spin_lock, state);
	/* We are here. FIFO is ready to be written. */
	/* Write number of characters to be written */
	sys_writel(num_of_chars, MSM_BOOT_UART_DM_NO_CHARS_FOR_TX(base));

	/* Clear TX_READY interrupt */
	sys_writel(MSM_BOOT_UART_DM_GCMD_RES_TX_RDY_INT, MSM_BOOT_UART_DM_CR(base));

	/* We use four-character word FIFO. So we need to divide data into
	 * four characters and write in UART_DM_TF register */
	tx_word_count = (num_of_chars % 4) ? ((num_of_chars / 4) + 1) :
	    (num_of_chars / 4);
	tx_char_left = num_of_chars;

	for (i = 0; i < (int)tx_word_count; i++) {
		tx_char = (tx_char_left < 4) ? tx_char_left : 4;
		num_chars_written = pack_chars_into_words((uint8_t *)tx_data, tx_char, &tx_word);

		/* Wait till TX FIFO has space */
		while (!(sys_readl(MSM_BOOT_UART_DM_SR(base)) & MSM_BOOT_UART_DM_SR_TXRDY));
		/* TX FIFO has space. Write the chars */
		sys_writel(tx_word, MSM_BOOT_UART_DM_TF(base, 0));
		tx_char_left = num_of_chars - (i + 1) * 4;
		tx_data = tx_data + num_chars_written;
	}
    spin_unlock_irqrestore(&uart_tx_spin_lock, state);

	return MSM_BOOT_UART_DM_E_SUCCESS;
}

/* Defining functions that's exposed to outside world and in coformance to
 * existing uart implemention. These functions are being called to initialize
 * UART and print debug messages in bootloader.
 */
void uart_dm_init(uint8_t id, uint64_t gsbi_base, uint64_t uart_dm_base)
{
	static uint8_t port = 0;

	/* Configure the uart clock */
//	clock_config_uart_dm(id);//FIXME
	mb();

	/* Configure GPIO to provide connectivity between UART block
	   product ports and chip pads */
//	gpio_config_uart_dm(id);//FIXME
	mb();

	/* Configure clock selection register for tx and rx rates.
	 * Selecting 115.2k for both RX and TX.
	 */
	sys_writel(UART_DM_CLK_RX_TX_BIT_RATE, MSM_BOOT_UART_DM_CSR(uart_dm_base));
	mb();

	/* Intialize UART_DM */
	msm_boot_uart_dm_init(uart_dm_base);

	ASSERT(port < ARRAY_SIZE(port_lookup));
	port_lookup[port++] = uart_dm_base;

	/* Set UART init flag */
	uart_init_flag = 1;
}

/* UART_DM uses four character word FIFO where as UART core
 * uses a character FIFO. so it's really inefficient to try
 * to write single character. But that's how dprintf has been
 * implemented.
 */
int uart_putc(int port, char c)
{
	uint64_t uart_base = port_lookup[port];

	/* Don't do anything if UART is not initialized */
	if (!uart_init_flag)
		return -1;

	msm_boot_uart_dm_write(uart_base, &c, 1);

	return 0;
}

/* UART_DM uses four character word FIFO whereas uart_getc
 * is supposed to read only one character. So we need to
 * read a word and keep track of each character in the word.
 */
int uart_getc(int port, bool wait)
{
	int byte;
	static unsigned int word = 0;
	uint64_t uart_base = port_lookup[port];

	/* Don't do anything if UART is not initialized */
	if (!uart_init_flag)
		return -1;

	if (!word) {
		/* Read from FIFO only if it's a first read or all the four
		 * characters out of a word have been read */
		if (msm_boot_uart_dm_read(uart_base, &word, wait) != MSM_BOOT_UART_DM_E_SUCCESS) {
			return -1;
		}

	}

	byte = (int)word & 0xff;
	word = word >> 8;

	return byte;
}
