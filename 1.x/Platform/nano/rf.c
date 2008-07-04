/*
    NanoStack: MCU software and PC tools for IP-based wireless sensor networking.
		
    Copyright (C) 2006-2007 Sensinode Ltd.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

		Address:
		Sensinode Ltd.
		Teknologiantie 6	
		90570 Oulu, Finland

		E-mail:
		info@sensinode.com
*/


/**
 *
 * \file rf.c
 * \brief nano.4 PIO based RF driver.
 *
 *  Nano.4: RF control functions.
 *   
 *	
 */


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

#include <sys/inttypes.h>

#include "stack.h"
#include "debug.h"

#include "bus.h"
#include "dma.h"
#include "mac.h"
#include "gpio.h"
#include "rf.h"

#ifndef RF_DEFAULT_POWER
#define RF_DEFAULT_POWER 100
#endif

#ifndef RF_DEFAULT_CHANNEL
#define RF_DEFAULT_CHANNEL 18
#endif

#ifdef HAVE_RF_LED
#if HAVE_RF_LED == 1
#define RF_RX_LED_ON() LED1_ON()
#define RF_RX_LED_OFF() LED1_OFF()
#define RF_TX_LED_ON() LED1_ON()
#define RF_TX_LED_OFF() LED1_OFF()
#else
#define RF_RX_LED_ON() LED2_ON()
#define RF_RX_LED_OFF() LED2_OFF()
#define RF_TX_LED_ON() LED2_ON()
#define RF_TX_LED_OFF() LED2_OFF()
#endif /*RF_LED == 1 */
#else
#ifdef HAVE_RF_DUAL_LED
#define RF_RX_LED_ON() LED1_ON()
#define RF_RX_LED_OFF() LED1_OFF()
#define RF_TX_LED_ON() LED2_ON()
#define RF_TX_LED_OFF() LED2_OFF()
#else
#define RF_RX_LED_ON()
#define RF_RX_LED_OFF()
#define RF_TX_LED_ON()
#define RF_TX_LED_OFF()
#endif /*HAVE_RF_DUAL_LED*/
#endif /*HAVE_RF_LED*/

uint8_t rf_initialized = 0;

uint8_t rf_tx_power;
uint8_t rf_flags;

#define RX_ACTIVE 0x80
#define TX_ACK 0x40
#define TX_ON_AIR 0x20
#define RX_NO_DMA

uint16_t rf_manfid;

extern int8_t timer_rf_launch(uint8_t ticks);
extern void timer_rf_stop(void);


#ifdef HAVE_RF_ERROR
uint8_t rf_error = 0;
#endif

uint8_t rf_softack;

/**
 * Execute a single CSP command.
 *
 * \param command command to execute
 *
 */
void rf_command(uint8_t command)
{
	if (command >= 0xE0)
	{	/*immediate strobe*/
		uint8_t fifo_count;
		switch (command)
		{	/*hardware bug workaround*/
			case ISRFOFF:
			case ISRXON:
			case ISTXON:
				fifo_count = RXFIFOCNT;
				RFST = command;
				pause_us(1);
				if (fifo_count != RXFIFOCNT)
				{
					RFST = ISFLUSHRX;
					RFST = ISFLUSHRX;
				}
				break;
				
			default:
				RFST = command;
		}
	}
	else if (command == SSTART)
	{
		RFIF &= ~IRQ_CSP_STOP;	/*clear IRQ flag*/
		RFST = SSTOP;	/*make sure there is a stop in the end*/
		RFST = ISSTART;	/*start execution*/
		while((RFIF & IRQ_CSP_STOP) == 0);
	}
	else
	{
		RFST = command;	/*write command*/
	}
}

/**
 * Select RF channel.
 *
 * \param channel channel number to select
 *
 * \return channel value or negative (invalid channel number)
 */
 
 /* channel freqdiv = (2048 + FSCTRL(9:0)) / 4
            freq = (2048 + FSCTRL(9:0)) MHz */

portCHAR rf_channel_set(uint8_t channel)
{
	uint16_t freq;
	
	if ( (channel < 11) || (channel > 26) ) return -1;
	rf_command(ISSTOP);	/*make sure CSP is not running*/
	rf_command(ISRFOFF);
	/* Channel values: 11-26 */
	freq = (uint16_t) channel - 11;
	freq *= 5;	/*channel spacing*/
	freq += 357; /*correct channel range*/
	freq |= 0x4000; /*LOCK_THR = 1*/
	FSCTRLH = (freq >> 8);
	FSCTRLL = (uint8_t)freq;
	
	rf_command(ISRXON);
	RFST = ISFLUSHRX;
	RFST = ISFLUSHRX;
	return (int8_t) channel;
}

/*PA_LEVEL TXCTRL register Output Power [dBm] Current Consumption [mA] 
	31 0xA0FF 0 17.4 
	27 0xA0FB -1 16.5 
	23 0xA0F7 -3 15.2 
	19 0xA0F3 -5 13.9 
	15 0xA0EF -7 12.5 
	11 0xA0EB -10 11.2 
	 7 0xA0E7 -15 9.9 
	 3 0xA0E3 -25 8.5*/

/**
 * Select RF transmit power.
 *
 * \param new_power new power level (in per cent)
 *
 * \return new level or negative (value out of range)
 */
 
portCHAR rf_power_set(uint8_t new_power)
{
	uint16_t power;
	
	if (new_power > 100) return -1;
	
	power = 31 * new_power;
	power /= 100;
	power += 0xA160;
		
	/* Set transmitter power */
	TXCTRLH = (power >> 8);
	TXCTRLL = (uint8_t)power;	
	
	rf_tx_power = (int8_t) new_power;
	return rf_tx_power;
}

uint8_t rf_dma_buffer[5];

/**
 * Enable RF receiver.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR rf_rx_enable(void)
{
	if (!(rf_flags & RX_ACTIVE))
	{
  		IOCFG0 = 0x7f;   // Set the FIFOP threshold 127
		RSSIH = 0xd2; /* -84dbm = 0xd2 default, 0xe0 -70 dbm */
		rf_flags |= RX_ACTIVE;

		RFPWR &= ~RREG_RADIO_PD;	/*make sure it's powered*/
		while((RFIF & IRQ_RREG_ON) == 0);	/*wait for power up*/
		SLEEP &= ~OSC_PD; /*Osc on*/
		while((SLEEP & XOSC_STB) == 0);	/*wait for power up*/

		rf_command(ISRXON);		
	}
	RFST = ISFLUSHRX;
	RFST = ISFLUSHRX;
	return pdTRUE;
}

/**
 * Disable RF receiver.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR rf_rx_disable(void)
{
	rf_command(ISSTOP);	/*make sure CSP is not running*/
	rf_command(ISRFOFF);

	RFPWR |= RREG_RADIO_PD;		/*RF powerdown*/

	rf_flags = 0;
	return pdTRUE;
}


/**
 * Enable RF transmitter.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR rf_tx_enable(void)
{
	DMAARM = 0x80 + (1 << 0);	/*ABORT + channel bit*/

	return pdTRUE;
}
   
/**
 * Initialize RF.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free or init failed
 */

portCHAR rf_init(void)
{
	portCHAR retval = pdFALSE;

	if (rf_initialized) return pdTRUE;

	RFPWR &= ~RREG_RADIO_PD;	/*make sure it's powered*/
	while ((RFPWR & ADI_RADIO_PD) == 1);
	while((RFIF & IRQ_RREG_ON) == 0);	/*wait for power up*/
	SLEEP &= ~OSC_PD; /*Osc on*/
	while((SLEEP & XOSC_STB) == 0);	/*wait for power up*/
	
	rf_flags = 0;
	rf_softack = 0;
		
	retval = pdTRUE;

	FSMTC1 = 1;	/*don't abort reception, if enable called, accept ack, auto rx after tx*/
	
	MDMCTRL0H = 0x02;	 /* Generic client, standard hysteresis, decoder on 0x0a */
	MDMCTRL0L = 0xE2;	 /* automatic ACK and CRC, standard CCA and preamble 0xf2 */

	MDMCTRL1H = 0x30;			/*defaults*/
	MDMCTRL1L = 0x00;

	RXCTRL0H = 0x32;
	RXCTRL0L = 0xf5;
	
	/* get ID for MAC */
	rf_manfid = CHVER;
	rf_manfid <<= 8;		
	rf_manfid += CHIPID;

	rf_channel_set(RF_DEFAULT_CHANNEL);

	/* Set transmitter power */
	rf_power_set(RF_DEFAULT_POWER);

	retval = pdTRUE;
		
	rf_command(ISFLUSHTX);
	rf_command(ISFLUSHRX);	
	//RFIM = IRQ_FIFOP | IRQ_SFD | IRQ_TXDONE;
	//RFIF &= ~(IRQ_FIFOP | IRQ_SFD| IRQ_TXDONE);

	RFIM = IRQ_FIFOP;
	RFIF &= ~(IRQ_FIFOP);

	S1CON &= ~(RFIF_0 | RFIF_1);
	IEN2 |= RFIE;
	//IEN0_RFERRIE = 1;
	
	//rf_rx_enable();

	RF_TX_LED_OFF();
	RF_RX_LED_OFF();
	rf_initialized = 1;
	return retval;
}

/**
	* Set address decoder parameters
	*
	*	\param address address for decoder
	*/
void rf_set_address(sockaddr_t *address)
{
	uint8_t i;
	__xdata unsigned char *ptr;
	
	switch(address->addr_type)
	{
		case ADDR_802_15_4_PAN_LONG:
			PANIDH = address->address[8];
			PANIDL = address->address[9];
		case ADDR_802_15_4_LONG:
			ptr = &IEEE_ADDR0;
			for(i=0; i<8;i++)
			{
				*ptr++ = address->address[i];
			}
			break;
		
		case ADDR_802_15_4_PAN_SHORT:
			PANIDH = address->address[2];
			PANIDL = address->address[3];
		case ADDR_802_15_4_SHORT:
			SHORTADDRH = address->address[0];
			SHORTADDRL = address->address[1];
			break;
		
		default:
			break;
	}			
}
/**
 * Set address decoder on/off.
 *
 * \param param 1=on 0=off. 
 * \return pdTRUE operation successful
 */
portCHAR rf_address_decoder_mode(rf_address_mode_t mode)
{
	portCHAR retval = pdFALSE;

	rf_softack = 0;
	/* set oscillator on*/	
	switch(mode)
	{
		case RF_SOFTACK_MONITOR:
			rf_softack = 1;
		case RF_MONITOR:
			MDMCTRL0H |= 0x10;	 /*Address-decode off , coordinator*/
			MDMCTRL0L &= ~0x10;	 /*no automatic ACK */
			break;
			
		case RF_DECODER_COORDINATOR:
			MDMCTRL0H |= 0x18;	 /*Address-decode on , coordinator*/
			MDMCTRL0L |= 0x10;	 /*automatic ACK */
			break;
			
		case RF_DECODER_ON:
			MDMCTRL0H |= 0x08;	 /*Address-decode on */
			//MDMCTRL0H &= ~0x18;	 /* Generic client */
			//MDMCTRL0L |= 0x10;	 /*automatic ACK */
			MDMCTRL0L &= ~0x10;	 /* no automatic ACK */
			break;

		default:
			MDMCTRL0H &= ~0x18;	 /* Generic client */
			MDMCTRL0L &= ~0x10;	 /* no automatic ACK */
			break;
	}
	retval = pdTRUE;
	return retval; 
}



/**
 * Channel energy detect.
 *
 * Coordinator use this function detect best channel for PAN-network.
 * \return RSSI-energy level dBm.
 * \return 0	operation failed.
 */
 
int8_t rf_analyze_rssi(void)
{
	int8_t retval = -128;
	pause_us(128);	

	retval = (int8_t)RSSIL;
	retval -= 45;
	return retval; 
}

/**
 * Clear channel assesment check.
 *
 * \return pdTRUE	CCA clear
 * \return pdFALSE	CCA reserved
 */
portCHAR rf_cca_check(uint8_t backoff_count, uint8_t slotted)
{
	uint8_t counter, cca=1;
	portCHAR retval = pdTRUE;
	backoff_count;
	rf_command(ISRXON);

	pause_us(16);
	pause_us(16);
	pause_us(16);
	pause_us(16);
	switch (slotted)
	{
		case 1:

		if(RFSTATUS & CCA)
		{
			counter=0;
			cca=1;
			while(cca!=0) 
			{
				if(counter > 1)
					cca=0;
				pause_us(250);
				if(!(RFSTATUS & CCA))
				{
					cca=0;
					retval = pdFALSE;
				}
				counter++;
			}
		}
		else
			retval = pdFALSE;
		break;

		case 0:
			if(!(RFSTATUS & CCA))
			{
				retval = pdFALSE;
			}
			else
			{
			}
			break;
	}
	return retval;		
}



/**
 * Send ACK.
 *
 *\param pending set up pending flag if pending > 0. 
 */
void rf_send_ack(uint8_t pending)
{
	if(pending)
	{
		rf_command(ISACKPEND);
	}
	else
	{
		rf_command(ISACK);
	}
}

/**
 * Transmit packet.
 *
 * Missing feature: address type check
 *
 * \param mac RF HW address
 * \param dst destination HW address
 * \param buffer data buffer pointer
 * \param length length of data
 *
 * \return pdTRUE+1 channel occupied
 * \return pdTRUE
 * \return pdFALSE	bus reserved
 */
portCHAR rf_write(buffer_t *buffer)
{
	uint8_t counter, i;
	portCHAR retval = pdTRUE;
	int16_t length =  buffer->buf_end - buffer->buf_ptr;
	
	if (rf_flags & TX_ACK) return pdFALSE;
	if ((rf_flags & RX_ACTIVE) == 0) rf_rx_enable();

	if (length <= 128) 
	{
		uint8_t *ptr = buffer->buf + buffer->buf_ptr;
		rf_command(ISFLUSHTX);
		RFD = (length+2);
		for (i = 0 ; i < length ; i++)
		{
			RFD = *ptr++;
		}
		RFD = (0);
		RFD = (0);

		if (rf_cca_check(0,0) == pdFALSE)
		{
			return pdFALSE;
		}

		RFIF &= ~IRQ_TXDONE;
		rf_command(ISTXON);
		counter = 0;	
		while(!(RFSTATUS & TX_ACTIVE) && (counter++ < 3))
		{
			/*if (RFSTATUS & (SFD | TX_ACTIVE) == SFD)
			{
				rf_command(ISRXON);
				counter = 3;
			}
			else
			{*/
				pause_us(10);
			//}
		}

		if (!(RFSTATUS & TX_ACTIVE))
		{
#ifdef RF_DEBUG
			debug("\r\nRF: TX never active.\r\n");
#endif
			retval = pdFALSE;
				//retval =pdTRUE+1;
		}
		else
		{
			RF_RX_LED_OFF();
			RF_TX_LED_ON();
			//rf_flags |= TX_ACK | TX_ON_AIR;
			//rf_flags |= TX_ON_AIR;
		}
	}
	if ((retval == pdTRUE) && (length > 128))
	{
#ifdef RF_DEBUG
		debug("Packet too long(");
		debug_int(length);
		debug(").\r\n");
#endif
		retval = pdFALSE;
	}

	return retval;		
}

uint8_t ft_buffer[8];
/**
	* Get HW MAC address
	*
	* \param address address to set
	*/
portCHAR rf_mac_get(sockaddr_t *address)
{
	mac_get(address);
	if (address->addr_type == ADDR_NONE)
	{
		uint8_t i;
		flash_read(&ft_buffer[0], 0x1FFF8, 8);
		for (i=0; i<8; i++)
		{
			address->address[7-i] = ft_buffer[i];
		}
		address->addr_type = ADDR_802_15_4_PAN_LONG;
		if (stack_check_broadcast(address->address, ADDR_802_15_4_LONG) == pdTRUE)
		{
			debug("No address in flash.\n");
			address->address[0]=0x00;
			address->address[1]=0x00;
			address->address[2]=0x00;
			address->address[3]=0x00;
			address->address[4]=0x00;
			address->address[5]=0x00;
#ifdef SHORT_ADDRESS
			address->address[6]= (SHORT_ADDRESS & 0xff);
			address->address[7]= (SHORT_ADDRESS >> 8);
#else
#warning "SHORT_ADDRESS not defined"
			address->address[6]= 0x12;
			address->address[7]= 0x34;
#endif
		}
		address->address[8] = 0xff;
		address->address[9] = 0xff;
		mac_set(address);
	}

	return pdTRUE;
}

extern xQueueHandle buffers;
#ifdef HAVE_MAC_15_4
extern void mac_push(buffer_t *b);
extern void mac_rx_push(void);
extern uint8_t ack_handle(uint8_t sqn);
extern portCHAR check_mac_rx_size(void);
#endif
extern sockaddr_t mac_long;
#ifdef HAVE_NRP
extern sockaddr_t mac_short;
#endif

uint8_t rfi_tmp, rfi_pac_type ;

void rf_rx_callback(void *param)
{
	param;
	return;
}

void rf_timer_callback(void);
void rf_timer_callback(void)
{
	rf_flags &= ~TX_ACK;
}

/**
 * RF interrupt service routine.
 *
 */
void rf_ISR( void ) interrupt (RF_VECTOR)
{
	EA = 0;
	RF_TX_LED_ON();
	if (RFIF & IRQ_TXDONE)
	{
		
		if (rf_flags & TX_ON_AIR)
		{
			//rf_flags |= TX_ACK;
			rf_flags ^= TX_ON_AIR;
			//timer_rf_launch(MAC_IFS); //832 us in timer ticks
		}
		RF_TX_LED_OFF();
		RFIF &= ~IRQ_TXDONE;
	}
	if (RFIF & IRQ_FIFOP)
	{
		RF_TX_LED_OFF();
		if (RFSTATUS & FIFO)
		{
			uint8_t len = RFD;
			len &= 0x7f;
			if(len > 4)
			{
				rf_dma_buffer[0] = RFD;
				rfi_pac_type = rf_dma_buffer[0];
				rfi_pac_type &= 0x07;
				if(rfi_pac_type==0x02)
				{
					rf_dma_buffer[1] = RFD;
					rf_dma_buffer[2] = RFD;
					rf_dma_buffer[3] = RFD;
					rf_dma_buffer[4] = RFD;
					if (rf_dma_buffer[4] & 0x80)
					{
						ack_handle(rf_dma_buffer[2]);
					}
				}
				else
				{
					portBASE_TYPE task = pdFALSE;
					buffer_t *rf_buf = 0;
					len--;
					if(len && check_mac_rx_size() != pdTRUE)
					{
						len=0;
						rf_command(ISFLUSHRX);
					}	
#ifdef STACK_RING_BUFFER_MODE
					if(len)
					{
						rf_buf=0;
						rf_buf = stack_buffer_pull();
					}
					if (len && rf_buf)
#else
					if (len && (xQueueReceiveFromISR( buffers, &( rf_buf ), &prev_task) == pdTRUE))
#endif
					{
						uint8_t i;
						uint8_t *ptr2 = &(rf_buf->buf[0]);
						*ptr2++ = rf_dma_buffer[0];

						if(rf_dma_buffer[0] & 0x20)
						{
#ifdef HAVE_NRP 
							if ((rf_softack & 2) == 0)
							{
								RFST = ISACK;
							}
#else
								RFST = ISACK;		
#endif
						}
						len -= 2;
						for (i=0; i<len; i++)
						{
							*ptr2++ = RFD;
						}
			
						rf_buf->options.rf_dbm = ((int8_t) RFD) - 45;
						rf_buf->options.rf_lqi = RFD;
				
						
						if ((rf_buf->options.rf_lqi & 0x80) )
						{
							len++;
							rf_buf->options.type = BUFFER_DATA;
							rf_buf->socket = 0;
							rf_buf->buf_ptr = 0;
							rf_buf->buf_end = len;
							rf_buf->options.rf_lqi &= 0x7F;
							rf_buf->dir = BUFFER_UP;
		
			#ifdef HAVE_MAC_15_4
								mac_push(rf_buf);
								mac_rx_push();
			#else
							{
								event_t event;
								event.process = 0;
								event.param = (void *) rf_buf;
				
								prev_task = xQueueSendFromISR( events, ( void * ) &event, prev_task);
							}
			#endif
						}
						else
						{
							rf_buf->buf_end = 0;
		#ifdef STACK_RING_BUFFER_MODE
							stack_buffer_add(rf_buf);
		#else
							prev_task = xQueueSendFromISR( buffers, ( void * ) &rf_buf, prev_task);
		#endif
							rf_buf=0;
						}
			}
			else
			{
				rf_command(ISFLUSHRX);
			}
				}
			}
			else
			{
				rf_command(ISFLUSHRX);
				rf_command(ISFLUSHRX);
			}
		RF_RX_LED_ON();
		}
		else
		{
			rf_command(ISFLUSHRX);
			rf_command(ISFLUSHRX);
		}
		RFIF &= ~IRQ_FIFOP;
	}
	S1CON &= ~(RFIF_0 | RFIF_1);
	RFIM |= IRQ_FIFOP;
	
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
	EA = 1;
}

/**
 * RF error interrupt service routine.
 *
 */
void rf_error_ISR( void ) interrupt (RFERR_VECTOR)
{
	EA = 0;
	TCON_RFERRIF = 0;
#ifdef HAVE_RF_ERROR
	rf_error = 254;
#endif
	rf_command(ISRFOFF);
	rf_command(ISFLUSHRX);
	rf_command(ISFLUSHRX);
	rf_command(ISRXON);
	RF_RX_LED_OFF();
	RF_TX_LED_OFF();
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
	EA = 1;
}

