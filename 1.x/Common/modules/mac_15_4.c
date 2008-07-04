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
 * \file     mac_15_4.c
 * \brief    802.15.4  protocol module.
 *
 *  802.15.4 MAC and CSMA sequence: handler functions.
 *  Modular version for positioning support etc.
 */



#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "list.h"

#ifndef MAC_15_4_DEBUG
#undef HAVE_DEBUG
#endif

#include "debug.h"
#include "socket.h"
#include "buffer.h"
#include "bus.h"
#include "module.h"
/*#include "neighbor_routing_table.h"*/
#include "control_message.h"
/*#include "event_timer.h"*/
#include "cipv6.h"
#include "rf.h"
#include "mac.h"
#include "gpio.h"
#include "aes.h"

/*
[NAME]
MAC_15_4

[ID]
MODULE_MAC_15_4,

[INFO]
#ifdef HAVE_MAC_15_4
  {mac_init, mac_handle, mac_check, 0, MODULE_MAC_15_4, 23, ADDR_802_15_4_PAN_LONG, 0 },
#endif

[FUNCS]*/
extern portCHAR mac_init(buffer_t *buf);
extern portCHAR mac_handle( buffer_t *buf );
extern portCHAR mac_check( buffer_t *buf );

/*
	Externals
	*/
#define MAC_TICK_FACTOR 1/32
#define PLATFORM_RAND_SEED 32
//#define MAC_RING_BUFFER_SIZE (STACK_BUFFERS_MAX + 1)
//#define MAC_RING_BUFFER_SIZE_RX MAC_RING_BUFFER_SIZE
#define MAC_RING_BUFFER_SIZE_RX (4 + 1)
#define MAC_RING_BUFFER_SIZE_TX (3 + 1)
//#define MAC_RING_BUFFER_SIZE_TX MAC_RING_BUFFER_SIZE

extern void timer_mac_launch(uint16_t mac_ticks);
extern void timer_mac_stop(void);

extern void rf_rx_callback(void *param);
#ifdef HAVE_NRP
extern void nrp_channel_notify(void);
#endif
extern sockaddr_t mac_short;
extern sockaddr_t mac_long;

/*
	Internals
	*/
#undef FC_DST_MODE
#undef FC_DST_ADDR_NONE
#undef FC_DST_16_BITS
#undef FC_DST_64_BITS

#undef FC_SRC_MODE
#undef FC_SRC_ADDR_NONE
#undef FC_SRC_16_BITS
#undef FC_SRC_64_BITS

/** */
#undef FC_INTRA_PAN
#undef FC_ACK
#undef FC_SEC
#undef FC_PENDING
#undef FC_FRAME_TYPE_MASK
#undef FC_BEACON_FRAME	
#undef FC_DATA_FRAME		
#undef FC_ACK_FRAME		
#undef FC_COMMAND_FRAME

#define FC_FRAME_TYPE_SHIFT 0
/** MAC Header masks */
#define FC_DST_MODE				0x0C00
#define FC_DST_ADDR_NONE	0x0000
#define FC_DST_16_BITS		0x0800
#define FC_DST_64_BITS		0x0C00

#define FC_SRC_MODE				0xC000   
#define FC_SRC_ADDR_NONE	0x0000
#define FC_SRC_16_BITS		0x8000
#define FC_SRC_64_BITS		0xC000 

/** */
#define FC_INTRA_PAN				0x0040     
#define FC_ACK							0x0020     
#define FC_SEC							0x0008 
#define FC_PENDING					0x0010    
#define FC_FRAME_TYPE_MASK 	0x0007
#define FC_BEACON_FRAME			0x0000     
#define FC_DATA_FRAME				0x0001     
#define FC_ACK_FRAME				0x0002     
#define FC_COMMAND_FRAME		0x0003

#define FC_ACK_PENDING		(FC_ACK_FRAME+FC_PENDING)
#define FC_ACK_NO_PENDING	(FC_ACK_FRAME)

#define MAC_RETRY_MAX 3
#define MAC_CCA_RETRY_MAX 9
//#define CHK_SUM

void mac_task( void *pvParameters );
// 215 40 on noin 5 ms
#define MAC_CCA_TIME (random_generate(120)) + 40
//#define MAC_ACK_TIME MAC_IFS*2

//#define MAC_CCA_TIME (random_generate(MAC_IFS)/32)+MAC_IFS
#define MAC_ACK_TIME 6000/128

typedef enum
{
	MAC_INIT = 0,
	MAC_ADHOC,
	MAC_SCAN,
#ifdef HAVE_RANGING
	MAC_RANGING,
#endif
	MAC_INVALID
}mac_mode_t;
	
typedef enum
{
	MAC_BEACON =		0,
	MAC_DATA = 			1,
	MAC_ACK = 			2,
	MAC_CMD = 			3,
	MAC_RANGING =		14,
	MAC_TYPE_NONE =	15
}mac_frame_type_t;



typedef enum
{
	MAC_TX_OK,
	MAC_TX_OK_ACK,
	MAC_TX_CCA,
	MAC_TX_BUSY
}mac_tx_status_t;

#ifdef MAC_ENERGY_SCAN
uint8_t hq_chan_cnt=0;
uint8_t hq_chan_list[16];
uint8_t hq_chan_index=0;
#endif


mac_mode_t mac_mode;
uint8_t mac_sequence;
buffer_t *mac_tx_on_air;
uint8_t mac_tx_retry;
uint8_t mac_tx_cca_retry=0;

/* flags */
#define MAC_CLIENT 0x80
#define MAC_ASSOCIATED 0x40
#define COORD_SHORT 0x20
#define MAC_USE_SHORT_ADDR 0x01

uint8_t mac_flags;

volatile mac_event_t mac_timer_event;
volatile mac_event_t mac_timer_active=0;

xQueueHandle mac_events;

buffer_t *mac_rx[MAC_RING_BUFFER_SIZE_RX];
volatile uint8_t mac_rx_rd, mac_rx_wr;
buffer_t *mac_tx[MAC_RING_BUFFER_SIZE_TX];
uint8_t mac_tx_rd, mac_tx_wr;
#ifdef MALLFORMED_HEADERS
extern uint8_t mallformed_headers_cnt;
#endif

#ifndef AD_HOC_STATE
xList mac_ctl;
#endif

uint8_t mac_pan[2];
uint8_t mac_coord[8];
uint8_t cur_channel = RF_DEFAULT_CHANNEL;

void push_bl_event(buffer_t *b);
void push_ttl_update_event(buffer_t *b);
void mac_rx_add(buffer_t *buf);
portCHAR mac_tx_add(buffer_t *b);
void mac_tx_push(buffer_t *buf);
buffer_t *mac_tx_pull(void);
buffer_t *mac_rx_pull(void);
void mac_data_up(buffer_t *buf);
mac_tx_status_t mac_buffer_out(buffer_t *buf);
void mac_push(buffer_t *b);
portCHAR check_mac_rx_size(void);


mac_event_t mac_control(buffer_t **ppbuf);
void mac_adhoc(mac_event_t event, buffer_t *buf);
void mac_scan(mac_event_t curr_event, buffer_t *buf);

#ifdef CHK_SUM
void mac_calc_chk_sum(buffer_t *buf);
uint8_t mac_check_chk_sum(buffer_t *buf);
#endif


#ifndef AD_HOC_STATE
void mac_client(mac_event_t curr_event, buffer_t *buf);
#ifdef MAC_COORDINATOR
void mac_coord(mac_event_t curr_event, buffer_t *buf);
#endif

#ifdef MAC_SUPERFRAME
void mac_client_sf(mac_event_t curr_event, buffer_t *buf);
#ifdef MAC_COORDINATOR
void mac_coord_sf(mac_event_t curr_event, buffer_t *buf);
#endif
#endif

#endif

#ifdef MAC_RANGING
void mac_ranging(mac_event_t event, buffer_t *buf);
#endif

uint8_t mac_header_generate(buffer_t *buf);
mac_frame_type_t mac_buffer_parse(buffer_t *buf);

uint8_t *mac_address_push(uint8_t *ptr, sockaddr_t *addr);
uint8_t *mac_address_push_pan(uint8_t *ptr, sockaddr_t *addr);

void mac_timer_launch (mac_event_t event);
void mac_timer_stop(void);

void mac_timer_callback(void);

portCHAR mac_set_channel(uint8_t new_channel)
{
	portCHAR retval;
	EA = 0;
	retval = pdFALSE;
	if(rf_channel_set(new_channel))
	{
		cur_channel = new_channel;
		retval = pdTRUE;
	}
	EA = 1;
	return retval;
}

uint8_t mac_current_channel(void)
{
	return cur_channel;
}


buffer_t *mac_rx_pull(void)
{
	buffer_t *b;
	uint8_t tmp = mac_rx_rd;
	
	if (tmp == mac_rx_wr) return 0;
	b = mac_rx[tmp];
	mac_rx[tmp++] = 0;
	if (tmp >= MAC_RING_BUFFER_SIZE_RX) tmp = 0;
	mac_rx_rd = tmp;
	
	return b;
}

buffer_t *mac_tx_pull(void)
{
	buffer_t *b;
	uint8_t tmp = mac_tx_rd;
	
	if (tmp == mac_tx_wr) return 0;
	b = mac_tx[tmp];
	mac_tx[tmp++] = 0;
	if (tmp >= MAC_RING_BUFFER_SIZE_TX) tmp = 0;
	mac_tx_rd = tmp;
	return b;
}

portCHAR mac_tx_add(buffer_t *b)
{
	uint8_t tmp = mac_tx_wr;
	
	tmp++;
	if (tmp >= MAC_RING_BUFFER_SIZE_TX) 
	{
		tmp = 0;
	}
	if(tmp != mac_tx_rd)
	{
		b->to = MODULE_MAC_15_4;
		mac_tx[mac_tx_wr] = b;
		mac_tx_wr = tmp;
	}
	else
	{	/* buffer full, tx overrun error */
		stack_buffer_free(b);
		return pdFALSE;
	}
	return pdTRUE;
}

void mac_rx_add(buffer_t *b)
{
	uint8_t tmp = mac_rx_wr;
	
	tmp++;
	if (tmp >= MAC_RING_BUFFER_SIZE_RX) 
	{
		tmp = 0;
	}
	if(tmp != mac_rx_rd)
	{
		mac_rx[mac_rx_wr] = b;
		mac_rx_wr = tmp;
	}
	else
	{	/* buffer full, error */

	}
}

portCHAR check_mac_rx_size(void)
{
	uint8_t tmp = mac_rx_wr;
	tmp++;

	if (tmp >= MAC_RING_BUFFER_SIZE_RX) 
	{
		tmp = 0;
	}

	if(tmp != mac_rx_rd)
	{
		return pdTRUE;
	}
	else
	{	/* buffer full, error */
		return pdFALSE;
	}
}

/*
	Protocol part for core
	*/
xTaskHandle mac_task_handle;

portCHAR mac_mem_alloc(void)
{
	mac_rx_rd = mac_rx_wr = 0;
	mac_tx_rd = mac_tx_wr = 0;
	mac_pan[0] = 0xFF;
	mac_pan[1] = 0xFF;
	mac_timer_event = MAC_TIMER_NONE;
	mac_events = xQueueCreate( 20, sizeof( mac_event_t ) );
	xTaskCreate(mac_task, "MAC", configMAXIMUM_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3 ), &mac_task_handle);
	//rf_init();
	return pdTRUE;
}
/**
 *  Enable ED-scan.
 * 
 * Working only when MAC_ENERGY_SCAN defined
 */
void mac_start_ed_scan(void)
{
	
	mac_event_t mac_new_event = MAC_ED_SCAN;
	debug("Start ED\r\n");
	xQueueSend(mac_events, &mac_new_event, 0);
}
/**
 *  GW scan with channel change.
 *
 *  Change next channel and create & send ROUTER SOLICATION MESSAGE
 */
void mac_gw_discover(void)
{
	mac_event_t mac_new_event = MAC_GW_DIS;
	debug("GW dis\r\n");
	xQueueSend(mac_events, &mac_new_event, 0);
}


portCHAR mac_init(buffer_t *buf)
{
	buf;
#ifndef AD_HOC_STATE
#ifdef MAC_COORDINATOR
	mac_mode = MAC_INIT;
#else
	mac_mode = MAC_SCAN;	/*clients start in scan*/
#endif
	vListInitialise(&mac_ctl);
#else
	mac_mode = MAC_ADHOC;
#endif	
	return pdTRUE;
}

portCHAR mac_handle( buffer_t *buf )
{
	switch (buf->dir)
	{
			case BUFFER_UP:
				debug("MAC: U\r\n");
				break;
			case BUFFER_DOWN:
				debug("MAC: D\r\n");
				if(mac_mode == MAC_SCAN)
				{
					stack_buffer_free(buf);
				}
				else
				{

#ifdef CHK_SUM
					mac_calc_chk_sum(buf);
#endif
#ifdef HAVE_AES
					aes_crypt(0 , buf);
#endif
					mac_tx_add(buf);
					{
						mac_event_t mac_new_event = MAC_TRANSMIT;
						xQueueSend(mac_events, &mac_new_event, 0);
					}
				}
				return pdTRUE;
				
			default:
				debug("MAC: unknown dir\r\n");
				break; 
	}
	return pdFALSE;
}


portCHAR mac_check( buffer_t *buf )
{
	buf;
	return pdFALSE;
}

#ifdef HAVE_RF_ERROR
extern uint8_t rf_error;
#endif
#define HW_ADDR_DEC
/*
	Task and functionality part
	*/
/**
 * Mac task.
 *
 *
 * \param pvParameters not used
 */

void mac_task( void *pvParameters )
{
	buffer_t *buf=0;
	mac_event_t curr_event;
	portTickType xLastWakeTime=0;
	rf_init();

	pvParameters;
	debug("Mac task start\r\n");
	
	/* Waiting that stack init is ready */
	{
		sockaddr_t temp_mac;
		temp_mac.addr_type = ADDR_NONE;
		while (temp_mac.addr_type == ADDR_NONE)
		{
			mac_get(&temp_mac);
			vTaskDelay(200/portTICK_RATE_MS);
		}
	}
	
	//vTaskDelay(2000/portTICK_RATE_MS);
	mac_sequence = mac_long.address[0];
	debug("Mac task go\r\n");
	/*{
		uint8_t i;
		uint8_t tmp_8;	
		tmp_8= (mac_sequence % 15) + 8;
		for(i=0; i< tmp_8; i++)
		{
			mac_sequence += random_generate(16);
		}
	}*/
	/*debug("Set address.\r\n");
	mac_long.address[8] = 0xff;
	mac_long.address[9] = 0xff;

	debug_address(&(mac_long));
	debug("Set address.\r\n");
	rf_set_address(&mac_long);*/
#ifdef HAVE_NRP
	debug("Softack on.\r\n");
	//rf_address_decoder_mode(RF_SOFTACK_MONITOR);
	rf_address_decoder_mode(RF_DECODER_ON);
#else
#ifdef MAC_SOFTACK
	debug("Decoder off.\r\n");
	rf_address_decoder_mode(RF_SOFTACK_CLIENT);
#else
	debug("Decoder on.\r\n");
	rf_address_decoder_mode(RF_DECODER_ON);
#endif
#endif
	
	mac_long.address[8] = 0xff;
	mac_long.address[9] = 0xff;

	rf_rx_enable();
#ifdef MAC_ENERGY_SCAN
	mac_start_ed_scan();
#endif

	for (;;)
	{
		if(xQueueReceive(mac_events, &(curr_event),(portTickType) 500 / portTICK_RATE_MS) == pdTRUE)
		{
#ifdef HAVE_RF_ERROR
			if (rf_error)
			{
				debug("RFE");
				debug_hex(rf_error);
				rf_error = 0;
			}
#endif		

			switch(curr_event)
			{
#ifndef AD_HOC_STATE
				case MAC_CONTROL:
					curr_event = mac_control(&buf);
					break;
#endif					
				case MAC_TRANSMIT:
					debug("TX\r\n");
					if (mac_timer_event != MAC_TIMER_NONE)
					{
						curr_event = MAC_NONE;
					}
					else
					{
						buf = mac_tx_pull();
						if(buf)
						{
							mac_tx_on_air=buf;
							buf=0;
							curr_event = MAC_NONE;
							mac_timer_launch(MAC_TIMER_CCA);
						}
						else
						{
							curr_event = MAC_NONE;
						}
					}
					break;
					
				case MAC_RECEIVE:
					debug("RX\r\n");
					buf = 0;
					portDISABLE_INTERRUPTS();
					buf = mac_rx_pull();
					portENABLE_INTERRUPTS();
					if(buf == 0)
					{
						curr_event = MAC_LOOP;
					}
					break;

				case MAC_ED_SCAN:
					buf = 0;
					curr_event = MAC_ED_SCAN;
					mac_mode = MAC_SCAN;
					break;

				case MAC_GW_DIS:
					curr_event == MAC_NONE;
#ifndef MAC_ENERGY_SCAN
					if(cur_channel==26)
						cur_channel=11;
					else
						cur_channel++;
					rf_channel_set(cur_channel);
					gw_discover();
#endif
					break;
					
				default:
					break;
			}	/*end event switch*/

			switch(mac_mode)
			{
				case MAC_ADHOC:
					mac_adhoc(curr_event, buf);
					break;
				case MAC_SCAN:
					mac_scan(curr_event, buf);
					break;
#ifndef AD_HOC_STATE					
				case MAC_CLIENT:
					mac_client(curr_event, buf);
					break;
#ifdef MAC_COORDINATOR
				case MAC_COORD:
					mac_coord(curr_event, buf);
					break;
#endif	/*COORDINATOR*/
					
#ifdef MAC_SUPERFRAME
				case MAC_CLIENT_SF:
					mac_client_sf(curr_event, buf);
					break;
#ifdef MAC_COORDINATOR
				case MAC_COORD_SF:
					mac_coord_sf(curr_event, buf);
					break;
#endif	/*COORDINATOR*/
#endif	/*SUPERFRAME*/
#endif	/*not AD_HOC_STATE*/
					
#ifdef HAVE_RANGING
				case MAC_RANGING:
					mac_ranging(curr_event, buf);
					curr_event = MAC_NONE;
					break;
#endif
				default:
					stack_buffer_free(buf);
					curr_event = MAC_NONE;
			}/* end mode switch */
			buf = 0;
			curr_event = MAC_NONE;
			if ((mac_timer_event == MAC_TIMER_NONE) && (mac_tx_on_air == 0))
			{
				buf = mac_tx_pull();
				if(buf)
				{
					mac_tx_on_air=buf;
					mac_tx_cca_retry=0;
					buf=0;
					mac_timer_launch(MAC_TIMER_CCA);
				}
			}
		}/* end event received */
	} /*end task loop*/
}

/**
 *  MAC ad-hoc mode packet handle.
 *
 */
void mac_adhoc(mac_event_t curr_event, buffer_t *buf)
{
	mac_frame_type_t ftype;
/*	debug("ADHOC:");
	debug_int(curr_event);*/
	switch (curr_event)
	{

			case MAC_ACK_RX:
				mac_timer_stop();
				mac_tx_retry = 0;
				mac_timer_event = MAC_TIMER_NONE;
				push_ttl_update_event(mac_tx_on_air);
				mac_tx_on_air = 0;
				buf=0;
				break;

			case MAC_RECEIVE:
				ftype = mac_buffer_parse(buf); 
				switch(ftype)
				{
					case MAC_DATA:
#ifdef CHK_SUM
						if(mac_check_chk_sum(buf) == 1)
						{
							mac_data_up(buf);
							debug("d\r\n");
						}
						else
						{
#ifdef MALLFORMED_HEADERS
							mallformed_headers_cnt++;
#endif
							debug("chk err\r\n");
							stack_buffer_free(buf);
						}

#else
						mac_data_up(buf);
						debug("d\r\n");
#endif
						break;
					default:
						/*debug_hex(ftype);*/
						debug("d!\r\n");
						stack_buffer_free(buf);
				}
				buf = 0;
				break;
				
			case MAC_TIMER_ACK:	
				mac_tx_retry++;
				if (mac_tx_retry > MAC_RETRY_MAX)
				{
					mac_timer_stop();
					mac_tx_retry = 0;
					buf = mac_tx_on_air;
					mac_tx_on_air = 0;
					buf->buf_ptr = (buf->buf_end - buf->options.lowpan_compressed );
					#ifdef HAVE_AES
						aes_crypt(1 , buf);
					#endif
					push_bl_event(buf);
					buf = 0;
					//debug("TX fail.\r\n");
					return;			
				}				
				debug("RTR");
				mac_timer_launch(MAC_TIMER_CCA);
				buf=0;
				break;
				
			case MAC_TIMER_CCA:	/*retransmit*/
				buf = mac_tx_on_air;
				mac_tx_on_air = 0;
			case MAC_TRANSMIT:
				if (buf != 0)
				{
					switch (mac_buffer_out(buf))
					{
						case MAC_TX_OK:
							mac_tx_on_air = 0;
							//mac_timer_stop();
							mac_timer_event = MAC_TIMER_NONE;
							
							mac_tx_retry = 0;
							stack_buffer_free(buf);
							break;
						case MAC_TX_OK_ACK:
							debug("WACK\r\n");
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_ACK);
							break;
						case MAC_TX_BUSY:
							debug("MAC TX busy.\r\n");
							mac_tx_cca_retry++;
							if(mac_tx_cca_retry > MAC_CCA_RETRY_MAX)
							{
								mac_tx_cca_retry=0;
								mac_tx_retry = 0;
								mac_tx_on_air = 0;
								mac_timer_event = MAC_TIMER_NONE;
#ifdef MAC_ENERGY_SCAN
								debug("chan change\r\n");
								stack_buffer_free(buf);
								mac_start_ed_scan();
								break;
#else
								debug("CCA er\r\n");

								//stack_buffer_free(buf);
								//break;

								buf->buf_ptr = (buf->buf_end - buf->options.lowpan_compressed );
								#ifdef HAVE_AES
								aes_crypt(1 , buf);
								#endif
								push_bl_event(buf);
								buf = 0;
								return;	
#endif
							}
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_CCA);
							break;
						case MAC_TX_CCA:

							mac_tx_cca_retry++;
							if(mac_tx_cca_retry > MAC_CCA_RETRY_MAX)
							{
								mac_tx_cca_retry=0;
								mac_tx_on_air = 0;
								mac_timer_event = MAC_TIMER_NONE;
								mac_tx_retry = 0;
#ifdef MAC_ENERGY_SCAN
								debug("chan change\r\n");
								
								stack_buffer_free(buf);
								mac_start_ed_scan();
								break;
#else
								debug("CCA er\r\n");
								
								//stack_buffer_free(buf);
								//break;
//#if 0
								/*control_message_t *ptr;
								ptr = ( control_message_t*) buf->buf;
								ptr->message.ip_control.message_id = MAC_CCA_ERR;
								push_to_app(buf);
								buf=0;*/


								buf->buf_ptr = (buf->buf_end - buf->options.lowpan_compressed );
								#ifdef HAVE_AES
								aes_crypt(1 , buf);
								#endif
								push_bl_event(buf);
								buf = 0;
								return;	
//#endif
#endif
							}


						default:
							//debug("MAC TX CCA.\r\n");
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_CCA);
					}
				}
				buf = 0;
				break;
			default:
				break;
	}
	if (buf)
	{
		stack_buffer_free(buf); 
	}
}

/**
 *  MAC scan mode function.
 *
 *  When MAC_ENERGY_SCAN is defined (GW device) this function analyze all 16 channel Energy level and select first free channel.
 */
void mac_scan(mac_event_t curr_event, buffer_t *buf)
{
	#ifdef MAC_ENERGY_SCAN
	uint8_t j=0;
	uint16_t i;
	uint8_t chan = 11;
	int8_t max_rssi = -128, rssi=-128;
	
	if(hq_chan_cnt == hq_chan_index)
	{
		debug(" Full Scan\r\n");
		hq_chan_cnt = hq_chan_index = 0;
		for(j=0; j<16; j++)
		{
			rf_channel_set(chan);
			pause_us(500);
			
			i=0;
			max_rssi = -128;
			while(i++ < 500)
			{
				rssi = rf_analyze_rssi();
				if(rssi < 0)
				{
					if(rssi > max_rssi)
					{
						max_rssi = rssi;
					}
					
				}
				
				pause_us(500);
			}
			if(max_rssi < -92)
			{
				hq_chan_list[hq_chan_cnt] = chan;
				hq_chan_cnt++;
			}
#ifdef HAVE_DEBUG
			debug_int(max_rssi);
			debug(" dbm\r\n");
#endif
			vTaskDelay(4);
			chan++;
		}
		if(hq_chan_cnt)
		{
#ifdef HAVE_DEBUG
			debug_int(hq_chan_cnt);
			debug(" chan cnt\r\n");
			debug_int(hq_chan_list[hq_chan_index]);
			debug(" start channel\r\n");
#endif
			hq_chan_index=1;
			rf_channel_set(hq_chan_list[0]);
			cur_channel = hq_chan_list[0];
			pause_us(500);
			
#ifdef HAVE_NRP
			gw_advertisment();
			nrp_channel_notify();
#endif
		}
	}
	else
	{
#ifdef HAVE_DEBUG
			debug_int(hq_chan_list[hq_chan_index]);
			debug(" new channel\r\n");
#endif
			rf_channel_set(hq_chan_list[hq_chan_index]);
			cur_channel = hq_chan_list[hq_chan_index];
			hq_chan_index++;
			pause_us(500);
			
#ifdef HAVE_NRP
			gw_advertisment();
			nrp_channel_notify();
#endif
	}
#endif
	#ifndef AD_HOC_STATE
#ifdef MAC_COORDINATOR
	mac_mode = MAC_INIT;
#else
	mac_mode = MAC_SCAN;	/*clients start in scan*/
#endif
#else
	mac_mode = MAC_ADHOC;
#endif
	curr_event = MAC_NONE;
	buf; 
}
#ifndef AD_HOC_STATE
mac_mode_t mac_command(buffer_t *buf, uint8_t coordinator)
{
	return mac_mode;
}




void mac_client(mac_event_t curr_event, buffer_t *buf)
{
	mac_frame_type_t ftype;
	mac_mode_t new_mode = mac_mode;
	
	switch (curr_event)
	{
			case MAC_RECEIVE:
				ftype = mac_buffer_parse(buf); 
				switch(ftype)
				{
					case MAC_DATA:
						if (buf->src_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
						{	/*if address matches coordinator*/							
							mac_data_up(buf);
						}
						break;
					case MAC_CMD:
						new_mode = mac_command(buf, 0);
						break;
					case MAC_ACK:
						mac_timer_stop();
						stack_buffer_free(mac_tx_on_air);
						stack_buffer_free(buf);
						mac_tx_retry = 0;
						mac_tx_on_air = 0;
						break;
					default:
/*						debug_hex(ftype);*/
						stack_buffer_free(buf);
				}
				buf = 0;
				break;
				
			case MAC_TIMER_ACK:	
				mac_tx_retry++;
				if (mac_tx_retry > MAC_RETRY_MAX)
				{
					buf = mac_tx_on_air;
					mac_tx_on_air = 0;
					stack_buffer_free(buf);
					buf = 0;
					mac_tx_retry = 0;
					mac_timer_stop();
					//debug("TX fail.\r\n");
					return;			
				}				
				//debug("RTR");
				
			case MAC_TIMER_CCA:	/*retransmit*/
				buf = mac_tx_on_air;
				mac_tx_on_air = 0;
			case MAC_TRANSMIT:
				if (buf != 0)
				{
					switch (mac_buffer_out(buf))
					{
						case MAC_TX_OK:
							mac_tx_on_air = 0;
							mac_timer_stop();
							stack_buffer_free(buf);
							mac_tx_retry = 0;
							break;
						case MAC_TX_OK_ACK:
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_ACK);
							break;
						case MAC_TX_BUSY:
							//debug("MAC TX busy.\r\n");
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_CCA);
							break;
						case MAC_TX_CCA:
						default:
							//debug("MAC TX CCA.\r\n");
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_CCA);
					}
				}
				buf = 0;
				break;
			default:
				break;
	}
	if (buf) stack_buffer_free(buf);
}

#ifdef MAC_COORDINATOR
void mac_coord(mac_event_t curr_event, buffer_t *buf)
{
	switch (curr_event)
	{
			case MAC_RECEIVE:
				switch(mac_buffer_parse(buf))
				{
					case MAC_DATA:
						mac_data_up(buf);
						break;
					default:
						stack_buffer_free(buf);
				}
				buf = 0;
				break;
			case MAC_TRANSMIT:
				switch (mac_buffer_out(buf))
				{
					case OK:
						stack_buffer_free(buf);
						break;
					case CCA_RESERVED:
					default:
						mac_tx_add(buf);
						mac_timer_launch(MAC_TIMER_CCA);
				}
				buf = 0;
				break;
	}
	if (buf) stack_buffer_free(buf);
}
#endif

#endif /*AD_HOC_STATE*/

/* 
	General header functions 
	*/
mac_frame_type_t mac_buffer_parse(buffer_t *buf)
{
	uint8_t *ptr;
	uint8_t seq;
	uint16_t fc,tmp;
	mac_frame_type_t type = MAC_TYPE_NONE;

	if (buf == 0) return type;
		
	ptr = buf->buf;
	ptr += buf->buf_ptr;
	
	fc = *ptr++;
	fc += ((uint16_t) (*ptr++) << 8);
	
	seq = *ptr++;
	
	type = (mac_frame_type_t) ((fc & FC_FRAME_TYPE_MASK) >> FC_FRAME_TYPE_SHIFT); /* detect frametype */
	
	tmp = (fc & FC_DST_MODE);
	if(tmp == FC_DST_ADDR_NONE)
	{
		buf->dst_sa.addr_type = ADDR_NONE;
	}
	else
	{	/*read PAN*/
		buf->dst_sa.address[8] = *ptr++;
		buf->dst_sa.address[9] = *ptr++;
	}
	if(tmp == FC_DST_64_BITS)
	{
		/* Read dst pan */
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
		memcpy(buf->dst_sa.address,ptr, 8);
		ptr += 8;
		/*for (i = 0; i < 8 ; i++)
		{
			buf->dst_sa.address[i] = *ptr++;
		}*/
	}
	if(tmp == FC_DST_16_BITS)
	{
		/* Read dst pan */
		buf->dst_sa.address[2] = buf->dst_sa.address[8];
		buf->dst_sa.address[3] = buf->dst_sa.address[9];
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
		buf->dst_sa.address[0] = *ptr++;
		buf->dst_sa.address[1] = *ptr++;
	}
	buf->dst_sa.port = 0;
	/* Source address */
	tmp=(fc & FC_SRC_MODE);

	if(tmp == FC_SRC_ADDR_NONE)
	{
		buf->src_sa.addr_type = ADDR_NONE;
	}
	else
	{
		if (fc &  FC_INTRA_PAN)
		{
			buf->src_sa.address[8] = buf->dst_sa.address[8];
			buf->src_sa.address[9] = buf->dst_sa.address[9];
		}
		else
		{
			buf->src_sa.address[8] = *ptr++;
			buf->src_sa.address[9] = *ptr++;
		}
	}
	if(tmp == FC_SRC_64_BITS)
	{
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;
		memcpy(buf->src_sa.address,ptr, 8);
		ptr += 8;
		/*for (i = 0; i < 8 ; i++)
		{
			buf->src_sa.address[i] = *ptr++;
		}*/
	}
	if(tmp== FC_SRC_16_BITS)
	{
		buf->src_sa.address[2] = buf->src_sa.address[8];
		buf->src_sa.address[3] = buf->src_sa.address[9];
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
		buf->src_sa.address[0] = *ptr++;
		buf->src_sa.address[1] = *ptr++;
	}
	/*if(buf->src_sa.addr_type != ADDR_NONE)
	{
		buf->link_sa.addr_type = buf->src_sa.addr_type;
		memcpy(buf->link_sa.address, buf->src_sa.address,10);
	}*/

	buf->buf_ptr = (uint16_t) ptr;
	buf->buf_ptr -= (uint16_t) buf->buf;
	buf->options.lowpan_compressed = seq;


	if(fc & FC_SEC && type == MAC_DATA)
	{
#ifdef HAVE_AES
		aes_crypt(1 , buf);
#else
		type = MAC_TYPE_NONE;
#endif
	}
	return type;
}

/**
 * Function creates MAC-frame.
 *
 * \param buf indicates pointer for buffer structure.
 *  \return  ack flag which indicated ack-requirement for frame.
 */
uint8_t mac_header_generate(buffer_t *buf)
{
	uint8_t header_size = 3; /* including Frame Control and sqn */
	uint8_t address_length;
	uint8_t *ptr;
	uint16_t fc;
	
	fc = FC_DATA_FRAME;

#ifdef HAVE_AES
	fc |= FC_SEC;
#endif


	if (stack_check_broadcast(buf->dst_sa.address, buf->dst_sa.addr_type) == pdTRUE)
	{
		buf->dst_sa.addr_type = ADDR_BROADCAST;
	}
	if(buf->dst_sa.addr_type == ADDR_BROADCAST)
	{
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
		memset(buf->dst_sa.address, 0xff, 4);
	}
	else
	{
		fc |= FC_ACK;
	}
	/* Check Intra flag use */
	if((buf->dst_sa.addr_type == ADDR_COORDINATOR || mac_mode == MAC_CLIENT) 
			&& (mac_flags & MAC_ASSOCIATED))
	{
		buf->dst_sa.address[8] = mac_pan[0];
		buf->dst_sa.address[9] = mac_pan[1];
		
		if((mac_flags & COORD_SHORT))
		{
			buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
			buf->dst_sa.address[2] = buf->dst_sa.address[8];
			buf->dst_sa.address[3] = buf->dst_sa.address[9];
			buf->dst_sa.address[0] = mac_coord[0];
			buf->dst_sa.address[1] = mac_coord[1];
			
			address_length=2;
			fc |= FC_DST_16_BITS;
			header_size += 2;
		}
		else 
		{
			buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
			address_length=8;
			fc |= FC_DST_64_BITS;
			header_size += 8;
			memcpy(buf->dst_sa.address,mac_coord, 8);
		}
		fc |= FC_INTRA_PAN;
	}
	else
	{
		if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_SHORT)
		{
			fc |= FC_DST_16_BITS;
			header_size += 4;
			buf->dst_sa.address[8] = buf->dst_sa.address[2];
			buf->dst_sa.address[9] = buf->dst_sa.address[3];
		}
		if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
		{
			fc |= FC_DST_64_BITS;
			header_size += 10;
		}
		if(memcmp(&buf->dst_sa.address[8], mac_pan, 2 ) == 0 ) /* Compare PAN-IDs --> if match use intra-flag */
		{
			fc |= FC_INTRA_PAN;
			header_size -=2;
		}
	}	

	/* Checkout Source addresstype */
	if(((mac_flags & MAC_ASSOCIATED) || mac_mode==MAC_ADHOC) && (mac_flags & MAC_USE_SHORT_ADDR))
	{
		fc |= FC_SRC_16_BITS;
		header_size +=4;
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
	}
	else
	{
		fc |= FC_SRC_64_BITS;
		header_size +=10;
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;
		buf->src_sa.address[8] = mac_pan[0];
		buf->src_sa.address[9] = mac_pan[1];
		memcpy(buf->src_sa.address, mac_long.address, 8);
	}
	
	/* Pack up */
	if(stack_buffer_headroom( buf,(uint16_t)header_size)==pdFALSE)
	{
		stack_buffer_free(buf);
		return pdTRUE;
	}
	
	buf->buf_ptr -= header_size;
	ptr = buf->buf;
	ptr += buf->buf_ptr;
	
	*ptr++ = fc; 	/* FCF*/
	*ptr++ = fc>>8;
	*ptr++ = mac_sequence;		/*packet sequence number*/
	if(mac_sequence==0xff)
		mac_sequence=0;
	else
		mac_sequence++;
	
	/* Craete address field */
	/* Destination address */
	ptr = mac_address_push_pan(ptr, &(buf->dst_sa));
	ptr = mac_address_push(ptr, &(buf->dst_sa));
	if ((fc & FC_INTRA_PAN) == 0)
	{
		ptr = mac_address_push_pan(ptr, &(buf->src_sa));
	}
	ptr = mac_address_push(ptr, &(buf->src_sa));

	if (fc & FC_ACK) return 1;
	else return 0;
}

uint8_t *mac_address_push_pan(uint8_t *ptr, sockaddr_t *addr)
{
	uint8_t *ptr2 = addr->address;
	
	switch (addr->addr_type)
	{
		case ADDR_802_15_4_PAN_LONG:
				ptr2 += 6;
		case ADDR_802_15_4_PAN_SHORT:
				ptr2 += 2;
				*ptr++ = *ptr2++;
				*ptr++ = *ptr2++;
				break;
		default:
				*ptr++ = 0xff;
				*ptr++ = 0xff;
	}
	return ptr;
}

uint8_t *mac_address_push(uint8_t *ptr, sockaddr_t *addr)
{
	switch (addr->addr_type)
	{
		case ADDR_802_15_4_PAN_SHORT:
				*ptr++ = addr->address[0];
				*ptr++ = addr->address[1];
				break;
		case ADDR_802_15_4_PAN_LONG:
				memcpy(ptr,addr->address, 8);
				ptr +=8;
				break;
				
#ifndef AD_HOC_STATE
#endif
				
		default:
				*ptr++ = 0xff;
				*ptr++ = 0xff;
	}
	return ptr;
}

void mac_rx_push(void);
void mac_rx_push(void)
{
	mac_event_t mac_new_event = MAC_RECEIVE;
	xQueueSendFromISR(mac_events, &mac_new_event, pdFALSE);		
}

void mac_event_push(void);
void mac_event_push(void)
{
	mac_event_t mac_new_event = MAC_ACK_RX;
	xQueueSendFromISR(mac_events, &mac_new_event, pdFALSE);
}
void mac_push(buffer_t *b)
{
	b->to = MODULE_MAC_15_4;
	b->dir = BUFFER_UP;
	b->from = MODULE_NONE;
	mac_rx_add(b);
}
/**
 * Push Neighbour & Routing table TTL update event to cIPV module.
 *
 * \param buf indicates pointer for buffer structure.
 */
void push_ttl_update_event(buffer_t *b)
{
	b->to = MODULE_CIPV6;
	b->dir = BUFFER_UP;
	b->from = MODULE_MAC_15_4;
	b->options.handle_type = HANDLE_TTL_UPDATE;
	stack_buffer_push(b);
}
/**
 * Push Broken Link event to cIPV module.
 *
 * When ACK not received after 4 re-tx.
 * 
 * \param buf indicates pointer for buffer structure.
 */
void push_bl_event(buffer_t *b)
{
	b->to = MODULE_CIPV6;
	b->dir = BUFFER_UP;
	b->from = MODULE_MAC_15_4;
	b->options.handle_type = HANDLE_BROKEN_LINK;
	stack_buffer_push(b);
}
/**
 * RF RX interrupt ACK handle.
 *
 * Function check ACK status in the MAC.
 * 
 * \param sqn ack sequency number
 * 
 */
void ack_handle(uint8_t sqn)
{
	if(mac_timer_event == MAC_TIMER_ACK && mac_timer_active) //T4CTL
	{
		if (mac_tx_on_air)
		{
			if(mac_sequence == 0x00 && sqn == 0xff)
			{
				mac_timer_event = MAC_ACK_RX;
			}
			else if(mac_sequence != 0x00 && (sqn == (mac_sequence - 1)))
			{
				mac_timer_event  = MAC_ACK_RX;
			}
			
			if(mac_timer_event  == MAC_ACK_RX)
			{
				portBASE_TYPE prev_task = pdFALSE;
				mac_timer_active=0;
				prev_task = xQueueSendFromISR(mac_events, &mac_timer_event, prev_task);
				mac_timer_stop();
			}
		}
	}
}
/**
 * Push Data buffer to upper layer from MAC.
 *
 * \param buf indicates pointer for buffer structure.
 */
void mac_data_up(buffer_t *b)
{
	b->to = MODULE_CIPV6;
	b->dir = BUFFER_UP;
	b->from = MODULE_MAC_15_4;
	b->options.handle_type = HANDLE_DEFAULT;
	stack_buffer_push(b);
}
/**
 * Buffer send API for MAC module.
 *
 * \param buf indicates pointer for buffer structure.
 * 
 * \return MAC_TX_OK_ACK, TX ok waiting ACK
 * \return MAC_TX_OK TX complete
 * \return MAC_TX_BUSY, channel busy
 * \return MAC_TX_CCA, channel not free
 */
mac_tx_status_t mac_buffer_out(buffer_t *buf)
{
	uint8_t ack = 0;
	
	if (mac_tx_on_air) return MAC_TX_BUSY;
	
	if (buf->to != MODULE_NONE)
	{
		buf->options.lowpan_compressed = (uint8_t) buffer_data_length(buf);
		ack = mac_header_generate(buf);
		buf->to = MODULE_NONE;
	}
	else
	{
		uint16_t ind = buf->buf_ptr;
		uint16_t fc = buf->buf[ind+1];
		fc <<= 8;
		fc += buf->buf[ind];
		if (fc & FC_ACK) ack = 1;		
	}
	switch (rf_write(buf))
	{
		case pdTRUE:
			mac_tx_on_air = buf;
			mac_tx_cca_retry=0;
			if (ack) return MAC_TX_OK_ACK;
			else return MAC_TX_OK;
		case pdTRUE+1:
			return MAC_TX_CCA;
		default:
			return MAC_TX_BUSY;
	}
}

/*
 Timer functions
 */
void mac_timer_launch (mac_event_t event)
{
	timer_mac_stop();
	mac_timer_event = event;
	
	switch(event)
	{
		case MAC_TIMER_CCA:
			timer_mac_launch(MAC_CCA_TIME);
			break;
			
		case MAC_TIMER_ACK:
			timer_mac_launch(MAC_ACK_TIME);

		default:
			break;	
	}
	mac_timer_active=1;
}

#ifdef CHK_SUM
void mac_calc_chk_sum(buffer_t *buf)
{
	uint16_t chk_sum=0;
	uint8_t i,*ptr;

	ptr = buf->buf + buf->buf_ptr;
	for(i=buf->buf_ptr; i< buf->buf_end; i++)
	{
		chk_sum += *ptr++;
	}
	*ptr++ = (chk_sum >> 8);		
	*ptr++ = (uint8_t) chk_sum;
	buf->buf_end += 2;
}

uint8_t mac_check_chk_sum(buffer_t *buf)
{
	uint16_t chk_sum=0, result=0;
	uint8_t i,*ptr;

	ptr = buf->buf + (buf->buf_end - 2);
	chk_sum = *ptr++;
	chk_sum <<= 8;
	chk_sum += *ptr++;

	buf->buf_end -= 2;
	ptr = buf->buf + buf->buf_ptr;
	for(i=buf->buf_ptr; i< buf->buf_end; i++)
	{
		result += *ptr++;
	}
	if(chk_sum == result)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
#endif

void mac_timer_stop()
{
	mac_timer_active=0;
	timer_mac_stop();
	mac_timer_event = MAC_TIMER_NONE;
}
/**
 * MAC timer Callback.
 *
 *  CB send MAC timer event to MAC
 */
void mac_timer_callback(void)
{
	if(mac_timer_active)
	{
		portBASE_TYPE prev_task = pdFALSE;
		mac_timer_active=0;
		prev_task = xQueueSendFromISR(mac_events, &mac_timer_event, prev_task);
	}
}

/**
 * Function setup IP modules address mode and also forward MAC and short address information.
 * 
 *
 *  \param  support_short_addr 1=Support & 0=Not support.
 */
void rf_802_15_4_ip_layer_address_mode_set(uint8_t support_short_addr)
{
	if(support_short_addr)
	{
		ip_address_setup( 1, mac_short.address); 
	}
	else
		ip_address_setup( 0,NULL); 
}
