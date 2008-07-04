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
 * \file main.c
 * \brief Example bare skeleton for starting a new Nano series application.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>
#include <sys/inttypes.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* NanoStack includes */
#include "stack.h"
#include "socket.h"
#include "debug.h"
#include "control_message.h"

/* Platform includes */
#include "uart.h"
#include "rf.h"
#include "bus.h"
#include "dma.h"
#include "timer.h"
#include "gpio.h"

static void vAppTask( int8_t *pvParameters );

/* Setup a default address structure to port 61625 */
sockaddr_t datasensor_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0xFF, 0xFF, 0xFF, 0xFF, 
	  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
	61625
};

socket_t *test_socket=0;
stack_event_t stack_event;
uint8_t gw_assoc_state=0;
uint8_t scan_active=0;
portTickType	scan_start;
portTickType data_send_period=0;

/* Main task, initialize hardware and start the FreeRTOS scheduler */
int main( void )
{
	/* Initialize the Nano hardware */
	LED_INIT();
	bus_init();
	debug_init(115200);
	stack_init();
	/* Setup the application task and start the scheduler */
	xTaskCreate( vAppTask, "App", configMAXIMUM_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2 ), ( xTaskHandle * )NULL );
	vTaskStartScheduler();
	
	/* Scheduler has taken control, next vAppTask starts executing. */

	return 0;
}

/** 
 * Skeleton application task 
 */
static void vAppTask( int8_t *pvParameters )
{
	uint8_t *dptr;
	buffer_t *buf;
	/* Start the debug UART at 115k */
	
	LED1_ON();
	vTaskDelay( 500 / portTICK_RATE_MS );
	LED1_OFF();
	stack_event = stack_service_init(NULL);	 /* Open socket for stack status message */

	/* Open and bind a socket */
	
	test_socket = socket(MODULE_CUDP, 0);
	if (test_socket) {
		socket_bind(test_socket, &datasensor_address);
	}
	/* Change port for Nodeview Demo port */	
	datasensor_address.port = 61630;
	
	/* Start an endless task loop, we must sleep most of the time allowing execution of other tasks. */
	for (;;)
	{
		if(gw_assoc_state == 0 && scan_active == 0)
		{
			/* Start GW discover */
			LED1_OFF();
			scan_active=1;
			scan_start = xTaskGetTickCount(); /* Save scan start time */
			gw_discover();
		}
		
		/* Listen Socket */
 		buf = socket_read(test_socket, 200);
		if (buf)
		{
			debug("RX test packet\r\n");
			socket_buffer_free(buf);
			buf = 0;
		}
		/* Send a packet to gateway */
		if(gw_assoc_state)
		{
			if ((xTaskGetTickCount() - data_send_period)*portTICK_RATE_MS > 10000)
			{
				/* Get a buffer for data */
				buf = socket_buffer_get(test_socket);
				if (buf) {
					buf->buf_end=0;
					buf->buf_ptr=0;
					/*buf->buf[buf->buf_end++] = 'T';
					buf->buf[buf->buf_end++] = 'e';
					buf->buf[buf->buf_end++] = 's';
					buf->buf[buf->buf_end++] = 't';
					buf->buf[buf->buf_end++] = '!';*/
					//buf->buf_ptr=0;
					dptr = buf->buf + buf->buf_ptr;
	
					*dptr++ = 0x01;  /* Versio */
					*dptr++ = 0x01;  /* 1 Header */
					*dptr++ = 0x01;  /* Type header */
					*dptr++ = 0x02;   
					*dptr++ = 0x01;		/* Num of Messages */
					*dptr++ = 0x02;		/* Type SensorSend*/
					*dptr++ = 0x01;		/* 1 Tag (ResultElement) */
					*dptr++ = 0x0f;		/* Message Length */
					*dptr++ = 0xf5;		/* ArrayofMeasurementElement */
					*dptr++ = 0x00;		/* Array */
					*dptr++ = 0x01;	/* Tags in array */
			
			
					/* Dummy sensor */
					*dptr++ = 0xf3;		/* MeasurementElement */
					*dptr++ = 0x08;		/* Item */
					*dptr++ = 0x01;		/* First Item in Array */
					*dptr++ = 0xf9;		/* TimeStamp */
					*dptr++ = 0x01;		/* Byte */
					*dptr++ = 0;		/* 0 ms */
					*dptr++ = 0xf6;		/* SensorType */
					*dptr++ = 0x01;		/* Byte */
					*dptr++ = 0x00;		/* No Unit*/
					*dptr++ = 0xfa;		/* MeasurementValue */
					*dptr++ = 0x05;		/* string variable */
					*dptr++ = 0x04;		/* length 4 chars */
					*dptr++ = 'T';
					*dptr++ = 'e';
					*dptr++ = 's';
					*dptr++ = 't';
					buf->buf_end = (dptr - buf->buf);
					
				}
				if (socket_sendto(test_socket, &datasensor_address, buf) == pdTRUE) {
					debug("Packet sent to GW\r\n");
					data_send_period = xTaskGetTickCount();
				}
				else
				{
					socket_buffer_free(buf);
					debug("Socket send fail\r\n");
				}
			}
		}
		else
		{
			debug("GW discover...\r\n");	
		}
		
		buf = waiting_stack_event(200);
		if(buf)
		{
			switch (parse_event_message(buf))
			{
				case BROKEN_LINK:
						buf->dst_sa.port = datasensor_address.port;
						if(memcmp(&datasensor_address.address, &(buf->dst_sa.address), 8) == 0)
						{
							/* Connection to GW lost--> enable scan again */
							gw_assoc_state = 0;
							datasensor_address.addr_type = ADDR_NONE;
							LED1_OFF();
						}
					break;

				case ROUTER_DISCOVER_RESPONSE:
					scan_active=0;
					if(gw_assoc_state == 0)
					{
						/* Stop GW scan and save GW address */
						memcpy(datasensor_address.address, buf->src_sa.address, 8);
						datasensor_address.addr_type = buf->src_sa.addr_type ;
						gw_assoc_state=1;
						data_send_period = xTaskGetTickCount();
						LED1_ON();
					}
					break;
				
				default:

					break;
			}
			socket_buffer_free(buf);
			buf = 0;
		}
		if (scan_active == 1 && (xTaskGetTickCount() - scan_start)*portTICK_RATE_MS > 1000)
		{
			/* GW discover timeout witthout answers */
			mac_gw_discover(); /* Change next channel and create GW discover message and send */
			scan_start = xTaskGetTickCount();
		}
		/* Sleep for 500 ms */
		vTaskDelay( 500 / portTICK_RATE_MS );
		LED2_TOGGLE();
	}
}


