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

/* Platform includes */
#include "uart.h"
#include "rf.h"
#include "bus.h"
#include "dma.h"
#include "timer.h"
#include "gpio.h"

static void vAppTask( int8_t *pvParameters );
static void ListenTask( int8_t *pvParameters );
portCHAR my_handler_function_name(buffer_t *buffer);
void blink1(void);
void blink2(void);

// Setup a default address structure, short address, broadcast, to port 125
sockaddr_t broadcast = 
{
	ADDR_802_15_4_LONG,
	{ 0x00, 0x15, 0x20, 0x00, 0x00, 0x02, 0x0c, 0x8d },
	253
};
// sockaddr_t broadcast = 
// {
// 	ADDR_802_15_4_SHORT,
// 	{ 0xff, 0xff, 0xff, 0xff },
// 	253
// };

socket_t *broadcast_socket=0;

socket_t *listenSocket = 0;
sockaddr_t listen = 
{
	ADDR_802_15_4_PAN_SHORT, 
	{ 0xff, 0xff, 0xff, 0xff },
	253
};

/* Main task, initialize hardware and start the FreeRTOS scheduler */
int main( void )
{
	/* Initialize the Nano hardware */
	LED_INIT();
	// button init
	P0DIR &= ~0xc0;
	bus_init();

	/* Setup the application task and start the scheduler */
	xTaskCreate( vAppTask, "App", configMINIMAL_STACK_SIZE+200, NULL, (tskIDLE_PRIORITY + 2 ), ( xTaskHandle * )NULL );
	xTaskCreate( ListenTask, "Listen", configMINIMAL_STACK_SIZE+200, NULL, (tskIDLE_PRIORITY + 1 ), ( xTaskHandle * )NULL );
	vTaskStartScheduler();
	
	/* Scheduler has taken control, next vAppTask starts executing. */

	return 0;
}

// 
// Skeleton application task 
//

static void vAppTask( int8_t *pvParameters )
{
	
	// Start the debug UART at 115k 
	debug_init(115200);
	vTaskDelay( 200 / portTICK_RATE_MS );

	// Initialize NanoStack with default parameters, NanoStack task automatically created. 
	{
		if(stack_start(NULL)==START_SUCCESS)
		{
			debug("NanoStack Start Ok\r\n");
		}
	}
	LED1_ON();
	vTaskDelay( 500 / portTICK_RATE_MS );
	LED1_OFF();

	// Open and bind a socket 
	
	broadcast_socket = socket(MODULE_CUDP, my_handler_function_name);
	if (broadcast_socket) {
		if (socket_bind(broadcast_socket, &broadcast) != pdTRUE) debug("Socket bind failed.\r\n");
	}	
	
	// Start an endless task loop, we must sleep most of the time allowing execution of other tasks. 
	for (;;)
	{

		// button 1 pressed
		if(P0_6 == 0)
		{
			// Send a packet 
			buffer_t *buf = socket_buffer_get(broadcast_socket);
			if (buf) {
				buf->buf_end=0;
				buf->buf_ptr=0;
				buf->options.hop_count = 1;
				buf->buf[buf->buf_end++] = 'T';
				buf->buf[buf->buf_end++] = 'e';
				buf->buf[buf->buf_end++] = 's';
				buf->buf[buf->buf_end++] = 't';
				buf->buf[buf->buf_end++] = '!';
			}
			if (socket_sendto(broadcast_socket, &broadcast, buf) == pdTRUE) {
				debug("Packet sent\r\n");
			}
		
			// Sleep for 200 ms 
			vTaskDelay( 200 / portTICK_RATE_MS );
		}
/*
		else
		{
			//wait for packets, timeout 1 second
			buffer_t *buf = socket_read(broadcast_socket, 200);
			if (buf) {
				LED2_TOGGLE();
				socket_buffer_free(buf); //free buffer
			}
	
		// Sleep for 100 ms 
		vTaskDelay( 100 / portTICK_RATE_MS );
		}
*/
	}
}

// 
// Listen to incoming data
//

static void ListenTask( int8_t *pvParameters )
{
/*
	// Start the debug UART at 115k 
	debug_init(115200);
	vTaskDelay( 200 / portTICK_RATE_MS );

	// Initialize NanoStack with default parameters, NanoStack task automatically created. 
	{
		if(stack_start(NULL)==START_SUCCESS)
		{
			debug("NanoStack Start Ok\r\n");
		}
	}
	LED1_ON();
	vTaskDelay( 500 / portTICK_RATE_MS );
	LED1_OFF();

	// Open and bind a socket 
	
	broadcast_socket = socket(MODULE_CUDP, 0);
	if (broadcast_socket) {
		if (socket_bind(broadcast_socket, &broadcast) != pdTRUE) debug("Socket bind failed.\r\n");
	}	
	
	// Start an endless task loop, we must sleep most of the time allowing execution of other tasks. 
	for (;;)
	{
		{
			//wait for packets, timeout 1 second
			buffer_t *buf = socket_read(broadcast_socket, 200);
			if (buf) {
				LED2_TOGGLE();
				socket_buffer_free(buf); //free buffer
			}
	
		// Sleep for 100 ms 
		vTaskDelay( 100 / portTICK_RATE_MS );
		}
	}
*/

	buffer_t *buf = 0;

	// Start the debug UART at 115k 
	debug_init(115200);
	vTaskDelay( 200 / portTICK_RATE_MS );

	// Initialize NanoStack with default parameters, NanoStack task automatically created. 
	{
		if(stack_start(NULL)==START_SUCCESS)
		{
			debug("NanoStack Start Ok\r\n");
		}
	}

	LED1_ON();
	vTaskDelay( 500 / portTICK_RATE_MS );
	LED1_OFF();

	// Open and bind a socket 
	
	//listenSocket = socket(MODULE_CUDP, my_handler_function_name);
	listenSocket = socket(MODULE_CUDP, 0);
	//listen.port = 253;

	if (listenSocket) {
		socket_bind(listenSocket, &listen);
	}	
	debug("Start Ok\r\n");

	// Start an endless task loop, we must sleep most of the time allowing execution of other tasks. 
	for (;;)
	{
	
		//wait for packets, timeout 1 second
		buf = socket_read(listenSocket, 1000);
		if (buf) {
			debug("Received Ok\r\n");
			LED2_TOGGLE();
			socket_buffer_free(buf); //free buffer
		}
	
		// Sleep for 100 ms 
		vTaskDelay( 10000 / portTICK_RATE_MS );
	}

}

void blink1(void)
{
    unsigned long c, c2;

    for(c2 = 0; c2<100; c2++)
    {
	// about 1 second = 0x28000
	for(c = 0; c<0x08000; c++)
		continue;
	P0_4 ^= 1;
    }

}
void blink2(void)
{
    unsigned long c, c2;

    for(c2 = 0; c2<100; c2++)
    {
	// about 1 second = 0x28000
	for(c = 0; c<0x08000; c++)
		continue;
	P0_5 ^= 1;
    }

}

// this callback function starts boot loader
portCHAR my_handler_function_name(buffer_t *buffer)
{
#define BOOT_ADDR 0x7000
       LED2_TOGGLE();

	EA = 0;
	_asm
		LJMP 0x7000
	_endasm;

       // free buffer
       socket_buffer_free(buffer);
       return pdTRUE;
}
