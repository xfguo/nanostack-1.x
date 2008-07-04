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


/* Standard includes. */
#include <stdlib.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <sys/inttypes.h>

#include "stack.h"

/*these have to be included, this way interrupt vectors get defined*/
#include "uart.h"
#include "rf.h"
#include "bus.h"
#include "dma.h"
#include "timer.h"
#include "gpio.h"
#include "nrp_uart.h"

/*debug library*/
#include "debug.h"


/*-----------------------------------------------------------*/
static void vMain(int8_t *pvParameters );

int main( void )
{
	bus_init();

	LED1_ON();
	LED2_ON();
	stack_init();
	xTaskCreate( vMain, "Main", configMAXIMUM_STACK_SIZE, NULL, ( tskIDLE_PRIORITY + 0 ), ( xTaskHandle * )NULL );	

	vTaskStartScheduler();

	return 0;
}

extern portCHAR nrp_gw_advertisement(void);	/*from protocol_lib.c*/
extern uint8_t nrp_router_advertise_period;	/*in nrp.c*/

static void vMain(int8_t *pvParameters )
{
	pvParameters;
	
	debug_init(115200);
	debug("n600\r\n");

	LED2_OFF();
	LED1_OFF();
	
	for( ;; )
	{
		while (nrp_router_advertise_period == 0)
		{
			vTaskDelay(10000/portTICK_RATE_MS);
		}
		while (nrp_router_advertise_period)
		{
			uint16_t time = (nrp_router_advertise_period*1000); 
			vTaskDelay(time/portTICK_RATE_MS);
			
			nrp_gw_advertisement();
		}
	}
}


