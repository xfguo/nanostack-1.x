/*
    NanoStack: MCU software and PC tools for sensor networking.
		
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
		PO Box 1
		90571 Oulu, Finland

		E-mail:
		info@sensinode.com
*/


/* Standard includes. */
#include <stdlib.h>
#include <signal.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "bus.h"
#include "gpio.h"
#include "debug.h"
static void vmaxfor_example( void *pvParameters );


//#include "socket.h"
//#include "rf.h"

#include "stack.h"
#include "control_message.h"
#include "neighbor_routing_table.h"

//powersave
//#include "powersave.h"

#define DIDU_BUFFER_LENGTH 59
extern xQueueHandle     buffers;
extern sockaddr_t mac_long;

/* Here goes the code */
int main( void )
{	
	uint8_t i=0;
	uint8_t j=0;

	LED_INIT();
	
/*	for(i=0;i<10;i++)
	{
		LED1_ON();	
		pause(1000);
		LED1_OFF();
		pause(1000);		
	}

	for(i=0;i<10;i++)
	{		
		LED2_ON();			
		pause(1000);		
		LED2_OFF();		
		pause(1000);
	}

	for(i=0;i<10;i++)
	{
		LED3_ON();	
		pause(1000);		
		LED3_OFF();
		pause(1000);
	}*/


	if (bus_init() == pdFALSE)
	{
	}	
	for(i=0;i<2;i++)
	{
		LED3_ON();	
		pause(200);		
		LED3_OFF();
		pause(200);
	}
	pause(200);
	debug_init(115200);
	pause(300);

	xTaskCreate( vmaxfor_example, "main", 250, NULL, ( tskIDLE_PRIORITY + 1 ), NULL );	
	vTaskStartScheduler();
	return 0;
}
/*-----------------------------------------------------------*/
sockaddr_t datasensor_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0x00, 0x00, 0x00, 0x00, 
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	61619
};

sockaddr_t data_address = 
{
	ADDR_BROADCAST,
	{ 0x00, 0x00, 0x00, 0x00, 
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	7
};


static void vmaxfor_example( void *pvParameters )
{
	//uint8_t test[6]= "Hello";
	uint8_t i=0;
	int16_t rx;
	uint16_t led_count;
	socket_t *data_soc = 0;
	int8_t ping_active=0;
	portTickType ping_start;
	uint16_t echo_sqn=0;

	//debug("Task start.\r\n");
	vTaskDelay(300/portTICK_RATE_MS);	
	//debug("Init:\r\n");
	stack_init();

	{
		stack_init_t stack_rules;
		start_status_t status;
		memset(stack_rules.mac_address, (uint8_t) 0, 8);

		memset(stack_rules.mac_address, (uint8_t) 0, 8);

		stack_rules.channel = CC2420_DEFAULT_CHANNEL;
		stack_rules.type = AD_HOC_DEVICE;
		for(i=0; i<8; i++)
		{
			stack_rules.mac_address[i] = mac_long.address[i];
		}

		//debug("Start.\r\n");
		vTaskDelay(300/portTICK_RATE_MS);
		status = stack_start(&stack_rules);
		if(status==START_SUCCESS)
		{
			//debug("Start Ok\r\n");
		}
	}
	i=0;
	for(;;)
	{	
		//debug_printf("%d", i);	
		debug_hex(0x01);	
		//debug("Testi");
		vTaskDelay(5000/portTICK_RATE_MS);
		//i++;

		/*rx = debug_read_blocking(500/portTICK_RATE_MS);
		if (rx != -1)
		{
			switch(rx)
			{
				case 's':
					data_soc 			= socket(MODULE_CUDP, 		0);	// Socket for response data from port number 45 
					if (data_soc)
					{
						if (socket_bind(data_soc, &datasensor_address) != pdTRUE)
						{
						//	debug("Bind fail rx.\r\n");					
						}
					}
					else
					{
						//debug("No socket.\r\n");
					}
	
					break;

				case 'u':
					if(data_soc)
					{
						buffer_t *buf = socket_buffer_get(data_soc);
						if (buf)
						{
							buf->buf_end=0;
							buf->buf_ptr=0;
								buf->buf[buf->buf_end++] = echo_sqn >> 8;
							buf->buf[buf->buf_end++] = echo_sqn;
							buf->options.hop_count =5;
							if (socket_sendto(data_soc, &data_address, buf) == pdTRUE)
							{
								ping_start = xTaskGetTickCount();
								ping_active = 1;
								vTaskDelay(10/portTICK_RATE_MS);
								//debug("udp echo_req(");
								//debug_hex(echo_sqn);
							//	debug(")\r\n");
								echo_sqn++;
							}
						}

					}
					else
					{
						//debug("No socket.\r\n");
					}
					break;
				case '\r':
				//	debug("\r\n");
					break;
				default:
					debug_put(rx);
			}
		}*/
		led_count++;
		if (led_count & 1)
		{
			LED1_ON();
		}
		else
		{
			LED1_OFF();
		}
	}
}


