/*
	NanoStack: MCU software and PC tools for IP-based wireless sensor networking.

	Copyright (C) 2008 Intelligent Systems Group (ISG)
	Computer Engineering Laboratory, Department of Electrical and Information Engineering
	University of Oulu, Finland

	Read RFID tag information and send it to nRoute (Initial version)

	Authors:	Santtu Seppänen (Initial development)
				Markus Paldanius (Extended development)
	


 *------------------------------------------------------------------------------------------
 * Implementation of NanoModule functions.
 * File main.c implements functions for sending RFID data to N770 over 6LoWPAN connection. 
 * File mifare.c implements the functions for initializing PN512 reader chip and the 
 * functions for reading Mifare Classic and Ultralight RFID tags.
 *-----------------------------------------------------------------------------------------*/


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
#include "debug.h"

/* Platform includes */
#include "uart.h"
#include "rf.h"
#include "bus.h"

/* PN512 functions include */
#include "mifare.h"

void IO_INIT(void) {P0DIR |= 0xFB;}
void RESET_ON(void) {P0_6 = 1;}
void RESET_OFF(void) {P0_6 = 0;}

void IRQ_ON(void) {P0_7 = 1;}
void IRQ_OFF(void) {P0_7 = 0;}

void SS_ON(void){P0_4 = 1;}
void SS_OFF(void){P0_4 = 0;}

void LED_ON(void) {P0_1 = 1;}
void LED_OFF(void) {P0_1 = 0;}


/*-------------------------------------------------------------------------------------------*/


static void vReader(int8_t *pvParameters );

/* Setup address structures, 16-bit PAN with 64-bit 802.15.4 address short address. */

/* Data address to port 61619. */
sockaddr_t data_address =
{
	ADDR_802_15_4_PAN_LONG,
	{ 0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 },
	  61619
	};

/* Broadcast address to port 254. */
sockaddr_t broadcast_address =
{
	ADDR_802_15_4_PAN_LONG,
	{ 0xFF, 0xFF, 0xFF, 0xFF,
	  0xFF, 0xFF, 0xFF, 0xFF },
	  254
	};

/* Buffer and socket definitions. */
buffer_t *payload = 0;
socket_t *datasoc = 0;


/* Main task, initialize hardware and start the FreeRTOS scheduler */
int main(void)
{
	/* Initialize the Nano hardware */
	IO_INIT();
	RESET_ON();
	IRQ_ON();
	SS_ON();
	bus_init();
	
	/* Setup the application task and start the scheduler */
	xTaskCreate(vReader, "RFID Reader", configMAXIMUM_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 0), (xTaskHandle *) NULL);

	vTaskStartScheduler();
	/* Scheduler has taken control, next vReader starts executing. */

	return 0;
}

/* Application task. */
static void vReader(int8_t *pvParameters)
{
	uint8_t i = 0;
	uint8_t NDEF = 0;
	uint8_t ping_active = 0;
	uint8_t dta[56];

	debug_init(115200);
    
	/* Sleep for 100 ms */
	vTaskDelay(100/portTICK_RATE_MS);
	
	/* Initialize NanoStack. */
	stack_init();

	/* Activate the PN512 chip. */
	ActivateReader();
	
	vTaskDelay(100/portTICK_RATE_MS);

	/* Start an endless task loop, we must sleep most of the time allowing execution of other tasks. */
	for ( ;; )
	{
		LED_OFF();

		/* Change power mode to 3 if 0 and turn radio off */
		if (!(SLEEP & 0x03))
		{	
			SLEEP |=0x03;	//Enter power mode 3
			PCON |= 0x01; 	//Enable power mode 3
						
			RFPWR = 0x0C; 	//Turn radio off
			CLKCON |= 0x40; //Select 16 MHz RC oscillator
			while ((!HFRC_STB) || (!RFPWR & 0x10)); //Wait until stable
			SLEEP |= 0x04; 	//Turn off 16 MHz RC oscillator
		}

		/* User presses the button. */
		if (P0_0 == 0)
		{
			LED_ON();
			
			/* Change power mode to 0 and turn radio on */
			RFPWR = 0x04; 	//Turn radio on
			RFIF = 0;
			RFIM = 0x80; 	// Enable IRQ_RREG_PD interrupt		    
		    
			SLEEP &= ~0x04; // Turn on 32 MHz crystal oscillator
			vTaskDelay(100/portTICK_RATE_MS);

			SLEEP &= ~0x03; //Enter power mode 0
			PCON |= 0x01; 	//Enable power mode 0
			
			while ((!XOSC_STB) || (RFPWR & 0x10)); //Wait until stable
			CLKCON &= ~0x47;//Change from 16 MHz RC osc to 32 MHz crystal
			SLEEP |= 0x04; 	// Turn off 16 MHz RC oscillator
		    
			/* If there is no open socket, open and bind a socket */
			if (datasoc == 0)
			{
				/* Open a 6LoWPAN socket. */
				datasoc = socket(MODULE_CUDP, 0);
                
				/* Socket is now open if TRUE. */
				if (datasoc)
				{
					/* Bind the socket. */
					if (socket_bind(datasoc, &data_address) == pdTRUE)
					{
						payload = 0;
					}

					/* Binding failed */
					else
					{
						debug("Socket binding failed.\r\n");
					}

				}

				/* Socket creation failed. */
				else
				{
					debug("Socket creation failed.\r\n");
				}

			}

			else
			{
				socket_close(datasoc);
				debug("Socket closed.\r\n");
				datasoc = 0;
				payload = 0;
			}

			/* Read a tag and send a data packet. */
			if (datasoc != 0 && ping_active == 0)
			{
				/* Allocate buffer. */
				buffer_t *buffer = socket_buffer_get(datasoc);

				if (buffer)
				{
					/* Execute the function, that reads a tag. */
					RequestMifare(dta);
					
					vTaskDelay(50 / portTICK_RATE_MS );

					/* Mifare UltraLight data received. */
					if (dta[0] == 0x88)
					{
						/* Add the data to buffer. */
						for (i = 0; i <= 55; i++)
						{
							buffer->buf[buffer->buf_end++] = dta[i];

							if (dta[i] == 0xE1)
								NDEF = 1;

							else if (dta[i] == 0xFE && NDEF == 1)
								break;
						
						}

					}

					/* Mifare Classic data received. */
					else
					{
						/* Add the data to buffer. */
						for (i = 0; i <= 63; i++)
						{
							buffer->buf[buffer->buf_end++] = dta[i];

							if (dta[i] == 0xE1)
								NDEF = 1;

							else if (dta[i] == 0xFE && NDEF == 1)
								break;
						
						}

					}

					payload = 0;

					/* Send the tag data. */
					if (socket_sendto(datasoc, &broadcast_address, buffer) == pdTRUE)
					{
						ping_active = 1;
					}
                    
					else
					{
						stack_buffer_free(buffer);
						debug("Ping send failed.\r\n");
					}

				}

				else
				{
					debug("No buffer.\r\n");
				}
                
				socket_buffer_free(buffer);

			}

			else
			{
				if (!datasoc)
				{
					debug("No socket.\r\n");
				}

				else
				{
					ping_active = 0;
				}

			}

			while (P0_0 == 0);

		}

		/* Free buffer and close socket. */
		if (ping_active)
		{
			socket_close(datasoc);
			//debug("Socket closed.\r\n");
			payload = 0;
			datasoc = 0;
			ping_active = 0;
			NDEF = 0;
		}

	}//for

}//vReader
