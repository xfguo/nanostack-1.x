/*
	NanoStack: MCU software and PC tools for IP-based wireless sensor networking.

	Copyright (C) 2008 Intelligent Systems Group (ISG)
	Computer Engineering Laboratory, Department of Electrical and Information Engineering
	University of Oulu, Finland

	Read and write RFID tag information and communicate with nRoute (Initial version)

	Authors:	Santtu Seppänen (Initial development)
				Markus Paldanius (Extended development)
	


 *------------------------------------------------------------------------------------------
 * Implementation of NanoModule functions.
 * File main.c implements functions for communicating via 6LoWPAN connection with the
 * nReader application that runs on PC or N770 mobile terminal.
 * File mifare.c implements the functions for initializing PN512 reader chip and the 
 * algorithms for reading and writing Mifare Classic and Ultralight RFID tags.
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


/*---------------------------------------------------------------------------------------*/


static void vTerminal(int8_t *pvParameters );

/* Setup address structures, 16-bit PAN with 64-bit 802.15.4 address short address. */

/* Broadcast address to port 254. */
sockaddr_t broadcast_address =
{
	ADDR_802_15_4_PAN_LONG,
	{ 0xFF, 0xFF, 0xFF, 0xFF, 
	  0xFF, 0xFF, 0xFF, 0xFF },
	  254
};

/* Data address to port 61619. */
sockaddr_t data_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 },
	  61619
};

/* Buffer and socket definitions. */
buffer_t *payload = 0;
buffer_t *buffer = 0;
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
	xTaskCreate(vTerminal, "Term", configMAXIMUM_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 0), (xTaskHandle *) NULL);

	vTaskStartScheduler();
	/* Scheduler has taken control, next vTerminal starts executing. */

	return 0;
}

/* Application task. */
static void vTerminal(int8_t *pvParameters)
{
	uint8_t i = 0;
	uint8_t m = 0;
	uint8_t active = 0;
	uint8_t NDEF = 0;
	uint8_t c = 0;
	uint8_t mode = 0; // 1 = read, 2 = write
	uint8_t dta[56];
	uint8_t length = 0;

	payload = 0;
	
	/* Start the debug UART at 115k */
	debug_init(115200);
    
	/* Sleep for 100 ms */
	vTaskDelay(100/portTICK_RATE_MS);
	
	/* Initialize NanoStack. */
	stack_init();

	/* Activate the PN512 reader. */
	ActivateReader();
	
	vTaskDelay(100/portTICK_RATE_MS);

	/* Start an endless task loop, we must sleep most of the time allowing execution of other tasks. */
	for ( ;; )
	{
		mode = 0;
		buffer = 0;
		LED_OFF();

		vTaskDelay(50/portTICK_RATE_MS);

		/* If there is no active socket, open and bind a socket */
		if (active == 0 && datasoc == 0)
		{
			/* Open a 6LoWPAN socket. */
			datasoc = socket(MODULE_CUDP, 0);

				/* Socket is now open if TRUE.*/
				if (datasoc)
				{				
					/* Bind the socket. */
					if (socket_bind(datasoc, &data_address) == pdTRUE)
					{
						active = 1;
						payload = 0;
						c = 0;
					}
					
					/* Binding failed */
					else
					{
						debug("Bind failed.\r\n");
					}								
				}
	
				/* Socket creation failed. */
				else
				{
					debug("Socket creation failed.\r\n");							
				}

			vTaskDelay(100/portTICK_RATE_MS);
		}


		if (datasoc)
		{
			/* Wait for data packets, timeout 10 ms. */
			payload = socket_read(datasoc, 10);
			
			vTaskDelay(50/portTICK_RATE_MS);

			/* Data received. */
			if (payload)
			{
				LED_ON();
				
				vTaskDelay(50/portTICK_RATE_MS);
					
				/* Process data. */
				c = (payload->buf[payload->buf_ptr]);

				mode = c;				
				
				vTaskDelay(100/portTICK_RATE_MS);

				LED_OFF();
				active = 0;
				c = 0;
			}
			
			/* Free payload buffer. */
			socket_buffer_free(payload);
			payload = 0;
			
			//vTaskDelay(50/portTICK_RATE_MS);				
		}

		vTaskDelay(100/portTICK_RATE_MS);

		/* If received command is improper, reset settings and close socket. */ 
		if (mode > 2)
		{
			mode = 0;
			active = 0;
			socket_buffer_free(payload);
			payload = 0;
			socket_close(datasoc);
			
			vTaskDelay(50/portTICK_RATE_MS);
		}

		/* Enter reading mode. */
		while (mode == 1)
		{
			/* User presses the button. */
			if (P0_0 == 0)
			{
				LED_ON();
				
				vTaskDelay(100 / portTICK_RATE_MS );
			
				/* Read a tag and send a data packet. */
				if (datasoc)
				{
					/* Allocate buffer. */
					buffer = socket_buffer_get(datasoc);

					if (buffer)
					{
						/* Execute the function, that reads a tag. */
						ReadMifare(dta);
						
						vTaskDelay(100 / portTICK_RATE_MS );
					
						/* Mifare UltraLight data received. */
						if (dta[0] == 0x88)
						{ 
							for(i = 0; i <= 55; i++)
							{
								/* Add the data to buffer. */
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
							for (i = 0; i <= 51; i++)
							{
								/* Add the data to buffer. */
								buffer->buf[buffer->buf_end++] = dta[i];

								if (dta[i] == 0xE1)
									NDEF = 1;

								else if (dta[i] == 0xFE && NDEF == 1)
									break;
						
							}
					
						}

						vTaskDelay(50/portTICK_RATE_MS);
					
						/* Send the tag data. */
						if (socket_sendto(datasoc, &broadcast_address, buffer) == pdTRUE)
						{
							mode = 0;
							active = 0;
							buffer = 0;
							
							vTaskDelay(50/portTICK_RATE_MS);
						}
                              
						else
						{
							stack_buffer_free(buffer);
							buffer = 0;
						}

					}

					else
					{
						debug("No buffer.\r\n");
					}
                
					/* Free buffer and close socket. */
					socket_buffer_free(buffer);
					buffer = 0;
					NDEF = 0;
					socket_close(datasoc);
					datasoc = 0;
					
					vTaskDelay(100/portTICK_RATE_MS);

				}

				else if (!datasoc)
					debug("No socket.\r\n");

			}

		}//while


		/* Enter writing mode. */
		while (mode == 2)
		{
			/* User presses the button. */
			if (P0_0 == 0)
			{
				LED_ON();		    

				/* Check if the socket is open. */
				if (datasoc)
				{
					/* Read the socket if it contains data. */
					payload = socket_read(datasoc, 10);
					
					vTaskDelay(50/portTICK_RATE_MS);
					
					/* Payload contains data. */
					if (payload)
					{
						/* Set the length of the payload. */
						length = payload->buf_end - payload->buf_ptr;
						
						vTaskDelay(50/portTICK_RATE_MS);
				
						/* Construct the NDEF headers. */
						dta[0] = 0xE1;			//Start mark of NDEF
						dta[1] = 0x10;			//Version of NDEF
						dta[2] = 0x80;			//Tag memory size for Classic, modified for UL later
						dta[3] = 0x00;			//Unrestricted write and read access
						dta[4] = 0x03;			//Indicates the presence of NDEF
						dta[5] = length + 4;//Length of the NDEF message
						dta[6] = 0xD1;			//NDEF header (11010001)
						dta[7] = 0x01;			//Length of TYPE field
						dta[8] = length;		//Length of payload
						dta[9] = 0x54;			//Payload is text
				
						/* Get the payload data from buffer and place it to the PAYLOAD field. */
						for (i = 10; i < (length + 10); i++)
						{
							dta[i] = (payload->buf[payload->buf_ptr+m]);
							//debug_hex(dta[i]);
							m++;
						}
				
						dta[length + 10] = 0xFE; //Indicate end of NDEF message	
				
						vTaskDelay(50/portTICK_RATE_MS);

						/* Execute the function, that writes a tag. */
						WriteMifare(dta);
						
						vTaskDelay(100/portTICK_RATE_MS);
				
						/* Reset variables. */
						c = 0;
						m = 0;
						mode = 0;
						payload = 0;
				
						/* Free buffer and close socket. */
						socket_buffer_free(payload);
						socket_close(datasoc);
						active = 0;
					}
			
					LED_OFF();
					
					vTaskDelay(50/portTICK_RATE_MS);			
				}
	
			}

		}//while

	}//for

}//vTerminal

