/* Standard includes. */
#include <stdlib.h>
#include <string.h>
#include <sys/inttypes.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Debug library. */
#include "debug.h"

/* PN512 functions include */
#include "mifare.h"

uint8_t block = 1;
uint8_t dta[64];
uint8_t bcc = 0;
uint8_t a = 0;
uint8_t type = 0;
uint8_t success = 0;

void SPIinit(void)
{
	/* Baud rate = ((256+UxBAUD) * 2^UxGCR) * crystal / (2^28) */
	PERCFG &= ~U0CFG;	//Alternative port 1 = P0.5-2
	P0SEL |= 0x2C;		//Peripheral select for CLK, NOT SS, MOSI and MISO (00101100)
	U0BAUD = 64;		//Baud rate mantissa value for SPI 5MHz
	U0GCR = 17;			//Baud rate exponent value for SPI 5MHz
	U0GCR |= 0x20;		//Bit order for transfer (MSB first)
}

/* Function for sending a data byte to FIFO buffer. */
void SR(uint8_t addr, uint8_t dta)
{
	SS_OFF();
	addr <<= 1;
	U0BUF = addr;
	while (U0CSR ==U_START);
	U0BUF = dta;
	while (U0CSR == U_START);
	SS_ON();
}

/* Function for reading a data byte from FIFO buffer. */
uint8_t GR(uint8_t addr)
{
	SS_OFF();
	addr <<= 1;
	U0BUF = 0x80 | addr;
	while (U0CSR == U_START);
	SS_ON();
	return U0BUF;
}

/* Activate PN512 reader chip and set power down mode. */
void ActivateReader(void)
{	
	RESET_OFF();
	vTaskDelay(50 / portTICK_RATE_MS );
	RESET_ON();
	SPIinit();
	vTaskDelay(50 / portTICK_RATE_MS );
	SR(COMMANDREG, 0x30);//Power down mode
}

/* Authentivate Mifare Classic tag. */
void Authenticate(uint8_t dta[])
{
	SR(COMMANDREG, 0x0C);	//Activate transceive
	SR(FIFOLEVELREG, 0x80);	//Flush FIFO
	SR(FIFODATAREG, 0x60);	//Use key A
	SR(FIFODATAREG, block);	//Block number
    
	/* 6-byte key A (FFFFFFFFFFFF) for activation. */
	for (a = 0; a <= 5; a++)
		SR(FIFODATAREG, 0xFF);//Authent KEY A
    
	/* Classic tag serial numbers 0-3. */
	SR(FIFODATAREG, *dta);
	SR(FIFODATAREG, *(dta+1));
	SR(FIFODATAREG, *(dta+2));
	SR(FIFODATAREG, *(dta+3));
    
	/* Perform authentication. */
	SR(COMMANDREG, 0x0E);	//MFAuthent
	GR(ERRORREG);			//Read ErrReg
	vTaskDelay(10 / portTICK_RATE_MS );
}

/* Read Mifare Classic or UltraLight tag. */
void RequestMifare(uint8_t *dta)
{
	uint8_t cnt = 0;
	uint8_t page = 4;
	block = 1;
	success = 0;

	/* Come back here to read data again in case of a reading error. */
	reread:

	type = 0;

	SR(COMMANDREG, 0x0F);    // Softreset
	vTaskDelay(5 / portTICK_RATE_MS );

	/* Configure PN512. */
	SR(CONTROLREG, 0x10);	//Act as initiator
	SR(TXAUTOREG, 0x40);	//Force 100% ASK modulation
	SR(RXTRESHOLDREG, 0x55);//Define MinLevel CollLevel
	SR(DEMODREG, 0x4D);		//Demodulator settings
	SR(RFCFGREG, 0x59);		//Receiver gain and RF level detector sensitivity
	SR(GSNONREG, 0xF4);		//Conductance for the antennas N-driver when switched ON
	SR(CWGSPREG, 0x3F);		//Conductance for the antennas P-driver when modulation OFF
	SR(TXCONTROLREG, 0x83);	//Enable InvTx2RFOn, Tx2RFEn, Tx1RFEn

	SR(TXMODEREG, 0x00);	//Deactivate CRC
	SR(RXMODEREG, 0x00);	//Deactivate CRC
	SR(GSNOFFREG, 0x6F);	//Conductance for the antennas N-driver when switched OFF
	SR(MODWIDTHREG, 0x26);	//Modulation width settings
	SR(TXBITPHASEREG, 0x8F);//Bitphase adjustment during transmission
	SR(MODGSPREG, 0x11);	//Conductance for the antennas P-driver when modulation ON
	vTaskDelay(5 / portTICK_RATE_MS );

	SR(COMMANDREG,0x0C);	//Transceiving settings (transmit - receive)

	/* Send Mifare request. */
	SR(FIFOLEVELREG, 0x80);	//Flush FIFO
	SR(FIFODATAREG, 0x26);	//Request command (7 bit)
	GR(FIFOLEVELREG);		//Read FIFOLevel
	SR(BITFRAMINGREG, 0x87);//StartSend, 7 bits
	vTaskDelay(5 / portTICK_RATE_MS );

	/* Response ATQ. */
	GR(ERRORREG);			//Read ErrReg
	GR(FIFOLEVELREG);		//Read FIFOLevel
	GR(FIFODATAREG);		//Get ATQ LSB
	GR(FIFODATAREG);		//Get ATQ MSB
	vTaskDelay(5 / portTICK_RATE_MS );
	
	/* Anticollision of Cascade Level1. */
	SR(FIFOLEVELREG, 0x80);	//Flush FIFO
	SR(FIFODATAREG, 0x93);	//Command code
	SR(FIFODATAREG, 0x20);	//Anticollision CL1 parameter
	SR(BITFRAMINGREG, 0x80);//StartSend
	vTaskDelay(5 / portTICK_RATE_MS );

	/* Response the first part the of UID. */
	GR(ERRORREG);			//Read ErrReg
	GR(FIFOLEVELREG);		//Read FIFOLevel
	GR(FIFODATAREG);		//Read CT
	*dta = GR(FIFODATAREG);	//Read SNR 0
	*(dta+1) = GR(FIFODATAREG);	//Read SNR 1
	*(dta+2) = GR(FIFODATAREG);	//Read SNR 2
	*(dta+3) = GR(FIFODATAREG);	//Read SNR 3
	bcc = GR(0x00);				//BCC1
	vTaskDelay(5 / portTICK_RATE_MS );
    
	/* Select of Cascade Level1. */
	SR(FIFOLEVELREG, 0x80);	//Flush FIFO
	SR(FIFODATAREG, 0x93);	//Command code
	SR(FIFODATAREG, 0x70);	//Select CL1 parameter
	SR(TXMODEREG, 0x80);	//Activate CRC
	SR(RXMODEREG, 0x80);	//Activate CRC
	SR(FIFODATAREG, *dta);	//Write SNR 0 to FIFO
	SR(FIFODATAREG, *(dta+1));	//Write SNR 1 to FIFO
	SR(FIFODATAREG, *(dta+2));	//Write SNR 2 to FIFO
	SR(FIFODATAREG, *(dta+3));	//Write SNR 3 to FIFO
	SR(FIFODATAREG, bcc);		//Write BCC1 (check byte) to FIFO
    
	SR(BITFRAMINGREG, 0x80);//StartSend
	vTaskDelay(5 / portTICK_RATE_MS );

	/* Response SAK. */
	GR(ERRORREG);			//Read ErrReg
	GR(FIFOLEVELREG);		//Read FIFOLevel
	GR(FIFODATAREG);		//Read SAK
	type = GR(0x00);		//Set tag type
	vTaskDelay(5 / portTICK_RATE_MS );	

	/* If the SAK response was 08h, read a Classic tag. */
	if (type == 0x08)
	{
		success = 1;
		//debug("Classic tag. ");

		/* Come back here to read next block until all data is read. */	
		nextblock:

		vTaskDelay(30 / portTICK_RATE_MS );

		/* Authenticate the current block. */	
		Authenticate(dta);

		
		SR(BITFRAMINGREG, 0x80);//StartSend
		SR(COMMANDREG, 0x0C);	//Transceiving settings (transmit - receive)
    
		/* Active state: perform read command to Classic tag. */
		SR(FIFOLEVELREG, 0x80);	//Flush FIFO
		SR(FIFODATAREG, 0x30);	//Read command
		SR(FIFODATAREG, block);	//Block number to read
		SR(BITFRAMINGREG, 0x80);//BitframingReg
		vTaskDelay(5 / portTICK_RATE_MS );

		GR(ERRORREG);			//Read ErrReg
    
		/* If an error occurred, read again. */
		if (GR(0x00) != 0x00)
		{
			success = 0;
			goto reread;
		}

		GR(FIFODATAREG);
		vTaskDelay(5 / portTICK_RATE_MS );

		/* After authentication, a block of data is read. */
		if (block < 3)
		{
			/* Read NDEF data. */
			for (cnt = 4; cnt <= 35; cnt++)
			{    
				/* If block 1 is done, go to block 2. */ 
				if (block == 2 && cnt == 4)
					cnt = 20;
		
				/* Get data byte from FIFO and add it to array. */
				*(dta+cnt) = GR(FIFODATAREG);

				/* Check if byte is NDEF start mark. */
				if (*(dta+cnt) == 0xE1)
					success = 2;

				/* Check if byte is NDEF end mark. */
				if (*(dta+cnt) == 0xFE)
				{
					success = 3;
					break;
				}

				/* Check if the byte is last byte on the 1. block. */
				if (cnt == 19 && success != 3)
				{
					block++;
					goto nextblock;
				}
		
				/* Check if the byte is last byte on the 2. block. */
				if (cnt == 35 && block == 2)
				{
					block = block + 2;
					goto nextblock;
				}

			}

			vTaskDelay(5 / portTICK_RATE_MS );

		}

		/* Move to 2. sector. */
		if (block > 3)
		{
			/* Read NDEF data. */
			for (cnt = 36; cnt <= 51; cnt++)
			{
				/* Get data byte from FIFO and add it to array. */
				*(dta+cnt) = GR(FIFODATAREG);

				/* Check if byte is NDEF start mark. */
				if (*(dta+cnt) == 0xE1)
					success = 2;

				/* Check if byte is NDEF end mark. End reading if TRUE.*/
				if (*(dta+cnt) == 0xFE)
				{
					success = 3;
					break;
				}

			}

			vTaskDelay(5 / portTICK_RATE_MS );
		}

	}

	/* If the SAK response was 04h, read an UltraLight tag. */
	if (type == 0x04)
	{
		success = 1;
		//debug("UltraLight ");
    
		/* Anticollision of Cascade Level2. */
		SR(FIFOLEVELREG, 0x80);	//Flush FIFO
		SR(TXMODEREG, 0x00);	//Deactivate CRC
		SR(RXMODEREG, 0x00);	//Deactivate CRC
		SR(FIFODATAREG, 0x95);	//Command code
		SR(FIFODATAREG, 0x20);	//Anticollision CL2 parameter
		SR(BITFRAMINGREG, 0x80);//StartSend
		vTaskDelay(5 / portTICK_RATE_MS );

		/* Response the second part of the UID. */
		GR(ERRORREG);			//Read ErrReg
		GR(FIFOLEVELREG);		//Read FIFOLevel
		GR(FIFODATAREG);		//Read FIFO byte
		*(dta+4) = GR(FIFODATAREG);	//Read SNR 4
		*(dta+5) = GR(FIFODATAREG);	//Read SNR 5
		*(dta+6) = GR(FIFODATAREG);	//Read SNR 6
		*(dta+7) = GR(FIFODATAREG);	//Read SNR 7
		bcc = GR(0x00);				//BCC2
		vTaskDelay(5 / portTICK_RATE_MS );
	
		/* Select of Cascade Level2. */
		SR(FIFOLEVELREG, 0x80);	//Flush FIFO
		SR(FIFODATAREG, 0x95);	//Command code
		SR(FIFODATAREG, 0x70);	//Select CL2 parameter
		SR(TXMODEREG, 0x80);	//Activate CRC
		SR(RXMODEREG, 0x80);	//Activate CRC
		SR(FIFODATAREG, *(dta+4));	//Write SNR 4 to FIFO
		SR(FIFODATAREG, *(dta+5));	//Write SNR 5 to FIFO
		SR(FIFODATAREG, *(dta+6));	//Write SNR 6 to FIFO
		SR(FIFODATAREG, *(dta+7));	//Write SNR 7 to FIFO
		SR(FIFODATAREG, bcc);		//Write BCC2 (check byte) to FIFO

		SR(BITFRAMINGREG, 0x80);//StartSend
		vTaskDelay(5 / portTICK_RATE_MS );

		/* Response SAK. */
		GR(ERRORREG);			//Read ErrReg
		GR(FIFOLEVELREG);		//Read FIFOLevel
		GR(FIFODATAREG);		//Read SAK
		type = GR(0x00);		//Foobar
		vTaskDelay(5 / portTICK_RATE_MS );
	
		/* Data starts from page 4. */
		page = 4;

		/* Come back here to read next page until all data is read. */	
		nextpage:
		
		/* Active state: perform read command to UltraLight tag. */
		SR(FIFOLEVELREG, 0x80);	//Flush FIFO
		SR(FIFODATAREG, 0x30);	//Read command
		SR(FIFODATAREG, page);	//Page number to read
		SR(TXMODEREG, 0x80);	//Activate CRC
		SR(RXMODEREG, 0x80);	//Activate CRC
		SR(BITFRAMINGREG, 0x80);//StartSend
		vTaskDelay(5 / portTICK_RATE_MS );
		
		GR(ERRORREG);			//Read ErrReg
    
		/* If an error occurred, read again. */
		if (GR(0x00) != 0x00)
		{
			success = 0;
			goto reread;
		}

		GR(FIFOLEVELREG);		//Read FIFOLevel
		GR(FIFODATAREG);
		vTaskDelay(5 / portTICK_RATE_MS );
		
		/* Read NDEF data. */
		for (cnt = 8; cnt <= 55; cnt++)
		{    	
			/* If pages 4-7 are done, go to page 8. */ 
			if (page == 8 && cnt == 8)
				cnt = 24;
			
			/* If pages 8-11 are done, go to page 12. */ 	        		
			if (page == 12 && cnt == 8)
				cnt = 40;

			/* Get data byte from FIFO and add it to array. */
			*(dta+cnt) = GR(FIFODATAREG);

			/* Check if byte is NDEF start mark. */
			if (*(dta+cnt) == 0xE1)
				success = 2;

			/* Check if byte is NDEF end mark. End reading if TRUE.*/
			if (*(dta+cnt) == 0xFE)
			{
				success = 3;
				break;
			}

			/* Check if the byte is last byte on the defined reading area. */
			if (cnt == 23 || cnt == 39)
			{
				page = page + 4;//Update the readable address
				goto nextpage;
			}

		}
		
		vTaskDelay(5 / portTICK_RATE_MS );
	}

	GR(ERRORREG);			//Read ErrReg
	
	/* If an error occurred or there's no tag in the field, go back and read again. */
	if (GR(0x00) != 0x00 || (success != 1 && success != 3))
	{
		//debug("No tag. ");
		success = 0;
		goto reread;
	}

	SR(COMMANDREG, 0x0F);	//Softreset
	//vTaskDelay(5 / portTICK_RATE_MS );
	SR(COMMANDREG, 0x30);	//Power down mode
	return;

}//RequestMifare
