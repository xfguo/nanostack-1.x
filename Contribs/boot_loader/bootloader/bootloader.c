/*
    bootloader: application for reprogramming TI's CC2430/31 microcontrollers 
		using Sensinode's NanoStack
		
    Copyright (C) 2008 Tallinn University of Technology

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
		Tallinn University of Technology
		Ehitajate tee 5
		19086 Tallinn, Estonia

		E-mail:
		rasmus_raag@users.sourceforge.net
*/

#include "ioCC2430.h"
#include "inttypes.h"
//#include "redirectint.h"

#define LED_INIT()	{ P0DIR |= 0x30; }
#define LED1_ON() 	{ P0_4 = 1; }
#define LED1_TOGGLE() 	{ P0_4 ^= 1; }
#define LED1_OFF()	{ P0_4 = 0; }
#define LED2_ON()	{ P0_5 = 1; }
#define LED2_TOGGLE() 	{ P0_5 ^= 1; }
#define LED2_OFF()	{ P0_5 = 0; }
#define START_APP ((void(code*)(void))0x1000)();

#define FLASH_SIZE	0x00020000
#define BANK_SIZE	0x00008000
#define PAGE_SIZE	0x00000800

#define SNOP		0xC0
#define STXCALN 	0xC1
#define SRXON		0xC2
#define STXON		0xC3
#define STXONCCA	0xC4
#define SRFOFF		0xC5
#define SFLUSHRX	0xC6
#define SFLUSHTX	0xC7
#define SACK		0xC8
#define SACKPEND	0xC9

#define ISTXCALN 	0xE1
#define ISRXON		0xE2
#define ISTXON		0xE3
#define ISTXONCCA	0xE4
#define ISRFOFF		0xE5
#define ISFLUSHRX	0xE6
#define ISFLUSHTX	0xE7
#define ISACK		0xE8
#define ISACKPEND	0xE9

#define ISSTOP		0xFF
#define ISSTART		0xFE

__xdata __at (0xDF64) IRQSRC;
#define TXACK 0x01

#define ADDR_DECODE 0x08
#define AUTOCRC 0x20
#define AUTOACK 0x10

// #define TIMING

#define LOCATION_PAGE 0x0E
#define LOCATION_SIZE 0x02

unsigned long a[2];

#ifdef TIMING
#define HISTORY 10
	unsigned char times[HISTORY];
	signed short timescounter = 0;
#endif
unsigned char * gen_pointer;
__code unsigned char * code_pointer;
volatile unsigned char radiopacket[132];
unsigned char counter = 0;
unsigned long int addr, timer;
unsigned char frame_length, data_length, data_start, addr_start, last_seq_no;
unsigned char frame_sequence_number = 1;
__xdata unsigned char * flash_address;
uint8_t backup;

// enough room for 64-bit destination address and PAN, is divisble by 4
unsigned char dest[12];
// indices simplify address swapping greatly
unsigned char src_index, dest_index;

// void blink1();

void erasePage(unsigned char pageNum);
void ListenToRadio(void);

void rf_command(unsigned char command);
char rf_cca_check(unsigned char backoff_count, unsigned char slotted);
char rf_tx_enable(void);
char rf_write(unsigned char *buffer, unsigned char length);
char rf_rx_enable(void);

// void rf_test(void);

void CRC16(unsigned char length);

void pause_us(unsigned short time);
void pause(unsigned short time);

void readflash(unsigned long addr, unsigned char length);
void programflash(unsigned long addr, unsigned char length);
void reset(void) _naked;

uint8_t reverse_packet(void);

// DMA Sturcture
typedef struct
{
  unsigned char  SRCADDRH;
  unsigned char  SRCADDRL;
  unsigned char  DESTADDRH;	
  unsigned char  DESTADDRL;
  unsigned char  VLEN;
  unsigned char  LEN;
  unsigned char  TRIG;
  unsigned char  INC_IRQ;
} DMA_SETTINGS;

static DMA_SETTINGS  __xdata FlashDMA;

void FlashDmaTrigger() _naked  //_naked, so it does not use stack
{
     // move trigger statement to 4 byte boundary
     // unfortunately, there is no better way if using SDCC
/*
     _asm
         nop
         nop
         nop
         nop
     _endasm;
*/
     FCTL |= F_WRITE;

	// must manually return from naked function
	_asm
		ret
	_endasm;

     return;
}

void DMAinit()
{

    // does not work without these two
    DMAARM = 0xFF;
    DMAIRQ = 0x00;

    FlashDMA.DESTADDRH  = 0xdf;
    FlashDMA.DESTADDRL  = 0xaf;
    FlashDMA.VLEN    = 0x00;
    FlashDMA.LEN     = 0x04;
    FlashDMA.TRIG    = 0x12;
    FlashDMA.INC_IRQ = 0x42;

    DMA0CFGH = (unsigned short)&FlashDMA >> 8;
    DMA0CFGL = (unsigned short)&FlashDMA;

}

void WriteFlash(unsigned long byte_address, unsigned char *buf, unsigned short len)
{
	// later, get rid of this variable, and use byte_address directly
	unsigned short address;

	// does not overwrite itself
	if((addr+len)/PAGE_SIZE >= LOCATION_PAGE && addr/PAGE_SIZE < LOCATION_PAGE + LOCATION_SIZE)
		  return;
	// does not work without these two
	DMAARM = 0xFF;
	DMAIRQ = 0x00;
	
	address = byte_address >> 2;
	// use DMA Channel 0
	// Set up Flash DMA
	
	FlashDMA.LEN     = len;

	// Set source address
	FlashDMA.SRCADDRH  = ((unsigned short)buf >> 8) & 0xFF;
	FlashDMA.SRCADDRL  = (unsigned char)buf;

	// Set destination address
	FADDRH = (address >> 8) & 0xFF;
	FADDRL = address;


	// Arm DMA channel 0
	DMAARM |= DMAARM0;

	// seems to work without nops, in spite of cc2430 datasheet at 13.5.1 requiring 9 clocks between
	// arming a DMA channel and triggering the transfer
	// maybe calling FlashDmaTrigger function uses that many cycles?

// 	_asm
// 		nop
// 		nop
// 		nop
// 		nop
// 		nop
// 		nop
// 		nop
// 		nop
// 		nop
// 	_endasm;


	FlashDmaTrigger();

	// not stated in datasheet and works without it, use only if something breaks
	//while (!(DMAIRQ & 0x01));   // wait for DMA transfer

	// wait until Flash controller not busy
	// no need to check only one bit, because "continuous read enable" bit is zero
	while (FCTL)
		continue;


	// manually clear DMA Interrupt Flag 0. Otherwise a false interrupt can be generated.
	// if having space problems, the need for this statement can be reviewed (cc2430 datasheet at 13.5.5)
	// for single transfers, it is automatically cleared by hardware
	// DMAIRQ &= ~DMAIF0;  

}


void main( void )
{
// something needs to be done here if WATCHDOG is activated!!!
    EA=0;
    LED_INIT();
    LED2_OFF();
    LED1_OFF();
    
    // over- and underflow
    TCON &= ~TCON_RFERRIF;

    // only first two lines of DMAinit needed right now. Clean it up later!
 	DMAinit();

  	ListenToRadio();
}

// set LED1 to blink
void blink1(void)
{
    unsigned long c;

    while(1)
    {
	// about 1 second = 0x28000
	for(c = 0; c<0x28000; c++)
		continue;
	LED1_TOGGLE();
    }

}
// void blink2(void)
// {
//     unsigned long c;
// 
//     while(1)
//     {
// 	// about 1 second = 0x28000
// 	for(c = 0; c<0x28000; c++)
// 		continue;
// 	LED2_TOGGLE();
//     }
// 
// }

// erase a flash page
void erasePage(unsigned char pageNum)
{
	// Save interrupts state
	unsigned char EAstate = EA;
	// diasble interrupts
	EA=0;
	
	// does not erase itself
	if(pageNum >= LOCATION_PAGE && pageNum < LOCATION_PAGE + LOCATION_SIZE)
		return;
	// set page
	FADDRH = (pageNum<<1);
	
	// set flash timing, this should be later moved to an init function
	FWT = 0x2A;
	
	// erase flash page
	FCTL |= F_ERASE;
	
	// Erase must be followed by a NOP, when executed from FLASH
	_asm
	nop
	_endasm;
	
	// Restore previous state
	EA = EAstate;
}

// this contains the main loop of the program
void ListenToRadio()
{

   	MDMCTRL0H |= ADDR_DECODE;
  	MDMCTRL0L |= (AUTOCRC & AUTOACK);

	if(RFSTATUS & FIFO)
	{
		while (RFSTATUS & SFD)
			continue;
		rf_command(ISFLUSHRX);
		rf_command(ISFLUSHRX);
	}

#ifdef TIMING
	CLKCON |= TICKSPD0 & TICKSPD1 & TICKSPD2;
	T1CTL = 0x05; //start the clock
	T1CNTL = 0; //reset the timer, may not be needed, but I have no time to check that now
#endif

	// main loop
	while(1)
	{
#ifdef TIMING
		times[timescounter++] = 0xEE;
		T1CNTL = 0; //reset the timer
#endif
		pause(1);
#ifdef TIMING
		times[timescounter++] = T1CNTH;
		times[timescounter++] = T1CNTL;
#endif
		counter = 0;
		// wait for incoming data
		while (!(RFSTATUS & FIFO))
			continue;

		gen_pointer = &radiopacket;

		// wait while packet is received and buffered
		while (RFSTATUS & SFD)
			continue;	

		if(RFSTATUS & FIFO == 0)	// if(!(RFSTATUS & FIFO)) gives a different result
		{
			continue;
		}

		// read the packet to another buffer
		while (RFSTATUS & FIFO > 0)
		{
			*gen_pointer = RFD;
			gen_pointer++;
			counter++;
		}

		frame_length = radiopacket[0];
		
		// drop my own ACK packets, although I should not be able to read them. Bug?
		if(frame_length > 127 || frame_length < 0x21)
		{
			continue;
		}
		last_seq_no = radiopacket[3];
		
 		if(radiopacket[1] != 0x61 && radiopacket[1] != 0x41)
		{
 			continue;
		}

		// process the headers		
		addr_start = reverse_packet();

		// without this, you will fail!
		if(addr_start == 0)
		{
			continue;
		}
#ifdef TIMING
		if(timescounter>=HISTORY-1)
		{
			radiopacket[0] += timescounter + 2;	// packet size
			for(counter = 0; timescounter > 0; timescounter--)
				radiopacket[addr_start++]= times[counter++];
			pause_us(1);
			if(rf_write(&(radiopacket[1]), radiopacket[0]-2))
				LED1_TOGGLE();
			continue;
		}
#endif	
		

		data_start = addr_start + 4;
		data_length = frame_length - data_start -2 + 1;
		
		//get addr
		addr=(((unsigned long)radiopacket[addr_start])<<24) + (((unsigned long)radiopacket[addr_start+1])<<16) +
				(((unsigned long)radiopacket[addr_start+2])<<8) + ((unsigned long)radiopacket[addr_start+3]);
		
		// reset request?
		if(addr == 0xFFFFFFFF)
			reset();
		
		// read request?
		if(data_length == 1)
		{
			// somewhy, it does not work without a little rest
			//pause(1);
			readflash(addr, radiopacket[data_start]);
// 			rf_test();
		}
		// erase request
		else if(data_length == 0)
		{
			// erase page
			erasePage((unsigned char)(addr >> 11));
			CRC16(PAGE_SIZE);
			
			radiopacket[data_start]= RNDH;
			radiopacket[data_start+1]= RNDL;
			
			radiopacket[0] += 4 + 2 + 1;	// packet size
			pause(1);
			if(rf_write(&(radiopacket[1]), radiopacket[0]-2))
				LED1_TOGGLE();
		}
		else if(data_length > 1)
		{
			// write data and answer with checksum
			// for now, only checksum
			data_length--;

			WriteFlash(addr, &(radiopacket[data_start+1]), data_length);

			CRC16(data_length);
			
			radiopacket[data_start]= RNDH;
			radiopacket[data_start+1]= RNDL;
	
			radiopacket[0] += 4 + 2 + 1;	// packet size
			pause(1);
			if(rf_write(&(radiopacket[1]), radiopacket[0]-2))
				LED2_TOGGLE();

		}
		else
		{
		
// 			if(addr < 0x800 || addr >= 0x1000)
// 				continue;
// 			DMAinit();
// 			WriteFlash(addr, &(radiopacket[0]), 64);
// 			//WriteFlash(0x00000800, &RFD, 8);
// 			addr += 64;

// 			if(radiopacket[1] & 0x0F == 1)

			// somewhy, it does not work without a little rest
			pause(1);
// 			rf_test();
		}
// 		
	}

}


// send a short message to test connection and pass debugging data
// void rf_test(void)
// {
// //	radiopacket[36]= *gen_pointer;
// 	radiopacket[0]=0x26;
// 	radiopacket[1]=0x41;
// 	radiopacket[2]=0xCC;
// //	radiopacket[3]=0x13;
//  	radiopacket[3]++;
// //	radiopacket[3]=frame_sequence_number++;
// 	radiopacket[4]=0xFF;
// 	radiopacket[5]=0xFF;
// 
// // 	radiopacket[6]=0x00;
// // 	radiopacket[7]=0x15;
// // 	radiopacket[8]=0x20;
// // 	radiopacket[9]=0x00;
// // 	radiopacket[10]=0x00;
// // 	radiopacket[11]=0x02;
// // 	radiopacket[12]=0x0C;
// // 	radiopacket[13]=0x8D;
// 
// 	radiopacket[6]=0x00;
// 	radiopacket[7]=0x15;
// 	radiopacket[8]=0x20;
// 	radiopacket[9]=0x00;
// 	radiopacket[10]=0x00;
// 	radiopacket[11]=0x02;
// 	radiopacket[12]=0x0E;
// 	radiopacket[13]=0xD3;
// 
// 	radiopacket[14]=0x00;
// 	radiopacket[15]=0x15;
// 	radiopacket[16]=0x20;
// 	radiopacket[17]=0x00;
// 	radiopacket[18]=0x00;
// 	radiopacket[19]=0x02;
// 	radiopacket[20]=0x0E;
// 	radiopacket[21]=0x43;
// 	radiopacket[22]=0x42;
// 	radiopacket[23]=0xBF;
// 	radiopacket[24]=0x04;
// 	radiopacket[25]=0x16;
// 	radiopacket[26]=0x00;
// 	radiopacket[27]=0xFD;
// 	radiopacket[28]=0x00;
// 	radiopacket[29]=0xFD;
// 	radiopacket[30]=0x00;
// 	radiopacket[31]=0x00;
// 
// 	flash_address = (__xdata unsigned char * )0x800;
// 	CRC16(1);
// 
// 	radiopacket[32]= frame_length;
// 	radiopacket[33]= data_length;
// 	radiopacket[34]= data_start;
// 	radiopacket[35]= addr_start;
// 	radiopacket[36]= radiopacket[3];
// 
// // 	radiopacket[32]=0x54;
// // 	radiopacket[33]=0x65;
// // 	radiopacket[34]=0x73;
// // 	radiopacket[35]=0x74;
// // 	radiopacket[36]=0x21;
// 	radiopacket[37]=0x00;
// 	radiopacket[38]=0x00;
// 
// 	if(rf_write(&(radiopacket[1]), radiopacket[0]-2))
// 		LED2_TOGGLE();
// // 		blink2();
// 
// }


// copied from NanoStack v1.0.3 file rf.h and modified
char rf_write(unsigned char *buffer, unsigned char length)
{
	unsigned char counter, i;
	unsigned char retval = 1;
	
	if ( (RFSTATUS & FIFOP) || (RFSTATUS & SFD) )
	{
		if ((RFSTATUS & FIFOP))
		{
			// check cc2430 errata and repair this section and others like this
			rf_command(ISFLUSHTX);
			rf_command(ISFLUSHRX);
			rf_command(ISFLUSHRX);
		}
		retval = 0;		
	}
	
	rf_tx_enable();
	if ( (length <= 128) && (retval == 1) )
	{
		rf_command(ISFLUSHTX);

		// +2 means room for crc
		RFD = (length+2);

		for (i = 0 ; i < length ; i++)
		{
			RFD = *buffer++;
		}
		RFD = (0);
		RFD = (0);

		if (rf_cca_check(0,0) == 0)
		{
			rf_command(ISFLUSHTX);
			rf_rx_enable();
			return 2;
		}

		i= 0;
		RFIF &= ~IRQ_TXDONE;
		while (i++ < 3)
		{
			rf_command(ISTXON);
			counter = 0;	
			while(!(RFSTATUS & TX_ACTIVE) && (counter++ < 200))
			{
				pause_us(10);
			}
			if (RFSTATUS & TX_ACTIVE) i = 200;
		}

		if (i ==3)
		{
			rf_command(ISRFOFF);
			retval = 0;
		}
		else
		{
			while(!(RFIF & IRQ_TXDONE))
			{
				pause_us(10);
			}
		}
	}
	if ((retval == 1) && (length > 128))
	{

		retval = 0;
	}
	rf_rx_enable();

	return retval;		
}
// copied from NanoStack v1.0.3 file rf.h and modified
char rf_tx_enable(void)
{
	RFIM &= ~IRQ_SFD;
	IEN2 &= ~RFIE;
	return 1;
}
// copied from NanoStack v1.0.3 file rf.h and modified
char rf_cca_check(unsigned char backoff_count, unsigned char slotted)
{
	unsigned char retval = 1;
	unsigned long timer;

	rf_command(ISRXON);

	pause_us(64);

	if(!(RFSTATUS & CCA))
	{
		retval = 0;
	}
	return retval;
}
// copied from NanoStack v1.0.3 file rf.h and modified
void rf_command(unsigned char command)
{
	unsigned long timer;
	if (command >= 0xE0)
	{	//immediate strobe
		unsigned char fifo_count;
		switch (command)
		{	//hardware bug workaround
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
	else
	{
		RFST = command;	//write command
	}
}
// copied from NanoStack v1.0.3 file rf.h and modified
char rf_rx_enable(void)
{

	RFIF &= ~(IRQ_SFD);

	IOCFG0 = 0x04;   // Set the FIFOP threshold
	S1CON &= ~(RFIF_0 | RFIF_1);
	RFPWR &= ~RREG_RADIO_PD;	//make sure it's powered
	while((RFIF & IRQ_RREG_ON) == 0);	//wait for power up
	SLEEP &= ~OSC_PD; //Osc on
	while((SLEEP & XOSC_STB) == 0);	//wait for power up
	
	RFIM |= IRQ_SFD;
	RFIF &= ~(IRQ_SFD);

	S1CON &= ~(RFIF_0 | RFIF_1);
	IEN2 |= RFIE;
	rf_command(ISRXON);
	
	return 1;
}
// copied from NanoStack v1.0.3 file rf.h and modified
void pause_us(unsigned short time)
{
	unsigned short i;
	for (i = 0; i< time; i++)
	{
		_asm
			NOP
		_endasm;
	}
}
// copied from NanoStack v1.0.3 file rf.h and modified
void pause(unsigned short time)
{
	unsigned short i;
	for (i = 0; i< time; i++)
	{
		pause_us(1000);
	}
}

// Polynomial: x^16 + x^15 + x^2 + 1 (0xA001), initialized with 0x0000
void CRC16(unsigned char length)
{
	// seed the CRC16 calculator with 0x0000
	RNDL = 0x00;
	RNDL = 0x00;


	// length can not exceed flash size
// 	if(length > FLASH_SIZE)
// 		length = FLASH_SIZE;
	if(addr > FLASH_SIZE - length)
		return;
	
	MEMCTR = ((addr >> 15 & 3) << 4) | 0x01; //bank select
	code_pointer = 0;
	code_pointer += ((addr & 0x7FFF) | 0x8000);
	
	while(length)
	{
// 		// go over bank edge
// 		if(code_pointer == 0 && addr > 0)
// 		{
// 			code_pointer += BANK_SIZE;
// 			// next bank
// 			MEMCTR += 0x10;
// 		}
		RNDH = *(code_pointer++);
		length--;
	}
	MEMCTR = 0x01; //bank select
	
}

void readflash(unsigned long addr, unsigned char length)
{
//	check the parameters
	if(addr + length > FLASH_SIZE || length + radiopacket[0] <= radiopacket[0])
		return;
	

	MEMCTR = ((addr >> 15 & 3) << 4) | 0x01; //bank select
	code_pointer = 0;
	code_pointer += (addr & 0x7FFF | 0x8000);

	for(gen_pointer = &(radiopacket[radiopacket[0]+4]); gen_pointer < &(radiopacket[radiopacket[0]+4+length]); gen_pointer++)
	{
		*gen_pointer = *(code_pointer++);
	}
	
	radiopacket[0] += length + 4 + 1;	// packet size
	
	MEMCTR = 0x01; //bank select

	pause(1);

 	if(rf_write(&(radiopacket[1]), radiopacket[0]-2))
		LED2_TOGGLE();
}

// Makes a transmit packet out of the received packet
// Returns received data start location
// If unsupported packet, return zero
// Merge it with reception to make everything faster
// Note: data start location is the location where received data starts and also where you can write data to be sent
//	 This is because header length will not be changed, only addresses are swapped
// Note2: packet size is left untouched
uint8_t reverse_packet(void)
{
	// we want unsecure data frames
	if((radiopacket[1] & 0x0F) != 0x01)
		return 0;
	// we want addresses of non-zero length (does not check for reserved addresses)
	if((radiopacket[2] & 0x0C) == 0x00 || (radiopacket[2] & 0xC0) == 0x00)
		return 0;
	// increment frame sequence number
	radiopacket[3]++;
	// backup dest address
	for(counter = 0; counter < 10; counter++)
		dest[counter] = radiopacket[counter+4];
	// copy source PAN ID if PAN compression is disabled
	if((radiopacket[1] >> 6) == 0)
		dest_index = radiopacket[4];
	else
		dest_index = 6;
	// source address length
	if((radiopacket[2] & 0xC0) == 0xC0)
		src_index = dest_index + 8;
	else
		src_index = dest_index + 2;
	// copy source to destination
	for(counter = src_index - dest_index; counter > 0; counter--)
		radiopacket[dest_index++] = radiopacket[src_index++];
	// dest address length
	if((radiopacket[2] & 0x0C) == 0x0C)
		counter = 8;
	else
		counter = 2;

	src_index = (radiopacket[1] >> 5) & 0x02;
	// copy dest PAN ID if PAN compression is disabled, also copy dest
	while(counter)
	{
		radiopacket[dest_index++] = dest[src_index++];
		counter--;
	}
	
	// finally, swap addressing mode bits
	// this generates optimized code
	radiopacket[2] = ((radiopacket[2] << 4) | (radiopacket[2] >> 4));
		
	// MAC headers are done, move on to IP headers
	// accept only 6lowpan headers
	// first, check for a mesh header
	if((radiopacket[dest_index] >> 6) == 0x10)
	{
		// set Hops Left to 0x6
		radiopacket[dest_index] |= 0x60;
		// dest address length
		src_index = dest_index + 8 - 6*((radiopacket[dest_index] >> 2) & 1);
		// backup dest address
		for(counter = 0; counter < src_index - dest_index; counter++)
			dest[0] = radiopacket[dest_index + counter];
		// copy source to destination
		for(counter = 0; counter < 8 - 6*((radiopacket[dest_index] >> 1) & 1); counter++)
			radiopacket[dest_index++] = radiopacket[src_index++];
		// copy destination to source
		for(counter = 0; counter < 8 - 6*((radiopacket[dest_index] >> 2) & 1); counter++)
			radiopacket[dest_index++] = dest[counter];
	}
	// next, expect a compressed IPv6 header
	if((radiopacket[dest_index++]) != 0x42)
		return 0;
	// next, expect a compressed prefix and interface identifier, Traffic Class and Flow Label are zero, next header is UDP
	if((radiopacket[dest_index++]) != 0xBF)
		return 0;
	// expect both UDP ports to be non-compressed, ignore length
	if((radiopacket[dest_index++] & 0x03) != 0x00)
		return 0;
	
	// ignore Hop Limit
	dest_index++;
	
	// check if incoming port value is 0x00FD
	if(radiopacket[dest_index+2] != 0x00 || radiopacket[dest_index+3] != 0xFD)
		return 0;
	// swap UDP ports
	radiopacket[dest_index+2] = radiopacket[dest_index];
	radiopacket[dest_index+3] = radiopacket[dest_index + 1];
	radiopacket[dest_index] = 0x00;
	radiopacket[dest_index+1] = 0xFD;
	// move on
	dest_index += 4;
	
	// ignore checksum
	dest_index += 2;
	
	// finally, we are there!
	// store data start (= header size) in data length byte
	radiopacket[0] = dest_index;
	return dest_index;
}

// should use it?
void programflash(unsigned long addr, unsigned char length)
{
	
}


// reset using watchdog timer
void reset(void) _naked
{
	radiopacket[0] = frame_length;
	pause(1);
	if(rf_write(&(radiopacket[1]), radiopacket[0]-2))
	{
		LED2_TOGGLE();
	}
	else
	{
		_asm
			ret
		_endasm;
	}
	// enable watchdog timer
	WDCTL = WDT_EN;
	// and wait...
	while(1)
		continue;
	
}

