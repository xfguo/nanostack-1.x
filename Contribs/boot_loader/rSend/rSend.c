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
/*
    rSend: send program to bootloader using NanoStack
		
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

/*
This application is based on Sensinode's SSI-Browser
*/

#define TIMING

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <math.h>

#include <unistd.h>

#include <strings.h>
#include <string.h>

#include <sys/poll.h>

#ifndef LIBNRP_H
#include "../libnRP/libnRP.h"
#endif

#ifndef LIBSSI_H
#include "../libSSI/libSSI.h"
#endif

#ifdef TIMING
#include <sys/time.h>
#endif

FILE * hexfile;
#define PAGESIZE 2048
#define PAGECOUNT 10
#define BANK_SIZE 0x8000
#define PACKET_FLASH_SIZE 0x58
#define MAX_RETRIES 24



#ifdef TIMING
	int times [10000];
	int timescounter = 0;
	int start_time, end_time;
	FILE * timefile;
	struct timeval tv;
	struct timezone tz;
#endif

unsigned char flash[PAGECOUNT*PAGESIZE];
unsigned char page_changed[PAGECOUNT];
unsigned short dataChecksum;

unsigned char parse_hex(void);
unsigned char create_hex(void);
void dataCRC16(unsigned char byte); // initialize with 0x0000
unsigned char read_flash(void);
int write_flash(int);
void reset(void);

int sockfd;

unsigned char sndbuf[128];

int packet_len = 0;
unsigned char SSI_request[128];
unsigned char sndbuf_conf[128];
unsigned char buffer[512];
unsigned char rbuf[512];
unsigned char flash_data[128];
int recvbytes;
unsigned int bytes;

unsigned char data[] = {0x00, 0x00, 0x08, 0x04, 'T','e','s','t',' ','d','a','t','a', 9};

int i = 1;

char nRd_address[32] = "127.0.0.1";
unsigned int nRd_port = 21870;
unsigned char target[8];
char argbuf[64];
int hits = 0;

struct pollfd pfds;
unsigned int nfds = 1;
int timed_out = 0;

int rval = 0;
int loop = 1;

unsigned short int portn;

struct jobs
{
	unsigned char write;
	unsigned char read;
	unsigned char verify;
}todo_list;

void usage(void)
{
		printf("rSend version 0.1\n\n");
		printf("USAGE: ./rSend [ADDR] [FILENAME.HEX]\n");
		printf("	Options:\n");
		printf("	-S [nRouted_address:port]	- The address and port number where the nRouted TCP server is located.\n");
		printf("								  (for example  -S 192.168.0.145:13022 )\n");
		printf("	[ADDR] 			- the 802.15.4 MAC address of the target.\n");
		printf("	[FILENAME.HEX] 		- the file containing program (in the Intel Hex format) to be loaded into\n");
		printf("					 the target device\n\n");
}

int main(int argc, char **argv)
{

	// clear TODO list
	todo_list.write = 0;
	todo_list.read = 0;
	todo_list.verify = 0;

/*	First check the command line parameters and parse them. */
	if(argc < 2 || argc > 4 || strncmp(argv[1], "--help", 6) == 0)
	{
		usage();
		return(-1);
	}

	while(i < argc)
	{
		sscanf(argv[i], "%s ", argbuf);
		if(strncmp(argbuf, "-S", 2) == 0)
		{
			i++;
			hits = sscanf(argv[i], "%[^:]:%d", nRd_address, &nRd_port);
			if(hits == 2)
				printf("Trying to connect nRouted at %s port %d\n", nRd_address, nRd_port);

			bzero(argbuf, 64);
		}
		else if (strncmp(argbuf, "-w", 2) == 0)
		{
			todo_list.write = 1;
			i++;
			hexfile = fopen(argv[i], "r");
			if(hexfile)
			{
				printf("Writing data from %s to FLASH\n", argv[i]);
				if(parse_hex())
				{
					fclose(hexfile);
				}
				else
				{
					printf("Invalid HEX file: %s\n", argv[i]);
					fclose(hexfile);
					return (-1);
				}
			}
			else
			{
				printf("Failed to open file %s\n", argv[i]);
				return (-1);
			}

			bzero(argbuf, 64);
		}
		else if (strncmp(argbuf, "-r", 2) == 0)
		{
			if(todo_list.write)
			{
				printf("Select only one of the operations: read, write, verify\n");
				return (-1);
			}
			todo_list.read = 1;
			i++;
			hexfile = fopen(argv[i], "w");
			if(hexfile)
			{
				printf("Writing data from FLASH to %s\n", argv[i]);
			}
			else
			{
				printf("Failed to open file %s\n", argv[i]);
				return (-1);
			}

			bzero(argbuf, 64);
		}
		else if (strncmp(argbuf, "-l", 2) == 0)
		{
			printf("Loop mode.\n");
			loop = 20;
		}
		else
		{
			hits = sscanf(argv[i], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &(target[0]), &(target[1]), &(target[2]), &(target[3]), &(target[4]), &(target[5]), &(target[6]), &(target[7]));

			printf("Requesting SSI data from: %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\n", target[0], target[1], target[2], target[3], target[4], target[5], target[6], target[7]);
		}
		
		bzero(argbuf, 64);
		i++;
	}

	printf("Trying to connect nRouted at %s port %d\n", nRd_address, nRd_port);

/*	Copying the Target address to the end of the nRouted Configuration packet, discovery packet and SSI request packet. */
	for(i=0;i<8;i++)
	{
		sndbuf_conf[i+12] = target[i];
		sndbuf[i+18] = target[i];
		SSI_request[i+12] = target[i];
	}

//	if((rval=libnrp_create_conf_pkt(sndbuf_conf, PROTO_UNDEFINED, target, NULL, ADDR_UNDEFINED, NULL, PORT_UNDEFINED)) < 0)
	if((rval=libnrp_create_conf_pkt(sndbuf_conf, PROTO_6LOWPAN, NULL, NULL, ADDR_UNDEFINED, NULL, PORT_UNDEFINED)) < 0)
	{
		printf("Failed to create an nRouted configuration packet.\n");
		return(-1);
	}
	
	if((sockfd = libnrp_snd_conf(sndbuf_conf, rval, nRd_address, nRd_port)) < 0)
	{
		printf("Failed to send the configuration packet to nRouted.\n");
		return(-1);
	}
	else
	{
		printf("nRouted configuration successful.\n");
	}

	if(todo_list.write)
	{
		int errors;
		int page = 0;
		for(errors = 0; errors < 10; errors++)
		{
			page = write_flash(page);
			if(page > 0)
			{
				printf("Flash write complete\n");
				reset();
				break;	
			}
			else
			{
				int errors2;
				printf("Flash write FAILED!\n");
				if((rval=libnrp_create_conf_pkt(sndbuf_conf, PROTO_6LOWPAN, NULL, NULL, ADDR_UNDEFINED, NULL, PORT_UNDEFINED)) < 0)
				{
					printf("Failed to create an nRouted configuration packet.\n");
					return(-1);
				}
				for(errors2 = 0; errors2 < 10; errors2++)	
				{
					if((sockfd = libnrp_snd_conf(sndbuf_conf, rval, nRd_address, nRd_port)) < 0)
					{
						printf("Failed to send the configuration packet to nRouted.\n");
					}
					else
					{
						printf("nRouted configuration successful.\n");
						break;
					}
				}
				page *= -1;
			}
		}
	}
	else if(todo_list.read)
	{
		if(read_flash())
		{
			printf("Flash read complete\n");
			if(create_hex())
			{
				printf("Flash data written to file\n");
				reset();
				return 1;
			}
			else
			{
				printf("Error writing to file");
				reset();
				return (-1);
			}
		}
		else
		{
			printf("Flash read FAILED!\n");
			reset();
			return (-1);
		}
	}
	else if(todo_list.verify)
	{
	
	}


	// Create test packet
	portn = 252;
	packet_len = libnrp_create_data_pkt_hdr(sndbuf, data, sizeof(data), PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &portn, 253, NULL);


	while(loop)
	{
		loop--;

	/*	Here we send the SSI discovery packet to the sensor device. */
		write(sockfd, sndbuf, packet_len);
	
		pfds.fd = (int)(sockfd);
		pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;
	
	/*	We try to read the answer and then process it. */
		i=0;
		timed_out = 0;
		while(!timed_out)
		{
			if((rval = poll(&pfds, nfds, 1000)) == 0)
			{
				printf("Timed out SSI discovery reply.\n");
				timed_out = -1;
			}
			else
			{
				recvbytes = read(sockfd, rbuf, 512);
				if(recvbytes>0)
				{
					printf("Received %d bytes ", recvbytes);
	
					libnrp_get_data(rbuf, recvbytes, buffer, &bytes);
					buffer[bytes]='\0';
					printf("\n%d data bytes: %s\n", bytes, buffer);

				}
			}
		}

	}

	close(sockfd);

	return(1);
}


unsigned char parse_hex(void)
{
	int i, line = 0;
	unsigned long  addr_base = 0, c;
	int endline = 0;
	
	// init flash image
	memset(&(flash[0]), 0xFF, sizeof(flash));
	memset(&(page_changed[0]), 0, sizeof(page_changed)); 
 
	while ((c=getc(hexfile))!=EOF && !endline)
	{
		if (c == ':')   /* Sync with beginning of line */
		{
			int n, check;
			unsigned char sum;
			unsigned long addr;
			int linetype;
			
			line++;
			
			// length
			if (fscanf(hexfile, "%2x", &n) != 1)
			{
				printf("Error in HEX file on line %d\n", line);
				return 0;
			}
			sum = n;
			//printf("length=%.2X\n", n);
			// address
			if (fscanf(hexfile, "%4lx", &addr) != 1)
			{
				printf("Error in HEX file on line %d\n", line);
				return 0;
			}
			sum += addr/256; //high byte
			sum += addr%256; //low byte
			//printf("addr=%.4hX\n", (unsigned short) addr);
			// line type
			if (fscanf(hexfile, "%2x", &linetype) != 1)
			{
				printf("Error in HEX file on line %d\n", line);
				return 0;
			}
			sum += linetype;
			//printf("linetype=%.2X\n", linetype);
			// data record
			if (linetype == 0)
			{
				addr = addr_base + (addr & 0x0000FFFF);;
				
				for (i=0;i<n;i++)
				{
					if (fscanf(hexfile, "%2lx", &c) != 1)
					{
						printf("Error in HEX file on line %d\n", line);
						return 0;
					}
					if (addr >= PAGECOUNT*PAGESIZE)
					{
						printf("Address on line %d is too big%lX,%lX\n", line, addr_base, addr);
						return 0;
					}
					
					page_changed[addr / PAGESIZE] = 1;
					
					flash[addr++] = c;
					
					sum += c;
				}
			}
			// End Of File record
			else if(linetype == 1)
			{
				endline = 1;
			}
			// Extended Segment Address Record
			else if(linetype == 2)
			{
				if(addr != 0 || n != 2)
				{
					printf("Invalid Extended Segment Address Record on line %d\n", line);
					return 0;
				}
				if (fscanf(hexfile, "%4lx", &c) != 1)
				{
					printf("Error in HEX file on line %d\n", line);
					return 0;
				}
				if(c & 0x0F)
				{
					printf("Invalid Extended Segment Address on line %d\n", line);
					return 0;
				}
				
				addr_base = c << 4;
				sum += c << 8;
				sum += c;
				
			}
			// Extended Linear Address Record
			// actually it chooses the memory bank
			else if(linetype == 4)
			{
				if(!(addr == 0 && n == 2))
				{
					printf("Invalid Extended Linear Address Record on line %d\n", line);
					return 0;
				}
				if (fscanf(hexfile, "%4lx", &c) != 1)
				{
					printf("Error in HEX file on line %d\n", line);
					return 0;
				}
				sum += c << 8;
				sum += c;
				if(c)
					c--;
				addr_base = c * BANK_SIZE;
			}
			// Skip this record
			else if(linetype == 5)
			{
				if(!(addr == 0 && n == 4))
				{
					printf("Invalid Start Linear Address Record on line %d\n", line);
					return 0;
				}
				if (fscanf(hexfile, "%8lx", &c) != 1)
				{
					printf("Error in HEX file on line %d\n", line);
					return 0;
				}
				printf("Skip line %d (record type 05)\n", line);
			}
			else
			{
				printf("Unsupported record type on line %d: %2X\n", line, linetype);
				return 0;
			}
			
			// checksum
			if (fscanf(hexfile, "%2x", &check) != 1)
			{
				printf("Error in HEX file on line %d\n", line);
				return 0;
			}
			
			sum = ~sum + 1;
			if (check != sum)
			{
				printf("Checksum mismatch on line %d (found %.2X instead of %.2X)\n", line, check, sum);
				return 0;
			}
		}
	}
	
	return 1;
}

unsigned char create_hex(void)
{
	int done = 0;
	unsigned long address = 0;
	unsigned long address_base = 0;
	while(!done)
	{
		int byte_num;
		unsigned char sum;
		// do not output empty pages
		// comment out this 'if' sentence
		if(!page_changed[address/PAGESIZE])
		{
			address += PAGESIZE;
			continue;
		}
		// this only works if flash size is divisible by 16
		if(address + 16 > sizeof(flash))
		{
			done = 1;
		}
		else
		{
			// Extended Segment Address Record
			// only allow flash sizes up to 1MB
			// do not use Extended Lineat Address Record, because SDCC outputs it a bit differently
			// nano_programmer interprets both the SDCC way, but outputs the Wikipedia way
			if(address_base != (address & 0xFFFF0000))
			{
				sum = 2;
				sum += 2;
				address_base = address & 0xFFFF0000;
				//sum += address_base >> 12;
				sum += address_base >> 4;
				sum ^= 0xFF;
				sum++;
				
				fprintf(hexfile, ":02000002%.4hX%.2X\n", (unsigned short) (address_base >> 4), sum);
			}
			sum = 16;
			sum += address >> 8;
			sum += address;
			fprintf(hexfile, ":10%.4hX00", (unsigned short) address);
			for(byte_num = 0; byte_num < 16; byte_num++)
			{
				sum += flash[address+byte_num];
				fprintf(hexfile, "%.2X", flash[address+byte_num]);
			}
			sum ^= 0xFF;
			sum++;
			fprintf(hexfile, "%.2X\n", sum);
			address += 16;
		}
	}
	fprintf(hexfile, ":00000001FF\n");
	return 1;
}

// Polynomial: x^16 + x^15 + x^2 + 1 (0x8005)
void dataCRC16(unsigned char byte) // initialize with 0x0000
{
	unsigned char i;
	dataChecksum ^= byte << 8;
	
	for (i = 0; i < 8; ++i)
	{
	    if (dataChecksum & 0x8000)
			dataChecksum = (dataChecksum << 1) ^ 0x8005;
	    else
			dataChecksum = (dataChecksum << 1);
	}
} // dataCRC16

unsigned char read_flash(void)
{
	unsigned long int flash_addr;
	unsigned char errorcount = 0;
	// size = 4 bytes of read start address, 1 byte of read length
	unsigned char req[5] = {0x00, 0x00, 0x00, 0x00, PACKET_FLASH_SIZE};
	unsigned long int read_size = PACKET_FLASH_SIZE;
	
	// init flash image
	memset(&(flash[0]), 0xFF, sizeof(flash));
	memset(&(page_changed[0]), 0, sizeof(page_changed)); 
	
	portn = 253;	// node listens on this port

#ifdef TIMING
	gettimeofday(&tv, &tz);
	start_time = tv.tv_usec;
#endif
	for(flash_addr = 0; flash_addr < PAGECOUNT*PAGESIZE; flash_addr+=read_size)
	{
// 		if(flash_addr > PACKET_FLASH_SIZE*128)
// 		{
// // 			for(i=0; i<256; i++)
// // 				printf("%.2X ", flash[i]);
// 			return 1;
// 		}
#ifdef TIMING
		gettimeofday(&tv, &tz);
		end_time = tv.tv_usec;
		if(end_time < start_time)
			end_time += 1000000;
		times[timescounter++] = (end_time - start_time);
		start_time = tv.tv_usec;
#endif
		if((flash_addr % PAGESIZE) == 0)
			printf("Reading page %lX\n", (flash_addr/PAGESIZE));
		
		// do not read over bank edge, because boot loader does not support it yet
		read_size = BANK_SIZE - (flash_addr % BANK_SIZE);
		// if last page to read, but not a bank edge, limit the size, too
		if(flash_addr / PAGESIZE == PAGECOUNT - 1)
			read_size = PAGESIZE - (flash_addr % PAGESIZE);
		
		if(read_size > PACKET_FLASH_SIZE)
			read_size = PACKET_FLASH_SIZE;
		
		req[4] = read_size;
		
		// Create request packet
		req[0] = flash_addr >> 24;
		req[1] = flash_addr >> 16;
		req[2] = flash_addr >> 8;
		req[3] = flash_addr;
		packet_len = libnrp_create_data_pkt_hdr(sndbuf, req, sizeof(req), PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &portn, 253, NULL);


		// send the packet
		write(sockfd, sndbuf, packet_len);
	
		pfds.fd = (int)(sockfd);
		pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;
	
		// read the answer
		i=0;
		timed_out = 0;
		while(!timed_out)
		{
			if((rval = poll(&pfds, nfds, 800)) == 0)
			{
				printf("Timed out reading address %lX...\n", flash_addr);
				errorcount++;
				// try again
				if(errorcount > MAX_RETRIES)
				{
					printf("Reading aborted at address %8lX: too many errors\n", flash_addr);
					return 0;
				}
				flash_addr-=read_size;
				timed_out = -1;
			}
			else
			{
				recvbytes = read(sockfd, rbuf, 512);
				if(recvbytes > 0)
				{
					libnrp_get_data(rbuf, recvbytes, buffer, &bytes);
// 					for(i=0; i<bytes; i++)
// 						printf("%.2X ", buffer[i]);
					if(bytes != read_size + 4)
					{
						printf("Unexpected data length: %d\n", bytes);
						for(i=0; i<bytes; i++)
							printf("%.2X ", buffer[i]);
						errorcount++;
					}
					else
					{
						if(!(buffer[0] == req[0] && buffer[1] == req[1] && buffer[2] == req[2] && buffer[3] == req[3]))
						{
							printf("Unexpected flash address: 0x%.2X%.2X%.2X%.2X\n", buffer[0], buffer[1], buffer[2], buffer[3]);
							errorcount++;
						}
						else
						{
							for(i = 0; i < read_size; i++)
								flash[flash_addr + i] = buffer[i + 4];
							
							page_changed[flash_addr/PAGESIZE] = 1;
							errorcount = 0;
							break;
						}
					}

				}
				else
				{
					errorcount++;
				}
			}
		}

	}
#ifdef TIMING
	timefile = fopen("readtimes.txt", "w");
	while (timescounter > 0)
		fprintf(timefile, "%d\n", times[timescounter--]);
	fclose(timefile);
#endif
	return 1;
}

int write_flash(int page)
{
	unsigned long int flash_addr;
	unsigned char errorcount = 0;
	unsigned char erase = 0;
	// size = 4 bytes of write start address, 1 byte of write length, PACKET_FLASH_SIZE of data
	unsigned char req[PACKET_FLASH_SIZE+5] = {0x00, 0x00, 0x00, 0x00, PACKET_FLASH_SIZE};
	unsigned long int write_size = PACKET_FLASH_SIZE;
		
	portn = 253;	// node listens on this port

#ifdef TIMING
	gettimeofday(&tv, &tz);
	start_time = tv.tv_usec;
#endif
	
	for(flash_addr = page*PAGESIZE; flash_addr < PAGECOUNT*PAGESIZE; flash_addr+=write_size)
	{
#ifdef TIMING
	gettimeofday(&tv, &tz);
	end_time = tv.tv_usec;
	if(end_time < start_time)
		end_time += 1000000;
	times[timescounter++] = (end_time - start_time);
	start_time = tv.tv_usec;
#endif
		// skip unmodified pages
		// this behaviour may be unwanted, because one might want to fill a page with 0xFF
		if(page_changed[flash_addr/PAGESIZE] == 0)
		{
			flash_addr += PAGESIZE - PACKET_FLASH_SIZE;
			continue;
		}
		
		// one packet can not write over page borders
		write_size = PAGESIZE - (flash_addr % PAGESIZE);
		if(write_size > PACKET_FLASH_SIZE)
			write_size = PACKET_FLASH_SIZE;
		
		// Change address
		req[0] = flash_addr >> 24;
		req[1] = flash_addr >> 16;
		req[2] = flash_addr >> 8;
		req[3] = flash_addr;
		
		// reset CRC16
		dataChecksum = 0x0000;
		
		if((flash_addr % PAGESIZE) == 0)
		{
			if(erase == 0)
				erase = 1;
			else
				erase = 0;
		}
		if(erase)
		{
			printf("Erasing page %lX\n", (flash_addr/PAGESIZE + erase - 1));
			
			// erase the page first
			// data size of 0 means an erase command
// 			req[4] = PACKET_FLASH_SIZE;
			
			// create packet
			packet_len = libnrp_create_data_pkt_hdr(sndbuf, req, 4, PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &portn, 253, NULL);
			
		}
		else
		{
			if((flash_addr % PAGESIZE) == 0)
				printf("Writing page %lX\n", (flash_addr/PAGESIZE));
			// set data size
			req[4] = write_size;
			
			// add data and calculte CRC16
			for(i = 0; i < write_size; i++)
			{
				req[5+i] = flash[flash_addr+i];
				dataCRC16(flash[flash_addr+i]);
			}
			
			// create packet
			packet_len = libnrp_create_data_pkt_hdr(sndbuf, req, 5+write_size, PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &portn, 253, NULL);
		}

		// send the packet
		write(sockfd, sndbuf, packet_len);
	
		// set polling properties
		pfds.fd = (int)(sockfd);
		pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;
	
		// read the answer
		i=0;
		timed_out = 0;
		while(!timed_out)
		{
			// try again
			if(errorcount > MAX_RETRIES)
			{
				printf("Writing aborted at address %8lX: too many errors\n", flash_addr);
				return -(flash_addr/PAGESIZE);
			}
			
			if((rval = poll(&pfds, nfds, 800)) == 0)
			{
				if(erase)
				{
					printf("Timed out erasing flash page %lX...\n", (flash_addr/PAGESIZE));
					erase = 0;
				}
				else
					printf("Timed out writing to address %lX...\n", flash_addr);
				errorcount++;

				flash_addr -= write_size;
				timed_out = -1;
			}
			else
			{
				recvbytes = read(sockfd, rbuf, 512);
				if(recvbytes > 0)
				{
					libnrp_get_data(rbuf, recvbytes, buffer, &bytes);
					// answer = 4 address bytes and 2 CRC16 bytes
					if(bytes != 6)
					{
						printf("Unexpected data length: %d\n", bytes);
						for(i=0; i<bytes; i++)
							printf("%.2X ", buffer[i]);
						errorcount++;
					}
					else
					{
						if(!(buffer[0] == req[0] && buffer[1] == req[1] && buffer[2] == req[2] && buffer[3] == req[3]))
						{
							printf("Unexpected flash address: 0x%.2X%.2X%.2X%.2X\n", buffer[0], buffer[1], buffer[2], buffer[3]);
							errorcount++;
						}
						else
						{
							if(erase)
							{
								if(buffer[4] != 0x00 || buffer[5] != 0x00)
								{
									printf("Erasing page %lX failed!\n", (flash_addr/PAGESIZE));
									erase = 0;
									errorcount++;
								}
								else
									errorcount = 0;
								
								flash_addr -= write_size;
								break;
							}
							else
							{
								if(buffer[4] != (unsigned char)(dataChecksum >> 8) || buffer[5] != (unsigned char) dataChecksum)
								{
									printf("Writing to address %lX failed! (got checksum %X%X instead of %hX)\n", flash_addr, buffer[4], buffer[5], dataChecksum);
									errorcount++;
									flash_addr -= write_size;
								}
								else
									errorcount = 0;
								
								break;
							}
						}
					}

				}
				else
				{
					errorcount++;
				}
			}
		}

	}
#ifdef TIMING
	timefile = fopen("writetimes.txt", "w");
	while (timescounter > 0)
		fprintf(timefile, "%d\n", times[timescounter--]);
	fclose(timefile);
#endif
	return 1;
	
}

void reset(void)
{
	unsigned char errorcount = 0;
	// size = 4 bytes of 0xFF
	unsigned char req[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	
	portn = 253;	// node listens on this port
	
	// create packet
	packet_len = libnrp_create_data_pkt_hdr(sndbuf, req, 4, PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &portn, 253, NULL);
	// send the packet
	write(sockfd, sndbuf, packet_len);

	// set polling properties
	pfds.fd = (int)(sockfd);
	pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;

	// read the answer
	i=0;
	timed_out = 0;
	while(!timed_out)
	{
		// try again
		if(errorcount > MAX_RETRIES)
		{
			printf("Reset aborted: too many errors\n");
			return;
		}
		
		if((rval = poll(&pfds, nfds, 1500)) == 0)
		{
			printf("Reset timed out...\n");
			errorcount++;
		}
		else
		{
			recvbytes = read(sockfd, rbuf, 512);
			if(recvbytes > 0)
			{
				libnrp_get_data(rbuf, recvbytes, buffer, &bytes);
				// answer = 4 bytes of 0xFF
				if(bytes != 4)
				{
					printf("Unexpected data length: %d\n", bytes);
					for(i=0; i<bytes; i++)
						printf("%.2X ", buffer[i]);
					errorcount++;
				}
				else
				{
					if(!(buffer[0] == req[0] && buffer[1] == req[1] && buffer[2] == req[2] && buffer[3] == req[3]))
					{
						printf("Unexpected flash address: 0x%.2X%.2X%.2X%.2X\n", buffer[0], buffer[1], buffer[2], buffer[3]);
						errorcount++;
					}
					else
					{
						printf("Reset succeeded!\n");
						break;
					}
				}
			}
			else
			{
				errorcount++;
			}
		}
	}
}
