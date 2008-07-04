/*
	NanoStack: MCU software and PC tools for IP-based wireless sensor networking.

	Copyright (C) 2008 Intelligent Systems Group (ISG)
	Computer Engineering Laboratory, Department of Electrical and Information Engineering
	University of Oulu, Finland

	Read and write RFID tag information and communicate with nRoute (Initial version)

	Author:	Markus Paldanius

*/

/**	@file nReader.c
 *	The PC application to connect nRouted daemon and control RFID reader/writer functionality.
 *
 *	This file contains the functions to connect nRouted daemon, send and receive RFID data via
 *	6LoWPAN connection and print the available information on the screen. 
 */


/* Standard includes. */
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>

#ifndef LIBNRP_H
	#include "libnRP/libnRP.h"
#endif

typedef void (*sighandler_t)(int);

/* Socket definitions. */
int sockcmd;
int sockrd;
int sockwr;

void *buffer;

/* Print the user commands. */
void usage(void)
{
	printf(" R - Read a tag   ");
	printf("W - Write a tag   ");
	printf("Q - Quit\r\n\n");
}

/* Simple function to get time of the day. */
void get_time_string(char time_str[])
{
	struct timeval now;
 	struct tm* ptm;
	char str[64];

	gettimeofday(&now, NULL);
 	ptm = localtime (&now.tv_sec);
	
	/* Format time down to a single second. */
	strftime (str, sizeof (str), "%H:%M:%S", ptm);
	sprintf(time_str, "%s", str);
}

/* Terminates the program when Ctrl+C pressed. */
void sig_interrupt(void)
{
	printf("\nProgram closed.\n----------------------------------------------------------------\n");
	close(sockcmd);
	close(sockrd);
	close(sockwr);
	exit(1);
}

/* Main task. */
int main(void)
{
	struct sockaddr_in srvaddr;

	unsigned char sndbuf_conf[128];
	unsigned char sndbuf[128];
	unsigned char rbuf[256];

	int i = 1;
	int d = 0;

	unsigned char *txBUF;
	unsigned char *command;
	unsigned char RBUF[256];
	unsigned char len;
	unsigned char target[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	unsigned char DATA[128];

	unsigned char d_len = 0;
	unsigned char prot = 255;

	unsigned char s_addr[12];
	unsigned char s_type = 0;

	unsigned char d_addr[12];
	unsigned char d_type = 0;

	unsigned short int s_port = 254;
	unsigned short int d_port = 61619;

	unsigned short int siglvl = 0;
	unsigned short int seq = 0;

	unsigned short int NDEF = 0;
	unsigned short int start = 0;
	unsigned short int end = 0;
	unsigned short int err = 0;

	unsigned int payload_len = 0;
	unsigned int packet_len = 0;

	unsigned short int mode = 0;
	unsigned short int c = 0;
	unsigned short int answer = 0;

	char time_string[128];

	int rval;
	struct pollfd pfds;

	/* Print when program started. */
	printf("\n------------- nReader version 0.1 -------------\n\n");
	printf(" Turn the RFID reader ON and choose an action.\n\n\n");
	usage();

	/* Endless loop. */
	while(1)
	{
		mode = 0;
		c = 0;
		answer = 0;
		command[0] = 0;
		
		printf("Command>");

		/* Waits user commands given by standard input. */
		while (mode == 0 && answer == 0)
		{
			answer = getc(stdin);
		
			if (answer != '\n') do
			{
				c = getc(stdin);
			
			} while(c != '\n' && c != EOF);
		
			/* Set the mode based on the input command. */
			switch(answer)
			{
				/* Reader mode. */
				case 'r':
				case 'R':
					mode = 1;
				break;
		
				/* Writer mode. */
				case 'w':
				case 'W':
					mode = 2;
				break;
		
				/* Quit program. */
				case 'q':
				case 'Q':
       		close(sockcmd);
       		close(sockrd);
       		close(sockwr);
					printf("\nProgram closed.\n");
					printf("\nTurn also the RFID reader OFF.\n\n");
					return -1;
				break;
		
				/* Do nothing. */
				default:
					mode = 0;
					printf("\nNot a proper command!\n\n");
					usage();
					printf("Command>");
				break;
			}

			answer = 0;
			c = 0;
		}

		/* Connect to nRouted and send the command byte to RFID reader. */
		if ((mode == 1) || (mode == 2))
		{
			/* Open a TCP socket. */
			sockcmd = socket(AF_INET, SOCK_STREAM, 0);
			
			/* Prepare the struct sockaddr with the nRouted address. */
			bzero(&srvaddr, sizeof(srvaddr));
			srvaddr.sin_family = AF_INET;
			srvaddr.sin_port = htons(21870);
			inet_pton(AF_INET, "127.0.0.1", &srvaddr.sin_addr);

			/* Connect to nRouted. */
			if ((connect(sockcmd, (struct sockaddr *)&srvaddr, sizeof(srvaddr))) != 0)
			{
				printf("Connect to nRouted failed...\n");
				return(-1);
			}
			
			else
				
				/* Create a configuration packet. */
				if ((rval = libnrp_create_conf_pkt(sndbuf_conf, PROTO_6LOWPAN, NULL, NULL, ADDR_UNDEFINED, NULL, PORT_UNDEFINED)) == -1)
				{
					printf("Failed to create nRouted configuration packet.\n");
					return(-1);
				}
 
 			/* Send the configuration packet. */
			write(sockcmd, sndbuf_conf, 13);


			/* Read the configuration reply. */
			i = read(sockcmd, rbuf, 256);

			/* Check for errors. */
			if((rval = libnrp_check_nRoute_reply(rbuf, i)) != 1)
			{
				printf("nRouted configuration error.\n");
				return(-1);
			}

			/* These are needed for the TCP socket polling. */
			pfds.fd = (int)(sockcmd);
			pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;

			/* Set the command byte as the first byte in buffer. */
			command[0] = mode;
		
			/* Create the nRP data packet. */	
			if ((packet_len = libnrp_create_data_pkt_hdr(sndbuf, command, 1, PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &s_port, 61619, NULL)) == -1)
			{
				printf("Failed to create nRP data packet.\n");
				return(-1);
			}

			/* Send the data packet (command byte). */
			write(sockcmd, sndbuf, packet_len);
		
			/* Set the variables to their default values and zero buffers. */
			packet_len = 0;
			payload_len = 0;		
			bzero(sndbuf, 128);
			bzero(sndbuf_conf, 128);
			bzero(command, 2);
			close(sockcmd);
		}


		/* Enter reader mode. */
		if (mode == 1)
		{
			/* Open a TCP socket. */
			sockrd = socket(AF_INET, SOCK_STREAM, 0);
			
			/* Prepare the struct sockaddr with the nRouted address. */
			bzero(&srvaddr, sizeof(srvaddr));
			srvaddr.sin_family = AF_INET;
			srvaddr.sin_port = htons(21870);
			inet_pton(AF_INET, "127.0.0.1", &srvaddr.sin_addr);

			/* Connect to nRouted. */
			if ((connect(sockrd, (struct sockaddr *)&srvaddr, sizeof(srvaddr))) != 0)
			{
				printf("Connect to nRouted failed...\n");
				return(-1);
			}
			
			else
				
				/* Create a configuration packet. */
				if ((rval = libnrp_create_conf_pkt(sndbuf_conf, PROTO_6LOWPAN, NULL, NULL, ADDR_UNDEFINED, NULL, PORT_UNDEFINED)) == -1)
				{
					printf("Failed to create nRouted configuration packet.\n");
					return(-1);
				}
       
			/* Send the configuration packet. */
			write(sockrd, sndbuf_conf, 13);

			/* Read the configuration reply. */
			i = read(sockrd, rbuf, 256);

			/* Check for errors. */
			if ((rval = libnrp_check_nRoute_reply(rbuf, i)) != 1)
			{
				printf("nRouted configuration error.\n");
				return(-1);
			}

			/* These are needed for the TCP socket polling. */
			pfds.fd = (int)(sockrd);
			pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;

			printf("\nRFID reader is now ready to read a tag.\n\n");

			/* Read the socket. */
			len = read(sockrd, RBUF, 256);
			err = 0;

			/* Socket contains data. */
			if (len > 0)
			{
				/* Notification about successful reading. */
				get_time_string(time_string);
				printf("Reading successful at time %s.\n", time_string);
				libnrp_parse_nrp_hdr(RBUF, len, DATA, &d_len, &prot, s_addr, &s_type, d_addr, &d_type, &s_port, &d_port, &siglvl, &seq);

				/* Check the protocol field. */
				if (prot != 255)
				{
					if (prot == 0x02)
					{
						printf("\nProtocol: 6LoWPAN");
					}
					
					fflush(stdout);
				}

				/* Check the source MAC address field. */
				if (s_type != 0)
				{
					printf("\nSource MAC Address: %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x", 
					s_addr[0], s_addr[1], s_addr[2], s_addr[3], s_addr[4], s_addr[5], s_addr[6], s_addr[7]);
					fflush(stdout);
				}

				/* Check the destination MAC address field. */
				if (d_type != 0)
				{
					printf("\nDestination MAC Address: %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x", 
					d_addr[0], d_addr[1], d_addr[2], d_addr[3], d_addr[4], d_addr[5], d_addr[6], d_addr[7]);
					fflush(stdout);
				}

				/* Check the source port field. */
				if (s_port != 0)
				{
					printf("\nSource port: %u", s_port);
					fflush(stdout);
				}

				/* Check the destination port field. */
				if (d_port != 0)
				{
					printf("\nDestination port: %u", d_port);
					fflush(stdout);
				}
		
				printf("\r\n");

				/* Process the payload data. */
				if (d_len != 0)
				{
					/* Check the first byte for "UltraLight byte" (0x88). */
					if (DATA[0] == 0x88)
					{
						printf("\nTag type: Mifare UltraLight");
						printf("\nSerial number: ");
						
						/* Print the next 7 bytes (UltraLight serial number). */
						for (d = 1; d <= 7; d++)
						{
							printf("%2.2X", DATA[d]);
						}
					}
			
					/* Otherwise tha data is from Classic tag. */
					else
					{
						printf("\nTag type: Mifare Classic");
						printf("\nSerial number: ");
			
						/* Print the next 4 bytes (Classic serial number). */
						for (d = 0; d <= 3; d++)
						{
							printf("%2.2X", DATA[d]);
						}
					}
								
					/* Check for proper NDEF headers. */
					for (d = 4; d <= 55; d++)
					{
						/* There's a NDEF start mark. */
						if (DATA[d] == 0xE1)
						{
							/* There's a NDEF message indicator. */
							if (DATA[d+4] == 0x03)
							{
								NDEF = 1;
								printf("\nPayload length: %d bytes", DATA[d+8]);						
								
								/* Define payload's start and end points. */
								start = d + 10;
								end = start + DATA[d+8] - 1;
							}
					
						}

					}
			
					/* Data not in NDEF format. */
					if (NDEF != 1)
						printf("\n\nUnknown message format (NOT NDEF!).");
			
					else
						printf("\n\nNDEF message: ");
				
					/* Print the payload data. */
					for (d = start; d <= end && NDEF == 1; d++)
					{
						/* Check for proper ASCII characters. */
						if (((DATA[d] < 32) || (DATA[d] > 255)) && (NDEF == 1))
						{
							printf("\nReading error! Try again.");
							err = 1;
						}
					
						/* Print a payload byte. */
						if (err != 1)
							printf("%c", DATA[d]);
					
						/* Stop printing in case of improper ASCII character. */
						else
							break;
						
					}
				
					printf("\r\n\n");
				
				}

				/* Set the variables to their default values and zero buffers. */
				packet_len = 0;
				payload_len = 0;
				start = 0;
				end = 0;
				NDEF = 0;
				err = 0;
				rval = 0;
				mode = 0;
				siglvl = 0;
				seq = 0;

				bzero(sndbuf, 128);
				close(sockrd);

			}

		}

		/* Enter writer mode. */
		if (mode == 2)
		{
			/* Open a TCP socket. */
			sockwr = socket(AF_INET, SOCK_STREAM, 0);
			
			/* Prepare the struct sockaddr with the nRouted address. */
			bzero(&srvaddr, sizeof(srvaddr));
			srvaddr.sin_family = AF_INET;
			srvaddr.sin_port = htons(21870);
			inet_pton(AF_INET, "127.0.0.1", &srvaddr.sin_addr);

			/* Connect to nRouted. */
			if ((connect(sockwr, (struct sockaddr *)&srvaddr, sizeof(srvaddr))) != 0)
			{
				printf("Connect to nRouted failed...\n");
				return(-1);
			}

			else

				/* Create a configuration packet. */
				if ((rval = libnrp_create_conf_pkt(sndbuf_conf, PROTO_6LOWPAN, NULL, NULL, ADDR_UNDEFINED, NULL, PORT_UNDEFINED)) == -1)
				{
					printf("Failed to create nRouted configuration packet.\n");
					return(-1);
				}
 
 			/* Send the configuration packet. */     
			write(sockwr, sndbuf_conf, 13);

			/* Read the configuration reply. */
			i = read(sockwr, rbuf, 256);

			/* Check for errors. */
			if ((rval = libnrp_check_nRoute_reply(rbuf, i)) != 1)
			{
				printf("nRouted configuration error.\n");
				return(-1);
			}

			/* These are needed for the TCP socket polling. */
			pfds.fd = (int)(sockwr);
			pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;


			reread:

			printf("\nRFID reader is now ready to receive data.\n\n");
			printf("Data >");
			
			/* Scan the user inputted data and place it to buffer. */
			scanf("%[^\t\n]", txBUF);
			
			/* Determine the length of data. */
			payload_len = strlen(txBUF);

			/* To fit in Classic and UltraLight tag, the length of payload data must not exceed over 37 characters. */
			if (payload_len > 37)
			{
				printf("\nYou can't send more than 37 characters. Try again.\n\n");
				bzero(txBUF, 128);
				getc(stdin);
				goto reread;
			}
	
			/*Create the nRP data packet. */	
			if ((packet_len = libnrp_create_data_pkt_hdr(sndbuf, txBUF, payload_len, PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &s_port, 61619, NULL)) == -1)
			{
				printf("Failed to create nRP data packet.\n");
				return(-1);
			}

			/* Send the data packet (payload). */
			write(sockwr, sndbuf, packet_len);
		
			printf("\nRFID reader has received %d bytes data and is ready for writing a tag.\n\n", strlen(txBUF));

			/* Set the variables to their default values and zero buffers. */
			packet_len = 0;
			payload_len = 0;
			bzero(sndbuf, 128);
			bzero(txBUF, 128);
			getc(stdin);
			close(sockwr);
			
		}

		/* Zero the buffers so nothing is left to mess things up in the next read/write. */
		bzero(txBUF, 128);
		bzero(DATA, 128);
		bzero(rbuf, 256);
		bzero(RBUF, 256);
		
		/* Also set the variables to their default values. */
		s_port = 254;
		d_port = 61619;
		d_len = 0;
		prot = 255;
		s_type = 0;
		d_type = 0;
		mode = 0;
		start = 0;
		end = 0;
		NDEF = 0;
		err = 0;
		sockcmd = 0;
		siglvl = 0;
		seq = 0;

	} //while(1)

	/* Close all sockets. */
	close(sockcmd);
	close(sockrd);
	close(sockwr);

	return(1);

} //main
