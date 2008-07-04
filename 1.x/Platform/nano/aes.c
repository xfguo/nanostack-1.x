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
 * \file aes.c
 * \brief nano AES coprocessor features.
 *
 *  Nano: AES coprocessor functionality.
 *  Key initialization and buffer en/decryption.
 *	Uses CBC mode.   
 *	
 */

 

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

#include "debug.h"
#include "bus.h"
#include "gpio.h"

#include "progmem.h"
#include "buffer.h"
#include "aes.h"

#ifdef HAVE_AES

#ifndef AES_KEY
#define AES_KEY { 0x83, 0x84, 0x63, 0x56, 0x36, 0x76, 0x53, 0xF3, 0xA5, 0xB3, 0xD8, 0x3A, 0x23, 0x7B, 0x3F, 0x07 }
#endif

#ifndef AES_IV
#define AES_IV {  0x63, 0x07, 0x56, 0x83, 0x84, 0x53, 0xF3, 0xA5, 0xB3, 0x36, 0x76, 0x7B, 0x3F, 0xD8, 0x3A, 0x23 }
#endif

uint8_t aes_key[16] = AES_KEY;
uint8_t aes_iv[16] = AES_IV;

/**
 * Set AES key and initialization vector. 
 *
 * \param key pointer to key data (16 bytes), 0 to preserve old key
 * \param iv pointer to IV/nonce data (16 bytes), 0 to preserve old IV
 */
void aes_init(uint8_t *key, uint8_t *iv)
{
	if (key) memcpy(aes_key, key, 16);
	if (iv) memcpy(aes_iv, iv, 16);
	return;
}

/**
 * Encrypt/decrypt buffer. 
 *
 * \param mode 0=encrypt, 1=decrypt
 * \param buffer buffer to en/decrypt
 */
void aes_crypt(uint8_t mode, buffer_t *buffer)
{
	uint8_t *dptr = buffer->buf;
	uint16_t d_index;
	uint8_t i;
	
	/* TODO: check source/destination to find correct key
	 * required for per-host keys */
	if (mode) mode = 1;
	
	if (mode == 0)
	{	/*pad with pad count*/
		d_index = buffer->buf_end - buffer->buf_ptr;
		
		dptr += buffer->buf_end;
		i = 1;
		do
		{
			*dptr++ = i++;
			d_index++;
			buffer->buf_end++;
		}while (d_index & 0x0F);
	}
	
	ENCCS = (0x02 << 1) + 1; /* start key download */
	for (i=0; i<16; i++)
	{
		ENCDI = aes_key[i];
	}
	while (!(ENCCS & 0x08));
	
	ENCCS = (0x03 << 1) + 1; /* start IV download */
	for (i=0; i<16; i++)
	{
		ENCDI = aes_iv[i];
	}
	while (!(ENCCS & 0x08));
	
	d_index = buffer->buf_ptr;
	dptr = buffer->buf;
	dptr += d_index;
	
	while (d_index < buffer->buf_end)
	{
		ENCCS = (mode << 1) + 1; /* start en/decryption */

		for (i=0; i<16; i++)
		{
			ENCDI = *dptr++;
		}
		dptr -= 16;
		for (i=0; i<16; i++)
		{
			*dptr++ = ENCDO;
		}
		d_index += 16;
		while (!(ENCCS & 0x08));
	}
	if (mode == 1)
	{	/*restore length*/
		dptr--;
		if (*dptr <= 0x10)
		{	/*TODO: additional counter check*/
			buffer->buf_end -= *dptr;
		}
	}
}

#endif /*HAVE_AES*/
