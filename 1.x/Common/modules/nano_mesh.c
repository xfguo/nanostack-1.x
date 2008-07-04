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
 * \file     nano_mesh.c
 * \brief    NanoMesh protocol module.
 *
 *  Module includes NanoMesh forwarding technic... .
 */



#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#undef HAVE_DEBUG
#include "debug.h"
#include "socket.h"
#include "control_message.h"
#include "neighbor_routing_table.h"
#include "cipv6.h"
#include "gpio.h"

/*
[NAME]
NANOMESH

[ID]
MODULE_NANOMESH,

[INFO]
#ifdef HAVE_NANOMESH
  {nanomesh_init, nanomesh_handle, nanomesh_check, 0, MODULE_NANOMESH, 16, ADDR_NONE, 0 },
#endif

[FUNCS]*/
extern portCHAR nanomesh_init(buffer_t *buf);
extern portCHAR nanomesh_handle( buffer_t *buf );
extern portCHAR nanomesh_check( buffer_t *buf );

extern namomesh_info_t namomesh_info;

/**
 *  Initialize NanoMesh module.
 *
 *  \return  pdTRUE    OK
 */
portCHAR nanomesh_init( buffer_t *buf )
{
	buf;
	return pdTRUE;
}

/**
 *  Main NanoMesh buffer handler.
 *
 *	\param buf pointer to buffer
 *  \return  pdTRUE    OK
 */
portCHAR nanomesh_handle( buffer_t *buf )
{
	uint8_t *dptr;
	dest_delivery_t table_check;
	uint8_t i=0, dest_length;
	nano_mesh_response_t nanomesh_delivery = NANOMESH_NO_ROUTE;
	uint8_t mesh_header=MESH_ROUTING_TYPE ,header_size=0;
	control_message_t *ptr=0;

	dptr=0;
	//debug("IP:Build\r\n");
#ifdef HAVE_ROUTING
		if(buf->dst_sa.addr_type == ADDR_802_15_4_LONG ||buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
		{
			buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;			
			mesh_header |= D_ADDRESSTYPE_64;
			header_size +=8;
		}
		else if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
		{
			mesh_header |= D_ADDRESSTYPE_16;
			header_size +=2;
		}
		table_check = check_neighbour_table(&(buf->dst_sa));
		if(table_check == NEIGHBOR)
		{
			nanomesh_delivery = NANOMESH_NEIGHBOUR;
		}
		else if(table_check==NEIGHBOR_LOW_RSSI || table_check==NOT_NEIGHBOR)
		{
			memcpy(&(namomesh_info.adr),&(buf->dst_sa), sizeof(sockaddr_t) );
			if(check_routing_table(&namomesh_info ) == pdTRUE)
			{
					nanomesh_delivery = NANOMESH_FORWARD;
			}
			else
			{
				if(table_check == NEIGHBOR_LOW_RSSI)
					nanomesh_delivery = NANOMESH_NEIGHBOUR;
				else
				{
					nanomesh_delivery = NANOMESH_NO_ROUTE;
#ifdef AD_HOC_STATE
					if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
					{
						buf->dst_sa.address[8] = 0xff;
						buf->dst_sa.address[9] = 0xff;
					}
#endif
				}
			}
		}
#else
		if(check_neighbour_table(&(buf->dst_sa)) == NOT_NEIGHBOR)
		{
			if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
			{
				buf->dst_sa.address[8] = 0xff;
				buf->dst_sa.address[9] = 0xff;
			}
		}
		nanomesh_delivery = NANOMESH_NEIGHBOUR;
#endif

if(buf->dir == BUFFER_DOWN)
{
#ifdef HAVE_ROUTING
	if(nanomesh_delivery == NANOMESH_FORWARD)
	{
		debug("MESH\r\n");
		header_size++;		/* Mesh options */
		if(buf->src_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
		{
			dest_length=2;
			header_size+=2;
		}
		else
		{
			header_size+=8;
			dest_length=8;
		}
		mesh_header |= BASIC_HOP_VALUE;

		if(stack_buffer_headroom(buf,header_size) == pdFALSE)
		{
			ptr = ( control_message_t*) buf->buf;
			ptr->message.ip_control.message_id = TOO_LONG_PACKET;
			push_to_app(buf);
			buf=0;
			return pdTRUE;
		}
		buf->buf_ptr -= header_size; /* lowpan-dispatch+HC1 = header space */
		dptr = buffer_data_pointer(buf);
		/* Mesh frame type */
		*dptr++ = mesh_header;
		/* Build Mesh delivery address field*/
		/* Originator address */

		for(i=0; i<dest_length ;i++)
		{
			*dptr++ = buf->src_sa.address[i];
		}

		if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
			dest_length=2;
		else
			dest_length=8;

		for(i=0; i<dest_length ;i++)
		{
			*dptr++ = buf->dst_sa.address[i];
		}
		/* Change next-hop address to rf_802_15_4 dest_address */
		for(i=0; i<10 ;i++)
		{
			buf->dst_sa.address[i] = namomesh_info.adr2.address[i];
		}
		buf->dst_sa.addr_type = namomesh_info.adr2.addr_type;
	}
#endif
	buf->src_sa.addr_type = ADDR_NONE;
	buf->from = MODULE_CIPV6;
#ifdef HAVE_MAC_15_4
	buf->to = MODULE_MAC_15_4;
#else
	buf->to = MODULE_RF_802_15_4;
#endif
	buf->dir = BUFFER_DOWN;
	stack_buffer_push(buf);
	return pdTRUE;
}


#ifdef HAVE_ROUTING
else if(buf->dir == BUFFER_UP)
{
	switch (nanomesh_delivery)
	{
		case NANOMESH_NO_ROUTE:
			debug("NR\r\n");
			ip_broken_link_notify(buf, 1);
			buf=0;
			break;

		case NANOMESH_FORWARD:
			debug("Rei\r\n");
			/* Change next-hop address to rf_802_15_4 dest_address */
			if(buf->src_sa.addr_type == ADDR_802_15_4_PAN_LONG)
				dest_length=8;
			else
				dest_length=2;

			if(memcmp(buf->src_sa.address, namomesh_info.adr2.address, dest_length)==0)
			{
				memcpy(&(namomesh_info.adr),&(buf->dst_sa), sizeof(sockaddr_t) );
				namomesh_info.lqi = 0;
				namomesh_info.event = REMOVE_ROUTE;
				update_routing_table(&namomesh_info);
			}
			/* Lisää tähän reititys loop detech */

			for(i=0; i<10;i++)
			{
				buf->dst_sa.address[i] = namomesh_info.adr2.address[i];

			}
			buf->dst_sa.addr_type = namomesh_info.adr2.addr_type;
		case NANOMESH_NEIGHBOUR:
			if(nanomesh_delivery==NANOMESH_NEIGHBOUR)
				debug("Neig\r\n");
			buf->src_sa.addr_type = ADDR_NONE;
			buf->from = MODULE_CIPV6;
			buf->to = MODULE_MAC_15_4;
			buf->dir = BUFFER_DOWN;
			stack_buffer_push(buf);
			buf=0;
			break;
		default:
			stack_buffer_free(buf);
			buf=0;
			break;
	}


}
#endif	
	return pdTRUE;

}

portCHAR nanomesh_check( buffer_t *buf )
{
	buf;
	return pdTRUE;
}

