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
 * \file     cIPv6.c
 * \brief    Lowpan and cIPv6 module.
 *
 *  Module includes LowPan adaptation module.
 */



#include <string.h>
#include <sys/inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#ifndef CIPV6_DEBUG
#undef HAVE_DEBUG
#endif
#include "debug.h"
#include "stack.h"
#include "buffer.h"
#include "address.h"

#include "module.h"
#include "neighbor_routing_table.h"
#include "control_message.h"
#include "event_timer.h"
#include "rf.h"
#include "cipv6.h"
#include "socket.h"
#include "gpio.h"
/*
[NAME]
CIPV6

[ID]
MODULE_CIPV6,

[INFO]
#ifdef HAVE_CIPV6
  {cipv6_init, cipv6_handle, cipv6_check, 0, MODULE_CIPV6, 41, ADDR_802_15_4_LONG, 0 },
#endif

[FUNCS]*/
extern portCHAR cipv6_init(buffer_t *buf);
extern portCHAR cipv6_handle( buffer_t *buf );
extern portCHAR cipv6_check( buffer_t *buf );

uint8_t check_broadcast_id(namomesh_info_t *bc_t);
uint8_t add_own_address(uint8_t *d_ptr);
extern sockaddr_t mac_long;

#ifdef HAVE_MAC_15_4
extern portCHAR mac_handle(	buffer_t *buf );
#endif

#ifdef MALLFORMED_HEADERS
extern uint8_t mallformed_headers_cnt;
#endif
cipv6_ib_t cipv6_pib;
flooding_table_t flooding_table;
namomesh_info_t namomesh_info;



#ifdef HAVE_ROUTING
buffer_t *forward_buffer = 0;
#endif

/**
 *  Initialize cIPv6 module.
 *
 *  \return  pdTRUE    OK
 */
portCHAR cipv6_init( buffer_t *buf )
{
	uint8_t i=0;	
	buf;
	flooding_table.count=0;
	for(i=0; i<FLOODING_INFO_SIZE; i++)
	{
		flooding_table.info[i].type = ADDR_NONE;
		flooding_table.info[i].last_sqn = 0;
	}
	cipv6_pib.own_brodcast_sqn = 0;
	cipv6_pib.use_short_address=0;
	cipv6_pib.short_address[0] = 0xfe;
	cipv6_pib.short_address[1] = 0xff;
	cipv6_pib.use_full_compress = 1;
	routing_init();
	return pdTRUE;
}

void ip_address_setup( uint8_t short_addr, uint8_t *ptr)
{
	uint8_t i;
	cipv6_pib.use_short_address = short_addr;
	if(short_addr)
	{
		for(i=0; i<2;i++)
		{
			cipv6_pib.short_address[i] = *ptr++;
		}	
	}
} 

/**
 *  Set LoWPAN's compression mode.
 *
 *  \return  pdTRUE    OK
 */
void cipv6_compress_mode( uint8_t mode )
{	
	cipv6_pib.use_full_compress = mode;
}

/**
 *  Main cUDP buffer handler.
 *
 *	\param buf pointer to buffer
 *  \return  pdTRUE    OK
 */
portCHAR cipv6_handle( buffer_t *buf )
{
	uint8_t tmp_8=0;
	uint8_t lowpan_dispatch=0;
	uint8_t *dptr;

  	/* Process the packet */		
	if(cipv6_pib.own_brodcast_sqn == 0)
	{
		cipv6_pib.own_brodcast_sqn = mac_long.address[0];
	}	
	debug("IP:");

	if(buf->options.handle_type == HANDLE_TTL_UPDATE)
	{
		debug("TTL\r\n");
		update_tables_ttl((&buf->dst_sa));
		stack_buffer_free(buf);
		return pdTRUE;
	}
	else if(buf->options.handle_type == HANDLE_BROKEN_LINK)
	{
		debug("BR\r\n");
		buf->options.handle_type = HANDLE_DEFAULT;
		ip_broken_link_notify(buf, 0);
		return pdTRUE;
	}

	if(buf->dir == BUFFER_UP)
	{
		memcpy(&(namomesh_info.adr),&(buf->src_sa), sizeof(sockaddr_t) );
		namomesh_info.last_sqn = buf->options.lowpan_compressed;
		namomesh_info.event = UPDATE_NEIGHBOUR;
		taskENTER_CRITICAL();
		tmp_8 = update_neighbour_table(&(namomesh_info));
		taskEXIT_CRITICAL();
		if(tmp_8 == 0)
		{
			debug("drop\r\n");
			stack_buffer_free(buf);
			return pdTRUE;
		}
		tmp_8 = 0;
		debug("UP");
		/* Lets check header start type */
		if(buf->buf[buf->buf_ptr] == IPV6)
		{
			//debug("IPV6\r\n");
#ifdef SUPPORT_UNCOMP_IPV6
			parse_ipv_header(buf);
			buf=0;
#else
#ifdef MALLFORMED_HEADERS
			mallformed_headers_cnt++;
#endif
			stack_buffer_free(buf);
			buf=0;
#endif
		}
		else
		{
			buf->options.lowpan_compressed=0;
			switch (buf->buf[buf->buf_ptr] & LOWPAN_TYPE_BM )
			{
			/* Normal unicast send-mode */
				case DISPATCH_TYPE:
					debug("DIS\r\n");
					/* Parse Unicast packet from neighbor */

					dptr = buf->buf + buf->buf_ptr;
					lowpan_dispatch = *dptr++;
					if(lowpan_dispatch == LOWPAN_HC1)
					{
						/* Check HC1 options */
						tmp_8= *dptr++;
						if((tmp_8 == HC1_NEXT_HEADER_COMPRESSED_UDP || tmp_8 == HC1_NEXT_HEADER_UNCOMPRES_UDP) || tmp_8 == IP_HEADER_FOR_ICMP)
						{
							if(tmp_8 == IP_HEADER_FOR_ICMP)
							{
								buf->to = MODULE_ICMP;
							}
							else
							{
								buf->to = MODULE_CUDP;
								if(tmp_8 == HC1_NEXT_HEADER_COMPRESSED_UDP) /* Read udp encode field */
									buf->options.lowpan_compressed = *dptr++;
							}
				
							*dptr -= 1;
							buf->options.hop_count = (GENERAL_HOPLIMIT - *dptr);
							dptr++;
							buf->buf_ptr = (dptr - buf->buf); /*cut header*/
							buf->from = MODULE_CIPV6;
							buf->to = MODULE_NONE;
							stack_buffer_push(buf);
							return pdTRUE;
						}
						else
						{
							debug("Dis:hc1\r\n");
							stack_buffer_free(buf);
							return pdTRUE;
						}
					}
					else
					{
						#ifdef MALLFORMED_HEADERS
							mallformed_headers_cnt++;
#endif
						stack_buffer_free(buf);
						return pdTRUE;
					}
					break;	
				
				/* Packet deliverymode is mesh */	
					case MESH_ROUTING_TYPE: /* Mesh Routing */
					{
						uint8_t hops_left=0,mesh_header,i, bc_filter=1;
						uint8_t tmp;
						uint8_t dest_length=0;
						match_type_t address_mode = NOT_OWN;
						debug("MES\r\n");
						/* Process the packet */
						dptr = buf->buf + buf->buf_ptr;

						/* Parse mesh header  */
						mesh_header = *dptr;
						hops_left = (mesh_header & 0xf0);
						hops_left = hops_left >> 4;
						hops_left--;
						namomesh_info.hop = hops_left;
						i = (mesh_header & 0x0f);
						tmp = hops_left;
						tmp <<= 4;
						*dptr++ = (tmp | i);
						/* Check Originator and Final-destination address */
						/* Originator */
						tmp=(mesh_header & MESH_ADDRESSTYPE_BM);
						tmp_8 = (tmp & O_ADDRESSTYPE_BM);
				
						if(tmp_8 == O_ADDRESSTYPE_16)
						{
							namomesh_info.adr.addr_type = ADDR_802_15_4_PAN_SHORT;
							dest_length=2;
						}
						else
						{
							namomesh_info.adr.addr_type = ADDR_802_15_4_PAN_LONG;
							dest_length=8;	
						}
						for(i=0;i<dest_length;i++)
						{
							namomesh_info.adr.address[i] = *dptr++;
						}

						/* Final Dst */
						tmp_8 = (tmp & D_ADDRESSTYPE_BM);

						if(tmp_8 == D_ADDRESSTYPE_16)
						{
							buf->dst_sa.addr_type =  ADDR_802_15_4_PAN_SHORT;
							dest_length=2;
						}
						else
						{
							buf->dst_sa.addr_type=ADDR_802_15_4_PAN_LONG;
							dest_length=8;
						}
						for(i=0;i<dest_length;i++)
						{
							buf->dst_sa.address[i] = *dptr++;
						}

						/* Parse rest of IP header */
						lowpan_dispatch = *dptr++;
						tmp=1;
						if(lowpan_dispatch == LOWPAN_BC0)
						{
							namomesh_info.last_sqn = *dptr++;
							//tmp = *dptr++;
							lowpan_dispatch = *dptr++;
							address_mode = BCAST;
							
							if(compare_ori_to_own(&(namomesh_info.adr)) == pdTRUE)
							{
								//debug(" Dis:BCO own\r\n");
								address_mode = DISCARD;
								tmp=0;
							}
							else
							{
								taskENTER_CRITICAL();
								bc_filter = check_broadcast_id(&namomesh_info);
								taskEXIT_CRITICAL();
								
								tmp=1;
								if(bc_filter != 1)
								{
									//debug(" Dis:BCO\r\n");
									address_mode = DISCARD;
									if(bc_filter == 0)
									{
										tmp = 0;
									}
								}	
							}
						}

						if(lowpan_dispatch == LOWPAN_HC1 && tmp == 1) 
						{
							/* Check HC1 options */
							tmp_8= *dptr++;
							if((tmp_8 == HC1_NEXT_HEADER_COMPRESSED_UDP || tmp_8 == HC1_NEXT_HEADER_UNCOMPRES_UDP) || tmp_8 == IP_HEADER_FOR_ICMP)
							{
								if(tmp_8 == IP_HEADER_FOR_ICMP)
								{
									buf->to = MODULE_ICMP;
								}
								else
								{
									buf->to = MODULE_CUDP;
									if(tmp_8 == HC1_NEXT_HEADER_COMPRESSED_UDP) /* Read udp encode field */
										buf->options.lowpan_compressed = *dptr++;
								}
					
								*dptr -= 1;
								buf->options.hop_count = (GENERAL_HOPLIMIT - *dptr);
								dptr++;

					#ifdef HAVE_ROUTING						
								if(buf->options.hop_count > 1 )
								{
									
									memcpy(&(namomesh_info.adr2),&(buf->src_sa), sizeof(sockaddr_t) );
									namomesh_info.hop = buf->options.hop_count;
									namomesh_info.lqi = buf->options.rf_lqi;
									namomesh_info.event = 0;
									taskENTER_CRITICAL();
									update_routing_table( &namomesh_info);
									taskEXIT_CRITICAL();
								}
					#endif
							}
							else
							{
								#ifdef MALLFORMED_HEADERS
									mallformed_headers_cnt++;
#endif
								//debug("Dis:HC1\r\n");
								address_mode = DISCARD;
							}
						}
						else
						{
							#ifdef MALLFORMED_HEADERS
							mallformed_headers_cnt++;
#endif
							//debug("NOT sup\r\n");
							address_mode = DISCARD;
						}
						

						if(address_mode == DISCARD)
						{
							stack_buffer_free(buf);
							return pdTRUE;
						}

						/* Check destination */
						if(address_mode == NOT_OWN)
						{
							if(compare_ori_to_own(&(namomesh_info.adr)) == pdTRUE)
							{
								control_message_t *msg;
					#ifdef HAVE_ROUTING
								memcpy(&(namomesh_info.adr),&(buf->dst_sa), sizeof(sockaddr_t) );
								namomesh_info.hop = 0;
									namomesh_info.lqi = 0;
									namomesh_info.event = REMOVE_ROUTE;
									taskENTER_CRITICAL();
									update_routing_table( &namomesh_info);
									taskEXIT_CRITICAL();	
					#endif
								buf->options.type = BUFFER_CONTROL;
								msg = ( control_message_t*) buf->buf;
								msg->message.ip_control.message_id = BROKEN_LINK;
								buf->from = MODULE_CIPV6;
								buf->socket = 0;
								msg->message.ip_control.message.broken_link_detect.reason = NO_ROUTE_TO_DESTINATION_TYPE;
								debug("Route loop err\r\n");
								push_to_app(buf);
								return pdTRUE;
							}
							if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
							{
								if (memcmp(buf->dst_sa.address, mac_long.address, 8) == 0)
								{
									address_mode = OWN;
								}
							}
							else
							{
								if (memcmp(buf->dst_sa.address, cipv6_pib.short_address, 2) == 0)
								{
									address_mode = OWN;
								}
							}		
						}

						if(namomesh_info.adr.addr_type == ADDR_802_15_4_PAN_LONG)
						{
							dest_length=8;
						}
						else
						{
							dest_length=2;
						}

						for(i=0; i<dest_length;i++)
						{
							buf->src_sa.address[i] = namomesh_info.adr.address[i];
						}
			
						buf->src_sa.addr_type = namomesh_info.adr.addr_type;
						if(address_mode==OWN || address_mode==BCAST)
						{
							#ifdef HAVE_ROUTING
							#ifndef HAVE_NRP
								if(hops_left != 0 && address_mode == BCAST)
								{
									#ifdef STACK_RING_BUFFER_MODE
									if(stack_buffer_count() > 2)
									#else
									if(uxQueueMessagesWaiting(buffers) > 2)
									#endif
									{
										forward_buffer=0;
										/* Get new buffer for forwarding */
										forward_buffer = stack_buffer_get(0);
										if (forward_buffer)
										{
											/* Copy original buffer for forwarding */
											memcpy(forward_buffer, buf, sizeof(buffer_t) + buf->buf_end);
											forward_buffer->src_sa.addr_type = ADDR_NONE;
											forward_buffer->dst_sa.addr_type = ADDR_BROADCAST;
											forward_buffer->from = MODULE_CIPV6;
											forward_buffer->to = MODULE_NONE;
											forward_buffer->dir = BUFFER_DOWN;
											forward_buffer->options.type = BUFFER_DATA;
											stack_buffer_push(forward_buffer);
											forward_buffer=0;
										}
									}
						
								}
							#endif
							#endif
								buf->buf_ptr = (dptr - buf->buf); /*cut header*/
								buf->from = MODULE_CIPV6;
								buf->to = MODULE_NONE;
								stack_buffer_push(buf);
								buf=0;
						}
						#ifdef HAVE_ROUTING	
						else if(address_mode == NOT_OWN)
						{
							if(compare_ori_to_own(&(buf->src_sa)) == pdTRUE)
							{
								control_message_t *msg;
								memcpy(&(namomesh_info.adr),&(buf->dst_sa), sizeof(sockaddr_t) );
								namomesh_info.hop = 0;
								namomesh_info.lqi = 0;
								namomesh_info.event = REMOVE_ROUTE;
								taskENTER_CRITICAL();
								update_routing_table( &namomesh_info);
								taskEXIT_CRITICAL();
								buf->options.type = BUFFER_CONTROL;
								msg = ( control_message_t*) buf->buf;
								msg->message.ip_control.message_id = BROKEN_LINK;
								buf->from = MODULE_CIPV6;
								buf->socket = 0;
								msg->message.ip_control.message.broken_link_detect.reason = NO_ROUTE_TO_DESTINATION_TYPE;
								debug("Route loop err\r\n");
								push_to_app(buf);
								return pdTRUE;
							}

							if(hops_left > 0)
							{

								buf->from = MODULE_CIPV6;
								buf->to = MODULE_NANOMESH;
								stack_buffer_push(buf);
								buf=0;
							}
							else
							{
								/* Route error support only 3 hops */
								//stack_buffer_free(buf);
								taskENTER_CRITICAL();
								ip_broken_link_notify(buf, 1);
								taskEXIT_CRITICAL();
								buf=0;
							}
						}
						#endif
					}
					break;	/* END_OF_MESH_ROUTING_TYPE */
				
				default:
					#ifdef MALLFORMED_HEADERS
							mallformed_headers_cnt++;
#endif
					debug("Dis\r\n");
					stack_buffer_free(buf);
					break;
			}
		}	
	}
	else if(buf->dir == BUFFER_DOWN)
	{
			debug("DOWN\r\n");
			if(cipv6_pib.use_full_compress)
			{
				uint8_t header_size, i;
				uint8_t mesh_header = MESH_ROUTING_TYPE;
				header_size = 3;
				if((buf->from == MODULE_CUDP) && buf->options.lowpan_compressed)
					header_size++;
			
				if(buf->dst_sa.addr_type != ADDR_COORDINATOR)
				{
					if(stack_check_broadcast(buf->dst_sa.address, buf->dst_sa.addr_type) == pdTRUE)
					{
						buf->dst_sa.addr_type = ADDR_BROADCAST;
					}
				}
				if(buf->dst_sa.addr_type == ADDR_BROADCAST)
				{
					
					mesh_header |= D_ADDRESSTYPE_16;
					header_size +=5;
					if(cipv6_pib.use_short_address)
					{
						mesh_header |= O_ADDRESSTYPE_16;
						header_size +=2;
					}
					else
					{
						mesh_header |= O_ADDRESSTYPE_64;
						header_size +=8;
					}
#ifdef HAVE_NRP
					mesh_header |= (GENERAL_HOPLIMIT<<4);
#else
#ifdef HAVE_ROUTING
					/*if(buf->options.hop_count)
						mesh_header |= (buf->options.hop_count<<4);
					else
						mesh_header |= (1<<4);*/

					mesh_header |= (GENERAL_HOPLIMIT<<4);
#else
					mesh_header |= (1<<4);
#endif/*HAVE_ROUTING*/
#endif/*HAVE_NRP*/
					memset(buf->dst_sa.address, 0xff, 4); 
				}

				if(stack_buffer_headroom(buf,header_size) == pdFALSE)
				{
					//control_message_t *ptr = ( control_message_t*) buf->buf;
					//ptr->message.ip_control.message_id = TOO_LONG_PACKET;
					//push_to_app(buf);
					//buf=0;
					stack_buffer_free(buf);
					return pdTRUE;
				}
				buf->buf_ptr -= header_size; /* lowpan-dispatch+HC1 = header space */
				dptr = buffer_data_pointer(buf);
				if(buf->dst_sa.addr_type == ADDR_BROADCAST)
				{
					*dptr++ = mesh_header;
					/* Build Mesh delivery address field*/
					/* Originator address */
					dptr += add_own_address(dptr);
					for(i=0; i<2 ;i++)
					{
						*dptr++ = buf->dst_sa.address[i];
					}
					buf->dst_sa.addr_type = ADDR_BROADCAST;
					buf->src_sa.addr_type = ADDR_NONE;
					/* Mesh broadcast header type and sequence number */
					*dptr++ = LOWPAN_BC0;
					*dptr++ = cipv6_pib.own_brodcast_sqn;
					update_ip_sqn();
#ifdef HAVE_MAC_15_4
					buf->to = MODULE_MAC_15_4;
#else
					buf->to = MODULE_RF_802_15_4;
#endif
				}
				else
				{
					buf->to = MODULE_NANOMESH;
					if(cipv6_pib.use_short_address)
					{
						memcpy(&(buf->src_sa.address),cipv6_pib.short_address,2);
						buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
						
					}
					else
					{
						memcpy(&(buf->src_sa.address),mac_long.address,8);
						buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;
					}
				}
				*dptr++ = LOWPAN_HC1; 	/* LowPan Dispatch	 */
				/* HC1 encode */
				if(buf->from == MODULE_ICMP)
					*dptr++ = IP_HEADER_FOR_ICMP;
				else
				{
					if(buf->options.lowpan_compressed)
					{
						*dptr++ = HC1_NEXT_HEADER_COMPRESSED_UDP;
						*dptr++ = buf->options.lowpan_compressed;
					}		
					else 
						*dptr++ = HC1_NEXT_HEADER_UNCOMPRES_UDP;
				}
				*dptr++ = GENERAL_HOPLIMIT;	/* Hop-limit	*/
				buf->from = MODULE_CIPV6;
				//buf->to = MODULE_NONE;
				stack_buffer_push(buf);
				buf=0;


			}
			else
			{
#ifdef SUPPORT_UNCOMP_IPV6
				build_ipv6_header(buf);
#else
				stack_buffer_free(buf);
				buf=0;
#endif
			}
	}

return pdTRUE;
}

/**
 *  The cIPv6 buffer checker.
 *
 *	\param buf pointer to buffer
 *
 *  \return  pdTRUE    is cIPv6
 *  \return  pdFALSE   is not cIPv6 or broken header
 */
portCHAR cipv6_check( buffer_t *buf )
{
	uint8_t temp;
	if(buf->buf[buf->buf_ptr] == IPV6)
	{
		/* Uncompressed IPv6 header dispatch */
		return pdTRUE; 
	}
	else
	{
		temp = (buf->buf[buf->buf_ptr] & LOWPAN_TYPE_BM);
		if(temp == DISPATCH_TYPE)
		{
			return pdTRUE; 
		}
#ifdef FRAGMENTATION
		else if(temp == FRAGMENT_TYPE)
		{
			return pdTRUE; 
		}
#endif
		else if(temp == MESH_ROUTING_TYPE)
		{
			return pdTRUE; 
		}
		else
		{
			return pdFALSE;
		}
	}
}

uint8_t add_own_address(uint8_t *d_ptr)
{
	uint8_t i;
	if(cipv6_pib.use_short_address)
	{
		for (i=0; i<2; i++)
		{
			*d_ptr++ = cipv6_pib.short_address[i];
		}
		return 2;
	}
	else
	{
		for (i=0; i<8; i++)
		{
			*d_ptr++ = mac_long.address[i];
		}
		return 8;
	}
}

void update_ip_sqn(void)
{
	/* Increace broadcast_counter */
	if(cipv6_pib.own_brodcast_sqn==0xff) cipv6_pib.own_brodcast_sqn=1;
	else cipv6_pib.own_brodcast_sqn++;
}

portCHAR compare_ori_to_own(sockaddr_t *adr)
{
	if(adr-> addr_type == ADDR_802_15_4_PAN_LONG)
	{
		if(memcmp(adr->address,mac_long.address , 8)==0)
			return pdTRUE;
	}
	else
	{
		if(memcmp(adr->address,cipv6_pib.short_address ,2)==0)
			return pdTRUE;
	}
	return pdFALSE;
}

/**
 * Function notify for Broken link.
 *
 * Function remove allways neighbour info which not answer and if packet is Mesh routing header it will also remove routing info to destination.
 * Then it check originator address, if address is device own it will notify application.
   if address is not own it will forward ctrl message to ICMP module which create ICMP error message to originator.
 * \param buf pointer to data
 */
void ip_broken_link_notify(buffer_t *buf, uint8_t no_route_reason)
{
	uint8_t i=0, length=0, tmp, *ind;
	control_message_t *msg;
	debug("IP:BR\r\n");
	memcpy(&(namomesh_info.adr),&(buf->dst_sa), sizeof(sockaddr_t) );
	namomesh_info.lqi = 0;
	namomesh_info.event = REMOVE_ROUTE;
	update_neighbour_table(&namomesh_info);

	if(buf->buf[buf->buf_ptr] == IPV6)
	{
		i=99;
	}
	else
	{
		buf->src_sa.addr_type = ADDR_NONE;
		if((buf->buf[buf->buf_ptr] & LOWPAN_TYPE_BM) ==  MESH_ROUTING_TYPE)
		{
			uint8_t mesh_header;
			ind = (buf->buf + buf->buf_ptr);
			mesh_header = *ind++;
			/* Check Originator and Final-destination address */
			tmp=(mesh_header & MESH_ADDRESSTYPE_BM);
			if((tmp & O_ADDRESSTYPE_BM) == O_ADDRESSTYPE_16)
			{
				buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
				length=2;
			}
			else
			{
				buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;
				length=8;	
			}
			for(i=0; i < length; i++)
			{
				buf->src_sa.address[i] = *ind++;
			}
		
			if((tmp & D_ADDRESSTYPE_BM) == D_ADDRESSTYPE_16)
			{
				buf->dst_sa.addr_type =  ADDR_802_15_4_PAN_SHORT;
				length=2;
			}
			else
			{
				buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
				length=8;
			}
			for(i=0; i < length; i++)
			{
				buf->dst_sa.address[i] = *ind++;
			}
#ifdef HAVE_ROUTING
			memcpy(&(namomesh_info.adr),&(buf->dst_sa), sizeof(sockaddr_t) );
			namomesh_info.lqi = 0;
			namomesh_info.event = ROUTE_ERR;
			
			update_routing_table(&namomesh_info);
#endif
		}
		else
		{
			i=99;	/* Destination is neighbour */
		}
	}

	buf->options.type = BUFFER_CONTROL;
	msg = ( control_message_t*) buf->buf;
	msg->message.ip_control.message_id = BROKEN_LINK;
	buf->from = MODULE_CIPV6;
	buf->socket = 0;

	if(no_route_reason)
		msg->message.ip_control.message.broken_link_detect.reason = NO_ROUTE_TO_DESTINATION_TYPE;
	else
		msg->message.ip_control.message.broken_link_detect.reason = BROKEN_LINK_TYPE;
	
	if(i==99 || buf->src_sa.addr_type == ADDR_NONE)
	{
		debug("--> APP\r\n");
		push_to_app(buf);
	}
	else
	{
		if(compare_ori_to_own(&(buf->src_sa)) == pdTRUE)
		{
			debug("--> APP\r\n");
			push_to_app(buf);
		}
		else
		{
			debug("--> ICMP\r\n");
			buf->to = MODULE_ICMP;
			buf->options.handle_type = HANDLE_DEFAULT;
			buf->dir = BUFFER_UP;
			stack_buffer_push(buf);
		}
	}
}

uint8_t flood_indexs=0;
/**
 * Flooding filter check.
 *
 * Function filter broadcast packet and discard if got same packet back from neighbour.
 *
 * \param bc_t pointer to broadcast packet information.
 * \return 1 when packet is new broadcast packet from originator.
 * \return 0 when function detetct same broadcast sqn number from originator.
 */
uint8_t check_broadcast_id(namomesh_info_t *bc_t)
{
	uint8_t i,j, length=0;
	if(bc_t->adr.addr_type == ADDR_802_15_4_PAN_SHORT)
		length=2;
	else
		length=8;

	if(flooding_table.count)
	{
		for(i=0; i<FLOODING_INFO_SIZE; i++)
		{
			if(flooding_table.info[i].type == bc_t->adr.addr_type)
			{
				if(memcmp(flooding_table.info[i].address, bc_t->adr.address, length)==0)
				{
					if(flooding_table.info[i].last_sqn != bc_t->last_sqn)
					{
						flooding_table.info[i].last_sqn = bc_t->last_sqn;
						flooding_table.info[i].hop = bc_t->hop;
						return 1;
					}
					else
					{
						if(flooding_table.info[i].hop != bc_t->hop)
						{
							return 0;
						}
						else
							return 2;
					}
				}
			}
		}
		if(flooding_table.count == FLOODING_INFO_SIZE)
		{
			flooding_table.info[flood_indexs].type = bc_t->adr.addr_type;
			//memcpy(flooding_table.info[flood_indexs].address, adr->address, length);
			for(j=0; j<length;j++)
			{
				flooding_table.info[flood_indexs].address[j] = bc_t->adr.address[j];
			}
			flooding_table.info[flood_indexs].last_sqn = bc_t->last_sqn;
			flooding_table.info[flood_indexs].hop = bc_t->hop;
			flood_indexs++;
			if(flood_indexs == FLOODING_INFO_SIZE)
				flood_indexs = 0;
			return 1;
		}
	}
	i=flooding_table.count;
	flooding_table.info[i].type = bc_t->adr.addr_type;
	for(j=0; j<length;j++)
	{
		flooding_table.info[i].address[j] = bc_t->adr.address[j];
	}
	flooding_table.info[i].last_sqn = bc_t->last_sqn;
	flooding_table.info[i].hop = bc_t->hop;
	flooding_table.count++;
	return 1;
}

#ifdef SUPPORT_UNCOMP_IPV6
/**
 * Build IPv6 header.
 *
 * Function create whole uncompressed header and add dispatch for that --> Header size 41 byte.
 * \param buf pointer to data
 */
portCHAR build_ipv6_header(buffer_t *buf)
{
	uint8_t next_header=0, i;
	uint8_t *dptr;
	uint16_t payload_length=0;
	
	dest_delivery_t destination_delivery = NOT_NEIGHBOR;
	memset(ipv6_address, 0,8);
	payload_length = (buf->buf_end - buf->buf_ptr);

	if(stack_buffer_headroom( buf,41)==pdFALSE)
	{
		control_message_t *ptr = ( control_message_t*) buf->buf;
		ptr->message.ip_control.message_id = TOO_LONG_PACKET;
		push_to_app(buf);
		buf=0;
		return pdTRUE;
	}
	buf->buf_ptr -= 41;
	/* Check deliverymode from neighbour table */
	if(buf->dst_sa.addr_type != ADDR_BROADCAST && buf->dst_sa.addr_type != ADDR_COORDINATOR)
	{
		destination_delivery = check_neighbour_table(&(buf->dst_sa));
		if(destination_delivery==BROADCAST)
		{
			buf->dst_sa.addr_type = ADDR_BROADCAST;
		}
	}
	if(buf->from == MODULE_CUDP)
		next_header = NEXT_HEADER_UDP;
	else
		next_header = NEXT_HEADER_ICMP6;

	dptr = buf->buf + buf->buf_ptr;
	*dptr++ = IPV6;
	*dptr++ = 0x06;
	*dptr++ = 0x00;
	*dptr++ = 0x00;
	*dptr++ = 0x00;
	*dptr++ = (payload_length >> 8);	
	*dptr++ = (uint8_t) payload_length;			/* Length */
	*dptr++ = next_header;
	*dptr++ = GENERAL_HOPLIMIT;
	/* Add source */
	if(cipv6_pib.use_short_address)
	{
		ipv6_address[4]=0xff;
		ipv6_address[3]=0xfe;
		ipv6_address[1]=cipv6_pib.short_address[1];
		ipv6_address[0]=cipv6_pib.short_address[0];
	}
	else
	{
		for(i=0; i<8; i++)
		{
			ipv6_address[i] = mac_long.address[7-i];
		}
	}
	*dptr++ =0xfe;
	*dptr++ = 0x80;
	for(i=0; i<6; i++)
	{
		*dptr++ = 0x00;
	}
	for(i=0; i<8; i++)
	{
		*dptr++ = ipv6_address[i];
	}
	memset(ipv6_address, 0,8);
	 /* Link Local IPV6 ADDRESS PREFIX */
	/* Check destination */
	/* Destination */
	if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT ||  buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
	{

		if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
		{
			for(i=0;i<8;i++)
			{
				ipv6_address[i] = buf->dst_sa.address[7-i];
			}
		}
		else
		{
			ipv6_address[3]=0xff;
			ipv6_address[4]=0xfe;
			ipv6_address[6]=buf->dst_sa.address[1];
			ipv6_address[7]=buf->dst_sa.address[0];
		}

		*dptr++ =0xfe;
		*dptr++ = 0x80;

		for(i=0; i<6; i++)
		{
			*dptr++ = 0x00;
		}

		/*for(i=0; i<8; i++)
		{
			*dptr++ = ipv6_address[i];
		}*/
		memcpy(dptr, ipv6_address, 8);
		dptr += 8;
	}
	if(buf->dst_sa.addr_type == ADDR_BROADCAST)
	{
		*dptr++ =0xff;
		*dptr++ = 0x02;	
		for(i=0; i<13; i++)
		{
			*dptr++ = 0x00;
		}
		*dptr++ = 0x01;
	}
	buf->from = MODULE_CIPV6;
	buf->to = MODULE_NONE;
	stack_buffer_push(buf);
	buf=0;
	return pdTRUE;
}

/**
 * Parse IPv6 header.
 *
 * Function support only link-local unicast and multicast for all nodes.
 * \param buf pointer to data
 */
void parse_ipv_header(buffer_t *buf)
{
	uint8_t ind=0, ip_version,hoplimit=0, next_header=0, i, dest_match=0;
	ipv6_address_t destination;
	ind = buf->buf_ptr;
	ind++;
	/* Check IP version */
	ip_version = buf->buf[ind++];
	if( ip_version == 0x06 )
	{	
		debug("IPv6 packet.\r\n");
		/* Check HC1 options */
		ind +=5;
		next_header = buf->buf[ind++];
		hoplimit= buf->buf[ind++];
		hoplimit--;
		ind +=16;
		for(i=0; i<16; i++)
		{
			destination[15-i] = buf->buf[ind++];
		}
		if(destination[15] == 0xfe && destination[14] == 0x80)
		{
			//debug("link local unicast\r\n");
			if(destination[3] == 0xfe && destination[3] == 0xff)
			{
				if(memcmp(destination, cipv6_pib.short_address, 2) ==0)
					dest_match=1;
			}
			else
			{
				if(memcmp(destination, mac_long.address, 8) ==0)
					dest_match=1;
			}	

		}
		if((destination[15] == 0xff && destination[14] == 0x02) && destination[0] == 0x01)
		{	
			//debug("link local multicast for all nodes\r\n");
			dest_match=1;
		}

		buf->options.lowpan_compressed = 0;
		if(next_header==NEXT_HEADER_ICMP6) buf->options.hop_count = 1;
		if(hoplimit > 0 && dest_match)
		{
			buf->buf_ptr = ind; /*cut header*/
			buf->to = MODULE_NONE;
			buf->from = MODULE_CIPV6;
			stack_buffer_push(buf);
			buf=0;
		}
		else
		{	
			//debug("destination not match\r\n");
		}
	}/* END_OF_LOWPAN_HC1 */
	/* General buffer discard part */	
	if(buf)
	{
		debug("Discard.\r\n");
		stack_buffer_free(buf);
		buf=0;
	}
}
#endif

#ifndef NO_FCS
uint8_t ipv6_in_use = 0;

uint8_t ipv6_pseudo[40] = 
{
    // source address
    0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // destination address
    0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // payload length
    0x00, 0x00, 0x00, 0x00,
    // next header
    0x00, 0x00, 0x00, 0x00
};
	
/**
 * Check & Calculate FCF.
 *
 * Function calculate typical IP FCF which is used IPv6 and ICMP protocols.
 * \param buf pointer to data
 * \param next_protocol NEXT_HEADER_UDP / NEXT_HEADER_ICMP6
 * \param rx_case flag when function now it have to check and when calculate
 *
 * \return fcf 16-values
 */
uint16_t ipv6_fcf(buffer_t *buf, uint8_t next_protocol, uint8_t rx_case)
{
	uint32_t sum = 0;
	uint16_t cksum;
	uint8_t i, tmp=0;
	uint8_t *p_payload = buffer_data_pointer(buf);
	uint16_t length = buffer_data_length(buf);
	
	while (ipv6_in_use)
	{
		vTaskDelay(200/portTICK_RATE_MS);
	};
	
	ipv6_in_use = 1;
	//Create and init IPv6 pseudo header array
	memset(ipv6_pseudo, (uint8_t) 0, 40);
	ipv6_pseudo[0] = 0xfe;
	ipv6_pseudo[1] = 0x80;
	ipv6_pseudo[16] = 0xfe;
	ipv6_pseudo[17] = 0x80;

#ifndef AD_HOC_STATE
#ifdef MAC_RFD
	if ((rx_case == 0) && (buf->dst_sa.addr_type == ADDR_COORDINATOR))
	{
		if(get_coord_address(&buf->dst_sa) !=pdTRUE)
		{

		}
	}
#endif
#endif	
	if(rx_case)
	{
		//RX case use buffers source address field
		if(buf->src_sa.addr_type == ADDR_802_15_4_PAN_LONG)
		{
			for(i=0;i<8;i++)
			{
				ipv6_pseudo[8+i] = buf->src_sa.address[7-i];
			}	
		}
		else
		{
			ipv6_pseudo[11]=0xff;
			ipv6_pseudo[12]=0xfe;
			ipv6_pseudo[14]=buf->src_sa.address[1];
			ipv6_pseudo[15]=buf->src_sa.address[0];	
		}
	}
	else
	{
		// Source address
		if(cipv6_pib.use_short_address)
		{
			ipv6_pseudo[11]=0xff;
			ipv6_pseudo[12]=0xfe;
			ipv6_pseudo[14]=cipv6_pib.short_address[1];
			ipv6_pseudo[15]=cipv6_pib.short_address[0];	
		}
		else
		{
			for(i=0; i<8; i++)
			{
				ipv6_pseudo[8+i] = mac_long.address[7-i];	
			}
		}
	}	
	//Destination address
	if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
	{
		tmp=0;
		for(i=0;i<8;i++)
		{
			if(buf->dst_sa.address[i] !=0xff)
			{
				tmp=1;
				i=8;
			}
		}

		if(tmp)
		{
			for(i=0;i<8;i++)
			{
				ipv6_pseudo[24+i] = buf->dst_sa.address[7-i];
			}	
		}
		else
			buf->dst_sa.addr_type = ADDR_BROADCAST;
	}
	if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
	{
		if((buf->dst_sa.address[1] !=0xff) && (buf->dst_sa.address[0] !=0xff))
		{
			ipv6_pseudo[27]=0xff;
			ipv6_pseudo[28]=0xfe;
			ipv6_pseudo[30]=buf->dst_sa.address[1];
			ipv6_pseudo[31]=buf->dst_sa.address[0];	
		}
		else
			buf->dst_sa.addr_type = ADDR_BROADCAST;	
	}
	if(buf->dst_sa.addr_type == ADDR_BROADCAST)
	{
		// Multicast to all nodes ff02: :01
		ipv6_pseudo[16] =0xff;
		ipv6_pseudo[17] = 0x02;	
		ipv6_pseudo[31] = 0x01;
	}

	ipv6_pseudo[35] = length;
	ipv6_pseudo[39] = next_protocol;

	//Calculate sum of ipv6_pseudo header
	for ( i = 0; i < sizeof(ipv6_pseudo); i += 2 )
	{
		uint16_t x = (uint16_t)(ipv6_pseudo[ i ] << 8) | ipv6_pseudo[ i+1 ];
		sum += x;
	}
	// Calculate sum of upper-layer payload
	for ( i = 0; i < length; i += 2 )
	{
		uint16_t x;
		// account for odd-length payload, pad with 0
		if ( (i+1) == length )
		x = (uint16_t)(p_payload[ i ] << 8);
		else
		x = (uint16_t)(p_payload[ i ] << 8) | p_payload[ i+1 ];
		sum += x;
	}
	cksum = (sum >> 16) + (sum & 0xffff);
	
	if(rx_case==0)
	{
		if ( cksum != 0xffff )
			cksum = ~cksum;
	}
	ipv6_in_use = 0;
	return cksum;
}
#endif /*NO_FCS*/
