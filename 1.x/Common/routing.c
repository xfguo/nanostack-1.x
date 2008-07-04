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
 * \file     routing.c
 * \brief    Routing module which handle routing & neighbourtable informations.
 *
 *  This module will be usefull for every modules who want check neighbor_tables and routing_tables
 *  
 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#undef HAVE_DEBUG
#include <sys/inttypes.h>
#include <string.h>
#include "address.h"
#include "neighbor_routing_table.h"
#include "socket.h"
#include "debug.h"
#include "bus.h"

#define TTL		15	
#define ROUTING_TTL	15
#ifdef HAVE_ROUTING
volatile routing_table_t routing_table;
#endif
volatile neighbor_table_t neighbor_table;

/**
 * Initialize Neighbour and Routing tables.
 *
 * Create also semaphore for tables.
 *
 */

void routing_init(void)
{
	uint8_t i=0;
#ifdef HAVE_ROUTING
	route_info_t *ptr;
#endif
	neighbor_info_t *b;
		/* Init table info */
		while (i < MAX_NEIGHBOR_COUNT)
		{
			b = &(neighbor_table.neighbor_info[i]);
			b->type=ADDR_NONE;
			b->tx_err_cnt=0;
			i++;
		}
		neighbor_table.count=0;
#ifdef HAVE_ROUTING
		i=0;
		routing_table.count=0;
		while (i < MAX_ROUTE_INFO_COUNT)
		{	
			ptr = &(routing_table.route_info[i]);
			ptr->dest_addr_type=ADDR_NONE;
			i++;
		}
#endif
}


/**
 * Update neighbor tables if necessary.
 *
 * 6LoWPAN-layer use this function every time when received packet which LQI > 0.
 *
 * \param *info pointer for neighbour information
 *
 * \return 1 when last_sqn is different than current
 * \return 0 when sqn is same, now MAC discard packet
 */
uint8_t update_neighbour_table(namomesh_info_t *info)
{
	neighbor_info_t *b=0;
	uint8_t i,j, sqn_check=0, length=0;
	dest_delivery_t delivery_mode;
	delivery_mode = NOT_NEIGHBOR;

		if(info->adr.addr_type==ADDR_802_15_4_PAN_LONG)
			length=8;					
		else if(info->adr.addr_type == ADDR_802_15_4_PAN_SHORT)
			length=4;

		delivery_mode = NOT_NEIGHBOR;
		if(neighbor_table.count > 0 && info->event != ADD_CHILD)
		{
			for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
					b=0;

				if(b && (info->adr.addr_type == b->type))
				{
					if(memcmp(b->address, info->adr.address,length) == 0)
					{
						delivery_mode = NEIGHBOR;
					}
					
					/* Update lqi and compare sqn to old one */
					if( delivery_mode == NEIGHBOR )
					{
						if(info->adr.addr_type != ADDR_802_15_4_PAN_SHORT)
						{
							for(j=0; j<2; j++)
							{
								b->address[length+j] = info->adr.address[length+j];
							}
						}
						if(info->event == REMOVE_NEIGHBOUR)
						{
							b->tx_err_cnt++;
							/*if(b->ttl > 2)
							{
								b->ttl -= 2;
							}
							else
							{
	
								b->type=ADDR_NONE;
								i=neighbor_table.count;
								neighbor_table.count--;
							}*/
							//return 0;
						}
						else
						{
							/* Duplicated packet check */
							if(b->last_sqn != info->last_sqn)
							{
								b->last_sqn = info->last_sqn;
								sqn_check=1;
								debug("sqn new\r\n");
							}
							b->ttl=TTL;
						}
						i=MAX_NEIGHBOR_COUNT;
						return sqn_check;
					}
				}
			}
		}
		/* Add new neighbor if addresstype is source */
		if((delivery_mode == NOT_NEIGHBOR && info->event != REMOVE_NEIGHBOUR) && neighbor_table.count < MAX_NEIGHBOR_COUNT)
		{
			for(i=0; i<MAX_NEIGHBOR_COUNT; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
				{
					i=MAX_NEIGHBOR_COUNT;
				}
			}

				if(info->adr.addr_type==ADDR_802_15_4_PAN_LONG)
						length+=2;

				for(j=0; j < length ; j++)
				{
					b->address[j] = info->adr.address[j];
				}
				/* add lqi value to neighbor */
				b->last_sqn  =    info->last_sqn;			
				sqn_check=1;
				b->ttl=TTL;
				b->type = info->adr.addr_type;
				b->tx_err_cnt=0;
				/* Increace Neigbor count */
				neighbor_table.count++;
				debug("added new\r\n");
		}
	return sqn_check;
}

/**
 * Update neighbor & routing table TTL when unicast TX working.
 *
 * \param type indicates type of address
 * \param address 
 */
void update_tables_ttl(sockaddr_t *adr)
{
	uint8_t i, length=0, final_length=0;
	neighbor_info_t *b;
#ifdef HAVE_ROUTING
	route_info_t *ptr;
#endif
		if(adr->addr_type ==ADDR_802_15_4_PAN_LONG)
		{
			length=8;
			final_length =10;					
		}
		if(adr->addr_type == ADDR_802_15_4_PAN_SHORT)
		{
			length=4;
			final_length=4;
		}
		if(neighbor_table.count > 0)
		{
			for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
					b=0;
	
				if(b && (adr->addr_type == b->type))
				{
					if(memcmp(b->address, adr->address,length) == 0)
					{
						if(b->tx_err_cnt)
						{
							b->tx_err_cnt--;
						}
						b->ttl=TTL;
						i=MAX_NEIGHBOR_COUNT;
					}
				}
			}
		}
#ifdef HAVE_ROUTING
		if(routing_table.count > 0)
		{
			for(i=0; i < MAX_ROUTE_INFO_COUNT ; i++)
			{
				ptr = &(routing_table.route_info[i]);
				if(ptr->dest_addr_type == ADDR_NONE)
					ptr=0;

				if(ptr && (adr->addr_type == ptr->next_hop_addr_type))
				{
					/* compare next hop address */
					if(memcmp(adr->address, ptr->next_hop, final_length) ==0)
					{
						ptr->ttl=ROUTING_TTL;
					}
				}
			}
		}
#endif
}

/**
 * Check destination address from neighbor.
 *
 * IP-layer uses this function when it has to forward packet.
 *
 * \param type indicates type of destination address mode
 * \param address destination address
 *
 * \return NEIGHBOR when address was already in list.
 * \return NO_SUPPORT when address type not valid.
 * \return NEIGHBOR_LOW_RSSI when link is asymmetric
 */
dest_delivery_t check_neighbour_table(sockaddr_t *adr)
{
	uint8_t i,j, length;
	dest_delivery_t delivery_mode = NOT_NEIGHBOR;
	neighbor_info_t *b;
	
		if(neighbor_table.count > 0)
		{
			if(adr->addr_type == ADDR_802_15_4_PAN_SHORT)
				length=2;
			else
				length=8;
			for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
					b=0;

				if(b && (b->type == adr->addr_type))
				{
					if(memcmp(b->address, adr->address,length) == 0)
					{
						if(b->tx_err_cnt)
							delivery_mode = NEIGHBOR_LOW_RSSI;
						else
							delivery_mode = NEIGHBOR;
						for(j=0; j<2; j++)
						{
							adr->address[length +j] = b->address[length+j];
						}
						i=MAX_NEIGHBOR_COUNT;
					}
				}
			}
		}
		else
			delivery_mode = NOT_NEIGHBOR;

	return delivery_mode;	
}

/**
 * Check route for destination address.
 *
 * If destination address is not neighbor IP-layer checks route to destination.
 *
 * \param *info pointer for route information sturucture
 */
#ifdef HAVE_ROUTING
portCHAR check_routing_table( namomesh_info_t *info)
{
	uint8_t i, j, final_length, next_hop_addr_length=0;
	portCHAR ret_val=pdFALSE;
	route_info_t *ptr;
	/* Check destination address from routing_table */
	
	if(routing_table.count > 0)
	{
		if(info->adr.addr_type==ADDR_802_15_4_PAN_LONG)
			final_length=8;
		else
			final_length=2;
		for(i=0; i < MAX_ROUTE_INFO_COUNT ; i++)
		{
			ptr = &(routing_table.route_info[i]);
			if(ptr->dest_addr_type == ADDR_NONE)
				ptr=0;
			if(ptr && (info->adr.addr_type == ptr->dest_addr_type))
			{
				if(memcmp(ptr->destination, info->adr.address,final_length) == 0)
				{
					if(ptr->next_hop_addr_type == ADDR_802_15_4_PAN_LONG)
						next_hop_addr_length=10;
					else
						next_hop_addr_length=4;

					for(j=0; j < next_hop_addr_length; j++)
					{
						info->adr2.address[j] = ptr->next_hop[j];
					}
					info->adr2.addr_type = ptr->next_hop_addr_type;
					info->hop = ptr->hop_count;
					i=MAX_ROUTE_INFO_COUNT;
					ret_val=pdTRUE;
				}
			}	
		}
	}
	return ret_val;
}
#endif

/**
 * Update route info to routing table.
 *
 * If IP-layer detects that distance to originator device is over 1, then layer updates route info to routing table.
 *
 * \param *info pointer for route information
 */
#ifdef HAVE_ROUTING
portCHAR update_routing_table(namomesh_info_t *info)
{
	uint8_t i=0, j,tmp_8=0, final_length=0, next_hop_length=0, compare=0, update=0;
	route_info_t *ptr=0;
	neighbor_info_t *b;

		if(info->adr.addr_type ==ADDR_802_15_4_PAN_LONG)
				final_length=8;
		else
			final_length=2;
		if(info->adr2.addr_type==ADDR_802_15_4_PAN_LONG)
			next_hop_length=8;
		else
			next_hop_length=4;


		tmp_8 = 0;
		/* Predict older route information and shuold use route */
		if(info->event != REMOVE_ROUTE && info->event != ROUTE_ERR)
		{
			
			if(neighbor_table.count > 0)
			{
				for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
				{
					b=&(neighbor_table.neighbor_info[i]);
					if(b->type==ADDR_NONE)
						b=0;
		
					if(b && (b->type == info->adr.addr_type))
					{
						if(memcmp(b->address, info->adr.address,final_length) == 0)
						{
							if(b->tx_err_cnt > 2)
							{
								info->event = 0;
							}
							else
								tmp_8=1;
						}
					}
				}
			}
		}

		if(routing_table.count > 0 && tmp_8==0)
		{
			for(i=0; i < MAX_ROUTE_INFO_COUNT ; i++)
			{
				ptr = &(routing_table.route_info[i]);
				if(ptr->dest_addr_type == ADDR_NONE)
					ptr=0;
				/* Check originator address from routing table */
				if(ptr && (info->adr.addr_type == ptr->dest_addr_type))
				{
					if(memcmp(ptr->destination,info->adr.address,final_length) ==0)
					{
						if(info->event == REMOVE_ROUTE)
						{
							
								ptr->dest_addr_type = ADDR_NONE;
								routing_table.count--;
								//return pdTRUE;
						}
						else if(info->event == ROUTE_ERR)
						{
							if(ptr->ttl > 2)
							{
								ptr->ttl -= 2;
							}
							else
							{
								ptr->dest_addr_type = ADDR_NONE;
								routing_table.count--;
							}
							//return pdTRUE;
						}
						else
						{
							if(info->adr2.addr_type == ptr->next_hop_addr_type)
							{
								/* compare next hop address */
								if(memcmp(info->adr2.address, ptr->next_hop, next_hop_length) !=0)
									compare=1;
								else
									update=2;
							}
							else
								compare=1;
	
							if(compare)
							{
#if 0
								compare = 0;
								/* Check next hop link quality */
								for(j=0; j < MAX_NEIGHBOR_COUNT ; j++)
								{
									b=&(neighbor_table.neighbor_info[j]);
									if(b->type==ADDR_NONE)
										b=0;
							
									if(b && (b->type == next_hop->addr_type))
									{
										if(memcmp(b->address, next_hop->address,next_hop_length) ==0)
										{
											if(b->tx_err_cnt < 2) /* Compare hop is valid */
											{
												compare=1;
											}
										}
									}
								}*/

#endif
								if(info->hop < ptr->hop_count)
								{
									update=1;	
								}
								/* First in Use when have a problem */
								else
								{
									if(info->hop == ptr->hop_count)
									{
										//update=1;
										if(info->lqi > ptr->last_lqi)
										{
											update=1;
										}
									}
								}
#if 0
								else
								{
									if(hop_count==ptr->hop_count && compare)
									{
										update=1;
										//if(last_rssi > ptr->last_rssi || (ptr->ttl  < (ROUTING_TTL - 2)  ))
											//update=1;
										/* Check current next hop link quality */
										/*for(j=0; j < MAX_NEIGHBOR_COUNT ; j++)
										{
											b=&(neighbor_table.neighbor_info[j]);
											if(b->type==ADDR_NONE)
												b=0;
									
											if(b && (b->type == ptr->next_hop_addr_type))
											{
												if(memcmp(b->address, ptr->next_hop,next_hop_length) ==0)
												{
													if(b->tx_err_cnt ) 
													{
														update=1;
													}
												}
											}
										}*/
									}
								}
#endif
							}

							if(update)
							{
								if(update != 2)
								{
									ptr->next_hop_addr_type = info->adr2.addr_type;
									next_hop_length+=2;
									/* added new next hop info */
									for(j=0; j < next_hop_length ; j++)
									{
										ptr->next_hop[j] = info->adr2.address[j];
									}
								}
								ptr->last_lqi=info->lqi;
								ptr->hop_count = info->hop;
								ptr->ttl=ROUTING_TTL;
							}
						}
						tmp_8=1;
						i=MAX_ROUTE_INFO_COUNT;
						//return pdTRUE;
					}
				}	
			}
		}
		
		if(info->event==0 && (tmp_8==0 && routing_table.count < MAX_ROUTE_INFO_COUNT ))
		{	
			for(i=0; i<MAX_ROUTE_INFO_COUNT; i++)
			{
				ptr = &(routing_table.route_info[i]);
				if(ptr->dest_addr_type == ADDR_NONE)
				{
					i=MAX_ROUTE_INFO_COUNT;
				}
			}
			for(j=0; j < final_length ; j++)
			{
				ptr->destination[j] = info->adr.address[j];	
			}
			next_hop_length+=2;
			for(j=0; j < next_hop_length ; j++)
			{
				ptr->next_hop[j] = info->adr2.address[j];
			}
			ptr->next_hop_addr_type = info->adr2.addr_type;
			ptr->dest_addr_type = info->adr.addr_type;
			ptr->hop_count = info->hop;
			ptr->last_lqi = info->lqi;
			ptr->ttl=ROUTING_TTL;
			routing_table.count++;
		}
return pdTRUE;
}
#endif

/**
 * Checkout routing- and neighbor tables time-stamp.
 *
 * If function detect that TTL, time to live value is coming zero it delete route or neighbor-info.
 *
 */
void check_tables_status(void)
{
	uint8_t i;
	neighbor_info_t *b;
#ifdef HAVE_ROUTING
	route_info_t *ptr;
#endif
		if(neighbor_table.count)
		{
			for(i=0; i < MAX_NEIGHBOR_COUNT; i++)
			{
				b=&(neighbor_table.neighbor_info[i]);
				if(b->type==ADDR_NONE)
					b=0;

				if(b)
				{
					if(b->ttl > 1)
						b->ttl--;
					else
					{
						b->type=ADDR_NONE;
						b->tx_err_cnt=0;
						neighbor_table.count--;
						
					}
				}
			}
		}
#ifdef HAVE_ROUTING
		if(routing_table.count)
		{
			for(i=0; i < MAX_ROUTE_INFO_COUNT; i++)
			{
				ptr=&(routing_table.route_info[i]);
				if(ptr->dest_addr_type==ADDR_NONE)
					ptr=0;

				if(ptr)
				{
					if(ptr->ttl > 1)
						ptr->ttl--;
					else
					{
						ptr->dest_addr_type=ADDR_NONE;
						routing_table.count--;
					}
				}
			}
		}
#endif
}
