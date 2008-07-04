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
 * \file neigbor_routing_table.h
 * \brief control_message type defined
 *
 *  nanoStack: These tables are for zigbee and 6lowpan modules
 *   
 *	
 */

 
#ifndef NEIGHBOR_ROUTING_TABLE_H
#define NEIGHBOR_ROUTING_TABLE_H
#ifdef NEIGHBOR_MAX
#define MAX_NEIGHBOR_COUNT	NEIGHBOR_MAX
#else
#define MAX_NEIGHBOR_COUNT	20
#endif

#ifdef ROUTE_MAX
#define MAX_ROUTE_INFO_COUNT	ROUTE_MAX
#else
#define MAX_ROUTE_INFO_COUNT	15
#endif

#define FLOODING_INFO_SIZE 16

#include "address.h"

/** Neighbor info structure. */
typedef struct
{
	address_t address;					/*!< neighbor address. */
	addrtype_t type;					/*!< address type. */
	uint8_t last_sqn;					/*!< Last sqn for detect duplicated packet */
	uint8_t ttl;						/*!< Neighbor infos Time to Live */
	uint8_t tx_err_cnt;					/*!< Indicate TX errors to neighbour */
} neighbor_info_t;

/** Neighbor tables structure. */
typedef struct
{
	uint8_t				count;									/*!< count of neighbor info. */
	neighbor_info_t 	neighbor_info[MAX_NEIGHBOR_COUNT];		/*!< list of neighbor infos. */	
} neighbor_table_t;
/** NanoMesh info structure. */
typedef struct
{
	sockaddr_t		adr;			/*!< primary address. */
	sockaddr_t		adr2;			/*!< secondary address. */
	uint8_t			last_sqn;		/*!< sequency numeber for nanomesh event. */							
	uint8_t			event;			/*!< indicate nanomesh event. */
	uint8_t 		hop;			/*!< Hop to orginator. */
	uint8_t			lqi;			/*!< LQI for next hop. */
} namomesh_info_t;


/** Flooding filter. */
typedef struct
{
	uint8_t		address[8];				/*!< device address. */
	addrtype_t type;					/*!< address type. */
	uint8_t last_sqn;					/*!< Last sqn for detect duplicated packet */
	uint8_t hop;						/*!< hop count. */
} flooding_info_t;

/** Flooding tables structure. */
typedef struct
{
	uint8_t				count;									/*!< count of neighbor info. */
	flooding_info_t	 	info[FLOODING_INFO_SIZE];								/*!< list of neighbor infos. */	
} flooding_table_t;

/** Delivery mode enumeration for neighbortable update function. */
typedef enum
{
	BROADCAST = 0,		/*!< address is broadcast. */
	NEIGHBOR,			/*!< address is Neighbors. */
	NEIGHBOR_LOW_RSSI,	/*!< address is Neighbors which have low RSSI value. */
	NOT_NEIGHBOR,		/*!< address is not neighbors. */
	NEW_NEIGHBOR,		/*!< address is not neighbors. */
    ROUTED_MESSAGE,
	NO_SUPPORT		/*!< No support. */
}dest_delivery_t;


typedef enum
{
	NANOMESH_BROADCAST = 0,		
	NANOMESH_FORWARD,
	NANOMESH_NEIGHBOUR,
	NANOMESH_NO_ROUTE
}nano_mesh_response_t;

/** Destination match type enumeration for received packet. */
typedef enum
{
	BCAST = 0xc0,		/*!< Destination address is broadcast --> use mesh routing. */
	OWN,				/*!< Destination address is device own. */
	NOT_OWN,			/*!< Destination address not own --> check route (neighbor/routing-table). */
	DISCARD				/*!< When header or bco sgn not valid. */
}match_type_t;

/** Gateway information structure. */
typedef struct
{
	addrtype_t	address_type;		/*!< Gateways address type. */
	address_t	address;			/*!< Gateways address. */	
	uint8_t		hop_distance;		/*!< Hop distance to Gateway. */
	uint8_t		lqi;				/*!< LQI to next-hop. */
	uint8_t		ttl;				/*!< Time live value that Gateway is valid. */
}gateway_info_t;


/** Gateway cache. */
typedef struct
{
	uint8_t				count;				/*!< Count of valid Gateways. */
	gateway_info_t		gateway_info[2];	/*!< Gateway information array. */
}gateway_cache_t;


#ifdef HAVE_ROUTING
/** Route info structure. */
typedef struct
{
	addrtype_t	dest_addr_type;		/*!< final destinatio address type. */
	uint8_t 	destination[8];		/*!< destination address. */
	addrtype_t	next_hop_addr_type;	/*!< next-hop address type. */
	address_t	next_hop;			/*!< next-hop address. */
	uint8_t 	hop_count;			/*!< Hop count to destination. */
	uint8_t 	ttl;				/*!< Routing infos Time to Live */
	uint8_t 	last_lqi;			/*!< Last LQI value for route info */
} route_info_t;

/** Routing tables structure. */
typedef struct
{
	uint8_t			count;		/*!< address destination count. */
	route_info_t 	route_info[MAX_ROUTE_INFO_COUNT];	/*!< list of route. */
} routing_table_t;
#endif
/** Route check functions response structure. */
typedef struct
{
	addrtype_t	address_type;		/*!< next-hop address type. */
	address_t	next_hop;			/*!< next-hop address. */
	uint8_t		hop_count;			/*!< Hop count to destination. */
}route_check_t;





/* Functions prototypes for handling routing and neighbor tables. */
extern void check_tables_status(void);
extern dest_delivery_t check_neighbour_table(sockaddr_t *adr);
extern uint8_t update_neighbour_table(namomesh_info_t *info);

extern void routing_init(void);
extern void update_tables_ttl(sockaddr_t *adr);
#ifdef HAVE_ROUTING
extern portCHAR update_routing_table(namomesh_info_t *info);
extern portCHAR check_routing_table( namomesh_info_t *info);
#endif

#endif /*NEIGHBOR_ROUTING_TABLE_H*/
