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


#ifndef _MODULE_ID_H
#define _MODULE_ID_H
/* module_id.h generated */

typedef enum { MODULE_NONE = 0,
MODULE_CIPV6,
MODULE_CUDP,
MODULE_ICMP,
MODULE_MAC_15_4,
MODULE_NANOMESH,
MODULE_NRP,
MODULE_SSI,
MODULE_APP,
MODULE_MAX } module_id_t;
#endif  /*_MODULE_ID_H*/
