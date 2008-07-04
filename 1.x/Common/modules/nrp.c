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
 * \file     nrp.c
 * \brief    nRP protocol module.
 *
 *  The nRoute protocol module: module definition.
 *   
 */

/*start module config*/
/*
[NAME]
NRP

[ID]
MODULE_NRP,

[INFO]
#ifdef HAVE_NRP
  {nrp_init, nrp_handle, nrp_check, 0, MODULE_NRP, 0, ADDR_NONE, 0 },
#endif

[FUNCS]*/
extern portCHAR nrp_init(buffer_t *buf);
extern portCHAR nrp_handle( buffer_t *buf );
extern portCHAR nrp_check( buffer_t *buf );


/*end module config*/

