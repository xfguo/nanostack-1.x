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


/* modules_conf.h generated */

#ifdef COMPILING_MODULE_C
extern portCHAR cipv6_init(buffer_t *buf);
extern portCHAR cipv6_handle( buffer_t *buf );
extern portCHAR cipv6_check( buffer_t *buf );

extern portCHAR cudp_init(buffer_t *buf);
extern portCHAR cudp_handle( buffer_t *buf );
extern portCHAR cudp_check( buffer_t *buf );

extern portCHAR icmp_init(buffer_t *buf);
extern portCHAR icmp_handle( buffer_t *buf );
extern portCHAR icmp_check( buffer_t *buf );

extern portCHAR mac_init(buffer_t *buf);
extern portCHAR mac_handle( buffer_t *buf );
extern portCHAR mac_check( buffer_t *buf );

extern portCHAR nanomesh_init(buffer_t *buf);
extern portCHAR nanomesh_handle( buffer_t *buf );
extern portCHAR nanomesh_check( buffer_t *buf );

extern portCHAR nrp_init(buffer_t *buf);
extern portCHAR nrp_handle( buffer_t *buf );
extern portCHAR nrp_check( buffer_t *buf );

extern portCHAR ssi_init(buffer_t *buf);
extern portCHAR ssi_handle( buffer_t *buf );
extern portCHAR ssi_check( buffer_t *buf );


module_t modules[] = {
#ifdef HAVE_CIPV6
  {cipv6_init, cipv6_handle, cipv6_check, 0, MODULE_CIPV6, 41, ADDR_802_15_4_LONG, 0 },
#endif
#ifdef HAVE_CUDP
  {cudp_init, cudp_handle, cudp_check, 0, MODULE_CUDP, 8, ADDR_NONE, 0 },
#endif
#ifdef HAVE_ICMP
  {icmp_init, icmp_handle, icmp_check, 0, MODULE_ICMP, 16, ADDR_NONE, 0 },
#endif
#ifdef HAVE_MAC_15_4
  {mac_init, mac_handle, mac_check, 0, MODULE_MAC_15_4, 23, ADDR_802_15_4_PAN_LONG, 0 },
#endif
#ifdef HAVE_NANOMESH
  {nanomesh_init, nanomesh_handle, nanomesh_check, 0, MODULE_NANOMESH, 16, ADDR_NONE, 0 },
#endif
#ifdef HAVE_NRP
  {nrp_init, nrp_handle, nrp_check, 0, MODULE_NRP, 0, ADDR_NONE, 0 },
#endif
#ifdef HAVE_SSI
  {ssi_init, ssi_handle, ssi_check, 0, MODULE_SSI, 0, ADDR_NONE, 0 },
#endif
{0, 0, 0, 0, MODULE_NONE, 0, ADDR_NONE, 0 } };
#else
extern module_t modules[];
#endif
