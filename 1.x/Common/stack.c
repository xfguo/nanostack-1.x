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
 * \file stack.c
 * \brief Protocol stack main module.
 *
 *  Protocol stack main module: handling of the buffer/event queue,
 *  buffer management, stack event poll.
 *   
 */


#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <sys/inttypes.h>

#include "stack.h"
#include "buffer.h"
#include "socket.h"
#include "module.h"

#ifndef STACK_DEBUG
#undef HAVE_DEBUG
#endif
#include "debug.h"

#include "control_message.h"
#include "rf.h"
#include "gpio.h"
/*
#ifndef STACK_TASK_SIZE
#ifdef SDCC_CC2430
#define STACK_TASK_SIZE configMAXIMUM_STACK_SIZE
#else
#define STACK_TASK_SIZE 300
#endif
#endif
*/
#ifndef STACK_PRIORITY

#define STACK_PRIORITY (tskIDLE_PRIORITY + 2)
#endif

#ifndef STACK_BUFFERS_MAX
#define STACK_BUFFERS_MAX 8
#endif

#ifndef STACK_BUFFERS_MIN
#define STACK_BUFFERS_MIN 4
#endif

#define STACK_RING_BUFFER_SIZE  ( STACK_BUFFERS_MAX + 1 )

/** Common variables */
#ifndef STACK_RING_BUFFER_MODE
xQueueHandle     buffers;
#else
buffer_t *stack_buffer_pool[STACK_RING_BUFFER_SIZE];
volatile uint8_t stack_buffer_wr, stack_buffer_rd;
#endif 

xQueueHandle     events = 0;
stack_event_t event_queue = 0;
 
extern socket_t  sockets[];
extern module_t modules[];
extern sockaddr_t mac_long;

uint8_t n_buffers;
#ifdef HAVE_MAC_15_4
stack_t stacks[] =
{
#ifdef HAVE_NRP
	
	{ 2, { MODULE_MAC_15_4, MODULE_NRP }},
#ifdef HAVE_CIPV6
#ifdef HAVE_ICMP
	{ 4, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_ICMP, MODULE_NRP }},
#endif	/*ICMP*/
#ifdef HAVE_CUDP
	{ 4, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_CUDP, MODULE_NRP}},
#endif	/*CUDP*/
#endif	/*CIPV6*/	
			
#else /*NRP*/

#ifdef HAVE_CIPV6
#ifdef HAVE_ICMP
	{ 3, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_ICMP }},
#endif	/*ICMP*/
#ifdef HAVE_CUDP
	{ 3, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_CUDP}},
#ifdef HAVE_SSI
 	{ 4, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_CUDP, MODULE_SSI }},
#endif	/*SSI*/
#endif	/*CUDP*/
#endif	/*CIPV6*/

#endif /*NRP*/
	{ 0, { 0 } }
}; 

#else
stack_t stacks[] =
{

#ifdef HAVE_NRP
#ifdef	HAVE_RF_802_15_4
  { 2, { MODULE_RF_802_15_4, MODULE_NRP }},
#ifdef HAVE_NUDP	
  { 3, { MODULE_RF_802_15_4, MODULE_NUDP, MODULE_NRP}},
#endif
#endif

#ifdef	HAVE_802_15_4_RAW
  { 2, { MODULE_802_15_4_RAW, MODULE_NRP }},
#ifdef HAVE_NUDP	
  { 3, { MODULE_802_15_4_RAW, MODULE_NUDP, MODULE_NRP}},
#endif
#endif

#ifdef HAVE_CUDP
#ifdef HAVE_CIPV6
#ifdef	HAVE_RF_802_15_4
  	{ 4, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_CUDP, MODULE_NRP }},
#endif	/*RF802_15_4*/
#ifdef HAVE_ICMP
	{ 3, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_ICMP }},
#endif
#ifdef	HAVE_802_15_4_RAW
  	{ 4, { MODULE_802_15_4_RAW, MODULE_CIPV6, MODULE_CUDP, MODULE_NRP }},
#endif	/*802_15_4_RAW*/
#endif /* HAVE_CIPV6 */
#endif /* HAVE_CUDP */

#else /*HAVE_NRP*/



#ifdef HAVE_CUDP
#ifdef HAVE_CIPV6
#ifdef	HAVE_RF_802_15_4
	{ 3, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_CUDP}},
#ifdef HAVE_ICMP
	{ 3, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_ICMP }},
#endif
#ifdef HAVE_SSI
  	{ 4, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_CUDP, MODULE_SSI }},
#endif
#endif	/*RF802_15_4*/
#ifdef	HAVE_802_15_4_RAW
  	{ 3, { MODULE_802_15_4_RAW, MODULE_CIPV6, MODULE_CUDP }},
#endif	/*802_15_4_RAW*/

#endif /* HAVE_CIPV6 */
#endif /* HAVE_CUDP */

#ifdef HAVE_RS232  
#ifdef HAVE_NUDP	
  { 2, { MODULE_RS232, MODULE_NUDP }},
#endif
#ifdef HAVE_NTCP			  
  { 2, { MODULE_RS232, MODULE_NTCP }},
#endif
#endif	/*RS232*/

#ifdef HAVE_ETH  
#ifdef HAVE_NUDP	
  { 2, { MODULE_ETH, MODULE_NUDP }},
#endif
#ifdef HAVE_NTCP			  
  { 2, { MODULE_ETH, MODULE_NTCP }},
#endif
#endif  /*ETH*/

#ifdef HAVE_802_15_4_RAW  
#ifdef HAVE_NUDP	
  { 2, { MODULE_802_15_4_RAW, MODULE_NUDP }},
#ifdef HAVE_SSI
  { 3, { MODULE_802_15_4_RAW, MODULE_NUDP, MODULE_SSI }},
#endif	/*HAVE_SSI*/

#endif
#ifdef HAVE_NTCP			  
  { 2, { MODULE_802_15_4_RAW, MODULE_NTCP }},
#endif

#endif	/*RF802_15_4_RAW*/

#ifdef HAVE_RF_802_15_4  
#ifdef HAVE_NUDP	
  { 2, { MODULE_RF_802_15_4, MODULE_NUDP }},
#endif


#ifdef HAVE_NTCP			  
  { 2, { MODULE_RF_802_15_4, MODULE_NTCP }},
#endif
  
#endif	/*RF802_15_4*/

#endif	/*NRP*/
	{ 0, { 0 } }
}; 
#endif

#define STACKS_MAX (sizeof(stacks)/sizeof(stack_t))

/** Internals */
void stack_main ( void *pvParameters );
void stack_buffer(buffer_t *b);

void stack_module_call(uint8_t stack_id, uint8_t layer, buffer_t *b);
portCHAR stack_module_check(uint8_t stack_id, uint8_t layer, buffer_t *b);
extern xTaskHandle mac_task_handle;
xTaskHandle core_handle;
#ifdef MALLFORMED_HEADERS
uint8_t mallformed_headers_cnt=0;
#endif
#ifdef STACK_RING_BUFFER_MODE



int8_t stack_buffer_count(void)
{
	int8_t size = stack_buffer_wr - stack_buffer_rd;
	if (size < 0) size += STACK_BUFFERS_MAX;
	return size;
}

/**
 *  Free buffer in the inrerrupt action.
 * 
 *  Use only in the interrupt and very carefully (STACK_RING_BUFFER_MODE enabled by default)
 *
 *	\param b  pointer to buffer dont give 0 pointer
 *
 *  \return	pdFALSE, when buffer pool full
 *  \return	pdTRUE, when buffer free ok.
 */
portCHAR stack_buffer_add(buffer_t *b)
{
	uint8_t tmp;
	if(b == 0)
	{
		return pdFALSE;	
	}
	tmp = stack_buffer_wr;
	tmp++;

	if(tmp >= STACK_RING_BUFFER_SIZE)
	{
		tmp=0;
	}
	if(tmp != stack_buffer_rd)
	{
		stack_buffer_pool[stack_buffer_wr] = b;
		stack_buffer_wr = tmp;
	}
	else
	{//Buffer full, err
		return pdFALSE;
	}
	return pdTRUE;
}
/**
 *  Get buffer in the inrerrupt action.
 * 
 *  Use only in the interrupt (STACK_RING_BUFFER_MODE enabled by default)
 *
 *
 *  \return	0 when no buffers available
 *  \return	pointer to buffer
 */
buffer_t *stack_buffer_pull(void)
{
	buffer_t *b;
	uint8_t tmp = stack_buffer_rd;
	
	if (tmp == stack_buffer_wr) return 0;
	b = stack_buffer_pool[tmp];
	stack_buffer_pool[tmp++] = 0;
	if (tmp >= STACK_RING_BUFFER_SIZE) tmp = 0;
	stack_buffer_rd = tmp;
	
	return b;
}
#endif

/**
 * Initialize protocol stack and start stack task.
 * Allocate initial set of buffers.
 *
 * \return pdTRUE
 * \return pdFALSE	insufficient memory
 */
portCHAR stack_init( void )
{
	uint8_t i;
	buffer_t *b;
#ifndef STACK_RING_BUFFER_MODE
  	buffers = xQueueCreate( STACK_BUFFERS_MAX, sizeof( buffer_t * ) );
#else
	stack_buffer_wr = stack_buffer_rd = 0;
#endif
  events = xQueueCreate( STACK_BUFFERS_MAX + 20, sizeof( event_t ) );
	n_buffers = 0;
	
	xTaskCreate( stack_main, "Stack", configMAXIMUM_STACK_SIZE, NULL, STACK_PRIORITY, &core_handle );
  /* Initialize modules */
  debug("Stack: mod inits.\r\n");
	
	i = 0;
	while (i < STACK_BUFFERS_MAX)
	{	/*Allocate a maximum of STACK_BUFFERS_MIN buffers */
		b = pvPortMalloc(sizeof(buffer_t)+BUFFER_SIZE);
		if (b)
		{
  			memset(b, 0, sizeof(buffer_t));
			b->size = BUFFER_SIZE;
			n_buffers++;
#ifdef STACK_RING_BUFFER_MODE
			if(stack_buffer_add(b) == pdFALSE)
			{
				return pdFALSE;
			}
#else
  			if (xQueueSend( buffers, ( void * ) &b,
					(portTickType) 0 ) == pdFALSE)
			{ 
				debug("Stack: buffers fail.\r\n");
				return pdFALSE;
			}
#endif
		}
		else
		{   
			debug("Stack: buffer allocation failed.\r\n");
			return pdFALSE;
		}
		if (i++ >= STACK_BUFFERS_MIN) i = STACK_BUFFERS_MAX;
	}
	debug_int(uxQueueMessagesWaiting(buffers));
	debug(" buffers available.\r\n"); 
	
	socket_init();
	#ifndef HAVE_NRP
	event_queue = xQueueCreate( 2, sizeof( buffer_t * ) );
	#endif
	{
		sockaddr_t init_addr;				/* Make sure that we have mac defined */
		init_addr.addr_type = ADDR_NONE;
		rf_mac_get(&init_addr);
		if(init_addr.addr_type != ADDR_NONE)
		{
			rf_set_address(&init_addr);
		}
	}
	
	mac_mem_alloc();
	return module_init();
}

/**
 * Initialize protocol stack logical type.
 * 
 *  !!! OLD API CALL !!!
 *  !!! Since release v1.1 dont use just use stack_init().
 * 
 *\param stack_parameters pointer for start parameters, 0 start stack on the default mode.
 * 
 * \return START_SUCCESS when start parameters are valid.
 * \return CHANNEL_NOT_SUPPORTED when given parameter is not valid.
 * \return TYPE_NOT_SUPPORTED when logical type is not supported.
 */
start_status_t stack_start(stack_init_t  *stack_parameters)
{
	if(stack_parameters!=NULL)
	{
		vPortFree(stack_parameters);
	}
	if(!(events))
	{
		stack_init(); /* Initialize stack core */
	}
	return START_SUCCESS;
}



/**
 * The main stack task
 *
 *	\param pvParameters 	not used
 *
 */
void stack_main ( void *pvParameters )
{
	event_t event;
	portTickType xLastWakeTime;
	
	xLastWakeTime = (portTickType) pvParameters;
	
  	xLastWakeTime = xTaskGetTickCount();
	
	vTaskDelayUntil( &xLastWakeTime, 200 / portTICK_RATE_MS );
	
	if (events == 0 )
	{
		debug("Stack: Initialization failure. Halted.\r\n");
		for(;;)
		{
			vTaskDelayUntil( &xLastWakeTime, 5000 / portTICK_RATE_MS );
		}
	}
	debug("Stack: Go.\r\n");

	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		if (xQueueReceive(events, &( event ), (portTickType) 600 / portTICK_RATE_MS) == pdTRUE)
		{
			if (event.process == 0)
			{	/*it is a buffer*/
				stack_buffer((buffer_t *) event.param);
			}
			else
			{	/*process event callback*/
				event.process(event.param);
			}
			
		}	/* end event handling */
		if ((xTaskGetTickCount() - xLastWakeTime)*portTICK_RATE_MS > 15000)
		{
			check_tables_status();
			xLastWakeTime = xTaskGetTickCount();
		}
	} /* end task loop */
}


/**
 * Buffer handler
 *
 *	\param b pointer to buffer
 */
void stack_buffer(buffer_t *b)
{
	portCHAR status = pdFAIL;
	uint8_t i;
	uint8_t found = 0;
	socket_t *si = (socket_t *) b->socket;
	
	if (b == 0) return;
	
	if ((b->to != MODULE_NONE) && (b->to != MODULE_APP))
	{	/*protocol identifier used*/
		debug_printf("buffer to: %d.\r\n", b->to);
		if (module_call(b->to, b) == pdFALSE)
		{
			debug("Packet refused by module -> free.\r\n");
			stack_buffer_free(b);
		}
	}
	else if (b->dir == BUFFER_DOWN)
	{	/*go layer down*/
		debug_printf("down:  from %d to %d.\r\n", b->from, b->to);
		if (si)
		{
			debug_printf("using socket:  stack %d.\r\n", si->stack_id);
			if ((b->from == MODULE_APP) || (b->from == MODULE_NRP))
			{	/*call top layer*/
				b->from = (module_id_t) stacks[si->stack_id].layers;
				stack_module_call(si->stack_id, (b->from) - 1, b);
				b=0;
			}
			else
			{
				i=0;
				while ( (i < stacks[si->stack_id].layers) && b)
				{
					if (stacks[si->stack_id].module[i] == (module_id_t) b->from)
					{
						stack_module_call(si->stack_id, i-1, b);
						b = 0;
					}
					i++;
				}
			}
			if(b)
			{
				debug("Stack: from layer 0! Free buffer.\r\n");
				stack_buffer_free(b);
				b=0;
			}
		}
		else
		{
			i = 0;
			debug("Stack: no socket, scan.\r\n");
			while (b && (stacks[i].layers > 0) )
			{
				uint8_t j = 1;
				
				while ( (j < stacks[i].layers) && b)
				{
					if (stacks[i].module[j] == (module_id_t) b->from)
					{
						debug_printf("Stack: call module %d.\r\n", stacks[i].module[j-1]);
						stack_module_call(i, j-1, b);
						b = 0;
					}
					j++;
				}
				i++;
			}
			
			if (b)
			{
				debug("Stack: no stack found, free buffer.\r\n");
				stack_buffer_free(b);
			}
		}
	}
	else /* if (b->dir == BUFFER_UP)*/
	{	/*one layer up*/
		uint8_t next_layer;
		debug_printf("up:  from %d to %d.\r\n", b->from, b->to);
		i = 0;
		status = socket_up(b);
		if (status != pdTRUE)
		{               					 
			while (b && (stacks[i].layers > 0))
			{
				uint8_t j;
				debug_printf("Scanning stack %d.\r\n", i);
				j=0;
				next_layer = LAYERS_MAX + 1 ;
				while (j < (stacks[i].layers-1))
				{
					debug_printf("Scanning layer %d.\r\n", j);
					if (stacks[i].module[j] == (module_id_t) b->from)
					{
						next_layer = j+1;
						j = stacks[i].layers;
					}
					else j++;
				}
				if (next_layer != (LAYERS_MAX + 1))
				{
					debug_printf("Hit module: %d.\r\n", next_layer);
					status = stack_module_check(i, next_layer, b);
					if (status == pdTRUE)
					{
						stack_module_call(i, next_layer, b);
						b = 0;
					}
				}
				i++;
			}
		}
		
		if (status != pdTRUE)
		{
			if (!found)
			{
				debug("No module found.\r\n");
			}
			else
			{
				debug("Unknown packet(1) -> free.\r\n");
			}
			if (b != 0)
			{ 
				stack_buffer_free(b);
			}
		}
		debug("Handler: eject.\r\n");
	}
}

/**
 *  Call module handler. Free buffer if module can't be called.
 *
 *	\param stack_id   stack identifier
 *  \param layer      which layer is to be called
 *  \param b          pointer to buffer
 */
void stack_module_call(uint8_t stack_id, uint8_t layer, buffer_t *b)
{
	uint8_t i;
	uint8_t found = 0;
	if (layer < stacks[stack_id].layers)
	{	/*valid layer*/
		i = 0;
		while(modules[i].id)
		{
			if (modules[i].id == stacks[stack_id].module[layer])
			{
				found = 1;


				modules[i].handle(b);

				break;
			}
			else i++;
		}
		if (!found)
		{
			debug("Invalid module ID.\r\n");
			stack_buffer_free(b);
		}
	}
}

/**
 *  Call module check function. Return pdFALSE if module can't be called.
 *
 *	\param stack_id   stack identifier
 *  \param layer      which layer is to be called
 *  \param b          pointer to buffer
 */
portCHAR stack_module_check(uint8_t stack_id, uint8_t layer, buffer_t *b)
{
	uint8_t i;
		
	if (layer < stacks[stack_id].layers)
	{	/*valid layer*/
		i = 0;
		while(modules[i].id)
		{
			if (modules[i].id == stacks[stack_id].module[layer])
			{
				return modules[i].check(b);
			}
			else i++;
		}
	}
	return pdFALSE;
}

/**
 *  Get buffer from stack.
 * 
 *  Buffer allocation in the interrupt should use stack_buffer_pull() function which return buffer pointer.
 *
 *	\param blocktime  time to wait for buffer
 *
 *  \return           pointer to buffer, 0 on failure (all buffers allocated)
 */
buffer_t* stack_buffer_get ( portTickType blocktime )
{
  buffer_t *b=0;
#ifdef STACK_RING_BUFFER_MODE
	if( blocktime)
	{
		blocktime >> 2;
		if(!blocktime) blocktime=1;
	}
	while(b == 0)
	{
		portENTER_CRITICAL();
		b=stack_buffer_pull();
		portEXIT_CRITICAL();
		if(b)
		{
			b->options.type = BUFFER_DATA;
			b->dir = BUFFER_DOWN;
			b->socket = 0;
			b->buf_ptr = 0;
			b->buf_end = 0;
			b->options.lowpan_compressed=0;
			b->options.handle_type = HANDLE_DEFAULT;
			return b;
		}
		else if(blocktime)
		{
			vTaskDelay((portTickType) 8);
			blocktime--;
		}
		else
		{
			return 0;
		}
	}
#else

  if (xQueueReceive( buffers, &( b ), 0) == pdTRUE)
	{
		b->options.type = BUFFER_DATA;
		b->dir = BUFFER_DOWN;
		b->socket = 0;
		b->buf_ptr = 0;
		b->buf_end = 0;
		b->options.lowpan_compressed=0;
		b->options.handle_type = HANDLE_DEFAULT;
		return b;
	}
	/*else if (n_buffers < STACK_BUFFERS_MAX)
	{
		b = pvPortMalloc(sizeof(buffer_t)+BUFFER_SIZE);
		if (b)
		{
  			memset(b, 0, sizeof(buffer_t));
			b->options.type = BUFFER_DATA;
			b->dir = BUFFER_DOWN;
			b->size = BUFFER_SIZE;
			b->buf_ptr = 0;
			b->buf_end = 0;
			b->options.lowpan_compressed=0;
			b->options.handle_type = HANDLE_DEFAULT;
			n_buffers++;
			return b;
		}
	}*/
    else if (xQueueReceive( buffers, &( b ), blocktime / portTICK_RATE_MS) == pdTRUE)
	{
		b->options.type = BUFFER_DATA;
		b->dir = BUFFER_DOWN;
		b->socket = 0;
		b->buf_ptr = 0;
		b->buf_end = 0;
		b->options.lowpan_compressed=0;
		b->options.handle_type = HANDLE_DEFAULT;
		return b;
	}
#endif
	return 0;
}

#ifdef HAVE_DYNAMIC_BUFFERS
/**
 *  Allocate a smaller buffer from stack (for data storage).
 *
 *	\param size  size of buffer
 *
 *  \return           pointer to buffer, 0 on failure (no memory)
 */
buffer_t* stack_buffer_allocate( uint8_t size )
{
  	buffer_t *b; 
	b = pvPortMalloc(sizeof(buffer_t)+ size);
	if (b)
	{
		memset(b, 0, sizeof(buffer_t));
		b->options.type = BUFFER_DATA;
		b->dir = BUFFER_DOWN;
		b->size = size;
		b->buf_ptr = 0;
		b->buf_end = 0;
		b->socket = 0;
		b->from = MODULE_NONE;
		b->to = MODULE_NONE;
		b->options.lowpan_compressed=0;
		b->options.handle_type = HANDLE_DEFAULT;
		return b;
	}
	return 0;
}
#endif

/**
 *  Free stack buffer.
 *
 * *  Buffer free in the interrupt shuold use stack_buffer_add(buffert_t *b).
 * 
 * 
 *	\param b					pointer to buffer
 *
 */
portCHAR stack_buffer_free ( buffer_t *b )
{
	if (b)
  	{ 
		portCHAR ret_val;
		b->socket = 0;
		b->from = MODULE_NONE;
		b->to = MODULE_NONE;
		b->options.lowpan_compressed=0;
		b->options.handle_type = HANDLE_DEFAULT;
	#ifdef HAVE_DYNAMIC_BUFFERS
		if (b->size == BUFFER_SIZE)
		{
		#ifdef STACK_RING_BUFFER_MODE
			portENTER_CRITICAL();
			ret_val = stack_buffer_add(b);
			portEXIT_CRITICAL();
			return ret_val;
		#else
			xQueueSend( buffers, ( void * ) &b, ( portTickType ) 0 );
		#endif
		}
		else
		{
			vPortFree(b);
		}
	#else
		#ifdef STACK_RING_BUFFER_MODE
			portENTER_CRITICAL();
			ret_val = stack_buffer_add(b);
			portEXIT_CRITICAL();
			return ret_val;
		#else
			xQueueSend( buffers, ( void * ) &b, ( portTickType ) 0 );
		#endif
	#endif
	}
	return pdFALSE;
}

/**
 *  Push buffer to stack.
 *
 *	\param b  pointer to buffer
 *
 *  \return  pdTRUE
 *  \return  pdFALSE queue full
 */
portCHAR stack_buffer_push( buffer_t * b)
{
	event_t event;
	event.process = 0;
	event.param = (void *) b;
	
	return xQueueSend( events, ( void * ) &event, ( portTickType ) 0 );
}

/**
 *  Check buffer headroom.
 *
 *	\param b  pointer to buffer
 *	\param size  required headroom space
 *
 *  \return  pdTRUE
 *  \return  pdFALSE packet too long
 */
portCHAR stack_buffer_headroom( buffer_t * b, uint16_t size)
{
	if (b->buf_ptr < size)
	{
		if ((b->buf_end + size) >  b->size)
		{
			return pdFALSE;
		}
		memmove(&(b->buf[(b->buf_ptr)+size]), 
		&(b->buf[b->buf_ptr]), 
		buffer_data_length(b));
		b->buf_end += size;
		b->buf_ptr += size;
	}
	return pdTRUE;
}

/**
 *  Get number of stacks.
 *
 *  \return  STACKS_MAX
 */
uint8_t stack_number_get()
{
	return STACKS_MAX;
}


stack_event_t open_stack_event_bus(void)
{
	if(event_queue)
	{
		return event_queue;
	}
	else
	{
		return 0;	
	}
}

/**
 *  Waiting stack event messages from stack.
 *
 *  \return  pointer to event message.
 */
buffer_t * waiting_stack_event(uint16_t time)
{
	buffer_t *b;
	if(xQueueReceive(event_queue, &(b), time / portTICK_RATE_MS) == pdTRUE)
		return b;
	else
		return 0;
}


/**
 *  Compare two addresses
 *
 *	\brief	Convert addresses to full format if fields are missing.
 *
 *	\param a1 first address to compare
 *	\param a2 second address to compare
 *
 *  \return  pdTRUE match
 *  \return  pdFALSE no match
 */
portCHAR stack_compare_address(sockaddr_t *a1, sockaddr_t *a2)
{
	uint8_t size[2] = {0,0};
	uint16_t pan[2] = {0xFFFF,0xFFFF};
	uint8_t match = 0;
	uint8_t i;
	
	{
		addrtype_t tmp_type;
		
		for (i=0; i<2; i++)
		{
			uint8_t *dptr = 0;
			sockaddr_t *tmp_addr;
			
			if (i==0)
			{ tmp_type = a1->addr_type;
				tmp_addr = a1;
			}
			else
			{ tmp_type = a2->addr_type;
				tmp_addr = a2;
			}
			
			switch(tmp_type)
			{					
				case ADDR_802_15_4_PAN_SHORT:
					dptr = tmp_addr->address;
					dptr += 2;
				case ADDR_802_15_4_SHORT:
					size[i] = 2;
					break;
					
				case ADDR_802_15_4_PAN_LONG:
					dptr = tmp_addr->address;
					dptr += 8;
				case ADDR_802_15_4_LONG:
					size[i] = 8;
					break;
				
				case ADDR_SHORT:
					size[i] = 1;
					break;
					
				case ADDR_PAN:
					dptr = tmp_addr->address;
				case ADDR_BROADCAST:
				case ADDR_NONE:
					size[i] = 0;
					break;
			}
			if (dptr)
			{
				pan[i] = *dptr++;
				pan[i] <<= 8;
				pan[i] = *dptr++;
			}
		}
	}
	if (stack_check_broadcast(a1->address, a1->addr_type) == pdTRUE)
	{
		match = 1;
	}
	if (stack_check_broadcast(a2->address, a2->addr_type) == pdTRUE)
	{
		match = 1;
	}

	if (!match)
	{
		uint8_t cmp_size;
		if (size[0] > size[1]) cmp_size = size[1];
		else 	cmp_size = size[0];
		if (memcmp(a2->address, a1->address, cmp_size) == 0)
		{
			match = 1;
		}
	}
	
	if (match)
	{
		if ((pan[0] != 0xFFFF) && (pan[1] != 0xFFFF))
		{
			if (pan[0] != pan[1]) return pdFALSE;
		}
	}
	else
	{
		return pdFALSE;
	}
	
	return pdTRUE;
}
		

/**
 *	Add an address to a given buffer
 *
 *	\param	dptr	Index to the data field of the buffer where to insert the address
 *	\param	type	Type of the address (ADDR_802_15_4_PAN_LONG, ADDR_802_15_4_LONG, ADDR_802_15_4_PAN_SHORT or ADDR_802_15_4_SHORT)
 *	\param	address	The actual address structure
 */
uint8_t * stack_insert_address_to_buffer(uint8_t *dptr, addrtype_t type, uint8_t *address)
{
	uint8_t i;
	if(type == ADDR_802_15_4_PAN_LONG || type == ADDR_802_15_4_LONG )
	{
		
		if(type == ADDR_802_15_4_PAN_LONG)
		{
			address += 8;
			*dptr++ = *address++;
			*dptr++ = *address++;
			address -= 10;
		}
		for (i = 0; i < 8; i++)
		{
			*dptr++ = *address++;
		}
	}
	else if(type == ADDR_802_15_4_PAN_SHORT  || type == ADDR_802_15_4_SHORT)
	{
		if(type == ADDR_802_15_4_PAN_SHORT)
		{
			address += 2;
			*dptr++ = *address++;
			*dptr++ = *address++;
			address -= 4;
		}
		for (i = 0; i < 2; i++)
		{
			*dptr++ = *address++;
		}
	}
	else
	{

	}
return dptr;
}

/**
 *	Check if all the bytes in the address field are 0xff
 *
 *	\param	address	The address field to check
 *	\param	type	The address type (so that the function knows how many bytes to check)
 *	\return	pdFALSE	Not a broadcast address
 *	\return	pdTRUE	A brodcast address
 */
portCHAR stack_check_broadcast(uint8_t *address, addrtype_t type)
{
	uint8_t i, len=4;
	if(type==ADDR_BROADCAST)
		return pdTRUE;

	if(type==ADDR_802_15_4_PAN_LONG || type==ADDR_802_15_4_LONG)
		len=8;
	if(type==ADDR_SHORT)
		len=2;
	for(i=0; i<len; i++)
	{
		if(*address++ != 0xff)
		{
			return pdFALSE;
		}
	}
	return pdTRUE;
}
