/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ST STR91x ARM9
 * port.
 *----------------------------------------------------------*/

/* Standard includes. */
#include <stdlib.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "task.h"

#if (NX_POWERSAVE)
#include "ps.h"
#endif //(NX_POWERSAVE)

#ifndef configUSE_WATCHDOG_TICK
	#error configUSE_WATCHDOG_TICK must be set to either 1 or 0 in FreeRTOSConfig.h to use either the Watchdog or timer 2 to generate the tick interrupt respectively.
#endif

/* Constants required to setup the initial stack. */
#define portINITIAL_SPSR 			( ( StackType_t ) 0x5f ) /* System mode, ARM mode, interrupts enabled. */

#define portINSTRUCTION_SIZE	    ( ( StackType_t ) 4 )

/* Constants required to handle critical sections. */
#define portNO_CRITICAL_NESTING 		((uint32_t ) 0 )

#ifndef abs
	#define abs(x) ((x)>0 ? (x) : -(x))
#endif

/*-----------------------------------------------------------*/

/* Setup the watchdog to generate the tick interrupts. */
static void prvSetupTimerInterrupt( void );

/* ulCriticalNesting will get set to zero when the first task starts.  It
cannot be initialised to 0 as this will cause interrupts to be enabled
during the kernel initialization process. */
uint32_t ulCriticalNesting = ( uint32_t ) 9999;
void * volatile pxNestChangedCurrentTCB = NULL;

/* Tick interrupt routines for cooperative and preemptive operation
respectively.  The preemptive version is not defined as __irq as it is called
from an asm wrapper function. */
#if configUSE_WATCHDOG_TICK == 0
	/* Used to update the OCR timer register */
	static uint16_t s_nPulseLength;
#endif

/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
#if (NX_POWERSAVE)
	#if CFG_USE_STA_PS
    #if PS_WAKEUP_MOTHOD_RW

        if(PS_STA_DTIM_CAN_SLEEP)
        {
        GLOBAL_INT_DECLARATION();
            GLOBAL_INT_DISABLE();
            PS_DEBUG_PWM_TRIGER ;

        ps_sleep_check();
        GLOBAL_INT_RESTORE();
        }        

    #endif
	#endif
#endif //(NX_POWERSAVE)
}

/*-----------------------------------------------------------*/
/*
 * Initialize the stack of a task to look exactly as if a call to
 * portSAVE_CONTEXT had been called.
 *
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
	StackType_t *pxOriginalTOS;

	pxOriginalTOS = pxTopOfStack;

	/* To ensure asserts in tasks.c don't fail, although in this case the assert
	is not really required. */
	pxTopOfStack--;

	/* Setup the initial stack of the task.  The stack is set exactly as
	expected by the portRESTORE_CONTEXT() macro. */

	/* First on the stack is the return address - which in this case is the
	start of the task.  The offset is added to make the return address appear
	as it would within an IRQ ISR. */
	*pxTopOfStack = ( StackType_t ) pxCode + portINSTRUCTION_SIZE;		
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xaaaaaaaa;	/* R14 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) pxOriginalTOS; /* Stack used when task starts goes in R13. */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x12121212;	/* R12 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x11111111;	/* R11 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x10101010;	/* R10 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x09090909;	/* R9 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x08080808;	/* R8 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x07070707;	/* R7 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x06060606;	/* R6 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x05050505;	/* R5 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x04040404;	/* R4 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x03030303;	/* R3 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x02020202;	/* R2 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x01010101;	/* R1 */
	pxTopOfStack--;	

	/* When the task starts is will expect to find the function parameter in
	R0. */
	*pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
	pxTopOfStack--;

    /* The status register is set for system mode, with interrupts enabled. */
    if ((uint32_t)pxCode & 0x01) { // thumb
        *pxTopOfStack = ( StackType_t ) (portINITIAL_SPSR | 0x20); // thumb
    } else {
        *pxTopOfStack = ( StackType_t ) portINITIAL_SPSR; // ARM
    }
	
	
	pxTopOfStack--;

	/* Interrupt flags cannot always be stored on the stack and will
	instead be stored in a variable, which is then saved as part of the
	tasks context. */
	*pxTopOfStack = portNO_CRITICAL_NESTING;

	return pxTopOfStack;	
}

/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the ARM port will require this function as there
	is nothing to return to.  */
}
/*-----------------------------------------------------------*/

/* This function is called from an asm wrapper, so does not require the __irq
keyword. */
#if configUSE_WATCHDOG_TICK == 1
void prvFindFactors(uint32_t n, uint16_t *a, uint32_t *b)
{
	/* This function is copied from the ST STR7 library and is
	copyright STMicroelectronics.  Reproduced with permission. */

	uint32_t b0;
	uint16_t a0;
	int32_t err, err_min=n;

	*a = a0 = ((n-1)/65536ul) + 1;
	*b = b0 = n / *a;

	for (; *a <= 256; (*a)++)
	{
		*b = n / *a;
		err = (int32_t)*a * (int32_t)*b - (int32_t)n;
		if (abs(err) > (*a / 2))
		{
			(*b)++;
			err = (int32_t)*a * (int32_t)*b - (int32_t)n;
		}
		if (abs(err) < abs(err_min))
		{
			err_min = err;
			a0 = *a;
			b0 = *b;
			if (err == 0) break;
		}
	}

	*a = a0;
	*b = b0;
}
/*-----------------------------------------------------------*/

static void prvSetupTimerInterrupt( void )
{
}
/*-----------------------------------------------------------*/

void WDG_IRQHandler( void )
{
	{
		/* Increment the tick counter. */
		if( xTaskIncrementTick() != pdFALSE )
		{		
			/* Select a new task to execute. */
			vTaskSwitchContext();
		}
	
		/* Clear the interrupt in the watchdog. */
		//WDG->SR &= ~0x0001;
	}
}
#else
void prvFindFactors(uint32_t n, uint16_t *a, uint32_t *b)
{
	/* This function is copied from the ST STR7 library and is
	copyright STMicroelectronics.  Reproduced with permission. */

	uint16_t b0;
	uint8_t a0;
	int32_t err, err_min=n;


	*a = a0 = ((n-1)/256) + 1;
	*b = b0 = n / *a;

	for (; *a <= 256; (*a)++)
	{
		*b = n / *a;
		err = (int32_t)*a * (int32_t)*b - (int32_t)n;
		if (abs(err) > (*a / 2))
		{
			(*b)++;
			err = (int32_t)*a * (int32_t)*b - (int32_t)n;
		}
		if (abs(err) < abs(err_min))
		{
			err_min = err;
			a0 = *a;
			b0 = *b;
			if (err == 0) break;
		}
	}

	*a = a0;
	*b = b0;
}
/*-----------------------------------------------------------*/

static void prvSetupTimerInterrupt( void )
{
    fclk_init();
}

#endif /* USE_WATCHDOG_TICK */

/*-----------------------------------------------------------*/
void vPortEnterCritical( void )
{
	portDISABLE_INTERRUPTS();

	ulCriticalNesting ++;
}

/*-----------------------------------------------------------*/
void vPortExitCritical( void )
{
	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		ulCriticalNesting --;

		if( ulCriticalNesting == portNO_CRITICAL_NESTING )
		{
			portENABLE_INTERRUPTS();
		}
	}
}

/*-----------------------------------------------------------*/
void prvDefaultHandler( void )
{
}

uint8_t platform_is_in_interrupt_context( void )
{    
    return ((platform_is_in_fiq_context()) 
                || (platform_is_in_irq_context()));
}

void platform_enable_intc_at_restore_context(void)
{
#define ARM968_SYS_MODE 	 0x1f
#define ARM968_MODE_MASK 	 0x1f

	//uint32_t spsr_content = platform_spsr_content();

	//if((spsr_content & ARM968_MODE_MASK) == ARM968_SYS_MODE)
	{
		if(!platform_is_in_interrupt_context())
			portENABLE_IRQ();
		
		if(!platform_is_in_fiq_context())
			portENABLE_FIQ();
	}	
}

void rtos_stack_overflow(char *taskname)
{
    bk_printf("of--%x\r\n", platform_sp_content());
	while(1);
}
//eof

