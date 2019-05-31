/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

#include "mxos_common.h"
#include "platform_cmsis.h"
//#include "platform_constants.h"
#include "platform_isr.h"
//#include "platform_isr_interface.h"
#include "platform_assert.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifdef _MXOS_DEBUG_
#define DEBUG_HARDFAULT
#endif /* ifdef _MXOS_DEBUG_ */

/* Bit Definitions for SCB_CFSR */
#define  SCB_CFSR_IACCVIOL                   ((uint32_t)0x00000001)
#define  SCB_CFSR_DACCVIOL                   ((uint32_t)0x00000002)
#define  SCB_CFSR_MUNSTKERR                  ((uint32_t)0x00000008)
#define  SCB_CFSR_MSTKERR                    ((uint32_t)0x00000010)
#define  SCB_CFSR_MMARVALID                  ((uint32_t)0x00000080)
#define  SCB_CFSR_IBUSERR                    ((uint32_t)0x00000100)
#define  SCB_CFSR_PRECISERR                  ((uint32_t)0x00000200)
#define  SCB_CFSR_IMPRECISERR                ((uint32_t)0x00000400)
#define  SCB_CFSR_UNSTKERR                   ((uint32_t)0x00000800)
#define  SCB_CFSR_STKERR                     ((uint32_t)0x00001000)
#define  SCB_CFSR_BFARVALID                  ((uint32_t)0x00008000)
#define  SCB_CFSR_UNDEFINSTR                 ((uint32_t)0x00010000)
#define  SCB_CFSR_INVSTATE                   ((uint32_t)0x00020000)
#define  SCB_CFSR_INVPC                      ((uint32_t)0x00040000)
#define  SCB_CFSR_NOCP                       ((uint32_t)0x00080000)
#define  SCB_CFSR_UNALIGNED                  ((uint32_t)0x01000000)
#define  SCB_CFSR_DIVBYZERO                  ((uint32_t)0x02000000)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct exception_stacked_registers_struct
{
    /* Stacked registers */
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R12;
    uint32_t LR;
    uint32_t PC;  /* (Return Address) */
    uint32_t PSR;
} exception_stacked_registers_t;

typedef enum EXC_RETURN_enum
{
    HANDLER_MSP_MSP = 0xF1, /* Return to Handler mode. Exception return gets state from MSP. Execution uses MSP after return. */
    THREAD_MSP_MSP  = 0xF9, /* Return to Thread mode.  Exception return gets state from MSP. Execution uses MSP after return. */
    THREAD_PSP_PSP  = 0xFD  /* Return to Thread mode.  Exception return gets state from PSP. Execution uses PSP after return. */
} EXC_RETURN_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

void HardFaultException_handler( uint32_t MSP, uint32_t PSP, uint32_t LR );

/******************************************************
 *               Variable Definitions
 ******************************************************/

extern unsigned int _stext, _etext, _estack;

/******************************************************
 *               Function Definitions
 ******************************************************/
merr_t stdio_hardfault( char* data, uint32_t size );
unsigned int * pTaskGetCurrentTaskEndOfStack( void );

#ifdef DEBUG_HARDFAULT

PLATFORM_DEFINE_NAKED_ISR( HardFault_Handler )
{
    __ASM("TST LR, #4" );
    __ASM("ITE EQ" );
    __ASM("MRSEQ R0, MSP" );
    __ASM("MRSNE R0, PSP" );
    __ASM("MRS R1, MSP" );
    __ASM("MRS R2, PSP" );
    __ASM("B hard_fault_handler_c");

}

void hard_fault_handler_c (unsigned int * sp, unsigned int * msp, unsigned int * psp)
{
    unsigned int *statck_end;
    unsigned int text_start = (uint32_t)&_stext;
    unsigned int text_end = (uint32_t)&_etext;

    printf("\n>>>>>>>>>>>>>>[");
    switch(__get_IPSR())
    {
    case 3:
    printf("Hard Fault");
    break;
    case    4:
    printf("Memory Manage");
    break;
    case 5:
    printf("Bus Fault");
    break;
    case 6:
    printf("Usage Fault");
    break;
    default:
    printf("Unknown Fault %ld", __get_IPSR());
    break;
    }
    printf(",corrupt,dump registers]>>>>>>>>>>>>>>>>>>\n\r");

    printf("R0 = 0x%08x\r\n", sp[0]);
    printf("R1 = 0x%08x\r\n", sp[1]);
    printf("R2 = 0x%08x\r\n", sp[2]);
    printf("R3 = 0x%08x\r\n", sp[3]);
    printf("R12 = 0x%08x\r\n", sp[4]);
    printf("SP [R13] = 0x%08x\r\n", (unsigned int)sp);
    printf("LR [R14] = 0x%08x\r\n", sp[5]);
    printf("PC [R15] = 0x%08X\r\n", sp[6]);
    printf("PSR = 0x%08X\r\n", sp[7]);
    printf("BFAR = 0x%08lx\r\n", SCB->BFAR);
    printf("CFSR = 0x%08lx\r\n", SCB->CFSR);
    printf("HFSR = 0x%08lx\r\n", SCB->HFSR);
    printf("DFSR = 0x%08lx\r\n", SCB->DFSR);
    printf("AFSR = 0x%08lx\r\n", SCB->AFSR);
    printf("MSP = 0x%08x  main stack pointer\r\n", (unsigned int)msp);
    printf("PSP = 0x%08x  process stack pointer\r\n", (unsigned int)psp);

    if(sp == msp)
    {
        statck_end = &_estack;
    }
    else
    {
        statck_end = pTaskGetCurrentTaskEndOfStack();
    }

    printf("Call stack:\r\n");
    while(sp++ <= statck_end)
    {
        if(*sp >= text_start && *sp <= text_end)
        {
            printf("0x%08x ", (unsigned int)*sp);
        }
    }
    printf("\r\n");

    while (1);
}

#if defined( __GNUC__ ) && ( ! defined( __clang__ ) )
#pragma GCC optimize ("O0")
#endif /* if defined( __GNUC__ ) && ( ! defined( __clang__ ) ) */

void HardFaultException_handler( uint32_t MSP, uint32_t PSP, uint32_t LR )
{
    exception_stacked_registers_t*  stackframe;
    uint32_t MMFAR = 0;
    uint32_t BFAR = 0;

    /* Get the Link Register value which contains the EXC_RETURN code */
    EXC_RETURN_t EXC_RETURN = (EXC_RETURN_t)(LR & 0xff);

    /* The location of the stack frame of the offending code is indicated by the EXC_RETURN code */
    if ( ( EXC_RETURN & 0x00000004 ) != 0 )
    {
        stackframe = (exception_stacked_registers_t*) PSP;
    }
    else
    {
        stackframe = (exception_stacked_registers_t*) MSP;
    }
    (void) stackframe; /* may be unused */

    /* Disable interrupts - this is so that when debugger continues, it will go to caller, not an interrupt routine */
    /* This will mean the system cannot run properly when returning */
    __set_PRIMASK( 0x01 );

    /* Find cause of hardfault */
    if ( ( SCB->HFSR & SCB_HFSR_VECTTBL_Msk ) != 0 )
    {
        MXOS_TRIGGER_BREAKPOINT(); /* Vector Table Hard Fault - Bus fault during vector table read during exception processing. */
    }
    else if ( ( SCB->HFSR & SCB_HFSR_FORCED_Msk ) != 0 )
    {
        /* Hard Fault is an escalated fault that was not handled */
        /* Need to read the other fault status registers */


        if ( ( SCB->CFSR & SCB_CFSR_MMARVALID    ) != 0 )
        {
            /* Memory Management Fault address register is valid - read it. */
            MMFAR = SCB->MMFAR;
        }

        if ( ( SCB->CFSR & SCB_CFSR_BFARVALID    ) != 0 )
        {
            /* Bus Fault address register is valid - read it. */
            BFAR = SCB->BFAR;
        }

        if ( ( SCB->CFSR & SCB_CFSR_IACCVIOL ) != 0 )
        {
            /* Memory Management Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Instruction Access Violation - Attempt to execute an instruction from a region marked Execute Never */
            (void) stackframe->LR; /* Check this variable for the jump instruction that jumped to an invalid region */
            (void) stackframe->PC; /* Check this variable for the location that was attempted to be executed */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */

        }
        else if ( ( SCB->CFSR & SCB_CFSR_DACCVIOL     ) != 0 )
        {
            /* Memory Management Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Data Access Violation */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
            (void) MMFAR;           /* Check this variable for the address of the attempted access */
            /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_MUNSTKERR    ) != 0 )
        {
            /* Memory Management Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Unstacking fault returning from an exception - stack possibly corrupted during exception handler */
                                   /* New stackframe is not saved in this case */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_MSTKERR      ) != 0 )
        {
            /* Memory Management Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Stacking fault whilst entering an exception - probably a bad stack pointer */
                                   /* Stack frame may be incorrect due to bad stack pointer */

        }
        else if ( ( SCB->CFSR & SCB_CFSR_IBUSERR      ) != 0 )
        {
            /* Bus Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Instruction Bus Error whilst fetching an instruction*/
        }
        else if ( ( SCB->CFSR & SCB_CFSR_PRECISERR    ) != 0 )
        {
            /* Bus Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Precise Data Bus Error - i.e. Data Bus fault at well defined location */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
            (void) BFAR;           /* Check this variable for the faulting address */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_IMPRECISERR  ) != 0 )
        {
            /* Bus Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Imprecise Data Bus Error - i.e. Data Bus fault occurred but details have been lost due to priorities delaying processing of the fault */
                                   /* No fault details are available in this case*/
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_UNSTKERR     ) != 0 )
        {
            /* Bus Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Unstacking fault returning from an exception - stack possibly corrupted during exception handler */
                                   /* New stackframe is not saved in this case */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_STKERR       ) != 0 )
        {
            /* Bus Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Stacking fault whilst entering an exception - probably a bad stack pointer */
                                   /* Stack frame may be incorrect due to bad stack pointer */

        }
        else if ( ( SCB->CFSR & SCB_CFSR_UNDEFINSTR   ) != 0 )
        {
            /* Usage Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Undefined Instruction Usage fault - probably corrupted memory in code space */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_INVSTATE     ) != 0 )
        {
            /* Usage Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Invalid State usage fault - This is probably due to a branch with the LSB=0 - i.e. attempt to execute non-thumb code - Illegal use of EPSR was attempted */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_INVPC        ) != 0 )
        {
            /* Usage Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Invalid PC load usage fault - the EXC_RETURN value in LR was invalid on return from an exception - possibly stack corruption in exception */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_NOCP         ) != 0 )
        {
            /* Usage Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* No Coprocessor usage fault - coprocessor instruction attempted on processor without support for them */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_UNALIGNED   ) != 0 )
        {
            /* Usage Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Unaligned access usage fault - Unaligned access whilst UNALIGN_TRP bit of SCB_CCR is set, or any unaligned access to LDM, STM, LDRD or STRD */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_DIVBYZERO    ) != 0 )
        {
            /* Usage Fault */
            MXOS_TRIGGER_BREAKPOINT();  /* Divide by zero usage fault */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else
        {
            /* Unknown Fault */
            MXOS_TRIGGER_BREAKPOINT();
            /* You may try stepping past the return of this handler, which may return near the location of the error */
        }

    }
    else
    {
        /* Unknown Hard Fault cause */
        MXOS_TRIGGER_BREAKPOINT();
        /* You may try stepping past the return of this handler, which may return near the location of the error */
    }

    (void) MMFAR; /* This is for debug usage and need not be used programmatically */
    (void) BFAR; /* This is for debug usage and need not be used programmatically */
}
#if defined( __GNUC__ ) && ( ! defined( __clang__ ) )
#pragma GCC reset_options
#endif /* if defined( __GNUC__ ) && ( ! defined( __clang__ ) ) */

#endif /* ifdef DEBUG_HARDFAULT */

