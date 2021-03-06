/**
 ******************************************************************************
 * @file    keys.c
 * @author  Eshen Wang
 * @version V1.0.0
 * @date    1-May-2015
 * @brief   user keys operation.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "mxos_platform.h"
#include "button.h"

#define keys_log(M, ...) custom_log("USER_KEYS", M, ##__VA_ARGS__)
#define keys_log_trace() custom_log_trace("USER_KEYS")

/*-------------------------------- VARIABLES ---------------------------------*/

/*------------------------------ USER INTERFACES -----------------------------*/

static void button_irq_handler( void* arg )
{
    button_context_t *_context = arg;

    int interval = -1;

    if ( mhal_gpio_value( _context->gpio ) == ((_context->idle == IOBUTTON_IDLE_STATE_HIGH) ? 0 : 1) ) {
        mhal_gpio_int_on( _context->gpio,
                           (_context->idle == IOBUTTON_IDLE_STATE_HIGH) ? IRQ_TRIGGER_RISING_EDGE : IRQ_TRIGGER_FALLING_EDGE,
                           button_irq_handler, _context );
        _context->start_time = mos_time() + 1;
        mos_timer_start(_context->_user_button_timer );
    }
    else {
        interval = (int) mos_time() + 1 - _context->start_time;
        if ( (_context->start_time != 0) && interval > 50 && interval < _context->long_pressed_timeout ) {
            /* button clicked once */
            if ( _context->pressed_func != NULL )
                (_context->pressed_func)();
        }
        mhal_gpio_int_on( _context->gpio,
                           (_context->idle == IOBUTTON_IDLE_STATE_HIGH) ? IRQ_TRIGGER_FALLING_EDGE : IRQ_TRIGGER_RISING_EDGE,
                           button_irq_handler, _context );
        mos_timer_stop(_context->_user_button_timer );
        _context->start_time = 0;
    }
}

static void button_timeout_handler( void* arg )
{
    button_context_t *_context = arg;

    _context->start_time = 0;
    if ( _context->long_pressed_func ) (_context->long_pressed_func)();
}


void button_init( button_context_t *btn_context )
{
    btn_context->start_time = 0;

    if ( btn_context->idle == IOBUTTON_IDLE_STATE_HIGH ) {
        mhal_gpio_open( btn_context->gpio, INPUT_PULL_UP );
        mhal_gpio_int_on( btn_context->gpio, IRQ_TRIGGER_FALLING_EDGE, button_irq_handler, btn_context );
    } else {
        mhal_gpio_open( btn_context->gpio, INPUT_PULL_DOWN );
        mhal_gpio_int_on( btn_context->gpio, IRQ_TRIGGER_RISING_EDGE, button_irq_handler, btn_context );
    }

    btn_context->_user_button_timer = mos_timer_new(btn_context->long_pressed_timeout,
                          button_timeout_handler, true, btn_context );

}




