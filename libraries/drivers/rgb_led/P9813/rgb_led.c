/**
 ******************************************************************************
 * @file    rgb_led.c
 * @author  Eshen Wang
 * @version V1.0.0
 * @date    17-Mar-2015
 * @brief   rgb led controller.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "rgb_led.h"
#include "mxos.h"

#define rgb_led_log(M, ...) custom_log("RGB_LED", M, ##__VA_ARGS__)
#define rgb_led_log_trace() custom_log_trace("RGB_LED")

/* use gpio */
static void P9813_PIN_write_frame(uint32_t data)
{ 
  uint8_t i;        
  uint32_t f_data = data;
  //rgb_led_log("P9813_PIN_write_frame = %X", f_data);
  
  for(i=0; i<32; i++){
    P9813_PIN_CIN_Clr();
    if(f_data & 0x80000000){
      P9813_PIN_DIN_Set();
    }
    else{
      P9813_PIN_DIN_Clr();
    }
    P9813_PIN_CIN_Set();  // raise edge to set data
    f_data = f_data << 1;
    //rgb_led_log("frame_data<<1 =%x", f_data);
  } 
  
  //P9813_PIN_CIN_Clr();
  //P9813_PIN_DIN_Clr();
}

static void P9813_PIN_write_start_frame(void)
{ 
  uint32_t start_frame = 0x00000000;
  P9813_PIN_write_frame(start_frame);     
}

static void P9813_PIN_write_data(uint8_t blue, uint8_t green, uint8_t red)
{
  uint8_t check_byte = 0xC0;  // starting flag "11"
  uint32_t send_data = 0;
  
  // calc check data
  check_byte |= (((~blue) >> 2) & 0x30);  // B7, B6
  check_byte |= (((~green) >> 4) & 0x0C);  // G7,G6
  check_byte |= (((~red) >> 6) & 0x03);   // R7,R6
  
  // create send data 32bit
  send_data |= (check_byte << 24) | (blue << 16) | (green << 8) | (red);
  
  //send_data = 0xFC0000FF;
  //rgb_led_log("P9813_PIN_write_data: %X", send_data);
  P9813_PIN_write_frame(send_data);
}
 
/*-------------------------------------------------- USER INTERFACES ------------------------------------------------*/

void rgb_led_init(void)
{
  MxosGpioInitialize( (mxos_gpio_t)P9813_PIN_CIN, OUTPUT_PUSH_PULL );
  MxosGpioInitialize( (mxos_gpio_t)P9813_PIN_DIN, OUTPUT_PUSH_PULL );
}

void rgb_led_open(uint8_t red, uint8_t green, uint8_t blue)
{
  P9813_PIN_write_start_frame();
  P9813_PIN_write_data(blue, green, red);
  P9813_PIN_write_start_frame();  // fix led bink bug
}

void rgb_led_close(void)
{
  rgb_led_init();
  rgb_led_open(0, 0, 0);
}
