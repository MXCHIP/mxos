/**
 ******************************************************************************
 * @file    mxoskit_ext.c
 * @author  Eshen Wang
 * @version V1.0.0
 * @date    8-May-2015
 * @brief   mxoskit extension board peripherals operations..
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
#include "mxoskit_ext.h"

#define mxoskit_ext_log(M, ...) custom_log("MXOSKIT_EXT", M, ##__VA_ARGS__)
#define mxoskit_ext_log_trace() custom_log_trace("MXOSKIT_EXT")

extern void user_key1_clicked_callback(void);
extern void user_key1_long_pressed_callback(void);
extern void user_key2_clicked_callback(void);
extern void user_key2_long_pressed_callback(void);

//------------------------------------- API ------------------------------------
OSStatus user_modules_init(void)
{
  OSStatus err = kUnknownErr;
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};   // max char each line
#if defined(MXOS_EXT_KEY1)||defined(MXOS_EXT_KEY2)
  button_init_t init;
#endif

#ifndef CONFIG_CPU_MX1290
  // init DC Motor(GPIO)
  dc_motor_init();
  dc_motor_set(0);   // off
#endif

  // init RGB LED(P9813)
  rgb_led_init();
  rgb_led_open(0, 0, 0);  // off
  
  // init OLED
  OLED_Init();
  OLED_Clear();
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", MODEL);
  OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", "MXOS            ");
  OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2,  oled_show_line);
  OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3,  "   Running...   ");
  OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4,  "                ");

  apds9930_sensor_init();

#ifndef CONFIG_CPU_MX1290
  // init Light sensor(ADC)
  light_sensor_init();

  // init infrared sensor(ADC)
  infrared_reflective_init();

  // init user key1 && key2
#ifdef MXOS_EXT_KEY1
  init.gpio = MXOS_EXT_KEY1;
  init.pressed_func = user_key1_clicked_callback;
  init.long_pressed_func = NULL;
  init.long_pressed_timeout = 5000;
  button_init( IOBUTTON_USER_1, init);
#endif

#ifdef MXOS_EXT_KEY2
  init.gpio = MXOS_EXT_KEY2;
  init.pressed_func = user_key2_clicked_callback;
  init.long_pressed_func = NULL;
  init.long_pressed_timeout = 5000;
  button_init( IOBUTTON_USER_2, init);
#endif
  
  err = temp_hum_sensor_init();
#endif
  
//  int32_t temperature;
//  uint32_t humidity;
//  
//  while(1){
//  err = temp_hum_sensor_read( &temperature,  &humidity );
//  if( err == kNoErr ){
//    platform_log( "temperature: %d, humidity = %d ", temperature, humidity);
//  }
//  sleep(1);
//  }
//exit:
  return err;
}

OSStatus mxoskit_ext_init(void)
{
  OSStatus err = kUnknownErr;
  err = user_modules_init();
  
  return err;
}
