/**
 ******************************************************************************
 * @file    oled.h
 * @author  Eshen Wang
 * @version V1.0.0
 * @date    17-Mar-2015
 * @brief   OLED control operations.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef __OLED_H_
#define __OLED_H_

#include "mxos.h"
#include "mxos_board.h"

/** @defgroup MXOS_Drivers_interface MXOS Drivers Interface
  * @brief Provide driver interface for MXOS external devices
  * @{
  */

/** @addtogroup MXOS_Drivers_interface
  * @{
  */

/** @defgroup MXOS_display_Driver MXOS display Driver
  * @brief Provide driver interface for display devices
  * @{
  */

/** @addtogroup MXOS_display_Driver
  * @{
  */

/** @defgroup MXOS_OLED_Driver MXOS OLED Driver
  * @brief Provide driver interface for OLED
  * @{
  */

#ifndef OLED_SPI_PORT
#define OLED_SPI_PORT       (MXOS_SPI_NONE)
#endif

#ifndef OLED_SPI_SCK
#define OLED_SPI_SCK        (MXOS_GPIO_NONE)
#endif

#ifndef OLED_SPI_DIN
#define OLED_SPI_DIN        (MXOS_GPIO_NONE)
#endif

#ifndef OLED_SPI_DC
#define OLED_SPI_DC         (MXOS_GPIO_NONE)
#endif

#ifndef OLED_SPI_CS
#define OLED_SPI_CS         (MXOS_GPIO_NONE)
#endif

#ifndef OLED_I2C_PORT
#define OLED_I2C_PORT       (MXOS_I2C_NONE)
#endif

#define OLED_MODE 0
#define SIZE 16
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	

// typedef
#define u8     uint8_t
#define u16    uint16_t
#define u32    uint32_t

//----------------- OLED PIN ----------------  	
#define OLED_DC_INIT()     mxos_gpio_init( (mxos_gpio_t)OLED_SPI_DC, OUTPUT_PUSH_PULL )  // in case spi flash is wrote

#define OLED_CS_Clr()      mxos_gpio_output_low(OLED_SPI_CS)  //CS
#define OLED_CS_Set()      mxos_gpio_output_high(OLED_SPI_CS)

#define OLED_DC_Clr()      mxos_gpio_output_low(OLED_SPI_DC)  //DC
#define OLED_DC_Set()      mxos_gpio_output_high(OLED_SPI_DC)

#define OLED_RST_Clr()
#define OLED_RST_Set()

//PC0~7,��Ϊ������
#define DATAOUT(x)           GPIO_Write(GPIOC,x);//���  
//ʹ��4�ߴ��нӿ�ʱʹ�� 

#define OLED_SCLK_Clr() mxos_gpio_output_low(OLED_SPI_SCK)  //CLK
#define OLED_SCLK_Set() mxos_gpio_output_high(OLED_SPI_SCK)

#define OLED_SDIN_Clr() mxos_gpio_output_low(OLED_SPI_DIN)  //DIN
#define OLED_SDIN_Set() mxos_gpio_output_high(OLED_SPI_DIN)
	     
#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

//-------------------------------- display define ------------------------------
// for 8*16 char can only display 4 rows, 16 chars each row.
#define OLED_DISPLAY_ROW_1    0    // yellow
#define OLED_DISPLAY_ROW_2    2    // blue
#define OLED_DISPLAY_ROW_3    4    // blue
#define OLED_DISPLAY_ROW_4    6    // blue

#define OLED_DISPLAY_COLUMN_START    0    // colloum from left pos 0

#define OLED_DISPLAY_MAX_CHAR_PER_ROW    16   // max 16 chars each row


//OLED�����ú���
void OLED_WR_Byte(u8 dat, u8 cmd);   
void OLED_WR_Bytes(u8 *dat, u8 len, u8 cmd);   
void OLED_Display_On(void);
void OLED_Display_Off(void);	  

void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);

void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);


/*-------------------------------------------------------- USER INTERFACES -----------------------------------------*/


/**
 * @brief Initialize OLED device.
 *
 * @return none
 */
void OLED_Init(void);



/**
 * @brief Clear up all data shown on OLED
 *
 * @return none
 */
void OLED_Clear(void);


/**
 * @brief show string in OLED specified place
 *
 * @param x: Position the X axis of the stiring to display
 * @param y: Position the Y axis of the string to display
 * @param p: String to be displayed in OLED
 * 
 * @return none
 */
void OLED_ShowString(u8 x,u8 y, char *p);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif  
	 



