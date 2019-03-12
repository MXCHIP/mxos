/**
 ******************************************************************************
 * @file    BootloaderEntrance.c
 * @author  William Xu
 * @version V2.0.0
 * @date    05-Oct-2014
 * @brief   MXOS bootloader main entrance.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2014 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */


#include "mxos.h"
#include "mxos_board.h"
#include "mxos_board_conf.h"

//#include "platform_internal.h"
#include "bootloader.h"

extern void Main_Menu(void);
extern merr_t update(void);

#ifdef SIZE_OPTIMIZE
char menu[] =
"\r\n"
"MXOS bootloader for %s, %s, HARDWARE_REVISION: %s\r\n"
"0:BOOTUPDATE,"
"1:FWUPDATE,"
"2:DRIVERUPDAT,"
"3:PARAUPDATE,"
"4:FLASHUPDATE,"
"5:MEMORYMAP,"
"6:BOOT,"
"7:REBOOT";
#else
const char menu[] =
"\r\n"
"MXOS bootloader for %s, %s, HARDWARE_REVISION: %s\r\n"
"+ command -------------------------+ function ------------+\r\n"
"| 1:FWUPDATE      <-r>             | Update application   |\r\n"
"| 3:PARUPDATE     <-id n><-r><-e>  | Update MXOS partition|\r\n"
"| 5:MEMORYMAP                      | List flash memory map|\r\n"
"| 6:BOOT                           | Excute application   |\r\n"
"| 7:REBOOT                         | Reboot               |\r\n"
"+----------------------------------+----------------------+\r\n"
"|    (C) COPYRIGHT 2015 MXCHIP Corporation  By William Xu |\r\n";
#endif

#ifdef MXOS_ENABLE_STDIO_TO_BOOT
extern int stdio_break_in(void);
#endif


WEAK bool mxos_should_enter_bootloader( void )
{
  return false;
}

WEAK bool mxos_should_enter_mfg_mode( void )
{
  return false;
}

WEAK bool mxos_should_enter_ate_mode( void )
{
  return false;
}

void bootloader_start_app( uint32_t app_addr )
{
  mxos_start_application( app_addr );
}


int main(void)
{
    mxos_logic_partition_t *partition;

    mxos_set_bootload_ver();

    update();

#ifdef MXOS_ENABLE_STDIO_TO_BOOT
    if (stdio_break_in() == 1)
    goto BOOT;
#endif

    if ( mxos_should_enter_bootloader() == false )
        bootloader_start_app((mxos_flash_get_info(MXOS_PARTITION_APPLICATION))->partition_start_addr);
    else if ( mxos_should_enter_mfg_mode() == true )
        bootloader_start_app((mxos_flash_get_info(MXOS_PARTITION_APPLICATION))->partition_start_addr);
    else if ( mxos_should_enter_ate_mode() ) {
        partition = mxos_flash_get_info(MXOS_PARTITION_ATE);
        if ( partition->partition_owner != MXOS_FLASH_NONE ) {
            bootloader_start_app(partition->partition_start_addr);
        }
    }

#ifdef MXOS_ENABLE_STDIO_TO_BOOT
    BOOT:
#endif

    printf(menu, MODEL, Bootloader_REVISION, HARDWARE_REVISION);

    while ( 1 ) {
        Main_Menu();
    }
}


