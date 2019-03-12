/**
 ******************************************************************************
 * @file    mxos_filesystem_internal.h
 * @author  You xx
 * @version V1.0.0
 * @date    01-Dec-2016
 * @brief   This file provide the struct used in mxos filesystem
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */


#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxos_filesystem.h"


#ifdef __cplusplus
extern "C" {
#endif
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define DEFAULT_SECTOR_SIZE  (512)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

struct mxos_filesystem_driver_struct
{
    mret_t (*init)             ( void );
    mret_t (*mount)            ( mxos_block_device_t* device, mxos_filesystem_t* fs_handle_out );
    mret_t (*unmount)          ( mxos_filesystem_t* fs_handle );
    mret_t (*file_get_details) ( mxos_filesystem_t* fs_handle, const char* filename, mxos_dir_entry_details_t* details_out );
    mret_t (*file_open)        ( mxos_filesystem_t* fs_handle, mxos_file_t* file_handle_out, const char* filename, mxos_filesystem_open_mode_t mode );
    mret_t (*file_seek)        ( mxos_file_t* file_handle, int64_t offset, mxos_filesystem_seek_type_t whence );
    mret_t (*file_tell)        ( mxos_file_t* file_handle, uint64_t* location );
    mret_t (*file_read)        ( mxos_file_t* file_handle, void* data, uint64_t bytes_to_read, uint64_t* returned_bytes_count );
    mret_t (*file_write)       ( mxos_file_t* file_handle, const void* data, uint64_t bytes_to_write, uint64_t* written_bytes_count );
    mret_t (*file_flush)       ( mxos_file_t* file_handle );
    int      (*file_end_reached) ( mxos_file_t* file_handle );
    mret_t (*file_close)       ( mxos_file_t* file_handle );
    mret_t (*file_delete)      ( mxos_filesystem_t* fs_handle, const char* filename );
    mret_t (*dir_open)         ( mxos_filesystem_t* fs_handle, mxos_dir_t* dir_handle, const char* dir_name );
    mret_t (*dir_read)         ( mxos_dir_t* dir_handle, char* name_buffer, unsigned int name_buffer_length, mxos_dir_entry_type_t* type, mxos_dir_entry_details_t* details );
    int      (*dir_end_reached)  ( mxos_dir_t* dir_handle );
    mret_t (*dir_rewind)       ( mxos_dir_t* dir_handle );
    mret_t (*dir_close)        ( mxos_dir_t* dir_handle );
    mret_t (*dir_create)       ( mxos_filesystem_t* fs_handle, const char* directory_name );
    mret_t (*format)           ( mxos_block_device_t* device );
    mret_t (*get_info)         ( mxos_filesystem_info* info,char* mounted_name );
    mret_t (*scan_files)       ( char* mounted_name, mxos_scan_file_handle arg );
};



/******************************************************
 *                 Static Variables
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
extern mxos_filesystem_driver_t mxos_filesystem_driver_ftfs;
extern mxos_filesystem_driver_t mxos_filesystem_driver_fatfs;

/******************************************************
 *               Function Definitions
 ******************************************************/

#ifdef __cplusplus
} /*extern "C" */
#endif
