/**
 ******************************************************************************
 * @file    mxos_filesystem.c
 * @author  You xx
 * @version V1.0.0
 * @date    01-Dec-2016
 * @brief   This file provide the function for mxos filesystem.
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

/** @file
 *  Implementation of the MXOSFS External-Use file system.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxos_filesystem.h"
#include "mxos_filesystem_internal.h"
#include "mxos_result.h"


/******************************************************
 *                      Macros
 ******************************************************/
#define os_filesystem_log(format, ...)  custom_log("filesystem", format, ##__VA_ARGS__)
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



/******************************************************
 *                 Static Variables
 ******************************************************/

static mxos_bool_t mxos_filesystem_inited = MXOS_FALSE;

static mxos_filesystem_mounted_device_t mounted_table[MXOS_FILESYSTEM_MOUNT_DEVICE_NUM_MAX];
static uint32_t total_mounted_index = 0;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static merr_t mxos_filesystem_add_mounted_device ( mxos_filesystem_t* fs_handle, const char* mounted_name);
static merr_t mxos_filesystem_del_mounted_device ( mxos_filesystem_t* fs_handle );

/******************************************************
 *               Variable Definitions
 ******************************************************/


/******************************************************
 *               Function Definitions
 ******************************************************/

merr_t mxos_filesystem_init ( void )
{
    if ( mxos_filesystem_inited == true )
    {
        return kNoErr;
    }

    memset( (void *) mounted_table, 0, sizeof(mounted_table) );
    total_mounted_index = 0;

#ifdef USING_FTFS
    mxos_filesystem_driver_ftfs.init();
#endif /* USING_mxosFS */
#ifdef USING_FATFS
    mxos_filesystem_driver_fatfs.init();
#endif /* USING_FATFS */
    mxos_filesystem_inited = true;
    return kNoErr;
}

static merr_t mxos_filesystem_add_mounted_device ( mxos_filesystem_t* fs_handle, const char* mounted_name)
{
    uint32_t i;

    /* Error checking */
    if ( (fs_handle == NULL) || (mounted_name == NULL) )
    {
        os_filesystem_log( "Null input!" );
        return kUnknownErr;
    }

    if ( total_mounted_index >= MXOS_FILESYSTEM_MOUNT_DEVICE_NUM_MAX )
    {
        os_filesystem_log( "Mounted device number exceeds upper limit!" );
        return kUnknownErr;
    }

    if ( strlen(mounted_name) > (MXOS_FILESYSTEM_MOUNT_NAME_LENGTH_MAX - 1) )
    {
        os_filesystem_log( "Device name length too long!" );
        return kUnknownErr;
    }

    /* Device duplicated checking */
    for ( i = 0; i < MXOS_FILESYSTEM_MOUNT_DEVICE_NUM_MAX; i++ )
    {
        if ( mounted_table[i].fs_handle == fs_handle )
        {
            os_filesystem_log( "Duplicated handle with index %ld", i );
            return kUnknownErr;
        }
        if ( strcmp(mounted_table[i].name, mounted_name) == 0 )
        {
            os_filesystem_log( "Duplicated name with index %ld\n", i );
            return kUnknownErr;
        }
    }

    /* Add into table */
    mounted_table[total_mounted_index].fs_handle = fs_handle;
    memcpy( mounted_table[total_mounted_index].name, mounted_name, MXOS_FILESYSTEM_MOUNT_NAME_LENGTH_MAX );
    mounted_table[total_mounted_index].name[strlen(mounted_name)] = '\0';
    os_filesystem_log( "Added %s into mounted table with index %lu", mounted_table[total_mounted_index].name, (unsigned long)total_mounted_index );

    /* Increase index if all right */
    total_mounted_index ++;

    /* Only for debug dump */
    #if 0
    printf( "\n--- Filesystem Mounted Table ---\n");
    for ( i = 0; i < MXOS_FILESYSTEM_MOUNT_DEVICE_NUM_MAX; i++ )
    {
        printf( "[%02ld]\t %s\t\t 0x%08lx\n", i, mounted_table[i].name, (uint32_t)mounted_table[i].fs_handle);
    }
    #endif

    return kNoErr;
}

static merr_t mxos_filesystem_del_mounted_device ( mxos_filesystem_t* fs_handle )
{
    mxos_bool_t is_handle_found = MXOS_FALSE;
    uint32_t shift_up_num = 0;
    uint32_t i;

    /* Error checking */
    if ( fs_handle == NULL )
    {
        os_filesystem_log( "Null input!" );
        return kUnknownErr;
    }

    /* Find device in mounted table */
    for ( i = 0; i < MXOS_FILESYSTEM_MOUNT_DEVICE_NUM_MAX; i++ )
    {
        if ( mounted_table[i].fs_handle == fs_handle )
        {
            os_filesystem_log( "Found device in mounted table with index %lu", (unsigned long)i );
            is_handle_found = MXOS_TRUE;
            break;
        }
    }
    if ( (is_handle_found == MXOS_FALSE) || (i > total_mounted_index) )
    {
        os_filesystem_log( "Not existing mounted device!" );
        return kUnknownErr;
    }

    /* Delete in table (shift up in table if deleted not the last one) */
    shift_up_num = (total_mounted_index - i);
    os_filesystem_log( "Deleting %s in mounted table with index %lu. (shift up %lu)", mounted_table[i].name, (unsigned long)i, (unsigned long)shift_up_num );

    if ( ( shift_up_num != 0 ) && ( i < ( MXOS_FILESYSTEM_MOUNT_DEVICE_NUM_MAX - 1 ) ) )
    {
        /* If shift up not zero, copy up before clearing the last entry */
        memcpy( (void *) &mounted_table[i], (void *) &mounted_table[i+1], (sizeof(mxos_filesystem_mounted_device_t) * shift_up_num) );
    }
    memset( (void *) &mounted_table[total_mounted_index], 0, sizeof(mxos_filesystem_mounted_device_t) );

    /* Decrease index if all right */
    total_mounted_index --;

    /* Only for debug dump */
    #if 0
    printf( "\n--- Filesystem Mounted Table ---\n");
    for ( i = 0; i < mxos_FILESYSTEM_MOUNT_DEVICE_NUM_MAX; i++ )
    {
        printf( "[%02ld]\t %s\t\t 0x%08lx\n", i, mounted_table[i].name, (uint32_t)mounted_table[i].fs_handle);
    }
    #endif

    return kNoErr;
}

merr_t mxos_filesystem_mount ( mxos_block_device_t* device, mxos_filesystem_handle_type_t fs_type, mxos_filesystem_t* fs_handle_out, const char* mounted_name,mxos_partition_t partition )
{
    merr_t result;

    /* These ifdefs ensure that the drivers are only pulled in if they are used */
    switch ( fs_type )
    {
#ifdef USING_FTFS
        case MXOS_FILESYSTEM_HANDLE_FTFS:
            fs_handle_out->driver = &mxos_filesystem_driver_ftfs;
            fs_handle_out->partition = partition;
            break;
#endif /* ifdef USING_mxosFS */
#ifdef USING_FATFS
        case MXOS_FILESYSTEM_HANDLE_FATFS:
            fs_handle_out->driver = &mxos_filesystem_driver_fatfs;
            fs_handle_out->partition = partition;
            break;
#endif /* ifdef USING_FATFS */

#ifndef USING_FTFS
        case MXOS_FILESYSTEM_HANDLE_FTFS:
#endif /* ifdef USING_mxosFS */
#ifndef USING_FATFS
        case MXOS_FILESYSTEM_HANDLE_FATFS:
#endif /* ifdef USING_FATFS */
        default:
            return MXOS_FILESYSTEM_ERROR;
    }

    fs_handle_out->device = device;

    result = fs_handle_out->driver->mount( device, fs_handle_out );
    if ( result == kNoErr )
    {
       result = mxos_filesystem_add_mounted_device ( fs_handle_out, mounted_name );
    }

    return result;
}

merr_t mxos_filesystem_unmount ( mxos_filesystem_t* fs_handle )
{
    merr_t result;

    result = fs_handle->driver->unmount( fs_handle );
    if ( result == kNoErr )
    {
        result = mxos_filesystem_del_mounted_device ( fs_handle );
    }

    return result;
}

mxos_filesystem_t* mxos_filesystem_retrieve_mounted_fs_handle ( const char* mounted_name )
{
    mxos_filesystem_t* fs_handle_get = NULL;
    uint32_t i;

    /* Find device in mounted table */
    for ( i = 0; i < MXOS_FILESYSTEM_MOUNT_DEVICE_NUM_MAX; i++ )
    {
        if ( strcmp(mounted_table[i].name, mounted_name) == 0 )
        {
            os_filesystem_log( "Found name in mounted table with index %lu", (unsigned long)i );
            fs_handle_get = mounted_table[i].fs_handle;
            break;
        }
    }

    return fs_handle_get;
}

merr_t mxos_filesystem_file_open ( mxos_filesystem_t* fs_handle, mxos_file_t* file_handle_out, const char* filename, mxos_filesystem_open_mode_t mode )
{
    file_handle_out->filesystem = fs_handle;
    file_handle_out->driver     = fs_handle->driver;
    return fs_handle->driver->file_open( fs_handle, file_handle_out, filename, mode );
}

merr_t mxos_filesystem_file_get_details ( mxos_filesystem_t* fs_handle, const char* filename, mxos_dir_entry_details_t* details_out )
{
    return fs_handle->driver->file_get_details( fs_handle, filename, details_out );
}

merr_t mxos_filesystem_file_close ( mxos_file_t* file_handle )
{
    return file_handle->driver->file_close( file_handle );
}

merr_t mxos_filesystem_file_delete ( mxos_filesystem_t* fs_handle, const char* filename )
{
    return fs_handle->driver->file_delete( fs_handle, filename );
}

merr_t mxos_filesystem_file_seek ( mxos_file_t* file_handle, int64_t offset, mxos_filesystem_seek_type_t whence )
{
    return file_handle->driver->file_seek( file_handle, offset, whence );
}

merr_t mxos_filesystem_file_tell ( mxos_file_t* file_handle, uint64_t* location )
{
    return file_handle->driver->file_tell( file_handle, location );
}

merr_t mxos_filesystem_file_read ( mxos_file_t* file_handle, void* data, uint64_t bytes_to_read, uint64_t* returned_bytes_count )
{
    return file_handle->driver->file_read( file_handle, data, bytes_to_read, returned_bytes_count );
}

merr_t mxos_filesystem_file_write( mxos_file_t* file_handle, const void* data, uint64_t bytes_to_write, uint64_t* written_bytes_count )
{
    return file_handle->driver->file_write( file_handle, data, bytes_to_write, written_bytes_count );
}

merr_t mxos_filesystem_file_flush ( mxos_file_t* file_handle )
{
    return file_handle->driver->file_flush( file_handle );
}


int mxos_filesystem_file_end_reached ( mxos_file_t* file_handle )
{
    return file_handle->driver->file_end_reached( file_handle );
}

merr_t mxos_filesystem_dir_open ( mxos_filesystem_t* fs_handle, mxos_dir_t* dir_handle, const char* dir_name )
{
    dir_handle->filesystem = fs_handle;
    dir_handle->driver     = fs_handle->driver;

    if ( ( dir_name == NULL ) || ( strlen(dir_name) <= 0 ) )
    {
        return MXOS_FILESYSTEM_BADARG;
    }

    return fs_handle->driver->dir_open( fs_handle, dir_handle, dir_name );
}

merr_t mxos_filesystem_dir_close ( mxos_dir_t* dir_handle )
{
    return dir_handle->driver->dir_close( dir_handle );
}

merr_t mxos_filesystem_dir_read( mxos_dir_t* dir_handle, char* name_buffer, unsigned int name_buffer_length, mxos_dir_entry_type_t* type, mxos_dir_entry_details_t* details )
{
    return dir_handle->driver->dir_read( dir_handle, name_buffer, name_buffer_length, type, details );
}

int mxos_filesystem_dir_end_reached ( mxos_dir_t* dir_handle )
{
    return dir_handle->driver->dir_end_reached( dir_handle );
}

merr_t mxos_filesystem_dir_rewind ( mxos_dir_t* dir_handle )
{
    return dir_handle->driver->dir_rewind( dir_handle );
}

merr_t mxos_filesystem_dir_create( mxos_filesystem_t* fs_handle, const char* directory_name )
{
    return fs_handle->driver->dir_create( fs_handle, directory_name );
}

merr_t mxos_filesystem_format( mxos_block_device_t* device, mxos_filesystem_handle_type_t fs_type )
{
    mxos_filesystem_driver_t* driver;
    /* These ifdefs ensure that the drivers are only pulled in if they are used */
    switch ( fs_type )
    {
#ifdef USING_FTFS
        case MXOS_FILESYSTEM_HANDLE_FTFS:
            driver = &mxos_filesystem_driver_ftfs;
            break;
#endif /* ifdef USING_mxosFS */
#ifdef USING_FATFS
        case MXOS_FILESYSTEM_HANDLE_FATFS:
            driver = &mxos_filesystem_driver_fatfs;
            break;
#endif /* ifdef USING_FATFS */

#ifndef USING_FTFS
        case MXOS_FILESYSTEM_HANDLE_FTFS:
#endif /* ifdef USING_mxosFS */
#ifndef USING_FATFS
        case MXOS_FILESYSTEM_HANDLE_FATFS:
#endif /* ifdef USING_FATFS */
        default:
            return MXOS_FILESYSTEM_ERROR;
    }

    return driver->format( device );
}

merr_t mxos_filesystem_get_info( mxos_filesystem_t* fs_handle,mxos_filesystem_info* info,char* mounted_name )
{
    return fs_handle->driver->get_info( info,mounted_name );
}

merr_t mxos_filesystem_scan_files( mxos_filesystem_t* fs_handle, char* mounted_name, mxos_scan_file_handle arg )
{
    return fs_handle->driver->scan_files(fs_handle, mounted_name, arg );
}
