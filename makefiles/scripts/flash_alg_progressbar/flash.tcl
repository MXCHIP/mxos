######################################
# Author : Snow Yang
# Date   : 2018-12-03
# Mail   : yangsw@mxchip.com
######################################

set FLASH_ALG_CONFIG_START 0x08000000
set FLASH_ALG_ENTRY_LOC     $($FLASH_ALG_CONFIG_START + 0x00)
set FLASH_ALG_BUF_SIZE_LOC  $($FLASH_ALG_CONFIG_START + 0x04)
set FLASH_ALG_RDY_LOC       $($FLASH_ALG_CONFIG_START + 0x08)
set FLASH_ALG_CMD_LOC       $($FLASH_ALG_CONFIG_START + 0x0C)
set FLASH_ALG_RET_LOC       $($FLASH_ALG_CONFIG_START + 0x10)
set FLASH_ALG_BUF_LOC       $($FLASH_ALG_CONFIG_START + 0x14)

proc memread32 {address} {
    mem2array memar 32 $address 1
    return $memar(0)
}

proc load_image_bin {fname foffset address length } {
    load_image $fname [expr $address - $foffset] bin $address $length
}

proc flash_alg_init { alg_elf } {

    global FLASH_ALG_BUF_SIZE

    reset halt

    load_image $alg_elf

    set FLASH_ALG_ENTRY [memread32 $::FLASH_ALG_ENTRY_LOC]
    set FLASH_ALG_BUF_SIZE [memread32 $::FLASH_ALG_BUF_SIZE_LOC]

    reg pc $FLASH_ALG_ENTRY

    resume
}

proc flash_alg_cmd_run { timeout } {

    mww $::FLASH_ALG_RDY_LOC 1

    loop t 0 $timeout 1 {
        after 1
        set ret [memread32 $::FLASH_ALG_RDY_LOC]  
        if { $ret == 0 } {
            set ret [memread32 $::FLASH_ALG_RET_LOC]
            return $ret
        }
    }
    
    error "error"
    exit -1;
}
