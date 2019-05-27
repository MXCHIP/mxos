set CMD_READ  0
set CMD_WRITE 1
set CMD_ERASE 2
set CMD_CRC   3

proc erase { addr size } {

    mww $::FLASH_ALG_CMD_LOC $::CMD_ERASE
    mww $($::FLASH_ALG_BUF_LOC + 0x00) $addr
    mww $($::FLASH_ALG_BUF_LOC + 0x04) $size

    flash_alg_cmd_run 10000
}

proc write { file addr } {

    set size [file size $file]
    set remain $size
    while {$remain > 0} {
        set n $($remain > $::FLASH_ALG_BUF_SIZE ? $::FLASH_ALG_BUF_SIZE : $remain)
        mww $::FLASH_ALG_CMD_LOC $::CMD_WRITE
        mww $($::FLASH_ALG_BUF_LOC + 0x00) $addr
        mww $($::FLASH_ALG_BUF_LOC + 0x04) $n
        load_image_bin $file $($size - $remain) $($::FLASH_ALG_BUF_LOC + 0x08) $n
        set remain $($remain - $n)
        set addr $($addr + $n)

        flash_alg_cmd_run 1000
        puts "0x[format %x $($size - $remain)]"
    }
}

proc read { file addr size } {
    exec echo -n > $file

    set remain $size
    set tmpfile "tmp.bin"
    while {$remain > 0} {
        set n $($remain > $::FLASH_ALG_BUF_SIZE ? $::FLASH_ALG_BUF_SIZE : $remain)
        mww $::FLASH_ALG_CMD_LOC $::CMD_READ
        mww $($::FLASH_ALG_BUF_LOC + 0x00) $addr
        mww $($::FLASH_ALG_BUF_LOC + 0x04) $n
        set remain $($remain - $n)
        set addr $($addr + $n)

        flash_alg_cmd_run 1000

        dump_image $tmpfile $::FLASH_ALG_BUF_LOC $n
        exec cat $tmpfile >> $file
        exec rm $tmpfile
    }
}

proc crc { addr size } {
    
    mww $::FLASH_ALG_CMD_LOC $::CMD_CRC
    mww $($::FLASH_ALG_BUF_LOC + 0x00) $addr
    mww $($::FLASH_ALG_BUF_LOC + 0x04) $size

    flash_alg_cmd_run 1000

    puts "0x[format %x [memread32 $::FLASH_ALG_BUF_LOC]]"
}

proc writefile { file addr } {

    set size [file size $file]

    set eradr $addr
    set ersiz 0

    set remain $size
    while {$remain > 0} {
        set n $($remain > $::FLASH_ALG_BUF_SIZE ? $::FLASH_ALG_BUF_SIZE : $remain)

        set wend $($addr + $n)
        set eend $($eradr + $ersiz)
        if { $wend >= $eend } {
            set ersiz $(($wend - $eend + 0x1000 - 1) & ~(0x1000 - 1))
            set eradr $($eend & ~(0x1000 - 1))
            erase $eradr $ersiz
        }

        mww $::FLASH_ALG_CMD_LOC $::CMD_WRITE
        mww $($::FLASH_ALG_BUF_LOC + 0x00) $addr
        mww $($::FLASH_ALG_BUF_LOC + 0x04) $n
        load_image_bin $file $($size - $remain) $($::FLASH_ALG_BUF_LOC + 0x08) $n
        set remain $($remain - $n)
        set addr $($addr + $n)

        flash_alg_cmd_run 1000
        puts "0x[format %x $($size - $remain)]"
    }
}

proc readfile { file addr size } {
    read $file $addr $size
}