# -*- coding: UTF-8 -*-
# Date    : 2018/08/07
# Author  : Snow Yang
# Mail    : yangsw@mxchip.com

import os
import sys
import struct
import argparse
from subprocess import Popen, CalledProcessError, PIPE
from progressbar import ProgressBar

MARKER = {'OSX': '█', 'Win32': '#'}
FILL = {'OSX': '░', 'Win32': '-'}


def sflasher(argv):
    parser = argparse.ArgumentParser(
        description='Download binary file to flash')
    parser.add_argument('-o', '--openocd', type=str,
                        required=True, help='openocd path')
    parser.add_argument('-m', '--mcu', type=str,
                        required=True, help='mcu name')
    parser.add_argument('-f', '--file', type=str,
                        required=True, help='file name')
    parser.add_argument('-a', '--addr', type=str,
                        required=True, help='address')
    args = parser.parse_args(argv)

    hostos = 'OSX' if sys.platform == 'darwin' else 'Linux64' if sys.platform == 'linux2' else 'Win32'
    pbar = ProgressBar(os.path.getsize(args.file),
                       marker=MARKER[hostos], fill=FILL[hostos])

    cmd = '%s \
    -f mxos/makefiles/OpenOCD/interface/jlink_swd.cfg \
    -f mxos/makefiles/OpenOCD/%s/%s.cfg \
    -f mxos/makefiles/scripts/flash_alg_progressbar/flash.tcl \
    -f mxos/makefiles/scripts/flash_alg_progressbar/cmd.tcl \
    -c init \
    -c "flash_alg_init mxos/platform/MCU/%s/flash_alg.elf" \
    -c "writefile %s %s" -c shutdown \
    2>build/openocd.log' \
    % (args.openocd, args.mcu, args.mcu, args.mcu, args.file, args.addr)
    proc = Popen(cmd, shell=True, universal_newlines=True, stdout=PIPE)

    while True:
        out = proc.stdout.readline().strip()
        if proc.poll() != None:
            if proc.poll():
                with open('build/openocd.log', 'r') as f:
                    print(f.read())
            return
        else:
            pbar.update(int(out, 16))


if __name__ == "__main__":
    try:
        sflasher(sys.argv[1:])
    except Exception as e:
        print(e)
        sys.exit(2)
