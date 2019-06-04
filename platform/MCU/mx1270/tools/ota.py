import os
import sys
import struct
import shutil

workdir = os.path.dirname(sys.argv[0])
xzpath = os.path.join(
    workdir, 'xz', 'osx/xz' if sys.platform == 'darwin' else 'win/xz.exe')

rawfile = sys.argv[1]
rawsize = os.path.getsize(rawfile)
xzfile = rawfile.replace('.bin', '.bin.xz')
otafile = rawfile.replace('.bin', '.ota.bin')
os.system('%s --lzma2=dict=32KiB --check=crc32 -k %s' % (xzpath, rawfile))
open(xzfile, 'ab').write(struct.pack('<L', rawsize))
shutil.move(xzfile, otafile)