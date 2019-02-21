import sys
import os

system = sys.platform
kill_cmd = None
if(system == 'win32'):
        cmd_path = os.getcwd() + '\\MiCoder\\cmd\\Win32\\'
        tasklist = cmd_path + 'tasklist.exe'
        taskkill = cmd_path + 'taskkill.exe'
        if('openocd_mxos.exe' in os.popen(tasklist).read()):
		kill_cmd = taskkill+' /F /IM openocd_mxos.exe'
elif(system == 'darwin'):
	if(os.system('/bin/ps xc | /usr/bin/grep openocd_mxos_run') == 0):
		kill_cmd = '/usr/bin/killall openocd_mxos_run'
elif(system == 'linux'):
	if(os.system('ps xc | grep openocd_mxos') == 0):
		kill_cmd = 'killall openocd_mxos'

if(kill_cmd):
	print 'WARNING: openocd process has already exits, kill it first'
	os.system(kill_cmd)
