#!/usr/bin/python

import sys
import re

map_file = sys.argv[1]

total_ram = 0
total_rom = 0
map_lines = []
with open(map_file, 'r') as f:
	s = f.read()

	# find the memory configuration
	mem_config_text = re.findall('Memory Configuration\n\nName             Origin             Length             Attributes\n([\s\S]+)\nLinker script and memory map', s)[0]
	# find the ROM configuration
	rom_config_text = re.findall('\w+\s+(0x\w+)\s+(0x\w+)\s+xr\n',mem_config_text)
	# get every ROM configuration's start - end address
	rom_config = []
	for rom in rom_config_text:
		rom_config += [{'start':int(rom[0], 16), 'end':int(rom[0], 16) + int(rom[1], 16)}]
	# find the RAM configuration
	ram_config_text = re.findall('\w+\s+(0x\w+)\s+(0x\w+)\s+xrw\n',mem_config_text)
	# get every RAM configuration's  start - end address
	ram_config = []
	for ram in ram_config_text:
		ram_config += [{'start':int(ram[0], 16), 'end':int(ram[0], 16) + int(ram[1], 16)}]

	# find memory map (without discard and debug sections)
	mem_map = re.findall('Linker script and memory map([\s\S]+?)START GROUP', s)[0]

	# find sections address - length in memory map
	modules = list(set(item[0] for item in re.findall('0x\w+\s+0x\w+\s+.+?([^/\\\]+\.[ao])(\(.+\.o\))?\n', mem_map)))
	modules.sort(key = lambda x : x.upper())
	modules += ['*fill*']

	for module in modules:
		rom_size = 0
		ram_size = 0
		module = module.replace('+', '\+')
		# get module's sections's address and size
		if(module == '*fill*'):
			sections = map(lambda arg : {'address':int(arg[0], 16), 'size':int(arg[1], 16)}, re.findall('\*fill\*[ \t]+(0x\w+)[ \t]+(0x\w+)[ \t]+\n', mem_map))
		else:
			sections = map(lambda arg : {'address':int(arg[0], 16), 'size':int(arg[1], 16)}, re.findall('(0x\w+)[ \t]+(0x\w+)[ \t]+.+[/\\\]'+module+'(\(.+\.o\))?\n', mem_map))
		if(not sections):
			continue


		def ram_size(arg):
			for ram_info in ram_config:
				if(ram_info['start'] < arg['address'] < ram_info['end']):
					return arg['size']
			return 0

		def rom_size(arg):
			for rom_info in rom_config:
				if(rom_info['start'] < arg['address'] < rom_info['end']):
					return arg['size']
			return 0

		ram_size = reduce(lambda x,y:x+y, map(ram_size, sections))
		rom_size = reduce(lambda x,y:x+y, map(rom_size, sections))

		total_ram += ram_size
		total_rom += rom_size

		map_lines.append('| %-40s | %-8d  | %-8d |'%(re.sub('\.[ao]','',module)[:40],rom_size,ram_size))

print '\n                        MXOS MEMORY MAP                            '	
print '|=================================================================|'	
print '| %-40s | %-8s  | %-8s |'%('MODULE','ROM','RAM')
print '|=================================================================|'	
for line in map_lines:
	print line
print '|=================================================================|'		
print '| %-40s | %-8d  | %-8d |'%('TOTAL (bytes)', total_rom, total_ram)
print '|=================================================================|'

