cur_path=$(cd `dirname $0`; pwd)
cp $cur_path/gnu_utility/gnu_script/rtl_gdb_flash_write.txt $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
gsed -i "s#^set \$KM0_BOOT_ALL_SIZE =.*#set \$KM0_BOOT_ALL_SIZE =$(stat -f %z $cur_path/../image/km0_boot_all.bin)#g" $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
gsed -i "s#^set \$KM4_BOOT_ALL_SIZE =.*#set \$KM4_BOOT_ALL_SIZE =$(stat -f %z $cur_path/../image/km4_boot_all.bin)#g" $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
cp $1 $cur_path/image/image.bin
gsed -i "s#^set \$KM0_KM4_IMAGE2_SIZE =.*#set \$KM0_KM4_IMAGE2_SIZE =$(stat -f %z $1)#g" $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
$2 -f mxos/makefiles/OpenOCD/interface/mxlink.cfg -c "transport select swd" -f mxos/makefiles/OpenOCD/rtl8721d/rtl8721d.cfg &
cd $cur_path
./arm-none-eabi-gdb -x gnu_utility/gnu_script/rtl_flash_write.txt