cur_path=$(cd `dirname $0`; pwd)
cp $cur_path/gnu_utility/gnu_script/rtl_gdb_flash_write_all.txt $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
gsed -i "s#^set \$KM0_BOOT_ALL_SIZE =.*#set \$KM0_BOOT_ALL_SIZE =$(stat -f %z $cur_path/../image/km0_boot_all.bin)#g" $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
gsed -i "s#^set \$KM4_BOOT_ALL_SIZE =.*#set \$KM4_BOOT_ALL_SIZE =$(stat -f %z $cur_path/../image/km4_boot_all.bin)#g" $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
gsed -i "s#^set \$ATE_IMAGE_SIZE =.*#set \$ATE_IMAGE_SIZE =$(stat -f %z $cur_path/../image/ate.bin)#g" $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
cp $1 $cur_path/image/image.bin
gsed -i "s#^set \$KM0_KM4_IMAGE2_SIZE =.*#set \$KM0_KM4_IMAGE2_SIZE =$(stat -f %z $1)#g" $cur_path/gnu_utility/gnu_script/rtl_flash_write.txt
cd $cur_path
./JLinkGDBServerCLExe -device cortex-m23 -if SWD -s &
./arm-none-eabi-gdb -x gnu_utility/gnu_script/rtl_flash_write.txt
