cur_path=$(cd `dirname $0`; pwd)
cp $1 $cur_path/image/image.bin
gsed -i "s#^set \$XIPBootSize =.*#set \$XIPBootSize =$(stat -f %z $1)#g" $cur_path/gnu_utility/gnu_script/rtl_gdb_flash_write.txt
cd $cur_path
./JLinkGDBServerCLExe -device cortex-m23 -if SWD -s &
./arm-none-eabi-gdb -x gnu_utility/gnu_script/rtl_gdb_flash_write.txt