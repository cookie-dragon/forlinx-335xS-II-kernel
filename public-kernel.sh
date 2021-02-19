#!/bin/sh
set -e
export PATH=/usr/local/arm/cross/am335xt3/devkit/bin:$PATH
sed -i-e "s/omap2.o/omap2.lo/" ./drivers/mtd/nand/Makefile
rm ./drivers/mtd/nand/omap2.c
mv ./drivers/mtd/nand/omap2.o ./drivers/mtd/nand/omap2.lo
sed -i-e "s/d_can.o/d_can.lo/" ./drivers/net/can/d_can/Makefile
rm ./drivers/net/can/d_can/d_can.c
mv ./drivers/net/can/d_can/d_can.o ./drivers/net/can/d_can/d_can.lo
cp ./drivers/net/wireless/realtek/rtl8189es/makefile ./drivers/net/wireless/realtek/rtl8189es/Makefile
rm ./drivers/net/wireless/realtek/rtl8189es/makefile
cp ./drivers/net/wireless/realtek/rtl8189es/8189es.o ./drivers/net/wireless/realtek/rtl8189es/8189es.lo
rm -fr ./drivers/net/wireless/realtek/rtl8189es/core
rm -fr ./drivers/net/wireless/realtek/rtl8189es/hal
rm -fr ./drivers/net/wireless/realtek/rtl8189es/include
rm -fr ./drivers/net/wireless/realtek/rtl8189es/os_dep
cp ./drivers/net/wireless/realtek/rtl8192cu/makefile ./drivers/net/wireless/realtek/rtl8192cu/Makefile
rm ./drivers/net/wireless/realtek/rtl8192cu/makefile
cp ./drivers/net/wireless/realtek/rtl8192cu/8192cu.o ./drivers/net/wireless/realtek/rtl8192cu/8192cu.lo
rm -fr ./drivers/net/wireless/realtek/rtl8192cu/core
rm -fr ./drivers/net/wireless/realtek/rtl8192cu/hal
rm -fr ./drivers/net/wireless/realtek/rtl8192cu/include
rm -fr ./drivers/net/wireless/realtek/rtl8192/os_dep
make CROSS_COMPILE=arm-arago-linux-gnueabi- ARCH=arm distclean

