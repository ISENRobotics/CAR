cmd_src/arm/BB-BONE-DVID-01-00A3.dtbo = cpp -Wp,-MD,src/arm/.BB-BONE-DVID-01-00A3.dtbo.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.BB-BONE-DVID-01-00A3.dtbo.dts.tmp src/arm/BB-BONE-DVID-01-00A3.dts ; /usr/local/bin/dtc -O dtb -o src/arm/BB-BONE-DVID-01-00A3.dtbo -b 0 -@ -i src/arm -d src/arm/.BB-BONE-DVID-01-00A3.dtbo.d.dtc.tmp src/arm/.BB-BONE-DVID-01-00A3.dtbo.dts.tmp ; cat src/arm/.BB-BONE-DVID-01-00A3.dtbo.d.pre.tmp src/arm/.BB-BONE-DVID-01-00A3.dtbo.d.dtc.tmp > src/arm/.BB-BONE-DVID-01-00A3.dtbo.d
BB-BONE-DVID-01-00A3.o: src/arm/BB-BONE-DVID-01-00A3.dts \
 include/dt-bindings/board/am335x-bbw-bbb-base.h \
 include/dt-bindings/pinctrl/am33xx.h include/dt-bindings/pinctrl/omap.h
src/arm/BB-BONE-DVID-01-00A3.dtbo: src/arm/.BB-BONE-DVID-01-00A3.dtbo.dts.tmp
