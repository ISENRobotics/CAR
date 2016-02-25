cmd_src/arm/BB-BONE-SERL-03-00A1.dtbo = cpp -Wp,-MD,src/arm/.BB-BONE-SERL-03-00A1.dtbo.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.BB-BONE-SERL-03-00A1.dtbo.dts.tmp src/arm/BB-BONE-SERL-03-00A1.dts ; /usr/local/bin/dtc -O dtb -o src/arm/BB-BONE-SERL-03-00A1.dtbo -b 0 -@ -i src/arm -d src/arm/.BB-BONE-SERL-03-00A1.dtbo.d.dtc.tmp src/arm/.BB-BONE-SERL-03-00A1.dtbo.dts.tmp ; cat src/arm/.BB-BONE-SERL-03-00A1.dtbo.d.pre.tmp src/arm/.BB-BONE-SERL-03-00A1.dtbo.d.dtc.tmp > src/arm/.BB-BONE-SERL-03-00A1.dtbo.d
BB-BONE-SERL-03-00A1.o: src/arm/BB-BONE-SERL-03-00A1.dts \
 include/dt-bindings/board/am335x-bbw-bbb-base.h \
 include/dt-bindings/pinctrl/am33xx.h include/dt-bindings/pinctrl/omap.h
src/arm/BB-BONE-SERL-03-00A1.dtbo: src/arm/.BB-BONE-SERL-03-00A1.dtbo.dts.tmp
