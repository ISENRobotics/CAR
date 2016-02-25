cmd_src/arm/bone_eqep0-00A0.dtbo = cpp -Wp,-MD,src/arm/.bone_eqep0-00A0.dtbo.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.bone_eqep0-00A0.dtbo.dts.tmp src/arm/bone_eqep0-00A0.dts ; /usr/local/bin/dtc -O dtb -o src/arm/bone_eqep0-00A0.dtbo -b 0 -@ -i src/arm -d src/arm/.bone_eqep0-00A0.dtbo.d.dtc.tmp src/arm/.bone_eqep0-00A0.dtbo.dts.tmp ; cat src/arm/.bone_eqep0-00A0.dtbo.d.pre.tmp src/arm/.bone_eqep0-00A0.dtbo.d.dtc.tmp > src/arm/.bone_eqep0-00A0.dtbo.d
bone_eqep0-00A0.o: src/arm/bone_eqep0-00A0.dts \
 include/dt-bindings/board/am335x-bbw-bbb-base.h \
 include/dt-bindings/gpio/gpio.h include/dt-bindings/pinctrl/am33xx.h \
 include/dt-bindings/pinctrl/omap.h
src/arm/bone_eqep0-00A0.dtbo: src/arm/.bone_eqep0-00A0.dtbo.dts.tmp
