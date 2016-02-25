cmd_src/arm/dev-USB-PWR-CTL-00A1.dtbo = cpp -Wp,-MD,src/arm/.dev-USB-PWR-CTL-00A1.dtbo.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.dev-USB-PWR-CTL-00A1.dtbo.dts.tmp src/arm/dev-USB-PWR-CTL-00A1.dts ; /usr/local/bin/dtc -O dtb -o src/arm/dev-USB-PWR-CTL-00A1.dtbo -b 0 -@ -i src/arm -d src/arm/.dev-USB-PWR-CTL-00A1.dtbo.d.dtc.tmp src/arm/.dev-USB-PWR-CTL-00A1.dtbo.dts.tmp ; cat src/arm/.dev-USB-PWR-CTL-00A1.dtbo.d.pre.tmp src/arm/.dev-USB-PWR-CTL-00A1.dtbo.d.dtc.tmp > src/arm/.dev-USB-PWR-CTL-00A1.dtbo.d
dev-USB-PWR-CTL-00A1.o: src/arm/dev-USB-PWR-CTL-00A1.dts \
 include/dt-bindings/board/am335x-bbw-bbb-base.h \
 include/dt-bindings/gpio/gpio.h include/dt-bindings/pinctrl/am33xx.h \
 include/dt-bindings/pinctrl/omap.h
src/arm/dev-USB-PWR-CTL-00A1.dtbo: src/arm/.dev-USB-PWR-CTL-00A1.dtbo.dts.tmp
