cmd_src/arm/univ-hdmi-00A0.dtbo = cpp -Wp,-MD,src/arm/.univ-hdmi-00A0.dtbo.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.univ-hdmi-00A0.dtbo.dts.tmp src/arm/univ-hdmi-00A0.dts ; /usr/local/bin/dtc -O dtb -o src/arm/univ-hdmi-00A0.dtbo -b 0 -@ -i src/arm -d src/arm/.univ-hdmi-00A0.dtbo.d.dtc.tmp src/arm/.univ-hdmi-00A0.dtbo.dts.tmp ; cat src/arm/.univ-hdmi-00A0.dtbo.d.pre.tmp src/arm/.univ-hdmi-00A0.dtbo.d.dtc.tmp > src/arm/.univ-hdmi-00A0.dtbo.d
univ-hdmi-00A0.o: src/arm/univ-hdmi-00A0.dts
src/arm/univ-hdmi-00A0.dtbo: src/arm/.univ-hdmi-00A0.dtbo.dts.tmp
