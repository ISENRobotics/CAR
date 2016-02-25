cmd_src/arm/BB-I2C2-0A00.dtbo = cpp -Wp,-MD,src/arm/.BB-I2C2-0A00.dtbo.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.BB-I2C2-0A00.dtbo.dts.tmp src/arm/BB-I2C2-0A00.dts ; /usr/local/bin/dtc -O dtb -o src/arm/BB-I2C2-0A00.dtbo -b 0 -@ -i src/arm -d src/arm/.BB-I2C2-0A00.dtbo.d.dtc.tmp src/arm/.BB-I2C2-0A00.dtbo.dts.tmp ; cat src/arm/.BB-I2C2-0A00.dtbo.d.pre.tmp src/arm/.BB-I2C2-0A00.dtbo.d.dtc.tmp > src/arm/.BB-I2C2-0A00.dtbo.d
BB-I2C2-0A00.o: src/arm/BB-I2C2-0A00.dts
src/arm/BB-I2C2-0A00.dtbo: src/arm/.BB-I2C2-0A00.dtbo.dts.tmp
