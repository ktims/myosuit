TRG=myosuit-bootloader.hex
ASM=avra

all: $(TRG)

%.hex: %.asm
	avra -fM -o $@ $<
