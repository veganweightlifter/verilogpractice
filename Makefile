serial.bin: serial.asc
	icepack serial.asc serial.bin

serial.asc: serial.json upduino3.pcf
	nextpnr-ice40 --up5k --package sg48 --json serial.json --pcf upduino3.pcf --asc serial.asc --pcf-allow-unconstrained # run place and route

serial.json: serial.v
	yosys -q -p "synth_ice40 -json serial.json" serial.v

.PHONY: flash
flash:
	iceprog -d i:0x0403:0x6014 serial.bin
#	iceprog -e 128 # Force a reset
.PHONY: clean
clean:
	$(RM) -f serial.json serial.asc serial.bin

