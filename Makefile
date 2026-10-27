# Compile the ST program with matiec (the IEC 61131-3 compiler OpenPLC uses),
# then build a shared library the Python tests drive scan by scan.
MATIEC ?= $(HOME)/matiec
BUILD  := build

all: $(BUILD)/libe84.so $(BUILD)/e84_sim.st $(BUILD)/e84_io.st check-io

$(BUILD)/e84_sim.st: plc/e84_load_port.st plc/wrap_sim.st
	mkdir -p $(BUILD)
	cat $^ > $@

$(BUILD)/e84_io.st: plc/e84_load_port.st plc/wrap_io.st
	mkdir -p $(BUILD)
	cat $^ > $@

$(BUILD)/sim/POUS.c: $(BUILD)/e84_sim.st
	mkdir -p $(BUILD)/sim
	$(MATIEC)/iec2c -I $(MATIEC)/lib -T $(BUILD)/sim $<

$(BUILD)/libe84.so: $(BUILD)/sim/POUS.c plc/glue.c
	gcc -shared -fPIC -O2 -w -I $(MATIEC)/lib/C -I $(BUILD)/sim \
	    plc/glue.c $(BUILD)/sim/Config0.c $(BUILD)/sim/Res0.c -o $@

check-io: $(BUILD)/e84_io.st
	mkdir -p $(BUILD)/io
	$(MATIEC)/iec2c -I $(MATIEC)/lib -T $(BUILD)/io $<
	@echo "hardware I/O variant compiles"

clean:
	rm -rf $(BUILD)
.PHONY: all clean check-io
