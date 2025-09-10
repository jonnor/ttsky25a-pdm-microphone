.POSIX:

ENVIRONMENT = PATH=$$(realpath venv/bin):$$PATH PDK=sky130A
PDK_ROOT=pdk

# https://github.com/TinyTapeout/volare-action/blob/main/action.yaml
# https://github.com/TinyTapeout/tt-gds-action/blob/main/gl_test/action.yml
PDK_NAME=sky130
PDK_VERSION=cd1748bb197f9b7af62a54507de6624e30363943

TOP_MODULE=tt_um_jonnor_pdm_microphone  

all:

distclean:
	rm -rf tt venv

fpga:
	cp pico_ice/pico_ice.pcf tt/fpga/tt_fpga_top.pcf
	$(ENVIRONMENT) venv/bin/python tt/tt_tool.py --create-fpga-bitstream

# Uses separate venv, just in case it messes with other dependencies
gl_test_setup:
	python3 -m venv test/venv
	test/venv/bin/pip install -r test/requirements.txt
	test/venv/bin/pip install volare==0.19.1
	. test/venv/bin/activate && PDK_ROOT=${PDK_ROOT} volare enable --pdk ${PDK_NAME} ${PDK_VERSION}

gl_test:
	cp ./runs/wokwi/final/pnl/${TOP_MODULE}.pnl.v test/gate_level_netlist.v
	. test/venv/bin/activate && PDK_ROOT=../${PDK_ROOT} make -C test -B GATES=yes

harden:
	$(ENVIRONMENT) venv/bin/python tt/tt_tool.py --harden

pico:
	mpremote cp build/tt_fpga.bin :fpga_bitstream.bin
	mpremote run scripts/fpga_flash_prog.py

record:
	mpremote run scripts/test_peripheral_spi.py
	mpremote cp :pcm.wav ./

png:
	$(ENVIRONMENT) venv/bin/python tt/tt_tool.py --create-png

test:
	cd test && . venv/bin/activate && make -B

tt:
	git clone -b ttsky25a https://github.com/TinyTapeout/tt-support-tools tt
	python3 -m venv venv
	venv/bin/pip install -r tt/requirements.txt
	venv/bin/pip install https://github.com/TinyTapeout/libparse-python/releases/download/0.3.1-dev1/libparse-0.3.1-cp313-cp313-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
	$(ENVIRONMENT) venv/bin/pip install openlane==2.2.9
	$(ENVIRONMENT) venv/bin/python tt/tt_tool.py --create-user-config

.PHONY: all distclean fpga gl_test gl_test_setup harden pico png test tt
