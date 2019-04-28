all: pipework_altera.vhd pipework_altera.qip qconv_altera.vhd qconv_altera.qip

pipework_altera.vhd :
	../../qconv-strip-vhdl/PipeWork/tools/vhdl-archiver.rb -v       \
            --library PipeWork                                          \
            --use 'SDPRAM(ALTERA_AUTO_SELECT)'                          \
            --use 'QUEUE_ARBITER(INTEGER_ARCH)'                         \
            --archive $@                                                \
            ../../qconv-strip-vhdl/PipeWork/src/                        \
            --library QCONV                                             \
            --top QCONV_STRIP_AXI_CORE                                  \
            ../../qconv-strip-vhdl/src/main/                            \
	    --library work                                              \
	    --top LED8_AXI                                              \
	    ./ikwzm_pipework_led8_axi_0.1/                              \
	    $(ENDLIST)

pipework_altera.qip : pipework_altera.vhd
	echo 'set_global_assignment -name VHDL_FILE -library PIPEWORK [file join $::quartus(qip_path) "pipework_altera.vhd"]' > $@

qconv_altera.vhd :
	../../qconv-strip-vhdl/PipeWork/tools/vhdl-archiver.rb -v       \
            --library PipeWork                                          \
            --use 'SDPRAM(ALTERA_AUTO_SELECT)'                          \
            --use 'QUEUE_ARBITER(INTEGER_ARCH)'                         \
            ../../qconv-strip-vhdl/PipeWork/src/                        \
            --library QCONV                                             \
            --top QCONV_STRIP_AXI_CORE                                  \
            --archive $@                                                \
            ../../qconv-strip-vhdl/src/main/                            \
	    $(ENDLIST)

qconv_altera.qip : qconv_altera.vhd
	echo 'set_global_assignment -name VHDL_FILE -library QCONV [file join $::quartus(qip_path) "qconv_altera.vhd"]' > $@
