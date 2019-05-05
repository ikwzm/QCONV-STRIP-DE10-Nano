package require -exact qsys 12.1

# +-----------------------------------
# | module axi_sample
# | 
set_module_property DESCRIPTION          "Quantized Convolution (strip) with AXI3 I/F"
set_module_property NAME                 qconv_strip_axi3
set_module_property VERSION              1.0
set_module_property INTERNAL             false
set_module_property OPAQUE_ADDRESS_MAP   true
set_module_property AUTHOR               "ikwm"
set_module_property DISPLAY_NAME         "qconv_strip_axi3"
set_module_property EDITABLE             true
set_module_property ANALYZE_HDL          AUTO
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property ALLOW_GREYBOX_GENERATION     false
set_module_property ELABORATION_CALLBACK elaborate
# | 
# +-----------------------------------

# +-----------------------------------
# | file sets
# | 
add_fileset          QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH TOP_LEVEL    QCONV_STRIP_AXI3
add_fileset_file     qconv_strip_axi3.vhd       VHDL PATH qconv_strip_axi3.vhd

add_fileset          SIM_VHDL SIM_VHDL "" ""
set_fileset_property SIM_VHDL ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property SIM_VHDL TOP_LEVEL         QCONV_STRIP_AXI3
add_fileset_file     qconv_strip_axi3.vhd       VHDL PATH qconv_strip_axi3.vhd

# | 
# +-----------------------------------

# +-----------------------------------
# | parameters
# | 
add_parameter          IN_C_UNROLL            INTEGER             1
set_parameter_property IN_C_UNROLL            DEFAULT_VALUE       1
set_parameter_property IN_C_UNROLL            DISPLAY_NAME        "IN_C Unroll"
set_parameter_property IN_C_UNROLL            HDL_PARAMETER       true
set_parameter_property IN_C_UNROLL            ALLOWED_RANGES      1:32

add_parameter          OUT_C_UNROLL           INTEGER             8
set_parameter_property OUT_C_UNROLL           DEFAULT_VALUE       8
set_parameter_property OUT_C_UNROLL           DISPLAY_NAME        "OUT_C Unroll"
set_parameter_property OUT_C_UNROLL           HDL_PARAMETER       true
set_parameter_property OUT_C_UNROLL           ALLOWED_RANGES      1:32

add_parameter          AXI_VERSION            STRING              "AXI3"
set_parameter_property AXI_VERSION            DISPLAY_NAME        "AXI Version"
set_parameter_property AXI_VERSION            TYPE                STRING
set_parameter_property AXI_VERSION            UNITS               None
set_parameter_property AXI_VERSION            DESCRIPTION         "Indicate this is AXI3 master"
set_parameter_property AXI_VERSION            AFFECTS_ELABORATION true
set_parameter_property AXI_VERSION            ALLOWED_RANGES      "AXI3"
set_parameter_property AXI_VERSION            DISPLAY_HINT        "boolean"
set_parameter_property AXI_VERSION            HDL_PARAMETER       false

add_parameter          S_AXI_ADDR_WIDTH       INTEGER             12
set_parameter_property S_AXI_ADDR_WIDTH       DEFAULT_VALUE       12
set_parameter_property S_AXI_ADDR_WIDTH       DISPLAY_NAME        "Control Status Register I/F Byte Address Width"
set_parameter_property S_AXI_ADDR_WIDTH       UNITS               "bits"
set_parameter_property S_AXI_ADDR_WIDTH       HDL_PARAMETER       true
set_parameter_property S_AXI_ADDR_WIDTH       AFFECTS_ELABORATION true
set_parameter_property S_AXI_ADDR_WIDTH       ALLOWED_RANGES      1:64

add_parameter          IO_AXI_DATA_WIDTH      INTEGER             64
set_parameter_property IO_AXI_DATA_WIDTH      DEFAULT_VALUE       64
set_parameter_property IO_AXI_DATA_WIDTH      DISPLAY_NAME        "In/Out Data I/F Data Width"
set_parameter_property IO_AXI_DATA_WIDTH      UNITS               "bits"
set_parameter_property IO_AXI_DATA_WIDTH      HDL_PARAMETER       true
set_parameter_property IO_AXI_DATA_WIDTH      ALLOWED_RANGES      "32,64,128,256,512,1024"

add_parameter          IO_AXI_ADDR_WIDTH      INTEGER             32
set_parameter_property IO_AXI_ADDR_WIDTH      DEFAULT_VALUE       32
set_parameter_property IO_AXI_ADDR_WIDTH      DISPLAY_NAME        "In/Out Data I/F Byte Address Width"
set_parameter_property IO_AXI_ADDR_WIDTH      UNITS               "bits"
set_parameter_property IO_AXI_ADDR_WIDTH      HDL_PARAMETER       true
set_parameter_property IO_AXI_ADDR_WIDTH      AFFECTS_ELABORATION true
set_parameter_property IO_AXI_ADDR_WIDTH      ALLOWED_RANGES      1:64

add_parameter          IO_AXI_ID_WIDTH        INTEGER             2
set_parameter_property IO_AXI_ID_WIDTH        DEFAULT_VALUE       2
set_parameter_property IO_AXI_ID_WIDTH        DISPLAY_NAME        "In/Out Data I/F AXI ID width"
set_parameter_property IO_AXI_ID_WIDTH        TYPE                INTEGER
set_parameter_property IO_AXI_ID_WIDTH        UNITS               None
set_parameter_property IO_AXI_ID_WIDTH        DESCRIPTION         "In/Out Data I/F AXI ID width"
set_parameter_property IO_AXI_ID_WIDTH        AFFECTS_ELABORATION true
set_parameter_property IO_AXI_ID_WIDTH        HDL_PARAMETER       true

add_parameter          IO_USE_ADDR_USER       INTEGER             1
set_parameter_property IO_USE_ADDR_USER       DEFAULT_VALUE       1
set_parameter_property IO_USE_ADDR_USER       DISPLAY_NAME        "In/Out Data I/F AXI AWUSER/ARUSER use"
set_parameter_property IO_USE_ADDR_USER       TYPE                INTEGER
set_parameter_property IO_USE_ADDR_USER       UNITS               None
set_parameter_property IO_USE_ADDR_USER       DESCRIPTION         "In/Out Data I/F AXI AWUSER/ARUSER use"
set_parameter_property IO_USE_ADDR_USER       AFFECTS_ELABORATION true
set_parameter_property IO_USE_ADDR_USER       ALLOWED_RANGES      0:1
set_parameter_property IO_USE_ADDR_USER       DISPLAY_HINT        "boolean"

add_parameter          IO_AXI_USER_WIDTH      INTEGER             2
set_parameter_property IO_AXI_USER_WIDTH      DEFAULT_VALUE       2
set_parameter_property IO_AXI_USER_WIDTH      DISPLAY_NAME        "In/Out Data I/F AXI AWUSER/ARUSER width"
set_parameter_property IO_AXI_USER_WIDTH      TYPE                INTEGER
set_parameter_property IO_AXI_USER_WIDTH      UNITS               None
set_parameter_property IO_AXI_USER_WIDTH      DESCRIPTION         "In/Out Data I/F AXI AWUSER/ARUSER width"
set_parameter_property IO_AXI_USER_WIDTH      AFFECTS_ELABORATION true
set_parameter_property IO_AXI_USER_WIDTH      HDL_PARAMETER       true

add_parameter          I_AXI_ID               INTEGER             0
set_parameter_property I_AXI_ID               DEFAULT_VALUE       0
set_parameter_property I_AXI_ID               DISPLAY_NAME        "In Data I/F AXI ID"
set_parameter_property I_AXI_ID               TYPE                INTEGER
set_parameter_property I_AXI_ID               UNITS               None
set_parameter_property I_AXI_ID               DESCRIPTION         "In Data I/F AXI ID"
set_parameter_property I_AXI_ID               AFFECTS_ELABORATION true
set_parameter_property I_AXI_ID               HDL_PARAMETER       true

add_parameter          I_AXI_PROT             INTEGER             0
set_parameter_property I_AXI_PROT             DEFAULT_VALUE       0
set_parameter_property I_AXI_PROT             DISPLAY_NAME        "In Data I/F AXI PROT"
set_parameter_property I_AXI_PROT             TYPE                INTEGER
set_parameter_property I_AXI_PROT             UNITS               None
set_parameter_property I_AXI_PROT             DESCRIPTION         "In Data I/F AXI PROT"
set_parameter_property I_AXI_PROT             AFFECTS_ELABORATION true
set_parameter_property I_AXI_PROT             HDL_PARAMETER       true

add_parameter          I_AXI_CACHE            INTEGER             15
set_parameter_property I_AXI_CACHE            DEFAULT_VALUE       15
set_parameter_property I_AXI_CACHE            DISPLAY_NAME        "In Data I/F AXI CACHE"
set_parameter_property I_AXI_CACHE            TYPE                INTEGER
set_parameter_property I_AXI_CACHE            UNITS               None
set_parameter_property I_AXI_CACHE            DESCRIPTION         "In Data I/F AXI CACHE"
set_parameter_property I_AXI_CACHE            AFFECTS_ELABORATION true
set_parameter_property I_AXI_CACHE            HDL_PARAMETER       true

add_parameter          I_AXI_AUSER            INTEGER             1
set_parameter_property I_AXI_AUSER            DEFAULT_VALUE       1
set_parameter_property I_AXI_AUSER            DISPLAY_NAME        "In Data I/F AXI ARUSER"
set_parameter_property I_AXI_AUSER            TYPE                INTEGER
set_parameter_property I_AXI_AUSER            UNITS               None
set_parameter_property I_AXI_AUSER            DESCRIPTION         "In Data I/F AXI ARUSER"
set_parameter_property I_AXI_AUSER            AFFECTS_ELABORATION true
set_parameter_property I_AXI_AUSER            HDL_PARAMETER       true

add_parameter          O_AXI_ID               INTEGER             0
set_parameter_property O_AXI_ID               DEFAULT_VALUE       0
set_parameter_property O_AXI_ID               DISPLAY_NAME        "Out Data I/F AXI ID"
set_parameter_property O_AXI_ID               TYPE                INTEGER
set_parameter_property O_AXI_ID               UNITS               None
set_parameter_property O_AXI_ID               DESCRIPTION         "Out Data I/F AXI ID"
set_parameter_property O_AXI_ID               AFFECTS_ELABORATION true
set_parameter_property O_AXI_ID               HDL_PARAMETER       true

add_parameter          O_AXI_PROT             INTEGER             0
set_parameter_property O_AXI_PROT             DEFAULT_VALUE       0
set_parameter_property O_AXI_PROT             DISPLAY_NAME        "Out Data I/F AXI PROT"
set_parameter_property O_AXI_PROT             TYPE                INTEGER
set_parameter_property O_AXI_PROT             UNITS               None
set_parameter_property O_AXI_PROT             DESCRIPTION         "Out Data I/F AXI PROT"
set_parameter_property O_AXI_PROT             AFFECTS_ELABORATION true
set_parameter_property O_AXI_PROT             HDL_PARAMETER       true

add_parameter          O_AXI_CACHE            INTEGER             15
set_parameter_property O_AXI_CACHE            DEFAULT_VALUE       15
set_parameter_property O_AXI_CACHE            DISPLAY_NAME        "Out Data I/F AXI CACHE"
set_parameter_property O_AXI_CACHE            TYPE                INTEGER
set_parameter_property O_AXI_CACHE            UNITS               None
set_parameter_property O_AXI_CACHE            DESCRIPTION         "Out Data I/F AXI CACHE"
set_parameter_property O_AXI_CACHE            AFFECTS_ELABORATION true
set_parameter_property O_AXI_CACHE            HDL_PARAMETER       true

add_parameter          O_AXI_AUSER            INTEGER             1
set_parameter_property O_AXI_AUSER            DEFAULT_VALUE       1
set_parameter_property O_AXI_AUSER            DISPLAY_NAME        "Out Data I/F AXI AWUSER"
set_parameter_property O_AXI_AUSER            TYPE                INTEGER
set_parameter_property O_AXI_AUSER            UNITS               None
set_parameter_property O_AXI_AUSER            DESCRIPTION         "Out Data I/F AXI AWUSER"
set_parameter_property O_AXI_AUSER            AFFECTS_ELABORATION true
set_parameter_property O_AXI_AUSER            HDL_PARAMETER       true

add_parameter          K_AXI_ID_WIDTH         INTEGER             2
set_parameter_property K_AXI_ID_WIDTH         DEFAULT_VALUE       2
set_parameter_property K_AXI_ID_WIDTH         DISPLAY_NAME        "K Data I/F AXI ID width"
set_parameter_property K_AXI_ID_WIDTH         TYPE                INTEGER
set_parameter_property K_AXI_ID_WIDTH         UNITS               None
set_parameter_property K_AXI_ID_WIDTH         DESCRIPTION         "K Data I/F AXI ID width"
set_parameter_property K_AXI_ID_WIDTH         AFFECTS_ELABORATION true
set_parameter_property K_AXI_ID_WIDTH         HDL_PARAMETER       true

add_parameter          K_AXI_DATA_WIDTH       INTEGER             64
set_parameter_property K_AXI_DATA_WIDTH       DEFAULT_VALUE       64
set_parameter_property K_AXI_DATA_WIDTH       DISPLAY_NAME        "K Data I/F Data Width"
set_parameter_property K_AXI_DATA_WIDTH       UNITS               "bits"
set_parameter_property K_AXI_DATA_WIDTH       HDL_PARAMETER       true
set_parameter_property K_AXI_DATA_WIDTH       ALLOWED_RANGES      "32,64,128,256,512,1024"

add_parameter          K_AXI_ADDR_WIDTH       INTEGER             32
set_parameter_property K_AXI_ADDR_WIDTH       DEFAULT_VALUE       32
set_parameter_property K_AXI_ADDR_WIDTH       DISPLAY_NAME        "K Data I/F Byte Address Width"
set_parameter_property K_AXI_ADDR_WIDTH       UNITS               "bits"
set_parameter_property K_AXI_ADDR_WIDTH       HDL_PARAMETER       true
set_parameter_property K_AXI_ADDR_WIDTH       AFFECTS_ELABORATION true
set_parameter_property K_AXI_ADDR_WIDTH       ALLOWED_RANGES      1:64

add_parameter          K_USE_ADDR_USER        INTEGER             1
set_parameter_property K_USE_ADDR_USER        DEFAULT_VALUE       1
set_parameter_property K_USE_ADDR_USER        DISPLAY_NAME        "K Data I/F AXI ARUSER/AWUSER use"
set_parameter_property K_USE_ADDR_USER        TYPE                INTEGER
set_parameter_property K_USE_ADDR_USER        UNITS               None
set_parameter_property K_USE_ADDR_USER        DESCRIPTION         "K Data I/F AXI ARUSER/AWUSER use"
set_parameter_property K_USE_ADDR_USER        AFFECTS_ELABORATION true
set_parameter_property K_USE_ADDR_USER        ALLOWED_RANGES      0:1
set_parameter_property K_USE_ADDR_USER        DISPLAY_HINT        "boolean"

add_parameter          K_AXI_USER_WIDTH       INTEGER             2
set_parameter_property K_AXI_USER_WIDTH       DEFAULT_VALUE       2
set_parameter_property K_AXI_USER_WIDTH       DISPLAY_NAME        "K Data I/F AXI AWUSER/ARUSER width"
set_parameter_property K_AXI_USER_WIDTH       TYPE                INTEGER
set_parameter_property K_AXI_USER_WIDTH       UNITS               None
set_parameter_property K_AXI_USER_WIDTH       DESCRIPTION         "K Data I/F AXI AWUSER/ARUSER width"
set_parameter_property K_AXI_USER_WIDTH       AFFECTS_ELABORATION true
set_parameter_property K_AXI_USER_WIDTH       HDL_PARAMETER       true

add_parameter          K_AXI_ID               INTEGER             1
set_parameter_property K_AXI_ID               DEFAULT_VALUE       1
set_parameter_property K_AXI_ID               DISPLAY_NAME        "K Data I/F AXI ID"
set_parameter_property K_AXI_ID               TYPE                INTEGER
set_parameter_property K_AXI_ID               UNITS               None
set_parameter_property K_AXI_ID               DESCRIPTION         "K Data I/F AXI ID"
set_parameter_property K_AXI_ID               AFFECTS_ELABORATION true
set_parameter_property K_AXI_ID               HDL_PARAMETER       true

add_parameter          K_AXI_PROT             INTEGER             0
set_parameter_property K_AXI_PROT             DEFAULT_VALUE       0
set_parameter_property K_AXI_PROT             DISPLAY_NAME        "K Data I/F AXI PROT"
set_parameter_property K_AXI_PROT             TYPE                INTEGER
set_parameter_property K_AXI_PROT             UNITS               None
set_parameter_property K_AXI_PROT             DESCRIPTION         "K Data I/F AXI PROT"
set_parameter_property K_AXI_PROT             AFFECTS_ELABORATION true
set_parameter_property K_AXI_PROT             HDL_PARAMETER       true

add_parameter          K_AXI_CACHE            INTEGER             15
set_parameter_property K_AXI_CACHE            DEFAULT_VALUE       15
set_parameter_property K_AXI_CACHE            DISPLAY_NAME        "K Data I/F AXI CACHE"
set_parameter_property K_AXI_CACHE            TYPE                INTEGER
set_parameter_property K_AXI_CACHE            UNITS               None
set_parameter_property K_AXI_CACHE            DESCRIPTION         "K Data I/F AXI CACHE"
set_parameter_property K_AXI_CACHE            AFFECTS_ELABORATION true
set_parameter_property K_AXI_CACHE            HDL_PARAMETER       true

add_parameter          K_AXI_AUSER            INTEGER             1
set_parameter_property K_AXI_AUSER            DEFAULT_VALUE       1
set_parameter_property K_AXI_AUSER            DISPLAY_NAME        "K Data I/F AXI ARUSER"
set_parameter_property K_AXI_AUSER            TYPE                INTEGER
set_parameter_property K_AXI_AUSER            UNITS               None
set_parameter_property K_AXI_AUSER            DESCRIPTION         "K Data I/F AXI ARUSER"
set_parameter_property K_AXI_AUSER            AFFECTS_ELABORATION true
set_parameter_property K_AXI_AUSER            HDL_PARAMETER       true

add_parameter          T_AXI_ID_WIDTH         INTEGER             2
set_parameter_property T_AXI_ID_WIDTH         DEFAULT_VALUE       2
set_parameter_property T_AXI_ID_WIDTH         DISPLAY_NAME        "T Data I/F AXI ID width"
set_parameter_property T_AXI_ID_WIDTH         TYPE                INTEGER
set_parameter_property T_AXI_ID_WIDTH         UNITS               None
set_parameter_property T_AXI_ID_WIDTH         DESCRIPTION         "T Data I/F AXI ID width"
set_parameter_property T_AXI_ID_WIDTH         AFFECTS_ELABORATION true
set_parameter_property T_AXI_ID_WIDTH         HDL_PARAMETER       true

add_parameter          T_AXI_DATA_WIDTH       INTEGER             64
set_parameter_property T_AXI_DATA_WIDTH       DEFAULT_VALUE       64
set_parameter_property T_AXI_DATA_WIDTH       DISPLAY_NAME        "T Data I/F Data Width"
set_parameter_property T_AXI_DATA_WIDTH       UNITS               "bits"
set_parameter_property T_AXI_DATA_WIDTH       HDL_PARAMETER       true
set_parameter_property T_AXI_DATA_WIDTH       ALLOWED_RANGES      "32,64,128,256,512,1024"

add_parameter          T_AXI_ADDR_WIDTH       INTEGER             32
set_parameter_property T_AXI_ADDR_WIDTH       DEFAULT_VALUE       32
set_parameter_property T_AXI_ADDR_WIDTH       DISPLAY_NAME        "T Data I/F Byte Address Width"
set_parameter_property T_AXI_ADDR_WIDTH       UNITS               "bits"
set_parameter_property T_AXI_ADDR_WIDTH       HDL_PARAMETER       true
set_parameter_property T_AXI_ADDR_WIDTH       AFFECTS_ELABORATION true
set_parameter_property T_AXI_ADDR_WIDTH       ALLOWED_RANGES      1:64

add_parameter          T_USE_ADDR_USER        INTEGER             1
set_parameter_property T_USE_ADDR_USER        DEFAULT_VALUE       1
set_parameter_property T_USE_ADDR_USER        DISPLAY_NAME        "T Data I/F AXI AWUSER/ARUSER use"
set_parameter_property T_USE_ADDR_USER        TYPE                INTEGER
set_parameter_property T_USE_ADDR_USER        UNITS               None
set_parameter_property T_USE_ADDR_USER        DESCRIPTION         "T Data I/F AXI AWUSER/ARUSER use"
set_parameter_property T_USE_ADDR_USER        AFFECTS_ELABORATION true
set_parameter_property T_USE_ADDR_USER        ALLOWED_RANGES      0:1
set_parameter_property T_USE_ADDR_USER        DISPLAY_HINT        "boolean"

add_parameter          T_AXI_USER_WIDTH       INTEGER             2
set_parameter_property T_AXI_USER_WIDTH       DEFAULT_VALUE       2
set_parameter_property T_AXI_USER_WIDTH       DISPLAY_NAME        "T Data I/F AXI AWUSER/ARUSER width"
set_parameter_property T_AXI_USER_WIDTH       TYPE                INTEGER
set_parameter_property T_AXI_USER_WIDTH       UNITS               None
set_parameter_property T_AXI_USER_WIDTH       DESCRIPTION         "T Data I/F AXI AWUSER/ARUSER width"
set_parameter_property T_AXI_USER_WIDTH       AFFECTS_ELABORATION true
set_parameter_property T_AXI_USER_WIDTH       HDL_PARAMETER       true

add_parameter          T_AXI_ID               INTEGER             2
set_parameter_property T_AXI_ID               DEFAULT_VALUE       2
set_parameter_property T_AXI_ID               DISPLAY_NAME        "T Data I/F AXI ID"
set_parameter_property T_AXI_ID               TYPE                INTEGER
set_parameter_property T_AXI_ID               UNITS               None
set_parameter_property T_AXI_ID               DESCRIPTION         "T Data I/F AXI ID"
set_parameter_property T_AXI_ID               AFFECTS_ELABORATION true
set_parameter_property T_AXI_ID               HDL_PARAMETER       true

add_parameter          T_AXI_PROT             INTEGER             0
set_parameter_property T_AXI_PROT             DEFAULT_VALUE       0
set_parameter_property T_AXI_PROT             DISPLAY_NAME        "T Data I/F AXI PROT"
set_parameter_property T_AXI_PROT             TYPE                INTEGER
set_parameter_property T_AXI_PROT             UNITS               None
set_parameter_property T_AXI_PROT             DESCRIPTION         "T Data I/F AXI PROT"
set_parameter_property T_AXI_PROT             AFFECTS_ELABORATION true
set_parameter_property T_AXI_PROT             HDL_PARAMETER       true

add_parameter          T_AXI_CACHE            INTEGER             15
set_parameter_property T_AXI_CACHE            DEFAULT_VALUE       15
set_parameter_property T_AXI_CACHE            DISPLAY_NAME        "T Data I/F AXI CACHE"
set_parameter_property T_AXI_CACHE            TYPE                INTEGER
set_parameter_property T_AXI_CACHE            UNITS               None
set_parameter_property T_AXI_CACHE            DESCRIPTION         "T Data I/F AXI CACHE"
set_parameter_property T_AXI_CACHE            AFFECTS_ELABORATION true
set_parameter_property T_AXI_CACHE            HDL_PARAMETER       true

add_parameter          T_AXI_AUSER            INTEGER             1
set_parameter_property T_AXI_AUSER            DEFAULT_VALUE       1
set_parameter_property T_AXI_AUSER            DISPLAY_NAME        "T Data I/F AXI ARUSER"
set_parameter_property T_AXI_AUSER            TYPE                INTEGER
set_parameter_property T_AXI_AUSER            UNITS               None
set_parameter_property T_AXI_AUSER            DESCRIPTION         "T Data I/F AXI ARUSER"
set_parameter_property T_AXI_AUSER            AFFECTS_ELABORATION true
set_parameter_property T_AXI_AUSER            HDL_PARAMETER       true

# | 
# +-----------------------------------

# +-----------------------------------
# | display items
# | 
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point ACLK
# | 
add_interface          ACLK    clock   end
set_interface_property ACLK    ENABLED true
add_interface_port     ACLK    ACLK    clk Input 1

# | 
# +-----------------------------------

# +-----------------------------------
# | connection point RESET
# | 
add_interface          ARESETn reset               sink
set_interface_property ARESETn associatedClock     ACLK
set_interface_property ARESETn synchronousEdges    DEASSERT
set_interface_property ARESETn ENABLED             true
set_interface_property ARESETn EXPORT_OF           ""
set_interface_property ARESETn PORT_NAME_MAP       ""
set_interface_property ARESETn CMSIS_SVD_VARIABLES ""
set_interface_property ARESETn SVD_ADDRESS_GROUP   ""
add_interface_port     ARESETn ARESETn reset_n Input 1
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point IRQ
# | 
add_interface          IRQ     interrupt end
set_interface_property IRQ     associatedAddressablePoint CSR
set_interface_property IRQ     ASSOCIATED_CLOCK ACLK
add_interface_port     IRQ     IRQ irq Output 1

# +-----------------------------------
# | Elaboration callback
# +-----------------------------------
proc elaborate {} {
    # +-----------------------------------
    # | connection point CSR
    # +-----------------------------------
    set s_axi_addr_width   [ get_parameter_value S_AXI_ADDR_WIDTH  ]
    set s_axi_data_width   32

    add_interface          S     axi4lite         end
    set_interface_property S     associatedClock  ACLK
    set_interface_property S     associatedReset  ARESETn
    set_interface_property S     readAcceptanceCapability     1
    set_interface_property S     writeAcceptanceCapability    1
    set_interface_property S     combinedAcceptanceCapability 1
    set_interface_property S     readDataReorderingDepth      1
    set_interface_property S     ENABLED        true
    add_interface_port     S     S_AXI_AWADDR   awaddr   Input  $s_axi_addr_width
    add_interface_port     S     S_AXI_AWPROT   awprot   Input  3
    add_interface_port     S     S_AXI_AWVALID  awvalid  Input  1
    add_interface_port     S     S_AXI_AWREADY  awready  Output 1
    add_interface_port     S     S_AXI_WDATA    wdata    Input  $s_axi_data_width
    add_interface_port     S     S_AXI_WSTRB    wstrb    Input  $s_axi_data_width/8
    add_interface_port     S     S_AXI_WVALID   wvalid   Input  1
    add_interface_port     S     S_AXI_WREADY   wready   Output 1
    add_interface_port     S     S_AXI_BRESP    bresp    Output 2
    add_interface_port     S     S_AXI_BVALID   bvalid   Output 1
    add_interface_port     S     S_AXI_BREADY   bready   Input  1
    add_interface_port     S     S_AXI_ARADDR   araddr   Input  $s_axi_addr_width
    add_interface_port     S     S_AXI_ARPROT   arprot   Input  3
    add_interface_port     S     S_AXI_ARVALID  arvalid  Input  1
    add_interface_port     S     S_AXI_ARREADY  arready  Output 1
    add_interface_port     S     S_AXI_RDATA    rdata    Output $s_axi_data_width
    add_interface_port     S     S_AXI_RRESP    rresp    Output 2
    add_interface_port     S     S_AXI_RVALID   rvalid   Output 1
    add_interface_port     S     S_AXI_RREADY   rready   Input  1

    # +-----------------------------------
    # | connection point IO
    # +-----------------------------------
    set io_axi_id_width        [ get_parameter_value IO_AXI_ID_WIDTH   ]
    set io_axi_data_width      [ get_parameter_value IO_AXI_DATA_WIDTH ]
    set io_axi_addr_width      [ get_parameter_value IO_AXI_ADDR_WIDTH ]
    set io_use_addr_user       [ get_parameter_value IO_USE_ADDR_USER  ]
    set io_axi_addr_user_width [ get_parameter_value IO_AXI_USER_WIDTH ]
    set io_axi_data_user_width 4
    set io_axi_burst_length    4
    set io_axi_lock_width      2

    add_interface          IO    axi              start
    set_interface_property IO    associatedClock  ACLK
    set_interface_property IO    associatedReset  ARESETn
    set_interface_property IO    ENABLED          true
    add_interface_port     IO    IO_AXI_AWID      awid     Output $io_axi_id_width
    add_interface_port     IO    IO_AXI_AWADDR    awaddr   Output $io_axi_addr_width
    add_interface_port     IO    IO_AXI_AWLEN     awlen    Output $io_axi_burst_length
    add_interface_port     IO    IO_AXI_AWSIZE    awsize   Output 3
    add_interface_port     IO    IO_AXI_AWBURST   awburst  Output 2
    add_interface_port     IO    IO_AXI_AWLOCK    awlock   Output $io_axi_lock_width
    add_interface_port     IO    IO_AXI_AWCACHE   awcache  Output 4
    add_interface_port     IO    IO_AXI_AWPROT    awprot   Output 3
    add_interface_port     IO    IO_AXI_AWUSER    awuser   Output $io_axi_addr_user_width
    add_interface_port     IO    IO_AXI_AWVALID   awvalid  Output 1
    add_interface_port     IO    IO_AXI_AWREADY   awready  Input  1
    add_interface_port     IO    IO_AXI_WID       wid      Output $io_axi_id_width
    add_interface_port     IO    IO_AXI_WDATA     wdata    Output $io_axi_data_width
    add_interface_port     IO    IO_AXI_WSTRB     wstrb    Output $io_axi_data_width/8
    add_interface_port     IO    IO_AXI_WLAST     wlast    Output 1
    add_interface_port     IO    IO_AXI_WVALID    wvalid   Output 1
    add_interface_port     IO    IO_AXI_WREADY    wready   Input  1
    add_interface_port     IO    IO_AXI_BID       bid      Input  $io_axi_id_width
    add_interface_port     IO    IO_AXI_BRESP     bresp    Input  2
    add_interface_port     IO    IO_AXI_BVALID    bvalid   Input  1
    add_interface_port     IO    IO_AXI_BREADY    bready   Output 1
    add_interface_port     IO    IO_AXI_ARID      arid     Output $io_axi_id_width
    add_interface_port     IO    IO_AXI_ARADDR    araddr   Output $io_axi_addr_width
    add_interface_port     IO    IO_AXI_ARLEN     arlen    Output $io_axi_burst_length
    add_interface_port     IO    IO_AXI_ARSIZE    arsize   Output 3
    add_interface_port     IO    IO_AXI_ARBURST   arburst  Output 2
    add_interface_port     IO    IO_AXI_ARLOCK    arlock   Output $io_axi_lock_width
    add_interface_port     IO    IO_AXI_ARCACHE   arcache  Output 4
    add_interface_port     IO    IO_AXI_ARPROT    arprot   Output 3
    add_interface_port     IO    IO_AXI_ARUSER    aruser   Output $io_axi_addr_user_width
    add_interface_port     IO    IO_AXI_ARVALID   arvalid  Output 1
    add_interface_port     IO    IO_AXI_ARREADY   arready  Input  1
    add_interface_port     IO    IO_AXI_RID       rid      Input  $io_axi_id_width
    add_interface_port     IO    IO_AXI_RDATA     rdata    Input  $io_axi_data_width
    add_interface_port     IO    IO_AXI_RRESP     rresp    Input  2
    add_interface_port     IO    IO_AXI_RLAST     rlast    Input  1
    add_interface_port     IO    IO_AXI_RVALID    rvalid   Input  1
    add_interface_port     IO    IO_AXI_RREADY    rready   Output 1

    if { $io_use_addr_user == 0 } {
        set_port_property IO_AXI_AWUSER TERMINATION true
        set_port_property IO_AXI_ARUSER TERMINATION true
    }

    # +-----------------------------------
    # | connection point K
    # +-----------------------------------
    set k_axi_id_width        [ get_parameter_value K_AXI_ID_WIDTH   ]
    set k_axi_data_width      [ get_parameter_value K_AXI_DATA_WIDTH ]
    set k_axi_addr_width      [ get_parameter_value K_AXI_ADDR_WIDTH ]
    set k_use_addr_user       [ get_parameter_value K_USE_ADDR_USER  ]
    set k_axi_addr_user_width [ get_parameter_value K_AXI_USER_WIDTH ]
    set k_axi_data_user_width 4
    set k_axi_burst_length    4
    set k_axi_lock_width      2

    add_interface          K     axi              start
    set_interface_property K     associatedClock  ACLK
    set_interface_property K     associatedReset  ARESETn
    set_interface_property K     ENABLED          true
    add_interface_port     K     K_AXI_AWID       awid     Output $k_axi_id_width
    add_interface_port     K     K_AXI_AWADDR     awaddr   Output $k_axi_addr_width
    add_interface_port     K     K_AXI_AWLEN      awlen    Output $k_axi_burst_length
    add_interface_port     K     K_AXI_AWSIZE     awsize   Output 3
    add_interface_port     K     K_AXI_AWBURST    awburst  Output 2
    add_interface_port     K     K_AXI_AWLOCK     awlock   Output $k_axi_lock_width
    add_interface_port     K     K_AXI_AWCACHE    awcache  Output 4
    add_interface_port     K     K_AXI_AWPROT     awprot   Output 3
    add_interface_port     K     K_AXI_AWUSER     awuser   Output $k_axi_addr_user_width
    add_interface_port     K     K_AXI_AWVALID    awvalid  Output 1
    add_interface_port     K     K_AXI_AWREADY    awready  Input  1
    add_interface_port     K     K_AXI_WID        wid      Output $k_axi_id_width
    add_interface_port     K     K_AXI_WDATA      wdata    Output $k_axi_data_width
    add_interface_port     K     K_AXI_WSTRB      wstrb    Output $k_axi_data_width/8
    add_interface_port     K     K_AXI_WLAST      wlast    Output 1
    add_interface_port     K     K_AXI_WVALID     wvalid   Output 1
    add_interface_port     K     K_AXI_WREADY     wready   Input  1
    add_interface_port     K     K_AXI_BID        bid      Input  $k_axi_id_width
    add_interface_port     K     K_AXI_BRESP      bresp    Input  2
    add_interface_port     K     K_AXI_BVALID     bvalid   Input  1
    add_interface_port     K     K_AXI_BREADY     bready   Output 1
    add_interface_port     K     K_AXI_ARID       arid     Output $k_axi_id_width
    add_interface_port     K     K_AXI_ARADDR     araddr   Output $k_axi_addr_width
    add_interface_port     K     K_AXI_ARLEN      arlen    Output $k_axi_burst_length
    add_interface_port     K     K_AXI_ARSIZE     arsize   Output 3
    add_interface_port     K     K_AXI_ARBURST    arburst  Output 2
    add_interface_port     K     K_AXI_ARLOCK     arlock   Output $k_axi_lock_width
    add_interface_port     K     K_AXI_ARCACHE    arcache  Output 4
    add_interface_port     K     K_AXI_ARPROT     arprot   Output 3
    add_interface_port     K     K_AXI_ARUSER     aruser   Output $k_axi_addr_user_width
    add_interface_port     K     K_AXI_ARVALID    arvalid  Output 1
    add_interface_port     K     K_AXI_ARREADY    arready  Input  1
    add_interface_port     K     K_AXI_RID        rid      Input  $k_axi_id_width
    add_interface_port     K     K_AXI_RDATA      rdata    Input  $k_axi_data_width
    add_interface_port     K     K_AXI_RRESP      rresp    Input  2
    add_interface_port     K     K_AXI_RLAST      rlast    Input  1
    add_interface_port     K     K_AXI_RVALID     rvalid   Input  1
    add_interface_port     K     K_AXI_RREADY     rready   Output 1

    if { $k_use_addr_user == 0 } {
        set_port_property K_AXI_AWUSER TERMINATION true
        set_port_property K_AXI_ARUSER TERMINATION true
    }

    # +-----------------------------------
    # | connection point T
    # +-----------------------------------
    set t_axi_id_width        [ get_parameter_value T_AXI_ID_WIDTH   ]
    set t_axi_data_width      [ get_parameter_value T_AXI_DATA_WIDTH ]
    set t_axi_addr_width      [ get_parameter_value T_AXI_ADDR_WIDTH ]
    set t_use_addr_user       [ get_parameter_value T_USE_ADDR_USER  ]
    set t_axi_addr_user_width [ get_parameter_value T_AXI_USER_WIDTH ]
    set t_axi_data_user_width 4
    set t_axi_burst_length    4
    set t_axi_lock_width      2

    add_interface          T     axi              start
    set_interface_property T     associatedClock  ACLK
    set_interface_property T     associatedReset  ARESETn
    set_interface_property T     ENABLED          true
    add_interface_port     T     T_AXI_AWID       awid     Output $t_axi_id_width
    add_interface_port     T     T_AXI_AWADDR     awaddr   Output $t_axi_addr_width
    add_interface_port     T     T_AXI_AWLEN      awlen    Output $t_axi_burst_length
    add_interface_port     T     T_AXI_AWSIZE     awsize   Output 3
    add_interface_port     T     T_AXI_AWBURST    awburst  Output 2
    add_interface_port     T     T_AXI_AWLOCK     awlock   Output $t_axi_lock_width
    add_interface_port     T     T_AXI_AWCACHE    awcache  Output 4
    add_interface_port     T     T_AXI_AWPROT     awprot   Output 3
    add_interface_port     T     T_AXI_AWUSER     awuser   Output $t_axi_addr_user_width
    add_interface_port     T     T_AXI_AWVALID    awvalid  Output 1
    add_interface_port     T     T_AXI_AWREADY    awready  Input  1
    add_interface_port     T     T_AXI_WID        wid      Output $t_axi_id_width
    add_interface_port     T     T_AXI_WDATA      wdata    Output $t_axi_data_width
    add_interface_port     T     T_AXI_WSTRB      wstrb    Output $t_axi_data_width/8
    add_interface_port     T     T_AXI_WLAST      wlast    Output 1
    add_interface_port     T     T_AXI_WVALID     wvalid   Output 1
    add_interface_port     T     T_AXI_WREADY     wready   Input  1
    add_interface_port     T     T_AXI_BID        bid      Input  $t_axi_id_width
    add_interface_port     T     T_AXI_BRESP      bresp    Input  2
    add_interface_port     T     T_AXI_BVALID     bvalid   Input  1
    add_interface_port     T     T_AXI_BREADY     bready   Output 1
    add_interface_port     T     T_AXI_ARID       arid     Output $t_axi_id_width
    add_interface_port     T     T_AXI_ARADDR     araddr   Output $t_axi_addr_width
    add_interface_port     T     T_AXI_ARLEN      arlen    Output $t_axi_burst_length
    add_interface_port     T     T_AXI_ARSIZE     arsize   Output 3
    add_interface_port     T     T_AXI_ARBURST    arburst  Output 2
    add_interface_port     T     T_AXI_ARLOCK     arlock   Output $t_axi_lock_width
    add_interface_port     T     T_AXI_ARCACHE    arcache  Output 4
    add_interface_port     T     T_AXI_ARPROT     arprot   Output 3
    add_interface_port     T     T_AXI_ARUSER     aruser   Output $t_axi_addr_user_width
    add_interface_port     T     T_AXI_ARVALID    arvalid  Output 1
    add_interface_port     T     T_AXI_ARREADY    arready  Input  1
    add_interface_port     T     T_AXI_RID        rid      Input  $t_axi_id_width
    add_interface_port     T     T_AXI_RDATA      rdata    Input  $t_axi_data_width
    add_interface_port     T     T_AXI_RRESP      rresp    Input  2
    add_interface_port     T     T_AXI_RLAST      rlast    Input  1
    add_interface_port     T     T_AXI_RVALID     rvalid   Input  1
    add_interface_port     T     T_AXI_RREADY     rready   Output 1

    if { $t_use_addr_user == 0 } {
        set_port_property T_AXI_AWUSER TERMINATION true
        set_port_property T_AXI_ARUSER TERMINATION true
    }
}
