Quantized Convolution (strip) for DE10-Nano
===========================================

Quantized Convolution (strip) binary and project and test code for DE10-Nano.

Quantized Convolution is a convolution method published by LeapMind Inc(https://leapmind.io) on Blueoil(https://github.com/blue-oil/blueoil).

### Requirement

* Board: DE10-Nano
* OS: https://github.com/ikwzm/FPGA-SoC-Linux

## Install

### Install FPGA-SoC-Linux

See https://github.com/ikwzm/FPGA-SoC-Linux

### Expand the CMA area

Add ```cma=256M``` to linux_boot_args in uEnv.txt.

```
linux_kernel_image=vmlinuz-4.14.34-armv7-fpga
linux_fdt_image=devicetree-4.14.34-socfpga.dtb
linux_boot_args=console=ttyS0,115200 root=/dev/mmcblk0p2 rw rootwait uio_pdrv_genirq.of_id=generic-uio cma=256M

linux_load_cmd=fatload mmc 0 ${loadaddr} ${linux_kernel_image} && fatload mmc 0 ${fdt_addr} ${linux_fdt_image}
linux_boot_cmd=setenv bootargs ${linux_boot_args} && bootz ${loadaddr} - ${fdt_addr}

uenvcmd=run linux_load_cmd && run linux_boot_cmd

bootmenu_0=Boot linux-4.14.34-armv7-fpga=boot
```

### Boot FPGA-SoC-Linux

### Login fpga user

### Install g++

```console
fpga% sudo apt-get install g++
```

### Download QCONV-STRIP-DE10-Nano to DE10-Nano

```console
fpga@debian-fpga:~/$ git clone https://github.com/ikwzm/QCONV-STRIP-DE10-Nano.git -b v0.2
fpga@debian-fpga:~/$ cd QCONV-STRIP-DE10-Nano
```

### Install FPGA Bitstream file

```console
fpga@debian-fpga:~/QCONV-STRIP-DE10-Nano$ sudo rake install
dtbocfg.rb --install qconv_strip --dts qconv_strip.dts
/config/device-tree/overlays/qconv_strip/dtbo: Warning (unit_address_vs_reg): 
[  465.444079] fpga_manager fpga0: writing qconv_strip_axi3.rbf to Altera SOCFPGA FPGA Manager
/uio-irq-test@0/__overlay__/uio_qconv_strip: node has a reg or ranges property, but no unit name
/config/device-tree/overlays/qconv_strip/dtbo: Warning (avoid_unnecessary_addr_size): /uio-irq-test@0: unnecessary #address-cells/#size-cells without "ranges" or child "reg" property
[  465.704702] udmabuf soc:fpga-region0:udmabuf_qconv_in: driver probe start.
[  465.724950] udmabuf udmabuf-qconv-in: driver installed
[  465.730114] udmabuf udmabuf-qconv-in: major number   = 245
[  465.735579] udmabuf udmabuf-qconv-in: minor number   = 0
[  465.740926] udmabuf udmabuf-qconv-in: phys address   = 0x30100000
[  465.746999] udmabuf udmabuf-qconv-in: buffer size    = 4194304
[  465.752951] udmabuf udmabuf-qconv-in: dma coherent   = 0
[  465.758325] udmabuf soc:fpga-region0:udmabuf_qconv_in: driver installed.
[  465.765884] udmabuf soc:fpga-region0:udmabuf_qconv_out: driver probe start.
[  465.796376] udmabuf udmabuf-qconv-out: driver installed
[  465.801625] udmabuf udmabuf-qconv-out: major number   = 245
[  465.807178] udmabuf udmabuf-qconv-out: minor number   = 1
[  465.812615] udmabuf udmabuf-qconv-out: phys address   = 0x30500000
[  465.818817] udmabuf udmabuf-qconv-out: buffer size    = 8388608
[  465.824716] udmabuf udmabuf-qconv-out: dma coherent   = 0
[  465.830134] udmabuf soc:fpga-region0:udmabuf_qconv_out: driver installed.
[  465.839095] udmabuf soc:fpga-region0:udmabuf_qconv_k: driver probe start.
[  465.858994] udmabuf udmabuf-qconv-k: driver installed
[  465.864035] udmabuf udmabuf-qconv-k: major number   = 245
[  465.869467] udmabuf udmabuf-qconv-k: minor number   = 2
[  465.874677] udmabuf udmabuf-qconv-k: phys address   = 0x30d00000
[  465.880708] udmabuf udmabuf-qconv-k: buffer size    = 4194304
[  465.886437] udmabuf udmabuf-qconv-k: dma coherent   = 0
[  465.891690] udmabuf soc:fpga-region0:udmabuf_qconv_k: driver installed.
[  465.899061] udmabuf soc:fpga-region0:udmabuf_qconv_th: driver probe start.
[  465.906909] udmabuf udmabuf-qconv-th: driver installed
[  465.912138] udmabuf udmabuf-qconv-th: major number   = 245
[  465.917610] udmabuf udmabuf-qconv-th: minor number   = 3
[  465.922949] udmabuf udmabuf-qconv-th: phys address   = 0x30050000
[  465.929066] udmabuf udmabuf-qconv-th: buffer size    = 65536
[  465.934707] udmabuf udmabuf-qconv-th: dma coherent   = 0
[  465.940035] udmabuf soc:fpga-region0:udmabuf_qconv_th: driver installed.
```

### Run Unit Test

```console
fpga@debian-fpga:~/QCONV-STRIP-DE10-Nano$ rake unit_test2_all
g++ -I ./include -Wpointer-arith -o unit_test src/tb/unit_test.cpp src/cpp/conv1x1.cpp src/cpp/conv3x3.cpp
./unit_test -iw 160 -ih 160 -ic 64 -oc 32 -kw 1 -kh 1 -th 1 random
FPGA exec time (160x160x64x32 1x1): 2242 [usec]
[qconv_strip] test success!!!
./unit_test -iw 160 -ih 160 -ic 32 -oc 8 -kw 3 -kh 3 -th 1 random
FPGA exec time (160x160x32x8 3x3): 582 [usec]
[qconv_strip] test success!!!
./unit_test -iw 80 -ih 80 -ic 32 -oc 16 -kw 3 -kh 3 -th 1 random
FPGA exec time (80x80x32x16 3x3): 299 [usec]
[qconv_strip] test success!!!
./unit_test -iw 40 -ih 40 -ic 64 -oc 32 -kw 3 -kh 3 -th 1 random
FPGA exec time (40x40x64x32 3x3): 173 [usec]
[qconv_strip] test success!!!
./unit_test -iw 20 -ih 20 -ic 128 -oc 64 -kw 3 -kh 3 -th 1 random
FPGA exec time (20x20x128x64 3x3): 170 [usec]
[qconv_strip] test success!!!
./unit_test -iw 10 -ih 10 -ic 256 -oc 128 -kw 3 -kh 3 -th 1 random
FPGA exec time (10x10x256x128 3x3): 239 [usec]
[qconv_strip] test success!!!
./unit_test -iw 5 -ih 5 -ic 512 -oc 128 -kw 3 -kh 3 -th 1 random
FPGA exec time (5x5x512x128 3x3): 267 [usec]
[qconv_strip] test success!!!
./unit_test -iw 5 -ih 5 -ic 128 -oc 256 -kw 3 -kh 3 -th 1 random
FPGA exec time (5x5x128x256 3x3): 144 [usec]
[qconv_strip] test success!!!
./unit_test -iw 10 -ih 10 -ic 128 -oc 32 -kw 1 -kh 1 -th 1 random
FPGA exec time (10x10x128x32 1x1): 29 [usec]
[qconv_strip] test success!!!
./unit_test -iw 20 -ih 20 -ic 64 -oc 16 -kw 1 -kh 1 -th 1 random
FPGA exec time (20x20x64x16 1x1): 37 [usec]
[qconv_strip] test success!!!
./unit_test -iw 40 -ih 40 -ic 32 -oc 4 -kw 1 -kh 1 -th 1 random
FPGA exec time (40x40x32x4 1x1): 42 [usec]
[qconv_strip] test success!!!
./unit_test -iw 5 -ih 5 -ic 1024 -oc 256 -kw 3 -kh 3 -th 1 random
FPGA exec time (5x5x1024x256 3x3): 1022 [usec]
[qconv_strip] test success!!!
./unit_test -iw 5 -ih 5 -ic 256 -oc 128 -kw 3 -kh 3 -th 1 random
FPGA exec time (5x5x256x128 3x3): 142 [usec]
[qconv_strip] test success!!!
./unit_test -iw 5 -ih 5 -ic 128 -oc 256 -kw 3 -kh 3 -th 1 random
FPGA exec time (5x5x128x256 3x3): 143 [usec]
[qconv_strip] test success!!!
./unit_test -iw 5 -ih 5 -ic 256 -oc 128 -kw 3 -kh 3 -th 1 random
FPGA exec time (5x5x256x128 3x3): 143 [usec]
[qconv_strip] test success!!!
./unit_test -iw 5 -ih 5 -ic 128 -oc 256 -kw 3 -kh 3 -th 1 random
FPGA exec time (5x5x128x256 3x3): 144 [usec]
[qconv_strip] test success!!!
```


## Build Bitstream file

### Requirement

* Intel Quartus Prime 18.1.0 Lite Edition 

### Download QCONV-STRIP-DE10-Nano

```console
shell$ git clone https://github.com/ikwzm/QCONV-STRIP-DE10-Nano.git
shell$ cd QCONV-STRIP-DE10-Nano
shell$ git submodule update --init --recursive
```

### Build DE10_NANO.rbf 

#### Open DE10_NANO Project

* File > Open Project > DE10_NANO.qpf

#### Compile

* Compile Design Start

#### Convert rbf

* File > Convert Programing Files
* Open Conversion Setup Data... > DE10_NANO.cof
* Generate

### Copy DE10_NANO.rbf to qconv_strip_axi3.rbf

```console
shell$ cd project
shell$ cp DE10_NANO.rbf ../qconv_strip_axi3.rbf
```

## Licensing

Distributed under the BSD 2-Clause License.

