Quantized Convolution (strip) for DE10-Nano
===========================================

Quantized Convolution (strip) binary and project and test code for DE10-Nano.

Quantized Convolution is a convolution method published by LeapMind Inc(https://leapmind.io) on Blueoil(https://github.com/blue-oil/blueoil).

### Requirement

* Board: DE10-Nano
* OS: [FPGA-SoC-Linux](https://github.com/ikwzm/FPGA-SoC-Linux.git)

## Install

  (T.B.D)

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

Run SoC EDS Command Shell

```console
shell$ cd project
shell$ make rbf
```

### Copy DE10_NANO.rbf to qconv_strip_axi3.rbf

```console
shell$ cd project
shell$ cp DE10_NANO.rbf ../qconv_strip_axi3.rbf
```

## Licensing

Distributed under the BSD 2-Clause License.

