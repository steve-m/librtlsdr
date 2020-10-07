[![librtlsdr version](https://img.shields.io/github/tag/librtlsdr/librtlsdr.svg?style=flat&label=librtlsdr)](https://github.com/librtlsdr/librtlsdr/releases)
[![GPLv2 License](http://img.shields.io/badge/license-GPLv2-brightgreen.svg)](https://tldrlegal.com/license/gnu-general-public-license-v2)

# Description

rtl-sdr turns your Realtek RTL2832 based DVB dongle into a SDR receiver


# New enhancements and features in this version

Many different developments have been taken in this release. For an overview, see [improvements](README_improvements.md)

# Build / Install (on debian/ubuntu)

## prerequisites
development tools have to be installed:
```
sudo apt-get install build-essential cmake git
```

install the libusb-1.0 development package::
```
sudo apt-get install libusb-dev libusb-1.0-0-dev
```

## retrieve the sources - right branch

```
git clone https://github.com/librtlsdr/librtlsdr.git
```

in case you want the *development* branch, e.g. for testing or preparing patches:
```
cd librtlsdr
git checkout development
```

by default, you should have the *master* branch, in doubt:
```
cd librtlsdr
git status
git checkout master
```

## build
run cmake and start compilation. cmake will accept some options, e.g.
* `-DINSTALL_UDEV_RULES=ON`, default is `OFF`
* `-DDETACH_KERNEL_DRIVER=ON`, default is `OFF`
* `-DPROVIDE_UDP_SERVER=ON`, default is `OFF`
* `-DWITH_RPC=ON`, default is `OFF`
* `-DLINK_RTLTOOLS_AGAINST_STATIC_LIB=ON`, default is `OFF`
* `-DRTL_STATIC_BUILD=OFF`, default is `ON`: for MINGW on WIN32

all cmake options are optional

```
mkdir build && cd build
cmake ../ -DINSTALL_UDEV_RULES=ON
make
```

## install
setup into prefix, usually will require `sudo`:
```
sudo make install
sudo ldconfig
```

# Development builds / binaries

[GitHub Actions](https://github.com/librtlsdr/librtlsdr/actions) is used for development builds - for Linux (x86), MacOS and Windows x86 32/64.
Cross-builds for Windows from a Linux machine: see [cross_build_mingw32.sh](cross_build_mingw32.sh) or [cross_build_mingw64.sh](cross_build_mingw64.sh)

# For more information see:

http://superkuh.com/rtlsdr.html

https://osmocom.org/projects/rtl-sdr/wiki/Rtl-sdr


# Setup for SDR only use - without DVB compatibility:

- a special USB vendor/product id got reserved at http://pid.codes/ : 0x1209/0x2832
- for such devices the linux kernel's DVB modules are not loaded automatically,
 thus can be used without blacklisting *dvb_usb_rtl28xxu* below /etc/modprobe.d/
- this allows to use a second RTL dongle for use with DVB in parallel
- the IDs can be programmed with '`rtl_eeprom -n`' or '`rtl_eeprom -g realtek_sdr`'
- for permanent blacklisting you might check/call following from the clone git directory
    ```./install-blacklist.sh```


# Contributing

Pull requests are always welcome but please make changes to, and pull request from, the development branch.

## Initial setup:

- fork the librtlsdr repo via GitHub
- clone your fork locally and cd to the cloned repo's folder
- add the upstream development repo:
    * `git remote add upstream git@github.com:librtlsdr/librtlsdr.git`
- track the development branch: 
    * `git branch --track development origin/development`

## Normal workflow:

- checkout the development branch and make your changes
- commit your changes
- sync your local development branch with the upstream development branch:
    * `git fetch upstream`
    * `git merge upstream/development`
- push your commit/s to your forked repo
- do a pull request via GitHub
