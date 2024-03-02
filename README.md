# librtlsdr
Turns your Realtek RTL2832 based DVB dongle into a SDR receiver.

## Building with CMake

```console
cd librtlsdr/
mkdir build
cd build
cmake ../
make
sudo make install
sudo ldconfig
```

## Building with autotools

```console
cd librtlsdr/
autoreconf -i
./configure
make
sudo make install
sudo ldconfig
```

For more information see: [https://osmocom.org/projects/rtl-sdr/wiki]()
