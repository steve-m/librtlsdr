#!/bin/bash

# requires debian/ubuntu packages: zip gcc-mingw-w64

REPO_DIR=$(pwd)

if [ -z "$1" ]; then
  echo "usage: $0 <zip-post> <any other cmake options>"
  exit 1
fi

ZIP_POST="$1"
shift

CROSS="i686-w64-mingw32"
WN="w32"
TOOLCHAIN="mingw-w32-i686.cmake"

# libusb
if /bin/true; then
  cd ${REPO_DIR} && rm -rf libusb_${WN}
  cd ${REPO_DIR} && git clone --branch v1.0.23 https://github.com/libusb/libusb.git libusb_${WN}
  echo -e "\n\n********************************************************"
  echo "start build of libusb_${WN}"
  cd ${REPO_DIR}/libusb_${WN} && ./bootstrap.sh && \
    CC=${CROSS}-gcc \
    AR=${CROSS}-ar \
    RANLIB=${CROSS}-ranlib \
    ./configure --prefix=${REPO_DIR}/mingw_libusb_${WN} --host=${CROSS} --disable-shared && \
    make && make install
    echo -e "\n\nlisting of ${REPO_DIR}/mingw_libusb_${WN}"
    ls -alh ${REPO_DIR}/mingw_libusb_${WN}
    echo -e "\nlisting of ${REPO_DIR}/mingw_libusb_${WN}/include"
    ls -alh ${REPO_DIR}/mingw_libusb_${WN}/include
    echo -e "\nlisting of ${REPO_DIR}/mingw_libusb_${WN}/lib"
    ls -alh ${REPO_DIR}/mingw_libusb_${WN}/lib
    echo -e "\n"
fi

# librtlsdr
if /bin/true; then
  cd ${REPO_DIR} && rm -rf build_${WN}
  echo -e "\n\n********************************************************"
  echo "start build of librtlsdr_${WN}"
  mkdir ${REPO_DIR}/build_${WN} && cd ${REPO_DIR}/build_${WN} && \
    cmake -DCMAKE_TOOLCHAIN_FILE=${REPO_DIR}/${TOOLCHAIN} \
      -DCMAKE_INSTALL_PREFIX=${REPO_DIR}/rtlsdr-bin-${WN}_${ZIP_POST} \
      -DRTL_STATIC_BUILD=ON "$@"  \
      -DLIBUSB_INCLUDE_DIR=${REPO_DIR}/mingw_libusb_${WN}/include/libusb-1.0 \
      -DLIBUSB_LIBRARIES=${REPO_DIR}/mingw_libusb_${WN}/lib/libusb-1.0.a \
      ../  && \
    make && make install
  md5sum  ${REPO_DIR}/rtlsdr-bin-${WN}_${ZIP_POST}/bin/* >${REPO_DIR}/rtlsdr-bin-${WN}_${ZIP_POST}/bin/md5sums.txt
  sha1sum ${REPO_DIR}/rtlsdr-bin-${WN}_${ZIP_POST}/bin/* >${REPO_DIR}/rtlsdr-bin-${WN}_${ZIP_POST}/bin/sha1sums.txt
fi

