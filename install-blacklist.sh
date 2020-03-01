#!/bin/bash

BLACKLIST_FN=""
if [ -f /etc/modprobe.d/rtlsdr-blacklist.conf ]; then
  BLACKLIST_FN="rtlsdr-blacklist.conf"
  echo "found /etc/modprobe.d/${BLACKLIST_FN}"
elif [ -f /etc/modprobe.d/blacklist-rtl8xxxu.conf ]; then
  BLACKLIST_FN="blacklist-rtl8xxxu.conf"
  echo "found /etc/modprobe.d/${BLACKLIST_FN}"
elif [ -f /etc/modprobe.d/raspi-blacklist.conf ]; then
  BLACKLIST_FN="raspi-blacklist.conf"
  echo "found /etc/modprobe.d/${BLACKLIST_FN}"
else
  BLACKLIST_FN="rtlsdr-blacklist.conf"
  echo "could not find existing blacklist. will use /etc/modprobe.d/${BLACKLIST_FN}"
fi

if [ -f /etc/modprobe.d/${BLACKLIST_FN} ]; then
  cat /etc/modprobe.d/${BLACKLIST_FN} rtlsdr-blacklist.conf | sort | uniq >/dev/shm/${BLACKLIST_FN}
  cp /dev/shm/${BLACKLIST_FN} /etc/modprobe.d/${BLACKLIST_FN}
  echo "updated /etc/modprobe.d/${BLACKLIST_FN} ; reboot to apply"
else
  cp rtlsdr-blacklist.conf /etc/modprobe.d/${BLACKLIST_FN}
  echo "created /etc/modprobe.d/${BLACKLIST_FN} ; reboot to apply"
fi

