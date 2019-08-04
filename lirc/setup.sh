#!/usr/bin/env bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root or sudo"
  exit
fi

cp lircd.conf /etc/lirc/lircd.conf

/etc/init.d/lirc restart
