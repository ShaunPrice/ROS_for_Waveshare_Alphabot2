#!/usr/bin/env bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root or sudo"
  exit
fi

cp wsrgb_service.py /usr/bin/wsrgb_service.py
cp wsrgb.service /lib/systemd/system/wsrgb.service

systemctl daemon-reload
systemctl enable wsrgb.service
systemctl start wsrgb.service
systemctl status wsrgb.service