#!/usr/bin/env bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root or sudo"
  exit
fi

chmod +x ~/catkin_ws/src/waveshare_alphabot2/nodes/*
chmod +x ~/catkin_ws/src/waveshare_alphabot2/scripts/*.*