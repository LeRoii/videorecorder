#!/bin/sh
#export DISPLAY=:0
#gst-launch-1.0 v4l2src device=/dev/video0 ! 'video/x-raw, format=(string)UYVY, width=(int)1920, height=(int)1080, framerate=(fraction)30/1' ! xvimagesink  -ev
taskset -c 4 ./videorecorder/build/nvvideoc1024 1024 768
