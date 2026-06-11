#!/bin/bash

PC_IP="192.168.2.149"
PORT="5600"

gst-launch-1.0 -v \
    v4l2src device=/dev/video0 ! \
    image/jpen,width=640,height=480,framerate=30/i ! \
    jpegdec ! videoconvert ! \
    x264nec tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 ! \
    rtph264pay condif-interval=1 pt=96 ! \
    udpsink host=$PC_IP port=$PORT sync=false