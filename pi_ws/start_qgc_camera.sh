#!/bin/bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video0 ! \
  image/jpeg,width=640,height=480,framerate=30/1 ! \
  jpegdec ! videoconvert ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.2.149 port=5600 sync=false