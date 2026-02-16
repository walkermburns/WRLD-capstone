# WRLD-capstone
Team WRLD's interdisciplinary capstone design project

# Raspberry Pi Setup

## Network Manager
Consistent UDP video streaming requries that the IP structure be set up correctly. Raspberry Pis use network manager to set up the network. I just completely nuked the default ethernet configuration and set up my own IPV4 static ip to make sure that there were no other issues.

View current interface configurations with

`nmcli con show`

Clear existing hardware configuration with

`nmcli con delete <connection_name>`

Set up a new connection with static IP:
```
nmcli con add type ethernet \
  ifname eth0 \
  con-name eth0-static \
  ipv4.method manual \
  ipv4.addresses 192.168.1.10/24 \
  ipv6.method disabled \
  connection.autoconnect yes
```

Bring up the connection

`nmcli con up eth0-static`

# Creating a daisy-chain network with a dedicated subnet

TODO: use network manager to set up a bridge between two network devices and 

# Streaming video over UDP

I used the following command to send UDP packets from the raspberry pi camera. The frames are encoded in h.264 using the raspberry pi hardware encoder.

`rpicam-vid -t 0 --inline --width 1920 --height 1080 --framerate 30 --codec h264 --bitrate 5000000 -o udp://192.168.1.8:4444 -n`

I then use ffmpeg to display the stream using this command:

`ffplay udp://192.168.1.10:4444 -vf "setpts=N/30" -fflags nobuffer -flags low_delay -framedrop`

To do CV on this stream, I will need to build a python or cpp app to pull the ffmpeg stream as frames

# Raspberry Pi image pipeline

With modern raspberry PIs there is no v4l2 driver available to make the camera appear as a webcam device. Instead, programs must use the modern libcamera to pull images from the cameras. This can be done using Gstreamer, LCCV, or the native libcamera API, although that is not recommended by gemini because of the difficulty of implementation.

If we are changing colorspace to work with opencv, we will need to make sure that gstreamer, lccv, etc is utilizing the hardware ISP for that function.

The Raspberry Pi has multiple hardware engines for image processing. The ISP already handles some color correction operations, but can also perform color space transformations that may be required to feed the footage into openCV.

The ISP can be leveraged

Once in openCV, we can use CPU functions to alter the image. We are interested in performing lens distortion correction (LDC) and image stabilization using an IMU.

These operations can be pretty expensive, we will test to see how long they take on the Pi in CPU, but it is also possible to use the GPU to handle some of the corrections. [lens correction](https://forums.raspberrypi.com/viewtopic.php?t=288505) for one has been done using OpenGL ES. The ISP also has instructions to perform these functions but aren't exposed to the API? and are quite slow. Using GLES shaders should be able to perform lens correction and stabilization.

Additionally, [Gstreamer](https://gemini.google.com/share/61ab6a969a65) can be used to perform some of these operations and is supposed to leverage the GPU. Yet to be determined is the cost of using the operations when it comes to memory handoff between the CPU and gpu (especially if memory has to be flushed from cache.)

It may be possible to use Gstreamer for everything from capture to encode.
