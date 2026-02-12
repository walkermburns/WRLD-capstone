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
