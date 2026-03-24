# WRLD-capstone
Team WRLD's interdisciplinary capstone design project

# Overall Software Details

The Mora system is split into two parts: a **buoy** and a **base station**. Buoys are the modular part of the system. Theoretically, as long as bandwith allows, and software scales, there can be infinite buoys. The hardware will have a camera and an IMU and will transmit this data through a daisy-chained network using bridged network interfaces.
The base station will receive all of the data from the buoys and will stitch together the video feeds to give the final image as well as record the data.

## Buoy Software
The buoy software is simple to reduce load on the limited hardware of the Raspberry Pi and to reduce the software complexity of each node.

Buoy Functions:
- Send low latency UDP video stream (timestamped)
- Send IMU data over UDP using protobuf
- Indicate status with LEDs?
- Detect leaks?

The buoy software is deployed using **scripts**, currently the command for this is `./deploy build`. The script will handle searching for nodes, verifying dependencies and network configuration, copying the files, and building. It will also eventually handle each of the running nodes, monitor for errors, and copy logs if needed

## Base Station Software
The base station software will be much more complex. At the cost of reducing software complexity on each of the buoys (in the interest of reducing distributed development headaches and not dealing with the limited compute) The base station must pick up a lot of the slack.

Base station functions:
- Time Sync Server (NTP or PTP)
- Recieve and synchronize IMU data with recieved video frames
- Perform lens distortion correction
- Perform video stabilization from IMU data
- Stitch together video feeds
- Record stitched feed
- Detect swimmer using OpenCV
- Zoom into stitched feed to highlight swimmer
- Overlay pose estimation data and other statistics. (stretch)
- Display augmented video on pool deck
- Record augmented video

Most of the heavy lifting of the base station software will be handled by GStreamer, an open platform that can handle the transmission of network video and live video compositing tasks using openGL plugins.

To build,
```
# setup cmake directories
cmake cmake -S src -B build
# To build
cd build
make
./base_station
```
Some debugging options have been added to the base station software, preceding the command with a flag will default all screens to use data from IMU source 1 for example:
`IMU_ALL=0 ./base_station `

## Scripts
Scripts will make or break the usability of Mora. In both development and deplyment, having to individually manage many devices is going to be a pain. We must be able to use scripts to deploy images to the Pis for development as well as to startup the system.

## Configs
There are two configuration files that are used for changing the behavior of the software, and this behavior can be altered without recompiling the program.

## Buoy IPs

| Hostname | User | IP Address | Notes | 
| --- | --- | --- | --- |
| raspberrypi | walker | 192.168.1.10 | SD Card corrupted? |
| mola1 | walker | 192.168.1.20 | |


# List of Software Dependencies
- cmake
- protobuf `sudo apt install protobuf-compiler libprotobuf-dev
`
- Gstreamer 1.0
``` 
sudo apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    gstreamer1.0-libcamera
```
- libcamera `sudo apt install libcamera-dev libcamera-tools libcamera-ipa`
- libyaml-cpp-dev