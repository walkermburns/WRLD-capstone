#pragma once

#include <string>
#include <vector>

// Configuration structures reflecting targets.yaml as used by the base station.
//
// Legacy format supported:
//   targets:
//     - name: buoy0
//       video_port: 5103
//       imu_port: 5003
//
// New multi-camera format supported:
//   targets:
//     - name: buoy0
//       imu_port: 5003
//       cameras:
//         - name: front
//           video_port: 5103
//           metadata_port: 5104
//           mount: front
//           layout: {x: 0, y: 0, width: 1920, height: 1080}
//         - name: rear
//           video_port: 5105
//           metadata_port: 5106
//           mount: rear_yaw_180
//           layout: {x: 1920, y: 0, width: 1920, height: 1080}

struct CameraConfig {
    std::string name;
    int videoPort = 0;
    int metadataPort = 0; // if omitted, defaults to videoPort + 1 in Config.cpp
    std::string mount = "front";
    int x = 0;
    int y = 0;
    int width = 1920;
    int height = 1080;
};

struct NodeConfig {
    std::string name;
    std::string host;
    std::string lanIp;
    std::string path;
    int videoPort = 0; // legacy single-camera field
    int imuPort = 0;
    std::vector<CameraConfig> cameras;
};

struct GlobalConfig {
    std::string baseLanIp;
};

struct TargetsConfig {
    GlobalConfig global;
    std::vector<NodeConfig> targets;
};

bool loadTargetsConfig(const std::string &path, TargetsConfig &cfg);
