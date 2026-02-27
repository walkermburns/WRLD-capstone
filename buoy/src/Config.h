#pragma once

#include <string>

// Represents the subset of targets.yaml that applies to the
// current build/installation.  The build system will copy the appropriate
// single-target YAML into src/configs/target.yaml before compiling.

struct TargetConfig {
    std::string baseIp;      // LAN IP of the base station
    std::string buoyIp;      // IP address of this buoy node
    int imuPort = 0;         // port for IMU protobuf messages (destination is baseIp)
    int videoPort = 0;       // port for video RTP stream (destination is baseIp)
};

// load the YAML file at the given path, return false on failure.  If multiple
// targets appear, the first element is used.
bool loadTargetConfig(const std::string &path, TargetConfig &cfg);
