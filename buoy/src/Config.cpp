#include "Config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

bool loadTargetConfig(const std::string &path, TargetConfig &cfg)
{
    try {
        YAML::Node root = YAML::LoadFile(path);
        if (root["global"] && root["global"]["base"] &&
            root["global"]["base"]["LAN_IP"])
        {
            cfg.baseIp = root["global"]["base"]["LAN_IP"].as<std::string>();
        }

        if (root["targets"] && root["targets"].IsSequence() &&
            root["targets"].size() > 0)
        {
            YAML::Node t = root["targets"][0];
            if (t["LAN_IP"]) cfg.buoyIp = t["LAN_IP"].as<std::string>();
            if (t["imu_port"]) cfg.imuPort = t["imu_port"].as<int>();

            // Legacy flat camera keys.
            if (t["video_port"]) cfg.videoPort = t["video_port"].as<int>();
            if (t["video2_port"]) cfg.video2Port = t["video2_port"].as<int>();
            if (t["camera_name"]) cfg.cameraName = t["camera_name"].as<std::string>();
            if (t["camera2_name"]) cfg.camera2Name = t["camera2_name"].as<std::string>();

            // New schema: targets[].cameras[] where cameras[0] is primary
            // (hardware encoder) and cameras[1] is secondary (software encoder).
            if (t["cameras"] && t["cameras"].IsSequence()) {
                const YAML::Node cameras = t["cameras"];
                if (cameras.size() > 0) {
                    const YAML::Node c0 = cameras[0];
                    if (c0["video_port"]) cfg.videoPort = c0["video_port"].as<int>();
                    if (c0["path"]) cfg.cameraName = c0["path"].as<std::string>();
                }
                if (cameras.size() > 1) {
                    const YAML::Node c1 = cameras[1];
                    if (c1["video_port"]) cfg.video2Port = c1["video_port"].as<int>();
                    if (c1["path"]) cfg.camera2Name = c1["path"].as<std::string>();
                }
            }
        }

        // fallback defaults if not provided
        if (cfg.buoyIp.empty())
            cfg.buoyIp = ""; // caller may ignore
        if (cfg.imuPort == 0)
            cfg.imuPort = 5000; // legacy default
        if (cfg.videoPort == 0)
            cfg.videoPort = 5001;
        if (cfg.cameraName.empty())
            cfg.cameraName = "0";

        return true;
    } catch (const YAML::Exception &e) {
        std::cerr << "failed to parse config " << path << ": " << e.what() << std::endl;
        return false;
    }
}
