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
            if (t["video_port"]) cfg.videoPort = t["video_port"].as<int>();
            if (t["imu_port"]) cfg.imuPort = t["imu_port"].as<int>();
        }

        // fallback defaults if not provided
        if (cfg.buoyIp.empty())
            cfg.buoyIp = ""; // caller may ignore
        if (cfg.imuPort == 0)
            cfg.imuPort = 5000; // legacy default
        if (cfg.videoPort == 0)
            cfg.videoPort = 5001;

        return true;
    } catch (const YAML::Exception &e) {
        std::cerr << "failed to parse config " << path << ": " << e.what() << std::endl;
        return false;
    }
}
