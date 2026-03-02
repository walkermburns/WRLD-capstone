#include "Config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

bool loadTargetsConfig(const std::string &path, TargetsConfig &cfg)
{
    try {
        YAML::Node root = YAML::LoadFile(path);

        // global/base settings
        if (root["global"] && root["global"]["base"]) {
            auto base = root["global"]["base"];
            if (base["LAN_IP"])
                cfg.global.baseLanIp = base["LAN_IP"].as<std::string>();
        }

        // targets array
        if (root["targets"] && root["targets"].IsSequence()) {
            for (auto t : root["targets"]) {
                NodeConfig node;
                if (t["name"])       node.name       = t["name"].as<std::string>();
                if (t["host"])       node.host       = t["host"].as<std::string>();
                if (t["LAN_IP"])     node.lanIp      = t["LAN_IP"].as<std::string>();
                if (t["path"])       node.path       = t["path"].as<std::string>();
                if (t["video_port"]) node.videoPort  = t["video_port"].as<int>();
                if (t["imu_port"])   node.imuPort    = t["imu_port"].as<int>();

                cfg.targets.push_back(std::move(node));
            }
        }

        return true;
    } catch (const YAML::Exception &e) {
        std::cerr << "failed to parse config " << path << ": " << e.what() << std::endl;
        return false;
    }
}
