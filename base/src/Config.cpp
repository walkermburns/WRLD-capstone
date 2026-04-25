#include "Config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <initializer_list>

namespace {

static int readIntWithAliases(const YAML::Node &node,
                              std::initializer_list<const char *> keys,
                              int fallback)
{
    for (const char *key : keys) {
        if (node[key]) {
            return node[key].as<int>();
        }
    }
    return fallback;
}

static std::string readStringWithAliases(const YAML::Node &node,
                                         std::initializer_list<const char *> keys,
                                         const std::string &fallback = {})
{
    for (const char *key : keys) {
        if (node[key]) {
            return node[key].as<std::string>();
        }
    }
    return fallback;
}

static CameraConfig parseCameraConfig(const YAML::Node &c, const std::string &defaultName)
{
    CameraConfig cam;
    cam.name = readStringWithAliases(c, {"name", "camera", "id"}, defaultName);
    cam.videoPort = readIntWithAliases(c, {"video_port", "videoPort"}, 0);
    cam.metadataPort = readIntWithAliases(c, {"metadata_port", "metadataPort", "meta_port", "metaPort"}, 0);
    cam.mount = readStringWithAliases(c, {"mount", "camera_mount"}, "front");

    if (c["layout"]) {
        const auto layout = c["layout"];
        cam.x = readIntWithAliases(layout, {"x", "xpos"}, cam.x);
        cam.y = readIntWithAliases(layout, {"y", "ypos"}, cam.y);
        cam.width = readIntWithAliases(layout, {"width", "w"}, cam.width);
        cam.height = readIntWithAliases(layout, {"height", "h"}, cam.height);
    } else {
        cam.x = readIntWithAliases(c, {"x", "xpos"}, cam.x);
        cam.y = readIntWithAliases(c, {"y", "ypos"}, cam.y);
        cam.width = readIntWithAliases(c, {"width", "w"}, cam.width);
        cam.height = readIntWithAliases(c, {"height", "h"}, cam.height);
    }

    if (cam.metadataPort == 0 && cam.videoPort != 0) {
        cam.metadataPort = cam.videoPort + 1;
    }

    return cam;
}

} // namespace

bool loadTargetsConfig(const std::string &path, TargetsConfig &cfg)
{
    try {
        YAML::Node root = YAML::LoadFile(path);

        if (root["global"] && root["global"]["base"]) {
            auto base = root["global"]["base"];
            if (base["LAN_IP"])
                cfg.global.baseLanIp = base["LAN_IP"].as<std::string>();
        }

        if (root["targets"] && root["targets"].IsSequence()) {
            for (auto t : root["targets"]) {
                NodeConfig node;
                if (t["name"])       node.name       = t["name"].as<std::string>();
                if (t["host"])       node.host       = t["host"].as<std::string>();
                if (t["LAN_IP"])     node.lanIp      = t["LAN_IP"].as<std::string>();
                if (t["path"])       node.path       = t["path"].as<std::string>();
                if (t["video_port"]) node.videoPort  = t["video_port"].as<int>();
                if (t["imu_port"])   node.imuPort    = t["imu_port"].as<int>();

                if (t["cameras"] && t["cameras"].IsSequence()) {
                    int k = 0;
                    for (const auto &c : t["cameras"]) {
                        node.cameras.push_back(parseCameraConfig(c, "camera" + std::to_string(k++)));
                    }
                } else if (node.videoPort != 0) {
                    CameraConfig cam;
                    cam.name = "front";
                    cam.videoPort = node.videoPort;
                    cam.metadataPort = node.videoPort + 1;
                    cam.mount = "front";
                    node.cameras.push_back(cam);
                }

                cfg.targets.push_back(std::move(node));
            }
        }

        return true;
    } catch (const YAML::Exception &e) {
        std::cerr << "failed to parse config " << path << ": " << e.what() << std::endl;
        return false;
    }
}
