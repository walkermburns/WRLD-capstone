#pragma once

#include <string>
#include <vector>

// Configuration structures reflecting the layout of targets.yaml as used by
// the *base station* software.  We only care about fields that the base
// needs (node name/ports, global base address) but the loader is intentionally
// permissive so that additional keys can be ignored without causing an error.

struct NodeConfig {
    std::string name;
    std::string host;
    std::string lanIp;
    std::string path;
    int videoPort = 0;
    int imuPort = 0;
};

struct GlobalConfig {
    std::string baseLanIp;
};

struct TargetsConfig {
    GlobalConfig global;
    std::vector<NodeConfig> targets;
};

// Load an entire targets.yaml file.  Returns true on success, false otherwise.
// The loader tolerates missing fields and will leave defaults in the struct.
// It is the caller's responsibility to verify that the meaningful values (eg
// imuPort) were actually provided.
bool loadTargetsConfig(const std::string &path, TargetsConfig &cfg);
