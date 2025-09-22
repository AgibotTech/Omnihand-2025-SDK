// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#include <c_agibot_hand_base.h>
#include "implementation/c_agibot_hand_can/c_agibot_hand_can.h"
#include "implementation/c_agibot_hand_rs/c_agibot_hand_rs.h"
#include "yaml-cpp/yaml.h"

namespace YAML {
template <>
struct convert<AgibotHandBase::Options> {
  static bool decode(const Node& node, AgibotHandBase::Options& options) {
    if (!node.IsMap()) return false;
    if (node["type"]) {
      options.device = node["type"].as<std::string>();
    }
    return true;
  }
};
}  // namespace YAML

std::unique_ptr<AgibotHandBase> AgibotHandBase::createHand(
    bool use_can,
    unsigned char device_id,
    EHandType hand_type,
    std::string_view cfg_path) {
  std::unique_ptr<AgibotHandBase> hand;

  Options options;
  if (!cfg_path.empty()) {
    YAML::Node config = YAML::LoadFile(cfg_path.data());
    options = config["device"].as<Options>();
    std::cout << config << std::endl;
  }

  // 根据类型创建具体实例
  if (options.device == "can") {
    hand = std::make_unique<AgibotHandCanO10>();
  } else {
    hand = std::make_unique<AgibotHandRsO10>();
  }

  hand->init(device_id, hand_type);

  return hand;
}
