// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#include <c_agibot_hand_base.h>
#include "implementation/c_agibot_hand_can/c_agibot_hand_can.h"
#include "implementation/c_agibot_hand_rs/c_agibot_hand_rs.h"

namespace YAML {
template <>
struct convert<AgibotHandO10::HardwareConf> {
  static bool decode(const Node& node, AgibotHandO10::HardwareConf& hardware_conf) {
    if (!node.IsMap()) return false;
    if (node["type"]) {
      hardware_conf.device = node["type"].as<std::string>();
    }
    if (node["options"]) {
      hardware_conf.options = node["options"];
    } else {
      hardware_conf.options = YAML::Node(YAML::NodeType::Null);
    }
    return true;
  }
};
}  // namespace YAML

std::unique_ptr<AgibotHandO10> AgibotHandO10::createHand(
    unsigned char device_id,
    EHandType hand_type,
    std::string_view cfg_path) {
  std::unique_ptr<AgibotHandO10> hand;

  HardwareConf hardware_conf;
  if (!cfg_path.empty()) {
    YAML::Node config = YAML::LoadFile(cfg_path.data());
    hardware_conf = config["device"].as<HardwareConf>();
  }

  // 根据类型创建具体实例
  if (hardware_conf.device == "can") {
    hand = std::make_unique<AgibotHandCanO10>(hardware_conf.options);
  } else {
    hand = std::make_unique<AgibotHandRsO10>(hardware_conf.options);
  }

  hand->init(device_id, hand_type);

  return hand;
}
