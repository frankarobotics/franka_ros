// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <stdint.h>
#include <list>
#include <map>
#include <string>
#include <vector>

#include <hardware_interface/controller_info.h>

#include <franka_hw/control_mode.h>

namespace franka_hw {

using ResourceWithClaimsMap = std::map<std::string, std::vector<std::vector<std::string>>>;

struct ResourceClaims {
  uint8_t joint_position_claims = 0;
  uint8_t joint_velocity_claims = 0;
  uint8_t joint_torque_claims = 0;
  uint8_t cartesian_velocity_claims = 0;
  uint8_t cartesian_pose_claims = 0;
};

using ArmClaimedMap = std::map<std::string, ResourceClaims>;

bool findArmIdInResourceId(const std::string& resource_id, std::string* arm_id);

ResourceWithClaimsMap getResourceMap(const std::list<hardware_interface::ControllerInfo>& info);

bool getArmClaimedMap(ResourceWithClaimsMap& resource_map, ArmClaimedMap& arm_claim_map);

ControlMode getControlMode(const std::string& arm_id, ArmClaimedMap& arm_claim_map);

bool hasConflictingMultiClaim(const ResourceWithClaimsMap& resource_map);

bool hasConflictingJointAndCartesianClaim(const ArmClaimedMap& arm_claim_map,
                                          const std::string& arm_id);

bool partiallyClaimsArmJoints(const ArmClaimedMap& arm_claim_map, const std::string& arm_id);

bool hasTrajectoryClaim(const ArmClaimedMap& arm_claim_map, const std::string& arm_id);

}  // namespace franka_hw
