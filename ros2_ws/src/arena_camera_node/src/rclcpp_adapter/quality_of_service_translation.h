#pragma once

#include <map>
#include <string>

#include "rmw/types.h"

// Bidirectional maps between YAML string values and RMW QoS policy enums.
extern std::map<std::string, rmw_qos_reliability_policy_t>
    K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY;

extern std::map<rmw_qos_reliability_policy_t, std::string>
    K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER;

extern std::map<std::string, rmw_qos_history_policy_t>
    K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY;

extern std::map<rmw_qos_history_policy_t, std::string>
    K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER;

bool is_supported_qos_reliability_policy(const std::string& policy);
bool is_supported_qos_history_policy(const std::string& policy);
