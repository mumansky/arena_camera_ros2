
#include "quality_of_service_translation.h"

std::map<std::string, rmw_qos_reliability_policy_t>
    K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY = {
        {"system_default", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
        {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
        {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT},
        {"unknown", RMW_QOS_POLICY_RELIABILITY_UNKNOWN}};

std::map<rmw_qos_reliability_policy_t, std::string>
    K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER = {
        {RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT, "system_default"},
        {RMW_QOS_POLICY_RELIABILITY_RELIABLE, "reliable"},
        {RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, "best_effort"},
        {RMW_QOS_POLICY_RELIABILITY_UNKNOWN, "unknown"}};

std::map<std::string, rmw_qos_history_policy_t>
    K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY = {
        {"system_default", RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
        {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
        {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL},
        {"unknown", RMW_QOS_POLICY_HISTORY_UNKNOWN}};

std::map<rmw_qos_history_policy_t, std::string>
    K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER = {
        {RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, "system_default"},
        {RMW_QOS_POLICY_HISTORY_KEEP_LAST, "keep_last"},
        {RMW_QOS_POLICY_HISTORY_KEEP_ALL, "keep_all"},
        {RMW_QOS_POLICY_HISTORY_UNKNOWN, "unknown"}};

bool is_supported_qos_reliability_policy(const std::string& policy)
{
  return K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY.find(policy) !=
         K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY.end();
}

bool is_supported_qos_history_policy(const std::string& policy)
{
  return K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY.find(policy) !=
         K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY.end();
}
