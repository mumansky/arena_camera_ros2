/**
 * @file test_qos_translation.cpp
 * @brief Unit tests for QoS policy translation utilities
 */

#include <gtest/gtest.h>
#include <string>

// Include the QoS translation header
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

/**
 * @brief Test reliability policy string to enum mapping
 */
TEST(QoSTranslationTest, ReliabilityPolicyStringToEnum)
{
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY["system_default"],
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY["reliable"],
            RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY["best_effort"],
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY["unknown"],
            RMW_QOS_POLICY_RELIABILITY_UNKNOWN);
}

/**
 * @brief Test reliability policy enum to string mapping
 */
TEST(QoSTranslationTest, ReliabilityPolicyEnumToString)
{
  EXPECT_EQ(K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT],
            "system_default");
  EXPECT_EQ(K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[RMW_QOS_POLICY_RELIABILITY_RELIABLE],
            "reliable");
  EXPECT_EQ(K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT],
            "best_effort");
  EXPECT_EQ(K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[RMW_QOS_POLICY_RELIABILITY_UNKNOWN],
            "unknown");
}

/**
 * @brief Test history policy string to enum mapping
 */
TEST(QoSTranslationTest, HistoryPolicyStringToEnum)
{
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY["system_default"],
            RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT);
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY["keep_last"],
            RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY["keep_all"],
            RMW_QOS_POLICY_HISTORY_KEEP_ALL);
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY["unknown"],
            RMW_QOS_POLICY_HISTORY_UNKNOWN);
}

/**
 * @brief Test history policy enum to string mapping
 */
TEST(QoSTranslationTest, HistoryPolicyEnumToString)
{
  EXPECT_EQ(K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT],
            "system_default");
  EXPECT_EQ(K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[RMW_QOS_POLICY_HISTORY_KEEP_LAST],
            "keep_last");
  EXPECT_EQ(K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[RMW_QOS_POLICY_HISTORY_KEEP_ALL],
            "keep_all");
  EXPECT_EQ(K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[RMW_QOS_POLICY_HISTORY_UNKNOWN],
            "unknown");
}

/**
 * @brief Test is_supported_qos_reliability_policy function
 */
TEST(QoSTranslationTest, IsSupportedReliabilityPolicy)
{
  // Supported policies
  EXPECT_TRUE(is_supported_qos_reliability_policy("system_default"));
  EXPECT_TRUE(is_supported_qos_reliability_policy("reliable"));
  EXPECT_TRUE(is_supported_qos_reliability_policy("best_effort"));
  EXPECT_TRUE(is_supported_qos_reliability_policy("unknown"));

  // Unsupported policies
  EXPECT_FALSE(is_supported_qos_reliability_policy("invalid"));
  EXPECT_FALSE(is_supported_qos_reliability_policy(""));
  EXPECT_FALSE(is_supported_qos_reliability_policy("RELIABLE"));  // Case sensitive
  EXPECT_FALSE(is_supported_qos_reliability_policy("Best_Effort"));
}

/**
 * @brief Test is_supported_qos_histroy_policy function
 */
TEST(QoSTranslationTest, IsSupportedHistoryPolicy)
{
  // Supported policies
  EXPECT_TRUE(is_supported_qos_histroy_policy("system_default"));
  EXPECT_TRUE(is_supported_qos_histroy_policy("keep_last"));
  EXPECT_TRUE(is_supported_qos_histroy_policy("keep_all"));
  EXPECT_TRUE(is_supported_qos_histroy_policy("unknown"));

  // Unsupported policies
  EXPECT_FALSE(is_supported_qos_histroy_policy("invalid"));
  EXPECT_FALSE(is_supported_qos_histroy_policy(""));
  EXPECT_FALSE(is_supported_qos_histroy_policy("KEEP_LAST"));  // Case sensitive
  EXPECT_FALSE(is_supported_qos_histroy_policy("Keep_All"));
}

/**
 * @brief Test reliability policy bidirectional consistency
 */
TEST(QoSTranslationTest, ReliabilityPolicyBidirectionalConsistency)
{
  for (const auto& pair : K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY) {
    const std::string& str_policy = pair.first;
    const rmw_qos_reliability_policy_t& enum_policy = pair.second;
    
    auto it = K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER.find(enum_policy);
    ASSERT_NE(it, K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER.end())
        << "Enum policy not found in reverse map for: " << str_policy;
    EXPECT_EQ(it->second, str_policy)
        << "Round-trip failed for reliability policy: " << str_policy;
  }
}

/**
 * @brief Test history policy bidirectional consistency
 */
TEST(QoSTranslationTest, HistoryPolicyBidirectionalConsistency)
{
  for (const auto& pair : K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY) {
    const std::string& str_policy = pair.first;
    const rmw_qos_history_policy_t& enum_policy = pair.second;
    
    auto it = K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER.find(enum_policy);
    ASSERT_NE(it, K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER.end())
        << "Enum policy not found in reverse map for: " << str_policy;
    EXPECT_EQ(it->second, str_policy)
        << "Round-trip failed for history policy: " << str_policy;
  }
}

/**
 * @brief Test map sizes match for reliability policies
 */
TEST(QoSTranslationTest, ReliabilityMapSizesMatch)
{
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY.size(),
            K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER.size());
}

/**
 * @brief Test map sizes match for history policies
 */
TEST(QoSTranslationTest, HistoryMapSizesMatch)
{
  EXPECT_EQ(K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY.size(),
            K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER.size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
