#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "autoware/scenario_gate/scenario_gate.hpp"

using autoware::scenario_gate::ScenarioGateNode;

class RclGuard : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RclGuard, LoadDefaultSelector_Succeeds)
{
  rclcpp::NodeOptions options;
  EXPECT_NO_THROW({
    auto node = std::make_shared<ScenarioGateNode>(options);
    (void)node;
  });
}

TEST_F(RclGuard, LoadExtraSelector_Succeeds)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("selector_type", "Extra"),
  });

  EXPECT_NO_THROW({
    auto node = std::make_shared<ScenarioGateNode>(options);
    (void)node;
  });
}

TEST_F(RclGuard, LoadUnknownSelector_Throws)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("selector_type", "DoesNotExist"),
  });

  EXPECT_THROW(
    {
      auto node = std::make_shared<ScenarioGateNode>(options);
      (void)node;
    },
    std::runtime_error);
}
