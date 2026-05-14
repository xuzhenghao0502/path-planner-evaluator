#include "decision_data/container/container_manager.h"

#include <gtest/gtest.h>
#include "common/test_utils.hpp"

namespace gpal {
namespace pnc {
namespace prediction {

class ContainerManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    planning::InitConfigManager();

    container_manager_ = std::make_unique<ContainerManager>();
  }

  std::unique_ptr<ContainerManager> container_manager_;
};

TEST_F(ContainerManagerTest, Init_EmptyConfig) {
  PredictionConfig config;
  ContainerConfig* container_config = config.mutable_container_config();

  container_manager_->Init(config);
}

TEST_F(ContainerManagerTest, Init_NonEmptyConfig) {
  PredictionConfig config;
  ContainerConfig* container_config = config.mutable_container_config();
  container_config->add_messages()->set_type(ContainerMessage::LOCALIZATION);
  container_config->add_messages()->set_type(ContainerMessage::ADC_TRAJECTORY);

  container_manager_->Init(config);
  auto* container = container_manager_->GetContainer<Container>(ContainerMessage::LOCALIZATION);
  EXPECT_NE(container, nullptr);
}

TEST_F(ContainerManagerTest, Reset) {
  PredictionConfig config;
  ContainerConfig* container_config = config.mutable_container_config();
  container_config->add_messages()->set_type(ContainerMessage::PERCEPTION_OBSTACLES);

  container_manager_->Init(config);
  EXPECT_NE(container_manager_->GetContainer<Container>(ContainerMessage::PERCEPTION_OBSTACLES), nullptr);

  container_manager_->Reset();
  EXPECT_EQ(container_manager_->GetContainer<Container>(ContainerMessage::PERCEPTION_OBSTACLES), nullptr);
}

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal