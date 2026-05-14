#include <gtest/gtest.h>
#include <any>
#include <filesystem>
#include <fmt/chrono.h>
#include <algorithm>
#include "config_manager/config_manager.h"
#define private public
#include "path_planner/local_path_optimizer.h"

#include "ocp/ocp_model.h"
#include "base/singleton.h"
#include "util/timer.h"
#include "decision_data/decision_define.h"

namespace gpal::pnc::planning {
// ====================================================================================
// Tester 派生类，用于实例化抽象基类并访问 protected 方法
// ====================================================================================
class LocalPathOptimizerTester : public LocalPathOptimizer {
 public:
  // 为基类中的纯虚函数提供最简单的实现
  std::string name() const override { return "LocalPathOptimizerTester"; }
  Status proc(const ReferenceLine&, const TrajectoryPt&, const PathBoundary&, PathData* const) override {
    return Status::OK();
  }

  // **关键**: 重写 init() 方法，让它使用我们传入的配置
  bool init(const LocalPathOptimizerConfig& test_config) {
    // 直接将测试用的配置赋值给内部成员变量
    auto config_manager = Singleton<ConfigManager>::get_instance();
    this->vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
    this->vehicle_param_ = std::make_shared<VehicleParam>(config_manager->vehicle_config().vehicle_param());
    this->optimizer_config_ = test_config;
    this->profile_type_ = "regular";
    return true;
  }

  // 将所有需要测试的 protected 方法公开
  bool callPreProcess(const TrajectoryPt& start_point, const DiscretizedPath& prev_path,
                      const std::string& profile_type) {
    return this->preProcess(start_point, prev_path, profile_type);
  }

  bool callGetSpeedPointsFromPrevSpeedData(const SpeedData& prev_speed_data, const int64_t curr_stamp,
                                           std::vector<gpal::pnc::SpeedPoint>* speed_points) {
    return this->getSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 1.0, 0.0, 10.0, 0.1, 100.0, speed_points);
  }

  bool callGetSpeedPointsFromCVModel(double v0, std::vector<gpal::pnc::SpeedPoint>* speed_points) {
    return this->getSpeedPointsFromCVModel(v0, 0.0, 10.0, 0.1, 100.0, speed_points);
  }

  const LocalPathOptimizerProfile& callProfile() const { return this->profile(); }
  // 将受保护的 getSpeedPointsFromPrevSpeedData 方法公开
  bool callGetSpeedPointsFromPrevSpeedData(const SpeedData& prev_speed_data, const int64_t curr_stamp,
                                           const double min_v, const double t0, const double t1, const double dt,
                                           const double max_s, std::vector<gpal::pnc::SpeedPoint>* speed_points) {
    return this->getSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, min_v, t0, t1, dt, max_s, speed_points);
  }
  // 将受保护的 getSpeedPointsFromCVModel 方法公开
  bool callGetSpeedPointsFromCVModel(const double v0, const double t0, const double t1, const double dt,
                                     const double max_s, std::vector<gpal::pnc::SpeedPoint>* speed_points) {
    return this->getSpeedPointsFromCVModel(v0, t0, t1, dt, max_s, speed_points);
  }

  // 将受保护的 initLocalPathModel 方法公开
  std::shared_ptr<OptimalControlProblem> callInitLocalPathModel(const LocalPathOptimizerProfile& profile,
                                                                const std::vector<gpal::pnc::SpeedPoint>& speed_points,
                                                                const DiscretizedPath& path,
                                                                const VehicleState& curr_state) {
    return this->initLocalPathModel(profile, speed_points, path, curr_state);
  }
};

// ====================================================================================
// GTest Fixture: 用于共享设置和辅助函数
// ====================================================================================
class LocalPathOptimizerTestPlus : public ::testing::Test {
 protected:
  void SetUp() override { optimizer_tester_ = std::make_unique<LocalPathOptimizerTester>(); }

  std::unique_ptr<LocalPathOptimizerTester> optimizer_tester_;
};

// ====================================================================================
// 测试用例
// ====================================================================================

// TEST_F(LocalPathOptimizerTestPlus, InitFunction) {
//   // 场景: 验证init函数能够成功执行
//   EXPECT_TRUE(optimizer_tester_->init());
// }

// **【新增测试用例】**
TEST_F(LocalPathOptimizerTestPlus, PreProcessCoverage) {
  SCOPED_TRACE("Testing all branches of preProcess");
  LocalPathOptimizerConfig config;
  optimizer_tester_->init(config);
  TrajectoryPt start_pt = {};

  // 场景1: 路径太短 (size < 2)
  EXPECT_FALSE(optimizer_tester_->callPreProcess(start_pt, {}, "regular"));

  // 场景2: 路径长度不足 (length < min_ref_path_length)
  DiscretizedPath short_path;
  // 创建一个点数足够，但s值长度不足的路径
  for (int i = 0; i <= 2; ++i) {
    PathPt pt;
    pt.set_x(0.0 + i * 0.5);
    pt.set_y(0.0);
    pt.set_theta(0.0);  // 左转航向
    pt.set_kappa(0.0);  // 左转曲率
    pt.set_s(0.0 + i * 0.5);
    short_path.push_back(pt);
  }
  EXPECT_FALSE(optimizer_tester_->callPreProcess(start_pt, short_path, "regular"));

  // 场景3: 正常路径
  DiscretizedPath valid_path;
  // **【代码修正】**: 创建一条长度足够长的路径 (s值达到100)
  for (int i = 0; i < 101; ++i) valid_path.emplace_back(i, 0, 0, 0, 0, 0, i, 0, 0);
  EXPECT_TRUE(optimizer_tester_->callPreProcess(start_pt, valid_path, "regular"));
}

TEST_F(LocalPathOptimizerTestPlus, GetSpeedPointsFromPrevSpeedDataCoverage) {
  SCOPED_TRACE("Testing all branches of getSpeedPointsFromPrevSpeedData");

  SpeedData prev_speed_data;
  std::vector<gpal::pnc::SpeedPoint> speed_points;
  int64_t curr_stamp = 200000;  // 0.2 seconds in microseconds

  // --- 场景1: speed_points 指针为空 ---
  EXPECT_FALSE(optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 1.0, 0.0, 1.0, 0.1,
                                                                      20.0, nullptr));

  // --- 场景2: prev_speed_data 为空 ---
  EXPECT_FALSE(optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 1.0, 0.0, 1.0, 0.1,
                                                                      20.0, &speed_points));

  // --- 准备一份有效的历史速度数据 ---
  prev_speed_data.setStamp(std::chrono::microseconds(0));
  for (int i = 0; i <= 20; ++i) {  // t from 0 to 2.0s
    prev_speed_data.AppendSpeedPoint(i * 0.5, static_cast<double>(i) * 0.1, 5.0, 0.0, 0.0);
  }

  // --- 场景3: 循环中 EvaluateByTime 失败 ---
  {
    SCOPED_TRACE("Scenario: EvaluateByTime fails inside the loop");
    // 请求一个超出 prev_speed_data 时间范围的时间点 (curr_stamp=0.2s, t=1.9s => 2.1s > 2.0s)
    EXPECT_TRUE(optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 1.0, 0.0, 2.0, 0.1,
                                                                       20.0, &speed_points));
    EXPECT_FALSE(speed_points.empty());
  }

  // --- 场景4: 速度低于min_v ---
  {
    SCOPED_TRACE("Scenario: Speed is lower than min_v");
    EXPECT_TRUE(optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 10.0, 0.0, 0.2, 0.1,
                                                                       20.0, &speed_points));
    for (const auto& pt : speed_points) {
      EXPECT_GE(pt.v(), 10.0);
    }
  }

  // --- 场景5: delta_t 过小 ---
  {
    SCOPED_TRACE("Scenario: Delta t is too small");
    EXPECT_TRUE(optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 1.0, 0.0, 0.2, 1e-7,
                                                                       20.0, &speed_points));
  }

  // --- 场景6: s 超出 max_s ---
  {
    SCOPED_TRACE("Scenario: s exceeds max_s");
    // **【断言修正】**: 此场景下，函数应返回false，因为生成的点数小于等于1
    EXPECT_FALSE(optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 1.0, 0.0, 1.0, 0.1,
                                                                        0.2, &speed_points));
  }
}

TEST_F(LocalPathOptimizerTestPlus, GetSpeedPointsFromCVModelCoverage) {
  SCOPED_TRACE("Testing all branches of getSpeedPointsFromCVModel");

  std::vector<gpal::pnc::SpeedPoint> speed_points;
  const double v0 = 10.0;
  const double max_s = 100.0;
  const double dt = 0.1;

  // --- 场景1: speed_points 指针为空 (覆盖第433行 if 为 true) ---
  {
    SCOPED_TRACE("Scenario: speed_points is nullptr");
    EXPECT_FALSE(optimizer_tester_->callGetSpeedPointsFromCVModel(v0, 0.0, 5.0, dt, max_s, nullptr));
  }

  // --- 场景2: 正常生成多个点 (覆盖 for 循环和 return true) ---
  {
    SCOPED_TRACE("Scenario: Normal generation with multiple points");
    EXPECT_TRUE(optimizer_tester_->callGetSpeedPointsFromCVModel(v0, 0.0, 5.0, dt, max_s, &speed_points));
    EXPECT_GT(speed_points.size(), 1);
  }
}

}  // namespace gpal::pnc::planning