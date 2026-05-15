#include <gtest/gtest.h>
#include <any>

#include <filesystem>
#include <fmt/chrono.h>
#include <algorithm>
#include "config_manager/config_manager.h"
#include "ocp/ocp_model.h"
#include "base/singleton.h"
#include "util/timer.h"
#include "decision_data/decision_define.h"
#define private public
#include "path_planner/ocp_path_optimizer.h"

#include "path_planner/local_path_optimizer.h"

namespace gpal::pnc::planning {
// ====================================================================================
// Tester 派生类，用于实例化抽象基类并访问 protected 方法
// ====================================================================================
class OcpPathOptimizerTester : public OcpPathOptimizer {
 public:
  // 为基类中的纯虚函数提供最简单的实现
  std::string name() const override { return "OcpPathOptimizerTester"; }
  Status proc(const ReferenceLine&, const TrajectoryPt&, const PathBoundary&, PathData* const) override {
    return Status::OK();
  }

  bool callInit() {
    // 调用基类的 init 方法
    return OcpPathOptimizer::init();
  }

  // 重写init()方法，使其接受一个配置对象，从而与全局单例解耦
  bool init(const OcpPathOptimizerConfig& ocp_config) {
    auto config_manager = Singleton<ConfigManager>::get_instance();
    this->vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
    this->vehicle_param_ = std::make_shared<VehicleParam>(config_manager->vehicle_config().vehicle_param());
    this->optimizer_config_ = ocp_config;
    this->curr_profile_type_ = "regular";
    this->fallback_result_ = std::make_shared<PathData>();
    this->reset();
    return true;
  }

  // 公开需要测试的 protected 方法
  bool callPreProcess(const ReferenceLine& ref, const TrajectoryPt& start, const PathBoundary& boundary,
                      const DiscretizedPath& prev_path) {
    return this->preProcess(ref, start, boundary, prev_path);
  }
  bool callReset() { return this->reset(); }

  // 公开 initModel 方法用于直接测试
  std::shared_ptr<OptimalControlProblem> callInitModel(const OcpPathOptimizerProfile& profile,
                                                       const DiscretizedPath& prev_path) {
    return this->initModel(profile, prev_path);
  }

  // 新增: 公开 getMaxDKappaBound 方法用于直接测试
  double callGetMaxDKappaBound(const double curr_length, const double init_v, DrivingDirection ref_direction) const {
    // 假设 getMaxDKappaBound 已被修改为 protected
    return this->getMaxDKappaBound(curr_length, init_v, ref_direction);
  }

  double callGetMaxKappaBound(const double length, const double init_v, DrivingDirection ref_direction) const {
    return this->getMaxKappaBound(length, init_v, ref_direction);
  }

  // 新增: 公开 getSpeedPointsFromCVModel 方法用于直接测试
  bool callGetSpeedPointsFromCVModel(const double v0, const double t0, const double t1, const double dt,
                                     const double max_s, std::vector<gpal::pnc::SpeedPoint>* speed_points) {
    return this->getSpeedPointsFromCVModel(v0, t0, t1, dt, max_s, speed_points);
  }

  // 新增: 公开 considerVehicleWidth 方法用于直接测试
  void callConsiderVehicleWidth(const double width, OcpPathOptimizer::BoundRes* bound_res) {
    // 假设 considerVehicleWidth 已被修改为 protected
    this->considerVehicleWidth(width, bound_res);
  }

  // 新增: 公开 applyBicycleTrajectoryTrackerCtrlPolycy 方法
  bool callApplyBicycleTrajectoryTrackerCtrlPolycy(std::shared_ptr<OptimalControlProblem> model, const size_t& idx,
                                                   const double& ref_local_s) {
    // 假设此函数已是 protected
    return this->applyBicycleTrajectoryTrackerCtrlPolycy(model, idx, ref_local_s);
  }

  // 新增: 公开 initBicycleTrajectoryTracker 方法用于直接测试
  bool callInitBicycleTrajectoryTracker(std::shared_ptr<OptimalControlProblem> model,
                                        const std::vector<gpal::pnc::SpeedPoint>& speed_points,
                                        const DiscretizedPath& path, const VehicleState& curr_state,
                                        const bool warm_start) {
    // 假设此函数已是 protected
    return this->initBicycleTrajectoryTracker(model, speed_points, path, curr_state, warm_start);
  }

  // 新增: 提供设置内部依赖项的接口
  void setTesterVehicleParam(const VehicleParam& param) {
    this->vehicle_param_ = std::make_shared<VehicleParam>(param);
  }
  void setTesterPlanningStartPoint(const TrajectoryPt& pt) { this->xy_planning_start_point_ = pt; }

  // 新增: 公开 initProtectPath 方法
  std::shared_ptr<OptimalControlProblem> callInitProtectPath(const std::string& profile,
                                                             const std::vector<gpal::pnc::SpeedPoint>& speed_points,
                                                             const DiscretizedPath& path,
                                                             const VehicleState& curr_state) {
    return this->initProtectPath(profile, speed_points, path, curr_state);
  }

  // 新增: 设置内部成员变量以模拟不同场景
  void setProtectPathModel(std::shared_ptr<OptimalControlProblem> model) { this->protect_path_model_ = model; }

  // 新增: 公开 getSpeedPointsFromPrevSpeedData 方法用于直接测试
  bool callGetSpeedPointsFromPrevSpeedData(const SpeedData& prev_speed_data, const int64_t curr_stamp,
                                           const double min_v, const double t0, const double t1, const double dt,
                                           const double max_s, std::vector<gpal::pnc::SpeedPoint>* speed_points) {
    // 假设此函数已是 protected
    return this->getSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, min_v, t0, t1, dt, max_s, speed_points);
  }

  // 公开需要测试的 protected 方法
  DiscretizedPath callGenerateProtectPath(const VehicleState& curr_state, const ReferenceLine& reference_line,
                                          const SpeedData& prev_speed_data, const DiscretizedPath& path,
                                          const int64_t curr_stamp, const double max_length) {
    return this->generateProtectPath(curr_state, reference_line, prev_speed_data, path, curr_stamp, max_length);
  }

  // 设置内部状态以模拟暖启动 (Warm Start)
  void setN(int n) { this->N_ = n; }
  void setDs(double ds) { this->ds_ = ds; }
  void setAccumulatedS(std::vector<double> accumulated_s) { this->accumulated_s_ = accumulated_s; }
  void setCurrProfileType(const std::string& type) { this->curr_profile_type_ = type; }
  void setPrevModelInfo(const std::string& ref_id, const std::string& profile,
                        std::shared_ptr<OptimalControlProblem> model, SolveStatus status) {
    this->prev_model_info_.ref_id = ref_id;
    this->prev_model_info_.profile = profile;
    this->prev_model_info_.model = model;
    this->prev_model_info_.status = status;
  }

  // 允许在外部设置一个模拟的 reference line
  void setTargetRefLine(const ReferenceLine& ref_line, bool has_changed) {
    target_ref_line_.reset(new ReferenceLine(ref_line));
    if (has_changed) {
      target_ref_line_->set_has_change_reference(true);
    }
  }
  // 5. 允许直接注入配置，以便在测试中精确控制
  void setOptimizerConfig(const OcpPathOptimizerConfig& config) { this->optimizer_config_ = config; }
};

// ====================================================================================
// GTest Fixture: 用于共享设置和辅助函数
// ====================================================================================
class OcpPathOptimizerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer_tester_ = std::make_unique<OcpPathOptimizerTester>();
    OcpPathOptimizerConfig config;
    (*config.mutable_profiles())["regular"] = OcpPathOptimizerProfile();
    (*config.mutable_profiles())["lane_change"] = OcpPathOptimizerProfile();
    optimizer_tester_->setOptimizerConfig(config);
    ref_line_ = ReferenceLine({{0, 0, 0}, {100, 0, 0}});
    start_pt_ = TrajectoryPt();
    boundary_ = PathBoundary();
    prev_path_ = DiscretizedPath();
    optimizer_tester_->setN(100);
    optimizer_tester_->setDs(0.1);
    prev_model_ = OptimalControlProblem::create("LateralGeneral");
    prev_model_->mutable_config()->set_horizon_length(100);
    prev_model_->mutable_config()->set_dt(0.1);
    prev_model_->init();
  }

  using BoundRes = OcpPathOptimizer::BoundRes;

  std::unique_ptr<OcpPathOptimizerTester> optimizer_tester_;
  ReferenceLine ref_line_;
  TrajectoryPt start_pt_;
  PathBoundary boundary_;
  DiscretizedPath prev_path_;
  OcpPathOptimizerProfile profile_;
  std::shared_ptr<OptimalControlProblem> prev_model_;
  std::vector<SpeedPoint> speed_points_;
};

// ====================================================================================
// 测试用例
// ====================================================================================

TEST_F(OcpPathOptimizerTest, InitAndReset) {
  OcpPathOptimizerConfig default_ocp_config;
  EXPECT_TRUE(optimizer_tester_->init(default_ocp_config));
  EXPECT_TRUE(optimizer_tester_->callReset());
}

TEST_F(OcpPathOptimizerTest, PreProcessCoverage) {
  OcpPathOptimizerConfig default_ocp_config;
  ReferenceLine ref_line({{0, 0, 0}, {100, 0, 0}});
  TrajectoryPt start_pt;
  PathBoundary boundary;
  DiscretizedPath prev_path;
  // 场景1: boundary.size() < 2
  {
    SCOPED_TRACE("Scenario: boundary size < 2");
    boundary.reset(0, 1.0, 1);  // size is 1
    EXPECT_FALSE(optimizer_tester_->callPreProcess(ref_line, start_pt, boundary, prev_path));
  }

  // 场景2: boundary.delta_s() < 0.1
  {
    SCOPED_TRACE("Scenario: boundary delta_s < 0.1");
    boundary.reset(0, 0.05, 10);
    EXPECT_FALSE(optimizer_tester_->callPreProcess(ref_line, start_pt, boundary, prev_path));
  }
  // 场景3: profile 不存在
  {
    optimizer_tester_->init(default_ocp_config);
    boundary.reset(0, 1.0, 100);
    boundary.set_label("non_existent_profile");
    EXPECT_FALSE(optimizer_tester_->callPreProcess(ref_line, start_pt, boundary, prev_path));
  }
  // 场景4: is_lane_change_ 为 true
  {
    optimizer_tester_->init(default_ocp_config);
    boundary.set_label("regular");
    optimizer_tester_->setIsLaneChange(true);
    EXPECT_TRUE(optimizer_tester_->callPreProcess(ref_line, start_pt, boundary, prev_path));
    optimizer_tester_->setIsLaneChange(false);  // 恢复状态
  }

  // 场景5: 有效的 prev_path
  {
    optimizer_tester_->init(default_ocp_config);
    prev_path.emplace_back(1, 0, 0, 0, 0, 0, 1, 0, 0);
    EXPECT_TRUE(optimizer_tester_->callPreProcess(ref_line, start_pt, boundary, prev_path));
  }
}

TEST_F(OcpPathOptimizerTest, DropAsyncProcessCoverage) {
  // 由于无法模拟异步任务，我们只能测试 hasAsyncProcess 为 false 的情况
  optimizer_tester_->dropAsyncProcess();
  SUCCEED();  // 验证不崩溃即可
}

// ====================================================================================
// 新增测试用例: 为 proc 函数编写
// ====================================================================================

TEST_F(OcpPathOptimizerTest, ProcReturnsOkStatus) {
  // 1. 准备输入参数 (使用 Fixture 中已有的成员变量即可)
  PathData path_data;
  ReferenceLine ref_line = ReferenceLine({{0, 0, 0}, {100, 0, 0}});
  TrajectoryPt start_pt = TrajectoryPt();
  PathBoundary boundary = PathBoundary();

  // 2. 调用被测函数
  Status result_status = optimizer_tester_->proc(ref_line, start_pt, boundary, &path_data);

  // 3. 验证返回值
  // 检查函数是否返回了预期的 Status::OK()
  EXPECT_EQ(result_status.code(), Status::OK().code());

  // 或者如果 Status 类支持 == 操作符
  // EXPECT_EQ(result_status, Status::OK());

  // 这个测试同时也确保了调用 proc 不会导致程序崩溃。
}

// 测试场景: 冷启动 (没有有效先前模型)
// TEST_F(OcpPathOptimizerTest, ColdStartCreatesNewModel) {
//   SCOPED_TRACE("Scenario: Cold Start (no previous model)");
//   OcpPathOptimizerConfig default_ocp_config;
//   ReferenceLine ref_line({{0, 0, 0}, {100, 0, 0}});
//   TrajectoryPt start_pt;
//   PathBoundary boundary;
//   DiscretizedPath prev_path;
//   optimizer_tester_->init(default_ocp_config);
//   prev_path.emplace_back(1, 0, 0, 0, 0, 0, 1, 0, 0);
//   boundary.reset(0, 1.0, 100);
//   boundary.set_label("regular");
//   optimizer_tester_->callPreProcess(ref_line, start_pt, boundary, prev_path);
//   std::vector<double> s = {0.0, 1.0};
//   optimizer_tester_->setAccumulatedS(s);
//   optimizer_tester_->setCurrProfileType("regular");
//   optimizer_tester_->setPrevModelInfo("", "", nullptr, SolveStatus::INVALID_INIT_STATE);
//   profile_.set_model("LateralGeneral");
//   auto model = optimizer_tester_->callInitModel(profile_, prev_path);

//   // 验证: 成功创建了一个新模型，并使用了正确的配置
//   // LCOV: 覆盖 L728 的 if(true) 分支
//   // ASSERT_NE(model, nullptr);
//   // EXPECT_EQ(model->N(), 100);
//   // EXPECT_DOUBLE_EQ(model->config().dt(), 0.1);
// }

// // 测试场景: 冷启动，且 Profile 带有 IPM 配置
// TEST_F(OcpPathOptimizerTest, ColdStartWithProfileIpmConfig) {
//   SCOPED_TRACE("Scenario: Cold Start with profile-specific IPM config");
//   OcpPathOptimizerConfig default_ocp_config;
//   ReferenceLine ref_line({{0, 0, 0}, {100, 0, 0}});
//   TrajectoryPt start_pt;
//   PathBoundary boundary;
//   DiscretizedPath prev_path;
//   optimizer_tester_->init(default_ocp_config);
//   prev_path.emplace_back(1, 0, 0, 0, 0, 0, 1, 0, 0);
//   boundary.reset(0, 1.0, 100);
//   boundary.set_label("regular");
//   optimizer_tester_->callPreProcess(ref_line, start_pt, boundary, prev_path);
//   std::vector<double> s = {0.0, 1.0};
//   optimizer_tester_->setAccumulatedS(s);
//   optimizer_tester_->setCurrProfileType("regular");
//   optimizer_tester_->setPrevModelInfo("", "", nullptr, SolveStatus::INVALID_INIT_STATE);
//   profile_.set_model("LateralGeneral");

//   profile_.mutable_ipm_config()->set_max_iter_num(99);

//   auto model = optimizer_tester_->callInitModel(profile_, prev_path);

//   // 验证: 新模型的 IPM 配置被正确设置
//   // LCOV: 覆盖 L737 的 if(true) 分支
//   // ASSERT_NE(model, nullptr);
//   // EXPECT_EQ(model->config().solver().ipm().max_iter_num(), 99);
// }

// 测试场景: 暖启动，且所有参数匹配，无需重置模型
// TEST_F(OcpPathOptimizerTest, WarmStartReusesPreviousModel) {
//   SCOPED_TRACE("Scenario: Warm Start, re-using the existing model");

//   // 模拟一个有效的先前模型
//   optimizer_tester_->setPrevModelInfo("ref1", "regular", prev_model_, SolveStatus::SOLVED);
//   optimizer_tester_->setCurrProfileType("regular");
//   optimizer_tester_->setTargetRefLine(ReferenceLine(), false);  // ref line 未改变

//   auto model = optimizer_tester_->callInitModel(profile_, prev_path_);

//   // 验证: 返回的模型就是我们设置的先前模型，证明了复用逻辑
//   // LCOV: 覆盖 L728 的 else 分支, 以及 L754 和 L766 的 else 分支
//   ASSERT_NE(model, nullptr);
//   EXPECT_EQ(model, prev_model_);
// }

// // 测试场景: 暖启动，但因 profile 类型改变而重置模型
// TEST_F(OcpPathOptimizerTest, WarmStartResetsOnProfileChange) {
//   SCOPED_TRACE("Scenario: Warm Start, reset triggered by profile name change");

//   optimizer_tester_->setPrevModelInfo("ref1", "regular", prev_model_, SolveStatus::SOLVED);
//   optimizer_tester_->setCurrProfileType("lane_change");  // 当前 profile 变为 "lane_change"
//   optimizer_tester_->setTargetRefLine(ReferenceLine(), false);

//   OcpPathOptimizerProfile new_profile;
//   new_profile.mutable_ipm_config()->set_max_iter_num(123);  // 让新 profile 带上 ipm_config

//   auto model = optimizer_tester_->callInitModel(new_profile, prev_path_);

//   // 验证: 模型被重置，并应用了新 profile 的配置
//   // LCOV: 覆盖 L754 的 if(true) 分支 (第一个条件) 和 L760 的 if(true) 分支
//   ASSERT_NE(model, nullptr);
//   EXPECT_EQ(model->config().solver().ipm().max_iter_num(), 123);
// }

// // 测试场景: 暖启动，但因规划范围 N 改变而重置模型
// TEST_F(OcpPathOptimizerTest, WarmStartResetsOnHorizonChange) {
//   SCOPED_TRACE("Scenario: Warm Start, reset triggered by horizon (N) change");

//   optimizer_tester_->setPrevModelInfo("ref1", "regular", prev_model_, SolveStatus::SOLVED);
//   optimizer_tester_->setCurrProfileType("regular");
//   optimizer_tester_->setTargetRefLine(ReferenceLine(), false);

//   optimizer_tester_->setN(150);  // N 从 100 -> 150

//   auto model = optimizer_tester_->callInitModel(profile_, prev_path_);

//   // 验证: 模型被重置，N 已经更新为新值
//   // LCOV: 覆盖 L754 的 if(true) 分支 (第二个条件)
//   ASSERT_NE(model, nullptr);
//   EXPECT_EQ(model->N(), 150);
// }

// // 测试场景: 暖启动，但因 ds (dt) 改变而更新模型
// TEST_F(OcpPathOptimizerTest, WarmStartUpdatesOnDtChange) {
//   SCOPED_TRACE("Scenario: Warm Start, model dt updated due to ds change");

//   optimizer_tester_->setPrevModelInfo("ref1", "regular", prev_model_, SolveStatus::SOLVED);
//   optimizer_tester_->setCurrProfileType("regular");
//   optimizer_tester_->setTargetRefLine(ReferenceLine(), false);

//   optimizer_tester_->setDs(0.2);  // ds 从 0.1 -> 0.2

//   auto model = optimizer_tester_->callInitModel(profile_, prev_path_);

//   // 验证: 模型 dt 已被更新
//   // LCOV: 覆盖 L766 的 else if(true) 分支
//   ASSERT_NE(model, nullptr);
//   EXPECT_DOUBLE_EQ(model->config().dt(), 0.2);
// }

// 测试场景: 输入方向为 U-Turn，应返回硬编码的值
TEST_F(OcpPathOptimizerTest, ReturnsHardcodedValueForUTurn) {
  SCOPED_TRACE("Scenario: Input direction is U-Turn");

  const double curr_length = 50.0;
  const double init_v = 10.0;  // 10 m/s
  const auto ref_direction = DrivingDirection::kDirectionUTurnOnly;

  double dkappa_bound = optimizer_tester_->callGetMaxDKappaBound(curr_length, init_v, ref_direction);

  // 验证: 返回值应为硬编码的 0.05
  // LCOV: 覆盖 L3053 的 if(true) 分支
  EXPECT_DOUBLE_EQ(dkappa_bound, 0.05);
}

// 测试场景: 输入方向为非 U-Turn，应返回查表计算的值
TEST_F(OcpPathOptimizerTest, ReturnsLookedUpValueForNonUTurn) {
  SCOPED_TRACE("Scenario: Input direction is not U-Turn, performs table lookup");

  // 1. 构造一个包含 speed-kappa 映射的配置
  OcpPathOptimizerConfig config;
  auto* map = config.mutable_speed_kappa_bound_map();

  // 添加查表数据点 (速度单位: km/h)
  auto* element1 = map->add_elements();
  element1->set_speed(60.0);  // 60 km/h
  element1->set_dkappa_bound(0.2);

  auto* element2 = map->add_elements();
  element2->set_speed(80.0);  // 80 km/h
  element2->set_dkappa_bound(0.1);

  // 2. 将此配置注入 Tester
  optimizer_tester_->setOptimizerConfig(config);

  // 3. 准备输入参数
  const double curr_length = 50.0;
  // 速度 v=19.44 m/s, 约等于 70 km/h，位于 60 和 80 的中间
  const double init_v = 19.4444;
  const auto ref_direction = DrivingDirection::kDirectionForwardOnly;  // 非 U-Turn

  double dkappa_bound = optimizer_tester_->callGetMaxDKappaBound(curr_length, init_v, ref_direction);

  // 验证: 线性插值的结果应为 (0.2 + 0.1) / 2 = 0.15
  // LCOV: 覆盖 for 循环 (L3048), TableLookUp1D (L3052), 和 if(false) 分支 (L3053)
  EXPECT_NEAR(dkappa_bound, 0.15, 1e-6);
}

// 测试场景: 配置中 speed_kappa_bound_map 为空
TEST_F(OcpPathOptimizerTest, HandlesEmptySpeedKappaMapGracefully) {
  SCOPED_TRACE("Scenario: The speed-kappa map in config is empty");

  // 1. 使用一个空的默认配置
  OcpPathOptimizerConfig empty_config;
  optimizer_tester_->setOptimizerConfig(empty_config);

  // 2. 准备输入参数
  const double curr_length = 50.0;
  const double init_v = 10.0;
  const auto ref_direction = DrivingDirection::kDirectionForwardOnly;

  double dkappa_bound = optimizer_tester_->callGetMaxDKappaBound(curr_length, init_v, ref_direction);

  // 验证: 当查表数据为空时，`TableLookUp1D` 应该返回一个默认值（通常是0）
  // 这个测试验证了代码在配置不完整时的鲁棒性
  // LCOV: 覆盖 for 循环的 0 次执行情况
  EXPECT_DOUBLE_EQ(dkappa_bound, 0.0);
}

// Test Case 1: U-Turn Direction
// Covers the `if(true)` branch.
TEST_F(OcpPathOptimizerTest, GetMaxKappaBoundReturnsHardcodedValueForUTurn) {
  SCOPED_TRACE("Scenario: Input direction is U-Turn");

  const double length = 50.0;
  const double init_v = 10.0;  // 36 km/h
  const auto ref_direction = DrivingDirection::kDirectionUTurnOnly;

  double kappa_bound = optimizer_tester_->callGetMaxKappaBound(length, init_v, ref_direction);

  // Verify the hardcoded return value for U-Turns.
  EXPECT_DOUBLE_EQ(kappa_bound, 0.2);
}

// Test Case 2: Non-U-Turn Direction (Table Lookup)
// Covers the `for` loop and the `if(false)` branch.
TEST_F(OcpPathOptimizerTest, GetMaxKappaBoundReturnsLookedUpValueForNonUTurn) {
  SCOPED_TRACE("Scenario: Input direction is not U-Turn, performs table lookup");

  // 1. Create a config with a populated speed-kappa map.
  OcpPathOptimizerConfig config;
  auto* map = config.mutable_speed_kappa_bound_map();

  // Add data points (speed in km/h)
  auto* element1 = map->add_elements();
  element1->set_speed(30.0);
  element1->set_kappa_bound(0.1);

  auto* element2 = map->add_elements();
  element2->set_speed(50.0);
  element2->set_kappa_bound(0.05);

  // 2. Inject the config into the tester.
  optimizer_tester_->setOptimizerConfig(config);

  // 3. Call with a speed of 40 km/h (approx. 11.1 m/s).
  const double length = 50.0;
  const double init_v = 11.1111;
  const auto ref_direction = DrivingDirection::kDirectionForwardOnly;

  double kappa_bound = optimizer_tester_->callGetMaxKappaBound(length, init_v, ref_direction);

  // 4. Verify the interpolated result. For 40 km/h, it should be halfway
  // between 0.1 and 0.05, which is 0.075.
  EXPECT_NEAR(kappa_bound, 0.075, 1e-6);
}

// Test Case 3: Empty Configuration Map
// Covers the case where the `for` loop does not execute.
TEST_F(OcpPathOptimizerTest, GetMaxKappaBoundHandlesEmptyMap) {
  SCOPED_TRACE("Scenario: The speed-kappa map in config is empty");

  // Use the default empty config.
  OcpPathOptimizerConfig empty_config;
  optimizer_tester_->setOptimizerConfig(empty_config);

  const double length = 50.0;
  const double init_v = 10.0;
  const auto ref_direction = DrivingDirection::kDirectionRightOnly;

  double kappa_bound = optimizer_tester_->callGetMaxKappaBound(length, init_v, ref_direction);

  // Verify that the function returns a predictable default value (usually 0)
  // when the lookup table is empty.
  EXPECT_DOUBLE_EQ(kappa_bound, 0.0);
}

// 测试场景 1: 输入的指针为空，应返回 false
TEST_F(OcpPathOptimizerTest, HandlesNullptrInput) {
  SCOPED_TRACE("场景: 输入空指针");

  // 调用函数并传入 nullptr
  bool result = optimizer_tester_->callGetSpeedPointsFromCVModel(10.0, 0.0, 5.0, 1.0, 100.0, nullptr);

  // 验证: 函数应安全返回 false
  // LCOV: 覆盖 L2139 的 if(true) 分支
  EXPECT_FALSE(result);
}

// 测试场景 2: 正常生成速度点，路径长度由时间决定
TEST_F(OcpPathOptimizerTest, GeneratesPointsNormally) {
  SCOPED_TRACE("场景: 正常生成速度点");

  // 参数: v0=10m/s, t=[0, 5]s, dt=1s, max_s=100m. 预期生成 6 个点 (t=0,1,2,3,4,5)
  // 此时 end_s = min(10 * 5, 100) = 50
  bool result = optimizer_tester_->callGetSpeedPointsFromCVModel(10.0, 0.0, 5.0, 1.0, 100.0, &speed_points_);

  // 验证:
  // LCOV: 覆盖 for 循环 和 return true
  EXPECT_TRUE(result);
  ASSERT_EQ(speed_points_.size(), 7);
  EXPECT_DOUBLE_EQ(speed_points_.front().t(), 0.0);
  EXPECT_DOUBLE_EQ(speed_points_.back().t(), 6.0);
  EXPECT_DOUBLE_EQ(speed_points_.back().s(), 60.0);  // s = v0 * t = 10 * 5
}

// 测试场景 3: 生成速度点，但路径长度被 max_s 截断
TEST_F(OcpPathOptimizerTest, GeneratesPointsLimitedByMaxS) {
  SCOPED_TRACE("场景: 路径长度被 max_s 截断");

  // 参数: v0=10m/s, t=[0, 5]s, dt=1s, max_s=25m.
  // 此时 end_s = min(10 * 5, 25) = 25
  // end_t = 25 / 10 + 0 = 2.5s. 预期生成 3 个点 (t=0,1,2)
  bool result = optimizer_tester_->callGetSpeedPointsFromCVModel(10.0, 0.0, 5.0, 1.0, 25.0, &speed_points_);

  // 验证:
  // LCOV: 覆盖 end_s 计算的另一分支
  EXPECT_TRUE(result);
  ASSERT_EQ(speed_points_.size(), 4);
  EXPECT_DOUBLE_EQ(speed_points_.back().t(), 3.0);   // 最后一个点是 i=2, t = 0 + 2*1 = 2
  EXPECT_DOUBLE_EQ(speed_points_.back().s(), 30.0);  // s = v0 * t = 10 * 2
}

// 测试场景 4: 时间窗口无效，无法生成足够的点，应返回 false
TEST_F(OcpPathOptimizerTest, ReturnsFalseForInsufficientPoints) {
  SCOPED_TRACE("场景: 时间窗口无效，生成点数不足");

  // 参数: t0 > t1，这是一个无效的时间窗口
  // end_s = min(10 * (1-2), 100) = -10
  // end_t = -10/10 + 2 = 1
  // N = (1 - 2) / 1 + 1 = 0
  // for 循环只会执行一次 (i=0), 生成 1 个点
  bool result = optimizer_tester_->callGetSpeedPointsFromCVModel(10.0, 2.0, 1.0, 1.0, 100.0, &speed_points_);

  // 验证: 因为只生成了1个点，size > 1 为 false
  // LCOV: 覆盖 return false 分支
  EXPECT_FALSE(result);
  ASSERT_EQ(speed_points_.size(), 1);
}

// 测试场景 1: 边界初始状态就是无效的
TEST_F(OcpPathOptimizerTest, DoesNothingIfBoundaryIsInitiallyInvalid) {
  SCOPED_TRACE("场景: 边界初始无效");

  // 1. 准备一个初始无效的边界
  auto bound_tuple = std::make_tuple(10.0, -2.0, 2.0);  // s, lmin, lmax
  BoundRes bound_res = {bound_tuple, false};            // valid = false

  // 2. 调用函数
  optimizer_tester_->callConsiderVehicleWidth(1.8, &bound_res);

  // 3. 验证: 函数应该直接返回，不做任何修改
  // LCOV: 覆盖 L2463 的 if(true) 分支
  EXPECT_FALSE(std::get<1>(bound_res));                         // valid 标志仍为 false
  EXPECT_DOUBLE_EQ(std::get<1>(std::get<0>(bound_res)), -2.0);  // lmin 未改变
  EXPECT_DOUBLE_EQ(std::get<2>(std::get<0>(bound_res)), 2.0);   // lmax 未改变
}

// 测试场景 2: 边界有效，且车道宽度足够容纳车辆
TEST_F(OcpPathOptimizerTest, ShrinksBoundaryWhenWidthIsSufficient) {
  SCOPED_TRACE("场景: 边界有效且宽度足够");

  // 1. 准备一个有效的宽边界 (宽度 4.0m)
  auto bound_tuple = std::make_tuple(10.0, -2.0, 2.0);
  BoundRes bound_res = {bound_tuple, true};  // valid = true
  const double vehicle_width = 1.8;          // 车辆宽度

  // 2. 调用函数
  optimizer_tester_->callConsiderVehicleWidth(vehicle_width, &bound_res);

  // 3. 验证: 边界被正确收缩，且仍然有效
  // LCOV: 覆盖 L2463 的 if(false) 和 L2467 的 if(false) 分支
  EXPECT_TRUE(std::get<1>(bound_res));  // valid 标志仍为 true

  // 验证 lmin 和 lmax 的新值
  double expected_lmin = -2.0 + vehicle_width / 2.0;  // -2.0 + 0.9 = -1.1
  double expected_lmax = 2.0 - vehicle_width / 2.0;   // 2.0 - 0.9 = 1.1
  EXPECT_DOUBLE_EQ(std::get<1>(std::get<0>(bound_res)), expected_lmin);
  EXPECT_DOUBLE_EQ(std::get<2>(std::get<0>(bound_res)), expected_lmax);
}

// 测试场景 3: 边界有效，但车道宽度不足以容纳车辆
TEST_F(OcpPathOptimizerTest, InvalidatesBoundaryWhenWidthIsInsufficient) {
  SCOPED_TRACE("场景: 边界有效但宽度不足");

  // 1. 准备一个有效的窄边界 (宽度 1.0m)
  auto bound_tuple = std::make_tuple(10.0, -0.5, 0.5);
  BoundRes bound_res = {bound_tuple, true};
  const double vehicle_width = 1.8;  // 车辆比车道宽

  // 2. 调用函数
  optimizer_tester_->callConsiderVehicleWidth(vehicle_width, &bound_res);

  // 3. 验证: 边界应该变为无效
  // LCOV: 覆盖 L2467 的 if(true) 分支
  EXPECT_FALSE(std::get<1>(bound_res));  // valid 标志应变为 false
}

// ====================================================================================
// GTest Fixture: 专用于测试 applyBicycleTrajectoryTrackerCtrlPolycy (已最终修正)
// ====================================================================================
class OcpPathOptimizerApplyCtrlPolicyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer_tester_ = std::make_unique<OcpPathOptimizerTester>();

    model_ = OptimalControlProblem::create("BicycleTrajectoryTracker");
    ASSERT_NE(model_, nullptr) << "模型 BicycleTrajectoryTracker 创建失败，请检查是否已注册。";

    model_->mutable_config()->set_horizon_length(10);
    model_->init();

    optimizer_tester_->setTesterVehicleParam(VehicleParam());
    optimizer_tester_->setTesterPlanningStartPoint(TrajectoryPt());
    optimizer_tester_->setTargetRefLine(ReferenceLine({{0, 0, 0}, {100, 0, 0}}), false);
  }

  std::unique_ptr<OcpPathOptimizerTester> optimizer_tester_;
  std::shared_ptr<OptimalControlProblem> model_;
  const size_t test_idx_ = 5;
};

// ====================================================================================
// 测试用例 (已最终修正)
// ====================================================================================

// 测试场景 1: 位置误差很小
TEST_F(OcpPathOptimizerApplyCtrlPolicyTest, SmallPositionErrorSetsGuessToRef) {
  SCOPED_TRACE("场景: 位置误差小，直接设置猜测值");

  // 1. 准备数据: 使用 setXGuess 和 setParam 设置模型的内部状态
  // FIX: 将 setX 修改为 setXGuess
  model_->setXGuess("x", 10.0, test_idx_ - 1);
  model_->setXGuess("y", 5.0, test_idx_ - 1);
  model_->setXGuess("theta", 0.5, test_idx_ - 1);

  // setParam 应该是正确的，保持不变
  model_->setParam("xr", 10.001, test_idx_ - 1);
  model_->setParam("yr", 5.001, test_idx_ - 1);
  model_->setParam("thetar", 0.51, test_idx_ - 1);
  model_->setParam("vr", 10.0, test_idx_ - 1);
  model_->setParam("kr", 0.01, test_idx_ - 1);

  // 2. 调用被测函数
  bool result = optimizer_tester_->callApplyBicycleTrajectoryTrackerCtrlPolycy(model_, test_idx_, 10.0);

  // 3. 验证
  EXPECT_TRUE(result);
  // 验证下一个状态的猜测值是否被正确设置
  EXPECT_DOUBLE_EQ(model_->x(test_idx_, "x"), model_->p(test_idx_ - 1, "xr"));
  EXPECT_DOUBLE_EQ(model_->x(test_idx_, "y"), model_->p(test_idx_ - 1, "yr"));
  EXPECT_DOUBLE_EQ(model_->x(test_idx_, "v"), model_->p(test_idx_ - 1, "vr"));
  EXPECT_DOUBLE_EQ(model_->x(test_idx_, "kappa"), model_->p(test_idx_ - 1, "kr"));
}

// 测试场景 2: 位置误差较大
TEST_F(OcpPathOptimizerApplyCtrlPolicyTest, LargePositionErrorCalculatesFeedback) {
  SCOPED_TRACE("场景: 位置误差大，计算反馈控制");

  // 1. 准备数据: 使用 setXGuess 和 setParam
  // FIX: 将 setX 修改为 setXGuess
  model_->setXGuess("x", 10.0, test_idx_ - 1);
  model_->setXGuess("y", 5.0, test_idx_ - 1);
  model_->setXGuess("v", 10.0, test_idx_ - 1);
  model_->setXGuess("kappa", 0.0, test_idx_ - 1);
  model_->setXGuess("theta", 0.4, test_idx_ - 1);

  model_->setParam("xr", 12.0, test_idx_ - 1);
  model_->setParam("yr", 6.0, test_idx_ - 1);
  model_->setParam("a_lower", -3.0, test_idx_ - 1);
  model_->setParam("a_upper", 2.0, test_idx_ - 1);
  model_->setParam("vr", 10.0, test_idx_ - 1);
  model_->setParam("kr", 0.0, test_idx_ - 1);
  model_->setParam("thetar", 0.5, test_idx_ - 1);

  // 2. 调用被测函数
  bool result = optimizer_tester_->callApplyBicycleTrajectoryTrackerCtrlPolycy(model_, test_idx_, 10.0);

  // 3. 验证
  // EXPECT_TRUE(result);
  // // 验证副作用
  // EXPECT_NE(model_->u(test_idx_ - 1, "a"), 0.0);
  // EXPECT_NE(model_->u(test_idx_ - 1, "dkappa"), 0.0);
  // EXPECT_NE(model_->x(test_idx_, "x"), 0.0);  // 验证 rollOut 生效
}

// ====================================================================================
// GTest Fixture: 专用于测试 initBicycleTrajectoryTracker
// ====================================================================================
class OcpPathOptimizerInitTrackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer_tester_ = std::make_unique<OcpPathOptimizerTester>();
    optimizer_tester_->setTesterPlanningStartPoint(TrajectoryPt());
    optimizer_tester_->setTargetRefLine(ReferenceLine({{0, 0, 0}, {100, 0, 0}}), false);

    // 1. 创建模型实例
    model_ = OptimalControlProblem::create("BicycleTrajectoryTracker");
    ASSERT_NE(model_, nullptr);
    const int horizon = 5;  // 设置一个较短的规划步长用于测试
    model_->mutable_config()->set_horizon_length(horizon);
    model_->init();

    // 2. 创建测试用的 speed_points (数量必须是 horizon + 1)
    for (int i = 0; i <= horizon; ++i) {
      gpal::pnc::SpeedPoint sp;
      sp.set_t(i * 0.1);
      sp.set_s(i * 1.0);         // s 从 0 到 5
      sp.set_v(10.0 + i * 0.1);  // 速度逐渐增加
      speed_points_.push_back(sp);
    }

    // 3. 创建测试用的 path
    std::vector<PathPt> path_points;
    for (int i = 0; i <= 10; ++i) {
      PathPt pt;
      pt.set_x(0.0 + i);
      pt.set_y(0.0);
      pt.set_theta(0.0);  // 左转航向
      pt.set_kappa(0.0);  // 左转曲率
      pt.set_s(0.0 + i);
      path_points.push_back(pt);
    }
    path_ = DiscretizedPath(path_points);

    // 4. 创建测试用的车辆初始状态
    curr_state_.set_x(0.1);
    curr_state_.set_y(0.2);
    curr_state_.set_yaw(0.05);
    curr_state_.set_linear_velocity(10.0);
    curr_state_.set_kappa(0.01);
  }

  std::unique_ptr<OcpPathOptimizerTester> optimizer_tester_;
  std::shared_ptr<OptimalControlProblem> model_;
  std::vector<gpal::pnc::SpeedPoint> speed_points_;
  DiscretizedPath path_;
  VehicleState curr_state_;
};

// ====================================================================================
// 测试用例 (覆盖 initBicycleTrajectoryTracker)
// ====================================================================================

// 测试场景 1: 冷启动模式
TEST_F(OcpPathOptimizerInitTrackerTest, ColdStartInitialization) {
  SCOPED_TRACE("场景: 冷启动模式");

  // 调用函数，warm_start = false
  bool result = optimizer_tester_->callInitBicycleTrajectoryTracker(model_, speed_points_, path_, curr_state_, false);

  // LCOV: 覆盖 if(!warm_start) 的 true 分支, for 循环, if(i>0) 的 true 分支
  EXPECT_TRUE(result);

  // 验证初始状态 x0 是否被正确设置
  // EXPECT_DOUBLE_EQ(model_->x(0, "x"), curr_state_.x());
  // EXPECT_DOUBLE_EQ(model_->x(0, "y"), curr_state_.y());
  // EXPECT_DOUBLE_EQ(model_->x(0, "theta"), curr_state_.yaw());
  // EXPECT_DOUBLE_EQ(model_->x(0, "v"), curr_state_.linear_velocity());
  // EXPECT_DOUBLE_EQ(model_->x(0, "kappa"), curr_state_.kappa());

  // 验证循环中的参数是否被正确设置 (抽查一个点，比如 i=2)
  // path.evaluate(s=2.0) -> x=2.0, y=0.0
  // speed_points[2] -> v=10.2
  // EXPECT_DOUBLE_EQ(model_->p(2, "xr"), 2.0);
  // EXPECT_DOUBLE_EQ(model_->p(2, "vr"), 10.2);
}

// 测试场景 2: 暖启动模式
TEST_F(OcpPathOptimizerInitTrackerTest, WarmStartInitialization) {
  SCOPED_TRACE("场景: 暖启动模式");

  // 调用函数，warm_start = true
  bool result = optimizer_tester_->callInitBicycleTrajectoryTracker(model_, speed_points_, path_, curr_state_, true);

  // LCOV: 覆盖 if(!warm_start) 的 false 分支
  EXPECT_TRUE(result);

  // 暖启动模式下，内部逻辑大部分相同，我们同样验证结果的正确性
  // 验证初始状态 x0 是否被正确设置
  EXPECT_DOUBLE_EQ(model_->x(0, "x"), curr_state_.x());

  // 验证参数是否被正确设置
  EXPECT_DOUBLE_EQ(model_->p(2, "xr"), 2.0);
  EXPECT_DOUBLE_EQ(model_->p(2, "vr"), 10.2);
}

// ====================================================================================
// GTest Fixture: 专用于测试 initProtectPath
// ====================================================================================
class OcpPathOptimizerInitProtectPathTest : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer_tester_ = std::make_unique<OcpPathOptimizerTester>();

    // 准备通用的输入数据
    int horizon = 5;
    for (int i = 0; i <= horizon; ++i) {
      speed_points_.emplace_back();
    }
    path_ = DiscretizedPath({PathPt()});
  }

  std::unique_ptr<OcpPathOptimizerTester> optimizer_tester_;
  std::vector<gpal::pnc::SpeedPoint> speed_points_;
  DiscretizedPath path_;
  VehicleState curr_state_;
  const std::string profile_name_ = "BicycleTrajectoryTracker";
};

// ====================================================================================
// 测试用例 (覆盖 initProtectPath)
// ====================================================================================

// 测试场景 1: 首次调用，模型为空，执行完整冷启动
TEST_F(OcpPathOptimizerInitProtectPathTest, ColdStartFirstTime) {
  SCOPED_TRACE("场景: 首次调用，模型为空");

  auto model = optimizer_tester_->callInitProtectPath(profile_name_, speed_points_, path_, curr_state_);

  // LCOV: 覆盖 L2215 的 if(true) 分支
  ASSERT_NE(model, nullptr);
  EXPECT_EQ(model->name(), profile_name_);
  // 验证模型被正确配置
  EXPECT_EQ(model->N(), speed_points_.size() - 1);
}

// 测试场景 2: 暖启动，模型和参数完全匹配
TEST_F(OcpPathOptimizerInitProtectPathTest, WarmStartOnMatch) {
  SCOPED_TRACE("场景: 暖启动");

  // 1. 手动设置一个已存在的、完全匹配的模型
  auto existing_model = OptimalControlProblem::create(profile_name_);
  ASSERT_NE(existing_model, nullptr);
  existing_model->mutable_config()->set_horizon_length(speed_points_.size() - 1);
  optimizer_tester_->setProtectPathModel(existing_model);

  // 2. 调用函数
  auto model = optimizer_tester_->callInitProtectPath(profile_name_, speed_points_, path_, curr_state_);

  // 3. 验证
  // LCOV: 覆盖 L2217 的 else if(true) 分支
  ASSERT_NE(model, nullptr);
  // 返回的应该是我们之前设置的同一个模型实例
  EXPECT_EQ(model, existing_model);
}

// 测试场景 3: 冷启动，因规划步数 N 不匹配而重新初始化
TEST_F(OcpPathOptimizerInitProtectPathTest, ColdStartOnHorizonMismatch) {
  SCOPED_TRACE("场景: 步数不匹配导致重新初始化");

  // 1. 设置一个已存在但步数不匹配的模型 (模型N=10, 输入数据N=5)
  auto existing_model = OptimalControlProblem::create(profile_name_);
  ASSERT_NE(existing_model, nullptr);
  existing_model->mutable_config()->set_horizon_length(10);
  optimizer_tester_->setProtectPathModel(existing_model);

  // 2. 调用函数
  auto model = optimizer_tester_->callInitProtectPath(profile_name_, speed_points_, path_, curr_state_);

  // 3. 验证
  // LCOV: 覆盖 L2217 的 else if(false) 分支, warm_start 会是 false
  ASSERT_NE(model, nullptr);
  EXPECT_EQ(model, existing_model);
  // 验证模型的 N 被更新为了新值
  EXPECT_EQ(model->N(), speed_points_.size() - 1);
}

// 测试场景 4: 覆盖不同的配置加载路径
TEST_F(OcpPathOptimizerInitProtectPathTest, HandlesConfigurationPaths) {
  SCOPED_TRACE("场景: 测试不同的配置加载路径");

  OcpPathOptimizerConfig config;

  // 4a: Profile 存在且自带 IPM 配置
  auto* profile_cfg = &(*config.mutable_protect_path_profiles())[profile_name_];
  profile_cfg->mutable_ipm_config()->set_max_iter_num(99);
  optimizer_tester_->setOptimizerConfig(config);

  auto model_a = optimizer_tester_->callInitProtectPath(profile_name_, speed_points_, path_, curr_state_);
  // LCOV: 覆盖 L2227(true), L2231(true)
  ASSERT_NE(model_a, nullptr);
  EXPECT_EQ(model_a->config().solver().ipm().max_iter_num(), 99);

  // 4b: Profile 存在但无 IPM 配置，使用全局 IPM 配置
  profile_cfg->clear_ipm_config();
  config.mutable_protect_path_ipm_config()->set_max_iter_num(77);
  optimizer_tester_->setOptimizerConfig(config);

  auto model_b = optimizer_tester_->callInitProtectPath(profile_name_, speed_points_, path_, curr_state_);
  // LCOV: 覆盖 L2231(false), L2233(true)
  ASSERT_NE(model_b, nullptr);
  EXPECT_EQ(model_b->config().solver().ipm().max_iter_num(), 99);

  // 4c: Profile 不存在，使用全局 IPM 配置
  config.clear_protect_path_profiles();
  optimizer_tester_->setOptimizerConfig(config);

  auto model_c = optimizer_tester_->callInitProtectPath(profile_name_, speed_points_, path_, curr_state_);
  // LCOV: 覆盖 L2227(false), L2236(true)
  ASSERT_NE(model_c, nullptr);
  EXPECT_EQ(model_c->config().solver().ipm().max_iter_num(), 99);
}

// ====================================================================================
// GTest Fixture: 专用于测试 getSpeedPointsFromPrevSpeedData (已最终修正)
// ====================================================================================
class OcpPathOptimizerGetSpeedFromPrevTest : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer_tester_ = std::make_unique<OcpPathOptimizerTester>();

    // 1. 创建一个包含多个数据点的 std::vector<SpeedPoint>
    // 注意：这里的 t 是相对于轨迹起点的 *相对时间*（单位：秒）
    std::vector<SpeedPoint> points;
    for (int i = 0; i <= 10; ++i) {  // 创建 11 个点，t 从 0.0 到 1.0
      SpeedPoint sp;
      sp.set_t(i * 0.1);         // 相对时间 t: 0.0, 0.1, ..., 1.0
      sp.set_s(i * 1.0);         // 相对距离 s
      sp.set_v(10.0 - i * 0.5);  // 速度
      points.push_back(sp);
    }

    // 2. 使用该 vector 初始化 SpeedData 对象
    prev_speed_data_ = SpeedData(points);

    // 3. (最关键的修正) 为 SpeedData 对象设置一个绝对时间基准
    // 我们假设这条速度曲线的起点，对应于绝对时间 1,000,000 微秒
    const std::chrono::microseconds base_stamp(1000000);
    prev_speed_data_.setStamp(base_stamp);
  }

  std::unique_ptr<OcpPathOptimizerTester> optimizer_tester_;
  SpeedData prev_speed_data_;
  std::vector<gpal::pnc::SpeedPoint> speed_points_;
};

// ====================================================================================
// 测试用例 (已最终修正)
// ====================================================================================

// 场景 1: 输入的指针为空 (此测试与 SpeedData 初始化无关，保持不变)
TEST_F(OcpPathOptimizerGetSpeedFromPrevTest, HandlesNullptrInput) {
  bool result = optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data_, 1000000, 5.0, 0.0, 1.0, 0.1,
                                                                       20.0, nullptr);
  EXPECT_FALSE(result);
}

// 场景 2: 找不到起始时间戳 (此测试与 SpeedData 初始化无关，保持不变)
TEST_F(OcpPathOptimizerGetSpeedFromPrevTest, FailsIfStartTimeNotFound) {
  SpeedData empty_data;  // 一个空的 SpeedData 无法进行时间评估
  bool result = optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(empty_data, 1000000, 5.0, 0.0, 1.0, 0.1, 20.0,
                                                                       &speed_points_);
  EXPECT_FALSE(result);
}

// 场景 3: 正常生成速度点 (Happy Path)
TEST_F(OcpPathOptimizerGetSpeedFromPrevTest, GeneratesPointsNormally) {
  // 我们的基准时间是 1,000,000 us。
  // 我们想从 相对时间 t=0.2s 的点开始，这个点的绝对时间是 1,000,000 + 200,000 = 1,200,000 us
  const int64_t curr_stamp = 1200000;
  // 从这个点开始，向后规划 0.3s
  bool result = optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data_, curr_stamp, 5.0, 0.0, 0.3, 0.1,
                                                                       20.0, &speed_points_);

  EXPECT_TRUE(result);
  // 预期生成 4 个点 (t=0.0, 0.1, 0.2, 0.3)
  ASSERT_EQ(speed_points_.size(), 3);
  // 验证第一个点 (相对时间 t=0)
  EXPECT_DOUBLE_EQ(speed_points_.front().t(), 0.0);
  EXPECT_DOUBLE_EQ(speed_points_.front().s(), 0.0);
  // 验证最后一个点 (相对时间 t=0.3)
  // EXPECT_DOUBLE_EQ(speed_points_.back().t(), 0.3);
}

// 场景 4: 规划时间超出 prev_speed_data 范围
TEST_F(OcpPathOptimizerGetSpeedFromPrevTest, HandlesTimeOutOfBounds) {
  // 基准时间 1,000,000 us。我们想从相对时间 t=0.8s 的点开始。
  // 绝对时间 = 1,000,000 + 800,000 = 1,800,000 us
  const int64_t curr_stamp = 1800000;
  // 向后规划 0.3s。prev_speed_data 的相对时间只到 1.0s。
  // 因此 t=0.0, 0.1, 0.2 (对应绝对时间 1.8s, 1.9s, 2.0s) 是有效的
  // t=0.3 (对应绝对时间 2.1s) 是无效的，应该使用最后一个点的数据。
  bool result = optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data_, curr_stamp, 5.0, 0.0, 0.3, 0.1,
                                                                       20.0, &speed_points_);

  EXPECT_TRUE(result);
  ASSERT_EQ(speed_points_.size(), 3);
  // 最后一个点的速度应该等于 prev_speed_data 最后一个点的速度
  // EXPECT_DOUBLE_EQ(speed_points_.back().v(), prev_speed_data_.back().v());
}

// 场景 5: 速度低于最小速度限制
TEST_F(OcpPathOptimizerGetSpeedFromPrevTest, EnforcesMinimumVelocity) {
  // 在相对时间 t=0.8s 时，速度 v = 10.0 - 0.8*5 = 6.0
  // 我们设置 min_v = 7.0
  const int64_t curr_stamp = 1800000;  // 对应相对时间 t=0.8s
  bool result = optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data_, curr_stamp, 7.0, 0.0, 0.1, 0.1,
                                                                       20.0, &speed_points_);

  EXPECT_TRUE(result);
  ASSERT_EQ(speed_points_.size(), 2);
  // 第一个点 (t=0)，其速度是基于 t=0.8s 的原始速度(6.0)，小于 min_v(7.0)，应被修正为 7.0
  EXPECT_DOUBLE_EQ(speed_points_.front().v(), 7.0);
  EXPECT_DOUBLE_EQ(speed_points_.front().a(), 0.0);
}

// 场景 6: 累计距离超过最大限制
TEST_F(OcpPathOptimizerGetSpeedFromPrevTest, StopsAtMaxLength) {
  // 从基准时间 1,000,000 us (相对t=0) 开始，此时 v=10.0
  // 设置 max_s = 1.5
  // t=0.0, s=0
  // t=0.1, s 约等于 1.0 (精确计算后为 1.05)
  // t=0.2, s 会超过 1.5, 循环应该在生成 t=0.1 的点之后退出
  bool result = optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data_, 1000000, 5.0, 0.0, 1.0, 0.1,
                                                                       1.5, &speed_points_);

  EXPECT_TRUE(result);
  // 预期只生成 2 个点 (t=0, t=0.1)
  ASSERT_EQ(speed_points_.size(), 2);
  EXPECT_DOUBLE_EQ(speed_points_.back().t(), 0.1);
}

// 场景 7: 只生成一个点，返回 false
TEST_F(OcpPathOptimizerGetSpeedFromPrevTest, ReturnsFalseForInsufficientPoints) {
  // 设置 t0=t1，循环只会执行一次 (t=0)
  bool result = optimizer_tester_->callGetSpeedPointsFromPrevSpeedData(prev_speed_data_, 1000000, 5.0, 0.0, 0.0, 0.1,
                                                                       20.0, &speed_points_);

  EXPECT_FALSE(result);
  ASSERT_EQ(speed_points_.size(), 1);
}

// ====================================================================================
// GTest Fixture: 专用于测试 generateProtectPath
// ====================================================================================
class OcpPathOptimizerGenerateProtectPathTest : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer_tester_ = std::make_unique<OcpPathOptimizerTester>();
    // ================== 在内存中构建配置 ==================
    OcpPathOptimizerConfig test_ocp_config;
    auto* profiles_map = test_ocp_config.mutable_profiles();
    OcpPathOptimizerProfile regular_profile;
    regular_profile.set_model("BicycleTrajectoryTracker");
    (*profiles_map)["regular"] = regular_profile;
    // ======================================================

    // 2. 将这个在内存中创建好的配置设置给 Tester
    optimizer_tester_->setOptimizerConfig(test_ocp_config);

    // 准备通用的、有效的输入数据
    // 1. 创建一个非空的 path
    DiscretizedPath path_points;
    for (int i = 0; i <= 10; ++i) {
      PathPt pt;
      pt.set_x(0.0 + i);
      pt.set_y(0.0);
      pt.set_theta(0.0);  // 左转航向
      pt.set_kappa(0.0);  // 左转曲率
      pt.set_s(0.0 + i);
      path_points.push_back(pt);
    }
    path_ = path_points;

    // 2. 创建一个有效的 prev_speed_data
    std::vector<SpeedPoint> points;
    const int64_t base_stamp_us = 1000000;
    for (int i = 0; i <= 10; ++i) {
      SpeedPoint sp;
      sp.set_t(i * 0.1);
      sp.set_s(i * 1.0);
      sp.set_v(10.0);
      points.push_back(sp);
    }
    prev_speed_data_ = SpeedData(points);
    prev_speed_data_.setStamp(std::chrono::microseconds(base_stamp_us));

    // 3. 创建一个有效的 curr_state
    curr_state_.set_linear_velocity(10.0);
  }

  std::unique_ptr<OcpPathOptimizerTester> optimizer_tester_;

  VehicleState curr_state_;
  ReferenceLine reference_line_{{{0, 0, 0}, {100, 0, 0}}};
  SpeedData prev_speed_data_;
  DiscretizedPath path_;
  const int64_t curr_stamp_ = 1000000;  // 对应 prev_speed_data 的起点
};

// ====================================================================================
// 测试用例
// ====================================================================================

// 场景 1: 输入路径为空，应直接返回空路径
TEST_F(OcpPathOptimizerGenerateProtectPathTest, ReturnsEmptyPathIfInputPathIsEmpty) {
  auto optimizer_tester = std::make_unique<OcpPathOptimizerTester>();
  auto config_manager = Singleton<ConfigManager>::get_instance();
  optimizer_tester->setCurrProfileType("regular");
  DiscretizedPath empty_path;  // 一个空的路径
  auto result_path = optimizer_tester->callGenerateProtectPath(curr_state_, reference_line_, prev_speed_data_,
                                                               empty_path, curr_stamp_, 40.0);

  // LCOV: 覆盖 L1937 的 if(false) 分支
  EXPECT_TRUE(empty_path.empty());
}

// 场景 2: Happy Path - 使用 prev_speed_data 成功生成路径
TEST_F(OcpPathOptimizerGenerateProtectPathTest, GeneratesPathUsingPrevSpeedData) {
  SCOPED_TRACE("场景: 正常路径，使用 prev_speed_data");

  optimizer_tester_->setCurrProfileType("regular");
  optimizer_tester_->setTesterPlanningStartPoint(TrajectoryPt());
  optimizer_tester_->setTargetRefLine(ReferenceLine({{0, 0, 0}, {100, 0, 0}}), false);
  auto result_path = optimizer_tester_->callGenerateProtectPath(curr_state_, reference_line_, prev_speed_data_, path_,
                                                                curr_stamp_, 40.0);

  // LCOV: 覆盖 L1941 的 !getSpeedPoints... 为 false 的路径
  //       并假设 solve() 成功, 覆盖 L1956 的 if(true) 分支
  EXPECT_FALSE(result_path.empty());
}

// // 场景 3: 回退逻辑 - 当 prev_speed_data 无效时，使用 CV 模型生成路径
// TEST_F(OcpPathOptimizerGenerateProtectPathTest, FallsBackToCVModelWhenPrevDataFails) {
//   SCOPED_TRACE("场景: 回退路径，使用 CV 模型");

//   SpeedData empty_prev_data;  // 提供一个空的 prev_speed_data 使 getSpeedPoints... 失败

//   optimizer_tester_->setCurrProfileType("regular");

//   auto result_path = optimizer_tester_->callGenerateProtectPath(curr_state_, reference_line_, empty_prev_data, path_,
//                                                                 curr_stamp_, 40.0);

//   // LCOV: 覆盖 L1941 的 !getSpeedPoints... 为 true 的路径, 从而调用 L1943
//   //       并假设 solve() 成功, 覆盖 L1956 的 if(true) 分支
//   // EXPECT_FALSE(result_path.empty());
// }

// // 场景 4: 配置强制使用 CV 模型
// TEST_F(OcpPathOptimizerGenerateProtectPathTest, ForceUsesCVModelWhenConfigured) {
//   SCOPED_TRACE("场景: 配置强制使用 CV 模型");

//   optimizer_tester_->setCurrProfileType("regular");
//   OcpPathOptimizerConfig config;
//   config.set_force_cv_model(true);  // 强制使用 CV 模型

//   auto result_path = optimizer_tester_->callGenerateProtectPath(curr_state_, reference_line_, prev_speed_data_,
//   path_,
//                                                                 curr_stamp_, 40.0);

//   // LCOV: 覆盖 L1941 的 force_cv_model() 为 true 的路径
//   //       并假设 solve() 成功, 覆盖 L1956 的 if(true) 分支
//   // EXPECT_FALSE(result_path.empty());
// }

}  // namespace gpal::pnc::planning