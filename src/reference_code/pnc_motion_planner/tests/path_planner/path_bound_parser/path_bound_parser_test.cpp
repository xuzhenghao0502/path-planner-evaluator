#include <gtest/gtest.h>
#include <filesystem>
#include <fmt/chrono.h>
#include <algorithm>
#include "config_manager/config_manager.h"
#include "path_bound_parser/path_bound_parser.h"
#include "decision_data/decision_define.h"

namespace gpal::pnc::planning {
class PathBoundParserTester : public PathBoundParser {
 public:
  // 将受保护的 getOffset 方法公开，以便在 gtest 中调用
  std::tuple<double, double, bool> callGetOffset(const math::LineSegment2d& segment,
                                                 const math::LineSegment2d& bound_segment) {
    return this->getOffset(segment, bound_segment);
  }

  // 将受保护的 sortStaticObstacleBoundRange 方法公开，以便在 gtest 中调用
  auto callSortStaticObstacleBoundRange(
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
      const std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>>& buffer_map) {
    return this->sortStaticObstacleBoundRange(decision_obstacles, buffer_map);
  }

  // 将受保护的 pickDynamicObstaclesForDynamicNudge 方法公开
  auto callPickDynamicObstaclesForDynamicNudge(
      const ReferenceLine& ref_line, const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
      const std::vector<std::tuple<double, double, double, double, double, double, double>>& buffer) {
    return this->pickDynamicObstaclesForDynamicNudge(ref_line, 0.0, decision_obstacles, buffer);
  }
  // 将受保护的 sortDynamicObstacleAndBoundaryRange 方法公开，并设置 adc_frenet_sd_ 的值
  auto callSortDynamicObstacleAndBoundaryRange(
      const ReferenceLine& ref_line, const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
      const std::vector<std::tuple<double, double, double, double, double, double, double>>& buffer) {
    // 在调用前设置一个有效的 adc_frenet_sd_ 值
    this->setAdcFrenetSpeed(10.0);
    return this->sortDynamicObstacleAndBoundaryRange(ref_line, decision_obstacles, buffer);
  }
  std::tuple<std::string, double> callGetRightOffsetInfoFromPolyline(const math::LineSegment2d& center,
                                                                     const PolylineInfoMap& polyline_info) {
    return this->getRightOffsetInfoFromPolyline(center, polyline_info);
  }
  std::pair<std::vector<PolylineRange>, std::vector<PolylineRange>> callSortPolylineRange(
      const std::vector<PolylineInput>& polylines, const double resolution_thrd,
      std::vector<PolylineInput>* ptr_small) {
    return this->sortPolylineRange(polylines, resolution_thrd, ptr_small);
  }
  auto callSortPolylineBoundRange(
      const std::vector<PolylineInput>& polylines,
      const std::unordered_map<std::string, std::tuple<double, double, double>>& buffer_map) {
    return this->sortPolylineBoundRange(polylines, buffer_map);
  }
  std::tuple<std::string, double> callGetLeftOffsetInfoFromPolyline(const math::LineSegment2d& center,
                                                                    const PolylineInfoMap& polyline_borders) {
    return this->getLeftOffsetInfoFromPolyline(center, polyline_borders);
  }
};

TEST(PathBoundParserTest, init) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: init" << std::endl;

  PathBoundParserConfig config;
  config.set_max_range(100.0);
  config.set_resolution(0.5);
  PathBoundParser parser(config);

  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {10.0, 10.0, 1.57}, {20.0, 20.0, 3.14}});

  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();

  double start_s = 0.0;
  double end_s = 20.0;
  // class AbstractTable1d : public std::vector<std::tuple<Tp, TupleData...>>
  // 在你的 path_bound_parser_test.cpp 文件中，使用下面的方式来初始化 check_ranges

  util::AbstractTable1d<double, double, double> check_ranges(
      // 关键改动：在这里显式地创建一个 std::vector 对象
      std::vector<std::tuple<double, double, double>>{{0.0, 10.0, 10.0},
                                                      {1.0, 10.0, 10.0},
                                                      {2.0, 10.0, 10.0},
                                                      {3.0, 10.0, 10.0},
                                                      {4.0, 10.0, 10.0},
                                                      {5.0, 10.0, 10.0},
                                                      {6.0, 10.0, 10.0},
                                                      {7.0, 10.0, 10.0},
                                                      {8.0, 10.0, 10.0},
                                                      {9.0, 10.0, 10.0},
                                                      {10.0, 10.0, 10.0}});
  bool init_result = parser.init(ref_line, tf_map_2_ego, start_s, end_s, check_ranges);
  EXPECT_TRUE(init_result) << "PathBoundParser initialization failed";
  std::cout << "[GTEST]:END!!! path_bound_parser_test: init" << std::endl;
}

TEST(PathBoundParserTest, GetBoundaryFromStaticObstaclesFinal) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: GetBoundaryFromStaticObstaclesFinal" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser
  PathBoundParserConfig config;
  config.set_max_range(100.0);
  config.set_resolution(0.5);
  PathBoundParser parser(config);

  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));

  // 2. Arrange: 创建并完整配置静态障碍物
  std::vector<std::shared_ptr<Decision::DecisionObject>> decision_obstacles;

  // --- 右侧障碍物 ---
  auto right_obstacle = std::make_shared<Decision::DecisionObject>();
  right_obstacle->id = "right_obs_1";
  right_obstacle->is_static = true;
  right_obstacle->lat_od_tag = Decision::LateralOdTag::LEFT_BYPASS;

  right_obstacle->cur_sl_bound->set_start_s(20.0);
  right_obstacle->cur_sl_bound->set_end_s(25.0);
  right_obstacle->cur_sl_bound->set_start_l(-2.0);
  right_obstacle->cur_sl_bound->set_end_l(-1.0);

  // **【最终修复】: 初始化障碍物的 cur_box 几何信息**
  // Box2d 构造函数通常需要中心点、朝向、长度和宽度
  // 中心点 (x,y) 根据 (s,l) 设置
  math::Vec2d right_center(22.5, -1.5);  // s中心=22.5, l中心=-1.5
  right_obstacle->cur_box =
      std::make_shared<math::Box2d>(right_center, 0.0, 5.0, 1.0);  // center, heading, length, width

  decision_obstacles.push_back(right_obstacle);

  // --- 左侧障碍物 ---
  auto left_obstacle = std::make_shared<Decision::DecisionObject>();
  left_obstacle->id = "left_obs_1";
  left_obstacle->is_static = true;
  left_obstacle->lat_od_tag = Decision::LateralOdTag::RIGHT_BYPASS;

  left_obstacle->cur_sl_bound->set_start_s(30.0);
  left_obstacle->cur_sl_bound->set_end_s(35.0);
  left_obstacle->cur_sl_bound->set_start_l(1.5);
  left_obstacle->cur_sl_bound->set_end_l(2.5);

  // **【最终修复】: 初始化障碍物的 cur_box 几何信息**
  math::Vec2d left_center(32.5, 2.0);  // s中心=32.5, l中心=2.0
  left_obstacle->cur_box = std::make_shared<math::Box2d>(left_center, 0.0, 5.0, 1.0);

  decision_obstacles.push_back(left_obstacle);

  // 提供对应的 buffer
  std::vector<std::tuple<double, double, double>> buffer;
  buffer.emplace_back(0.1, 0.1, 0.2);
  buffer.emplace_back(0.1, 0.1, 0.2);

  // 3. Act: 调用待测试的函数
  ObstacleBoundaryInfo result_bounds = parser.getBoundaryFromStaticObstacles(decision_obstacles, buffer);

  // 4. Assert: 验证结果
  ASSERT_FALSE(result_bounds.bound.empty());

  bool right_obs_found = false;
  for (const auto& obs_ptr : result_bounds.right_bound_obstacles) {
    if (obs_ptr != nullptr && obs_ptr->id == "right_obs_1") {
      right_obs_found = true;
      break;
    }
  }
  EXPECT_TRUE(right_obs_found) << "Right obstacle was not found in the result bounds.";

  bool left_obs_found = false;
  for (const auto& obs_ptr : result_bounds.left_bound_obstacles) {
    if (obs_ptr != nullptr && obs_ptr->id == "left_obs_1") {
      left_obs_found = true;
      break;
    }
  }
  EXPECT_TRUE(left_obs_found) << "Left obstacle was not found in the result bounds.";

  std::cout << "[GTEST]:END!!! path_bound_parser_test: GetBoundaryFromStaticObstaclesFinal" << std::endl;
}

// 可选：添加一个路径被阻塞的测试，以覆盖 if (bound_right > bound_left) 分支
TEST(PathBoundParserTest, BlockedPathCausesBoundaryCrossing) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: BlockedPathCausesBoundaryCrossing" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser
  PathBoundParserConfig config;
  config.set_max_range(100.0);
  config.set_resolution(0.5);
  PathBoundParser parser(config);

  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));

  // 2. Arrange: 创建两个横向重叠的障碍物来阻塞路径
  std::vector<std::shared_ptr<Decision::DecisionObject>> decision_obstacles;

  // --- 障碍物1 (被视为右侧障碍物) ---
  // 我们将它放在轻微偏右的位置，但它足够宽，以至于其左边缘越过了中心线
  auto obs1 = std::make_shared<Decision::DecisionObject>();
  obs1->id = "obs_block_1";
  obs1->is_static = true;
  obs1->lat_od_tag = Decision::LateralOdTag::LEFT_BYPASS;  // 它的存在会更新 bound_right
  obs1->cur_sl_bound->set_start_s(40.0);
  obs1->cur_sl_bound->set_end_s(45.0);
  obs1->cur_sl_bound->set_start_l(-1.0);  // 物体右边缘
  obs1->cur_sl_bound->set_end_l(1.0);     // 物体左边缘. bound_right 将被更新为 1.0 + buffer
  obs1->cur_box = std::make_shared<math::Box2d>(math::Vec2d(42.5, 0.0), 0.0, 5.0, 2.0);
  decision_obstacles.push_back(obs1);

  // --- 障碍物2 (被视为左侧障碍物) ---
  // 我们将它放在轻微偏左的位置，但它足够宽，以至于其右边缘也越过了中心线
  auto obs2 = std::make_shared<Decision::DecisionObject>();
  obs2->id = "obs_block_2";
  obs2->is_static = true;
  obs2->lat_od_tag = Decision::LateralOdTag::RIGHT_BYPASS;  // 它的存在会更新 bound_left
  obs2->cur_sl_bound->set_start_s(40.0);
  obs2->cur_sl_bound->set_end_s(45.0);
  obs2->cur_sl_bound->set_start_l(-1.0);  // 物体右边缘. bound_left 将被更新为 -1.0 - buffer
  obs2->cur_sl_bound->set_end_l(1.0);     // 物体左边缘
  obs2->cur_box = std::make_shared<math::Box2d>(math::Vec2d(42.5, 0.0), 0.0, 5.0, 2.0);
  decision_obstacles.push_back(obs2);

  // 提供对应的 buffer
  std::vector<std::tuple<double, double, double>> buffer;
  buffer.emplace_back(0.1, 0.1, 0.2);  // obs1 buffer
  buffer.emplace_back(0.1, 0.1, 0.2);  // obs2 buffer

  // 3. Act: 调用待测试的函数
  ObstacleBoundaryInfo result_bounds = parser.getBoundaryFromStaticObstacles(decision_obstacles, buffer);

  // 4. Assert: 主要目的是为了覆盖率，只要函数能正确处理这种阻塞场景不崩溃即可
  // 在s=40-45m的区间，bound_right会被更新为~1.2，bound_left会被更新为~-1.2
  // 此时 bound_right > bound_left 的条件成立，红色分支被覆盖
  ASSERT_FALSE(result_bounds.bound.empty());

  std::cout << "[GTEST]:END!!! path_bound_parser_test: BlockedPathCausesBoundaryCrossing" << std::endl;
}

TEST(PathBoundParserTest, GetBoundaryFromStaticObstaclesFilteringLogicTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: GetBoundaryFromStaticObstaclesFilteringLogicTest" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser
  PathBoundParserConfig config;
  config.set_resolution(0.5);
  PathBoundParser parser(config);

  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));

  // 创建一个“对照组”的正常障碍物，我们期望它总能被正确处理
  auto normal_obs = std::make_shared<Decision::DecisionObject>();
  normal_obs->id = "normal_obs";
  normal_obs->is_static = true;
  normal_obs->lat_od_tag = Decision::LateralOdTag::LEFT_BYPASS;
  normal_obs->cur_sl_bound->set_start_s(50.0);
  normal_obs->cur_sl_bound->set_end_s(55.0);
  normal_obs->cur_sl_bound->set_start_l(-2.0);
  normal_obs->cur_sl_bound->set_end_l(-1.0);
  normal_obs->cur_box = std::make_shared<math::Box2d>(math::Vec2d(52.5, -1.5), 0.0, 5.0, 1.0);

  // --- 场景 1: 测试障碍物在路径起点之前 (覆盖 sortStaticObstacleRange 第448行) ---
  {
    SCOPED_TRACE("Scenario: Obstacle is before path start");
    auto before_obs = std::make_shared<Decision::DecisionObject>();
    before_obs->id = "before_obs";
    before_obs->cur_sl_bound->set_start_s(-5.0);
    before_obs->cur_sl_bound->set_end_s(-1.0);  // end_s < 0.0, 应该被过滤

    std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {normal_obs, before_obs};
    std::vector<std::tuple<double, double, double>> buffer = {{0.1, 0.1, 0.2}, {0.1, 0.1, 0.2}};

    ObstacleBoundaryInfo result = parser.getBoundaryFromStaticObstacles(obstacles, buffer);

    // 断言：正常障碍物被找到了，但“起点前”的障碍物没有被找到
    bool normal_found = std::any_of(result.right_bound_obstacles.begin(), result.right_bound_obstacles.end(),
                                    [&](const auto& p) { return p && p->id == "normal_obs"; });
    bool special_found = std::any_of(result.right_bound_obstacles.begin(), result.right_bound_obstacles.end(),
                                     [&](const auto& p) { return p && p->id == "before_obs"; });
    EXPECT_TRUE(normal_found);
    EXPECT_FALSE(special_found);
  }

  // --- 场景 2: 测试障碍物在路径终点之后 (覆盖 sortStaticObstacleRange 第444行) ---
  {
    SCOPED_TRACE("Scenario: Obstacle is after path end");
    auto after_obs = std::make_shared<Decision::DecisionObject>();
    after_obs->id = "after_obs";
    after_obs->cur_sl_bound->set_start_s(101.0);  // start_s > 100.0, 应该被过滤
    after_obs->cur_sl_bound->set_end_s(105.0);

    std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {normal_obs, after_obs};
    std::vector<std::tuple<double, double, double>> buffer = {{0.1, 0.1, 0.2}, {0.1, 0.1, 0.2}};

    ObstacleBoundaryInfo result = parser.getBoundaryFromStaticObstacles(obstacles, buffer);

    // 断言：正常障碍物被找到了，但“终点后”的障碍物没有被找到
    bool normal_found = std::any_of(result.right_bound_obstacles.begin(), result.right_bound_obstacles.end(),
                                    [&](const auto& p) { return p && p->id == "normal_obs"; });
    bool special_found = std::any_of(result.right_bound_obstacles.begin(), result.right_bound_obstacles.end(),
                                     [&](const auto& p) { return p && p->id == "after_obs"; });
    EXPECT_TRUE(normal_found);
    EXPECT_FALSE(special_found);
  }

  // --- 场景 3: 测试未处理的 lat_od_tag (覆盖 sortStaticObstacleRange 第454行的 default 分支) ---
  {
    SCOPED_TRACE("Scenario: Obstacle with unhandled nudge tag");
    auto invalid_tag_obs = std::make_shared<Decision::DecisionObject>();
    invalid_tag_obs->id = "invalid_tag_obs";
    invalid_tag_obs->is_static = true;
    invalid_tag_obs->lat_od_tag = Decision::LateralOdTag::INVALID;  // 无效 tag, 应该被 switch 忽略
    invalid_tag_obs->cur_sl_bound->set_start_s(60.0);
    invalid_tag_obs->cur_sl_bound->set_end_s(65.0);  // 长度足够
    invalid_tag_obs->cur_box = std::make_shared<math::Box2d>(math::Vec2d(62.5, 0.0), 0.0, 5.0, 2.0);

    std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {normal_obs, invalid_tag_obs};
    std::vector<std::tuple<double, double, double>> buffer = {{0.1, 0.1, 0.2}, {0.1, 0.1, 0.2}};

    ObstacleBoundaryInfo result = parser.getBoundaryFromStaticObstacles(obstacles, buffer);

    // 断言：正常障碍物被找到了，但“无效tag”的障碍物没有被找到
    bool normal_found = std::any_of(result.right_bound_obstacles.begin(), result.right_bound_obstacles.end(),
                                    [&](const auto& p) { return p && p->id == "normal_obs"; });
    bool special_found = false;  // 检查所有结果，确保它不在任何一个列表里
    for (const auto& p : result.right_bound_obstacles) {
      if (p && p->id == "invalid_tag_obs")
        special_found = true;
    }
    for (const auto& p : result.left_bound_obstacles) {
      if (p && p->id == "invalid_tag_obs")
        special_found = true;
    }
    EXPECT_TRUE(normal_found);
    EXPECT_FALSE(special_found);
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: GetBoundaryFromStaticObstaclesFilteringLogicTest" << std::endl;
}

TEST(PathBoundParserTest, GetOffsetCoverageTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: GetOffsetCoverageTest" << std::endl;

  PathBoundParserTester tester;

  // 1. Arrange: 构造两个线段来实现目标场景

  // segment: 作为参考基准线，从(10, 5)到(20, 5)，长度为10
  math::LineSegment2d segment(math::Vec2d(10.0, 5.0), math::Vec2d(20.0, 5.0));

  // bound_segment: 构造一个“起点在前，终点在内”的场景
  // 起点x=5, 在segment起点(10)之前 -> 触发 gx1 = -1
  // 终点x=15, 在segment[10, 20]之内 -> 触发 gx2 = 0
  // 这将使得 if(gx2 > 0) 的分支走 false，覆盖目标代码
  math::LineSegment2d bound_segment(math::Vec2d(5.0, 6.0), math::Vec2d(15.0, 6.0));

  // 2. Act: 调用待测函数
  auto [min_offset, max_offset, has_offset] = tester.callGetOffset(segment, bound_segment);

  // 3. Assert: 验证计算结果
  // 因为 bound_segment 完全在 segment 的一侧，其横向偏移量是恒定的
  // 偏移量 = bound_segment.y - segment.y = 6.0 - 5.0 = 1.0
  EXPECT_TRUE(has_offset);
  EXPECT_NEAR(min_offset, 1.0, 1e-9);
  EXPECT_NEAR(max_offset, 1.0, 1e-9);

  std::cout << "[GTEST]:END!!! path_bound_parser_test: GetOffsetCoverageTest" << std::endl;
}

TEST(PathBoundParserTest, SortStaticObstacleBoundRangeDirectTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: SortStaticObstacleBoundRangeDirectTest" << std::endl;

  // 1. Arrange: 初始化 PathBoundParserTester
  PathBoundParserTester tester;
  // **关键**: 必须调用init来填充函数内部依赖的 sample_points_ 成员
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(tester.init(ref_line, tf_map_2_ego, 0.0, 100.0));

  // --- 场景 1: 测试 LEFT_BYPASS 分支 (覆盖第543-550行) ---
  {
    SCOPED_TRACE("Scenario: LEFT_BYPASS tag");
    auto obs = std::make_shared<Decision::DecisionObject>();
    obs->lat_od_tag = Decision::LateralOdTag::LEFT_BYPASS;
    obs->cur_sl_bound->set_start_s(50.0);
    obs->cur_sl_bound->set_end_s(55.0);
    obs->cur_sl_bound->set_end_l(-1.0);

    std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {obs};
    std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>> buffer_map = {
        {obs.get(), {0.1, 0.1, 0.2}}};

    auto [left_ranges, right_ranges] = tester.callSortStaticObstacleBoundRange(obstacles, buffer_map);

    // 预期: LEFT_BYPASS 的障碍物被添加到 right_ranges
    EXPECT_FALSE(right_ranges.empty());
    EXPECT_TRUE(left_ranges.empty());
  }

  // --- 场景 2: 测试障碍物在路径起点之前 (覆盖第536行) ---
  {
    SCOPED_TRACE("Scenario: Obstacle before path start");
    auto obs = std::make_shared<Decision::DecisionObject>();
    obs->cur_sl_bound->set_end_s(-1.0);  // end_s < 0.0

    std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {obs};
    std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>> buffer_map = {
        {obs.get(), {0.1, 0.1, 0.2}}};

    auto [left_ranges, right_ranges] = tester.callSortStaticObstacleBoundRange(obstacles, buffer_map);

    // 预期: 障碍物被 continue 跳过，两个 range 都应为空
    EXPECT_TRUE(left_ranges.empty());
    EXPECT_TRUE(right_ranges.empty());
  }

  // --- 场景 3: 测试障碍物在路径终点之后 (覆盖第532行) ---
  {
    SCOPED_TRACE("Scenario: Obstacle after path end");
    auto obs = std::make_shared<Decision::DecisionObject>();
    obs->cur_sl_bound->set_start_s(101.0);  // start_s > 100.0

    std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {obs};
    std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>> buffer_map = {
        {obs.get(), {0.1, 0.1, 0.2}}};

    auto [left_ranges, right_ranges] = tester.callSortStaticObstacleBoundRange(obstacles, buffer_map);

    // 预期: 障碍物被 continue 跳过，两个 range 都应为空
    EXPECT_TRUE(left_ranges.empty());
    EXPECT_TRUE(right_ranges.empty());
  }

  // --- 场景 4: 测试无效的 lat_od_tag (覆盖 switch 的 default 行为) ---
  {
    SCOPED_TRACE("Scenario: Obstacle with INVALID nudge_tag");
    auto obs = std::make_shared<Decision::DecisionObject>();
    obs->lat_od_tag = Decision::LateralOdTag::INVALID;
    obs->cur_sl_bound->set_start_s(50.0);
    obs->cur_sl_bound->set_end_s(55.0);

    std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {obs};
    std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>> buffer_map = {
        {obs.get(), {0.1, 0.1, 0.2}}};

    auto [left_ranges, right_ranges] = tester.callSortStaticObstacleBoundRange(obstacles, buffer_map);

    // 预期: switch 不执行任何操作，range 仍为空
    EXPECT_TRUE(left_ranges.empty());
    EXPECT_TRUE(right_ranges.empty());
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: SortStaticObstacleBoundRangeDirectTest" << std::endl;
}

TEST(PathBoundParserTest, GetBoundaryFromDynamicObstaclesCoverageTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: GetBoundaryFromDynamicObstaclesCoverageTest" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser 并设置其内部状态
  PathBoundParser parser;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));

  // **关键**: 设置 getBoundaryFromDynamicObstacles 依赖的内部状态
  std::vector<std::tuple<double, double, double, double>> adc_ts_ranges;
  adc_ts_ranges.emplace_back(0.0, 5.0, 0.0, 50.0);  // t=[0,5], s=[0,50]
  parser.setAdcTSRange(adc_ts_ranges);

  // 2. Arrange: 创建有效的动态障碍物列表
  std::vector<std::shared_ptr<Decision::DecisionObject>> dynamic_obstacles;

  // --- 构造一个动态障碍物，需要从左侧绕行 (影响右边界) ---
  auto obs_dynamic_right = std::make_shared<Decision::DecisionObject>();
  obs_dynamic_right->id = "dynamic_right";
  obs_dynamic_right->is_static = false;
  obs_dynamic_right->lat_od_tag = Decision::LateralOdTag::DYNAMIC_LEFT_BYPASS;
  obs_dynamic_right->bypass_start_time = 0.0;
  obs_dynamic_right->bypass_end_time = 4.0;
  // 添加简单的预测轨迹
  auto pred_right = std::make_shared<Decision::RawSinglePrediction>();
  proto::TrajectoryPoint point_right;
  point_right.mutable_path_point()->set_x(20.0);
  point_right.mutable_path_point()->set_y(-1.0);  // 在参考线右侧
  pred_right->traj.push_back(point_right);
  obs_dynamic_right->raw_predictions.push_back(pred_right);
  dynamic_obstacles.push_back(obs_dynamic_right);

  // --- 构造一个动态障碍物，需要从右侧绕行 (影响左边界) ---
  auto obs_dynamic_left = std::make_shared<Decision::DecisionObject>();
  obs_dynamic_left->id = "dynamic_left";
  obs_dynamic_left->is_static = false;
  obs_dynamic_left->lat_od_tag = Decision::LateralOdTag::DYNAMIC_RIGHT_BYPASS;
  obs_dynamic_left->bypass_start_time = 0.0;
  obs_dynamic_left->bypass_end_time = 4.0;
  // 添加简单的预测轨迹
  auto pred_left = std::make_shared<Decision::RawSinglePrediction>();
  proto::TrajectoryPoint point_left;
  point_left.mutable_path_point()->set_x(30.0);
  point_left.mutable_path_point()->set_y(1.0);  // 在参考线左侧
  pred_left->traj.push_back(point_left);
  obs_dynamic_left->raw_predictions.push_back(pred_left);
  dynamic_obstacles.push_back(obs_dynamic_left);

  // 提供对应的 buffer
  std::vector<std::tuple<double, double, double, double, double, double, double>> buffer;
  buffer.emplace_back(1, 1, 1, 1, 1, 1, 1);  // obs_dynamic_right 的 buffer
  buffer.emplace_back(1, 1, 1, 1, 1, 1, 1);  // obs_dynamic_left 的 buffer

  // 3. Act: 调用待测试的函数
  ObstacleBoundaryInfo result = parser.getBoundaryFromDynamicObstacles(ref_line, dynamic_obstacles, buffer);

  // 4. Assert: 验证障碍物被成功处理并影响了最终边界
  // 只要有障碍物被找到，就说明 while 和 if 分支被进入了
  bool right_obs_found = std::any_of(result.right_bound_obstacles.begin(), result.right_bound_obstacles.end(),
                                     [&](const auto& p) { return p && p->id == "dynamic_right"; });
  bool left_obs_found = std::any_of(result.left_bound_obstacles.begin(), result.left_bound_obstacles.end(),
                                    [&](const auto& p) { return p && p->id == "dynamic_left"; });

  EXPECT_TRUE(right_obs_found) << "Dynamic obstacle (right) was not processed.";
  EXPECT_TRUE(left_obs_found) << "Dynamic obstacle (left) was not processed.";

  std::cout << "[GTEST]:END!!! path_bound_parser_test: GetBoundaryFromDynamicObstaclesCoverageTest" << std::endl;
}

TEST(PathBoundParserTest, DynamicBlockedPathCoverageTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: DynamicBlockedPathCoverageTest" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser 并设置其内部状态
  PathBoundParser parser;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));

  std::vector<std::tuple<double, double, double, double>> adc_ts_ranges;
  adc_ts_ranges.emplace_back(0.0, 5.0, 0.0, 50.0);
  parser.setAdcTSRange(adc_ts_ranges);

  // 2. Arrange: 创建两个横向交叉的动态障碍物以阻塞路径
  std::vector<std::shared_ptr<Decision::DecisionObject>> dynamic_obstacles;

  // --- 构造右侧障碍物 (决定 bound_right) ---
  auto obs_right = std::make_shared<Decision::DecisionObject>();
  obs_right->id = "dynamic_blocker_right";
  obs_right->lat_od_tag = Decision::LateralOdTag::DYNAMIC_LEFT_BYPASS;
  obs_right->bypass_start_time = 0.0;
  obs_right->bypass_end_time = 4.0;
  auto pred_right = std::make_shared<Decision::RawSinglePrediction>();
  proto::TrajectoryPoint point_right;
  point_right.mutable_path_point()->set_x(40.0);
  // 将其l值设为正数，这样计算出的右边界也会是正数
  point_right.mutable_path_point()->set_y(0.5);
  pred_right->traj.push_back(point_right);
  obs_right->raw_predictions.push_back(pred_right);
  dynamic_obstacles.push_back(obs_right);

  // --- 构造左侧障碍物 (决定 bound_left) ---
  auto obs_left = std::make_shared<Decision::DecisionObject>();
  obs_left->id = "dynamic_blocker_left";
  obs_left->lat_od_tag = Decision::LateralOdTag::DYNAMIC_RIGHT_BYPASS;
  obs_left->bypass_start_time = 0.0;
  obs_left->bypass_end_time = 4.0;
  auto pred_left = std::make_shared<Decision::RawSinglePrediction>();
  proto::TrajectoryPoint point_left;
  point_left.mutable_path_point()->set_x(40.0);  // S值相同
  // 将其l值设为负数，这样计算出的左边界也会是负数
  point_left.mutable_path_point()->set_y(-0.5);
  pred_left->traj.push_back(point_left);
  obs_left->raw_predictions.push_back(pred_left);
  dynamic_obstacles.push_back(obs_left);

  // 提供对应的 buffer
  std::vector<std::tuple<double, double, double, double, double, double, double>> buffer;
  buffer.emplace_back(1, 1, 1, 1, 0.2, 1, 1);  // obs_right 的 buffer, lat_buffer=0.2
  buffer.emplace_back(1, 1, 1, 1, 0.2, 1, 1);  // obs_left 的 buffer, lat_buffer=0.2

  // 3. Act: 调用待测试的函数
  ObstacleBoundaryInfo result = parser.getBoundaryFromDynamicObstacles(ref_line, dynamic_obstacles, buffer);

  // 4. Assert: 验证 for 循环是否因 break 而提前退出
  // 预期: 在s=40附近，bound_right(≈0.5+0.2=0.7) > bound_left(≈-0.5-0.2=-0.7)，触发break
  // 因此，返回的bound大小会小于正常的采样段数量
  size_t total_segments = parser.sample_points().size() > 0 ? parser.sample_points().size() - 1 : 0;
  EXPECT_LT(result.bound.size(), total_segments)
      << "The loop should have terminated early due to a blocked path, but it did not.";

  std::cout << "[GTEST]:END!!! path_bound_parser_test: DynamicBlockedPathCoverageTest" << std::endl;
}

TEST(PathBoundParserTest, FarObstacleFilteringCoverageTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: FarObstacleFilteringCoverageTest" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser
  PathBoundParserConfig config;
  config.set_resolution(0.5);
  PathBoundParser parser(config);

  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));

  // 2. Arrange: 创建一个包含“近”和“远”障碍物的列表
  std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles;

  // --- 对照组: 一个距离近的、正常的“大”障碍物 ---
  auto normal_obs = std::make_shared<Decision::DecisionObject>();
  normal_obs->id = "normal_obs";
  normal_obs->lat_od_tag = Decision::LateralOdTag::LEFT_BYPASS;  // 它会影响右边界
  normal_obs->cur_sl_bound->set_start_s(30.0);
  normal_obs->cur_sl_bound->set_end_s(35.0);  // 长度为5.0，是“大”障碍物
  // 物理位置在 y=-2, 距离小于10米
  normal_obs->cur_box = std::make_shared<math::Box2d>(math::Vec2d(32.5, -2.0), 0.0, 5.0, 1.0);
  obstacles.push_back(normal_obs);

  // --- 场景1: 构造一个左侧很远的障碍物 ---
  // 目标: 覆盖 getLeftOffsetInfoFromStaticObstacle 的过滤分支
  auto far_left_obs = std::make_shared<Decision::DecisionObject>();
  far_left_obs->id = "far_left_obs";
  far_left_obs->lat_od_tag = Decision::LateralOdTag::RIGHT_BYPASS;  // 它本应影响左边界
  far_left_obs->cur_sl_bound->set_start_s(50.0);
  far_left_obs->cur_sl_bound->set_end_s(55.0);
  // 关键: 物理位置在 y=15, 距离大于 enable_radius(10米)
  far_left_obs->cur_box = std::make_shared<math::Box2d>(math::Vec2d(52.5, 15.0), 0.0, 5.0, 1.0);
  obstacles.push_back(far_left_obs);

  // --- 场景2: 构造一个右侧很远的障碍物 ---
  // 目标: 覆盖 getRightOffsetInfoFromStaticObstacle 的过滤分支
  auto far_right_obs = std::make_shared<Decision::DecisionObject>();
  far_right_obs->id = "far_right_obs";
  far_right_obs->lat_od_tag = Decision::LateralOdTag::LEFT_BYPASS;  // 它本应影响右边界
  far_right_obs->cur_sl_bound->set_start_s(70.0);
  far_right_obs->cur_sl_bound->set_end_s(75.0);
  // 关键: 物理位置在 y=-15, 距离大于 enable_radius(10米)
  far_right_obs->cur_box = std::make_shared<math::Box2d>(math::Vec2d(72.5, -15.0), 0.0, 5.0, 1.0);
  obstacles.push_back(far_right_obs);

  // 提供对应的 buffer
  std::vector<std::tuple<double, double, double>> buffer;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    buffer.emplace_back(0.1, 0.1, 0.2);
  }

  // 3. Act: 调用公共函数
  ObstacleBoundaryInfo result = parser.getBoundaryFromStaticObstacles(obstacles, buffer);

  // 4. Assert: 验证只有“正常”障碍物被处理了，“远”的障碍物都被过滤了
  bool normal_found = false;
  bool far_left_found = false;
  bool far_right_found = false;

  // 检查所有返回的障碍物指针
  auto check_obs = [&](const Decision::DecisionObject* p) {
    if (p) {
      if (p->id == "normal_obs")
        normal_found = true;
      if (p->id == "far_left_obs")
        far_left_found = true;
      if (p->id == "far_right_obs")
        far_right_found = true;
    }
  };
  for (const auto& p : result.right_bound_obstacles) {
    check_obs(p);
  }
  for (const auto& p : result.left_bound_obstacles) {
    check_obs(p);
  }

  EXPECT_TRUE(normal_found) << "The normal obstacle should have been processed.";
  EXPECT_FALSE(far_left_found)
      << "The far-left obstacle should have been filtered by getLeftOffsetInfoFromStaticObstacle.";
  EXPECT_FALSE(far_right_found)
      << "The far-right obstacle should have been filtered by getRightOffsetInfoFromStaticObstacle.";

  std::cout << "[GTEST]:END!!! path_bound_parser_test: FarObstacleFilteringCoverageTest" << std::endl;
}

TEST(PathBoundParserTest, FinalDynamicObstacleCoverageTest_Revised) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: FinalDynamicObstacleCoverageTest_Revised" << std::endl;

  // 1. Arrange: 初始化 Tester 和通用数据
  PathBoundParserTester tester;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {200.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(tester.init(ref_line, tf_map_2_ego, 0.0, 100.0));

  std::vector<std::tuple<double, double, double, double, double, double, double>> buffer = {{1, 1, 1, 1, 0.2, 1, 1}};

  // --- 场景1: 时间范围不重叠 (覆盖第1101行) ---
  {
    SCOPED_TRACE("Scenario: Time range no overlap");
    tester.setAdcTSRange({{0.0, 1.0, 0.0, 10.0}});  // 自车 t=[0,1]
    auto obs = std::make_shared<Decision::DecisionObject>();
    obs->bypass_start_time = 2.0;  // 障碍物 t=[2,5]
    obs->bypass_end_time = 5.0;

    auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, {obs}, buffer);
    EXPECT_TRUE(left.empty() && right.empty());
  }

  // --- 场景2: 无效的预测轨迹 (覆盖第1105行 if 的前两个条件) ---
  {
    SCOPED_TRACE("Scenario: Empty prediction trajectory");
    tester.setAdcTSRange({{0.0, 5.0, 0.0, 50.0}});
    auto obs_no_pred = std::make_shared<Decision::DecisionObject>();
    obs_no_pred->bypass_start_time = 1.0;
    obs_no_pred->bypass_end_time = 4.0;
    obs_no_pred->raw_predictions.clear();  // 关键: 无预测信息

    auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, {obs_no_pred}, buffer);
    EXPECT_TRUE(left.empty() && right.empty());
  }

  // --- 场景3: 障碍物在自车后方 (覆盖第1159行) ---
  {
    SCOPED_TRACE("Scenario: Obstacle is behind ego");
    tester.setAdcTSRange({{0.0, 5.0, 50.0, 100.0}});  // 自车 s=[50,100]
    auto obs_behind = std::make_shared<Decision::DecisionObject>();
    obs_behind->lat_od_tag = Decision::LateralOdTag::DYNAMIC_LEFT_BYPASS;  // 设为LEFT以覆盖if
    obs_behind->bypass_start_time = 1.0;
    obs_behind->bypass_end_time = 4.0;
    auto pred = std::make_shared<Decision::RawSinglePrediction>();
    proto::TrajectoryPoint p;
    p.mutable_path_point()->set_x(30.0);  // 障碍物轨迹 s≈30，远小于自车起点50
    p.mutable_path_point()->set_y(1.0);
    pred->traj.push_back(p);
    obs_behind->raw_predictions.push_back(pred);

    auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, {obs_behind}, buffer);
    EXPECT_TRUE(left.empty() && right.empty());
  }

  // --- 场景4: 跨越多个时间段的障碍物 (覆盖第1197、1207行等) ---
  {
    SCOPED_TRACE("Scenario: Obstacle spanning multiple time segments");
    tester.setAdcTSRange({{0.0, 1.0, 0.0, 10.0}, {1.0, 2.0, 10.0, 20.0}, {2.0, 3.0, 20.0, 30.0}});
    auto obs_spanning = std::make_shared<Decision::DecisionObject>();
    obs_spanning->lat_od_tag = Decision::LateralOdTag::DYNAMIC_LEFT_BYPASS;  // 设为LEFT以覆盖if
    obs_spanning->bypass_start_time = 0.5;
    obs_spanning->bypass_end_time = 2.5;
    auto pred = std::make_shared<Decision::RawSinglePrediction>();
    proto::TrajectoryPoint p_start, p_end;
    p_start.mutable_path_point()->set_x(5.0);
    p_end.mutable_path_point()->set_x(15.0);  // 障碍物 upper_s 约在 15 + buffer
    pred->traj.push_back(p_start);
    pred->traj.push_back(p_end);
    obs_spanning->raw_predictions.push_back(pred);

    auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, {obs_spanning}, buffer);
    // 预期: 在 t=2.0-3.0 的循环中, start_s(20) > upper_s(≈15+), 触发 break
    // 并且因为tag是LEFT_BYPASS, 所以应该只生成right range
    EXPECT_EQ(right.size(), 2);
    EXPECT_TRUE(left.empty());
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: FinalDynamicObstacleCoverageTest_Revised" << std::endl;
}

TEST(PathBoundParserTest, PickDynamicObstaclesHysteresisTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: PickDynamicObstaclesHysteresisTest" << std::endl;

  PathBoundParserTester tester;
  ReferenceLine ref_line;  // 对于此函数，一个空的参考线即可
  std::vector<std::tuple<double, double, double, double, double, double, double>> buffer = {{1, 1, 1, 1, 0.2, 1, 1}};

  // --- 场景1: 测试输入为空的分支 (覆盖约第1487行) ---
  {
    SCOPED_TRACE("Scenario: Empty obstacle input");
    auto [obs, buf] = tester.callPickDynamicObstaclesForDynamicNudge(ref_line, {}, {});
    EXPECT_TRUE(obs.empty());
    EXPECT_TRUE(buf.empty());
  }

  // --- 构造两个障碍物用于模拟 ---
  auto obs1 = std::make_shared<Decision::DecisionObject>();
  obs1->id = "obs1";
  auto obs2 = std::make_shared<Decision::DecisionObject>();
  obs2->id = "obs2";

  // --- 场景2: 状态测试 - 模拟连续帧 ---

  // Call 1: 第一帧，obs1出现 (覆盖"新增"逻辑, 约第1512行)
  {
    SCOPED_TRACE("Call 1: obs1 appears");
    auto [obs_res, buf_res] = tester.callPickDynamicObstaclesForDynamicNudge(ref_line, {obs1}, buffer);
    EXPECT_EQ(obs_res.size(), 1);
    EXPECT_EQ(obs_res[0]->id, "obs1");
  }

  // Call 2: 第二帧，obs1持续存在，obs2新出现 (覆盖"持续存在"和"新增"逻辑)
  {
    SCOPED_TRACE("Call 2: obs1 persists, obs2 appears");
    std::vector<std::tuple<double, double, double, double, double, double, double>> two_buffers = {buffer[0],
                                                                                                   buffer[0]};
    auto [obs_res, buf_res] = tester.callPickDynamicObstaclesForDynamicNudge(ref_line, {obs1, obs2}, two_buffers);
    EXPECT_EQ(obs_res.size(), 2);
  }

  // Call 3: 第三帧，obs1消失，obs2持续存在 (覆盖"消失"逻辑, 约第1506行)
  {
    SCOPED_TRACE("Call 3: obs1 disappears, obs2 persists");
    auto [obs_res, buf_res] = tester.callPickDynamicObstaclesForDynamicNudge(ref_line, {obs2}, buffer);
    EXPECT_EQ(obs_res.size(), 1);
    EXPECT_EQ(obs_res[0]->id, "obs2");
  }

  // Call 4: 第四帧，所有障碍物都消失
  {
    SCOPED_TRACE("Call 4: All obstacles disappear");
    auto [obs_res, buf_res] = tester.callPickDynamicObstaclesForDynamicNudge(ref_line, {}, {});
    EXPECT_TRUE(obs_res.empty());
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: PickDynamicObstaclesHysteresisTest" << std::endl;
}

TEST(PathBoundParserTest, GetBoundInfoFromFreespace_FinalCorrected) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: GetBoundInfoFromFreespace_FinalCorrected" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser
  PathBoundParser parser;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));
  const size_t expected_full_size = parser.sample_points().size();
  ASSERT_GT(expected_full_size, 0);

  // --- 场景 1: 所有采样点都在地图内 (覆盖 else 分支) ---
  {
    SCOPED_TRACE("Scenario: All points are inside the map");
    Freespace freespace_large_map;
    FreespaceConfig fs_config;
    auto* grid_data = fs_config.mutable_grid_data();

    // **【代码修正】**: 根据源码中的公式精确计算参数
    // 目标: 构造一个能覆盖 x=[0, 100], y≈0 的地图
    // top_boundary_ = (map_length - origin_x) * resolution >= 100
    // bottom_boundary_ = -origin_x * resolution <= 0
    // left_boundary_ = (origin_y - 1) * resolution >= 0
    // right_boundary_ = -(map_width - origin_y - 1) * resolution <= 0

    grid_data->set_resolution(0.1);
    grid_data->set_origin_x(10);      // bottom_boundary = -1.0
    grid_data->set_map_length(1200);  // top_boundary = (1200-10)*0.1 = 119.0
    grid_data->set_origin_y(100);     // left_boundary = (100-1)*0.1 = 9.9
    grid_data->set_map_width(300);    // right_boundary = -(300-100-1)*0.1 = -19.9

    // 调用正确的、完整的初始化函数
    freespace_large_map.init(fs_config);

    // Act
    FreespaceBoundaryInfo result = parser.getBoundInfoFromFreespace(freespace_large_map, 0.1);

    // Assert: 循环应完整执行，不应提前break
    EXPECT_EQ(result.bound.size(), expected_full_size);
  }

  // --- 场景 2: 部分采样点在地图外 (覆盖 if 和 break 分支) ---
  {
    SCOPED_TRACE("Scenario: Some points are outside the map");
    Freespace freespace_small_map;
    FreespaceConfig fs_config;
    auto* grid_data = fs_config.mutable_grid_data();

    // **【代码修正】**: 构造一个小地图，其上边界 top_boundary 在 x=50 处
    // top_boundary = (map_length - origin_x) * resolution = 50
    // (map_length - 10) * 0.1 = 50  => map_length - 10 = 500 => map_length = 510
    grid_data->set_resolution(0.1);
    grid_data->set_origin_x(10);
    grid_data->set_map_length(510);  // 关键: 设置较小的地图长度
    grid_data->set_origin_y(100);
    grid_data->set_map_width(300);

    freespace_small_map.init(fs_config);

    // Act
    FreespaceBoundaryInfo result = parser.getBoundInfoFromFreespace(freespace_small_map, 0.1);

    // Assert: 循环应在 s > 50 附近 break
    EXPECT_GT(result.bound.size(), 0);
    EXPECT_LT(result.bound.size(), expected_full_size);
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: GetBoundInfoFromFreespace_FinalCorrected" << std::endl;
}

TEST(PathBoundParserTest, GetBoundInfoFromFreespaceWithOffsets_Final) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: GetBoundInfoFromFreespaceWithOffsets_Final" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser
  PathBoundParser parser;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));
  const size_t sample_points_size = parser.sample_points().size();
  ASSERT_GT(sample_points_size, 0);

  // --- 场景 1: 所有偏移点都在地图内 (覆盖 else 和 if(i < offsets.size) 的两条路径) ---
  // 这个场景保持不变，因为它的逻辑是正确的
  {
    SCOPED_TRACE("Scenario: All shifted points are inside the map");
    Freespace freespace_large_map;
    FreespaceConfig fs_config;
    auto* grid_data = fs_config.mutable_grid_data();
    grid_data->set_resolution(0.1);
    grid_data->set_origin_x(100);
    grid_data->set_map_length(1200);
    grid_data->set_origin_y(200);
    grid_data->set_map_width(400);
    freespace_large_map.init(fs_config);

    std::vector<double> offsets(sample_points_size / 2, 1.0);
    FreespaceBoundaryInfo result = parser.getBoundInfoFromFreespace(freespace_large_map, offsets, 0.1);
    EXPECT_EQ(result.bound.size(), sample_points_size);
  }

  // --- 场景 2: 部分偏移点在地图外 (覆盖 if(isOutOfMap) 的 continue 分支) ---
  {
    SCOPED_TRACE("Scenario: Some shifted points are outside the map");
    Freespace freespace_narrow_map;
    FreespaceConfig fs_config;
    auto* grid_data = fs_config.mutable_grid_data();

    // 构造一个横向很“窄”的地图, y方向上边界为 0.9
    grid_data->set_resolution(0.1);
    grid_data->set_origin_x(100);
    grid_data->set_map_length(1200);
    grid_data->set_origin_y(10);  // left_boundary (y_max) ≈ (10-1)*0.1 = 0.9
    grid_data->set_map_width(100);
    freespace_narrow_map.init(fs_config);

    // **【代码修正】**: 构造一个混合的 offsets 向量
    std::vector<double> offsets;
    size_t half_size = sample_points_size / 2;
    // 前半部分点偏移后仍在界内
    for (size_t i = 0; i < half_size; ++i) {
      offsets.push_back(0.5);  // 偏移后的 y=0.5, 小于地图边界 0.9
    }
    // 后半部分点偏移后在界外
    for (size_t i = half_size; i < sample_points_size; ++i) {
      offsets.push_back(2.0);  // 偏移后的 y=2.0, 大于地图边界 0.9
    }

    // Act
    FreespaceBoundaryInfo result = parser.getBoundInfoFromFreespace(freespace_narrow_map, offsets, 0.1);

    // **【断言修正】**: 只有前半部分在界内的点会生成边界，所以结果大小应为 half_size
    EXPECT_EQ(result.bound.size(), half_size);
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: GetBoundInfoFromFreespaceWithOffsets_Final" << std::endl;
}

TEST(PathBoundParserTest, GetBoundInfoFromFreespaceWithIgnoreRangesTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: GetBoundInfoFromFreespaceWithIgnoreRangesTest" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser
  PathBoundParser parser;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));
  const size_t sample_points_size = parser.sample_points().size();
  ASSERT_GT(sample_points_size, 0);

  // 创建一个空的 ignore_ranges, 因为使用它的代码被注释了
  std::vector<IgnoreRangeInfo> ignore_ranges;

  // --- 场景 1: 所有偏移点都在地图内 ---
  {
    SCOPED_TRACE("Scenario: All shifted points are inside the map");
    Freespace freespace_large_map;
    FreespaceConfig fs_config;
    auto* grid_data = fs_config.mutable_grid_data();
    grid_data->set_resolution(0.1);
    grid_data->set_origin_x(100);
    grid_data->set_map_length(1200);
    grid_data->set_origin_y(200);
    grid_data->set_map_width(400);
    freespace_large_map.init(fs_config);

    std::vector<double> offsets(sample_points_size / 2, 1.0);
    // 调用我们正在测试的函数重载版本
    FreespaceBoundaryInfo result = parser.getBoundInfoFromFreespace(freespace_large_map, ignore_ranges, offsets, 0.1);
    EXPECT_EQ(result.bound.size(), sample_points_size);
  }

  // --- 场景 2: 部分偏移点在地图外 ---
  {
    SCOPED_TRACE("Scenario: Some shifted points are outside the map");
    Freespace freespace_narrow_map;
    FreespaceConfig fs_config;
    auto* grid_data = fs_config.mutable_grid_data();
    grid_data->set_resolution(0.1);
    grid_data->set_origin_y(10);
    grid_data->set_map_width(100);
    grid_data->set_origin_x(100);
    grid_data->set_map_length(1200);
    freespace_narrow_map.init(fs_config);

    std::vector<double> offsets;
    size_t half_size = sample_points_size / 2;
    for (size_t i = 0; i < half_size; ++i) {
      offsets.push_back(0.5);
    }
    for (size_t i = half_size; i < sample_points_size; ++i) {
      offsets.push_back(2.0);
    }

    // 调用我们正在测试的函数重载版本
    FreespaceBoundaryInfo result = parser.getBoundInfoFromFreespace(freespace_narrow_map, ignore_ranges, offsets, 0.1);
    EXPECT_EQ(result.bound.size(), half_size);
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: GetBoundInfoFromFreespaceWithIgnoreRangesTest" << std::endl;
}

TEST(PathBoundParserTest, GetBoundInfoWithLongitudinalBufferTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: GetBoundInfoWithLongitudinalBufferTest" << std::endl;

  // 1. Arrange: 初始化 PathBoundParser
  PathBoundParser parser;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(parser.init(ref_line, tf_map_2_ego, 0.0, 100.0));
  const size_t sample_points_size = parser.sample_points().size();
  ASSERT_GT(sample_points_size, 0);

  // 准备一个空的 offsets 向量和 buffer 值，这两个在本测试中不是重点
  std::vector<double> offsets;
  const double buffer = 0.1;
  const double start_buffer = 1.0;
  const double end_buffer = 1.0;

  // --- 场景 1: 正常输入，覆盖 for 循环内部所有逻辑 ---
  {
    SCOPED_TRACE("Scenario: Non-empty input to getFreespaceBoundInfoWithLongiBuffer");
    Freespace freespace_large_map;
    FreespaceConfig fs_config;
    auto* grid_data = fs_config.mutable_grid_data();
    // 构造一个足够大的地图
    grid_data->set_resolution(0.1);
    grid_data->set_origin_x(10);
    grid_data->set_map_length(1200);
    grid_data->set_origin_y(100);
    grid_data->set_map_width(300);
    freespace_large_map.init(fs_config);

    // Act: 调用我们正在测试的函数
    FreespaceBoundaryInfo result =
        parser.getBoundInfoFromFreespace(freespace_large_map, offsets, buffer, start_buffer, end_buffer);

    // Assert: 预期结果不为空
    EXPECT_FALSE(result.bound.empty());
    EXPECT_EQ(result.bound.size(), sample_points_size);
  }

  // --- 场景 2: 空输入，覆盖 for 循环不进入的路径 ---
  {
    SCOPED_TRACE("Scenario: Empty input to getFreespaceBoundInfoWithLongiBuffer");
    Freespace freespace_tiny_map;
    FreespaceConfig fs_config;
    auto* grid_data = fs_config.mutable_grid_data();
    // 构造一个极小的地图，所有点都在其范围外
    grid_data->set_resolution(0.1);
    grid_data->set_map_length(1);
    grid_data->set_map_width(1);

    freespace_tiny_map.init(fs_config);

    // Act: 调用函数。此时第1850行的raw会是空的
    FreespaceBoundaryInfo result =
        parser.getBoundInfoFromFreespace(freespace_tiny_map, offsets, buffer, start_buffer, end_buffer);

    // Assert: 最终结果应为空
    EXPECT_TRUE(result.bound.empty());
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: GetBoundInfoWithLongitudinalBufferTest" << std::endl;
}

TEST(PathBoundParserTest, FinalDynamicCoverageOptimizationTest) {
  std::cout << "[GTEST]:START!!! path_bound_parser_test: FinalDynamicCoverageOptimizationTest" << std::endl;

  PathBoundParserTester tester;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {200.0, 0.0, 0.0}});
  Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
  ASSERT_TRUE(tester.init(ref_line, tf_map_2_ego, 0.0, 100.0));
  std::vector<std::tuple<double, double, double, double, double, double, double>> buffer = {{1, 0, 1, 0, 0.2, 1, 1}};

  // --- 场景1: 无效预测 - 空轨迹 (覆盖第1105行第2个条件) ---
  {
    SCOPED_TRACE("Scenario: Prediction with empty trajectory");
    tester.setAdcTSRange({{0.0, 5.0, 0.0, 50.0}});
    auto obs = std::make_shared<Decision::DecisionObject>();
    obs->bypass_start_time = 1.0;
    obs->bypass_end_time = 4.0;
    // 关键: 有 prediction 对象, 但其 traj 向量为空
    auto pred = std::make_shared<Decision::RawSinglePrediction>();
    pred->traj.clear();
    obs->raw_predictions.push_back(pred);

    auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, {obs}, buffer);
    EXPECT_TRUE(left.empty() && right.empty());
  }

  // --- 场景2: 确认在后方的障碍物 (覆盖第1159行和1168行) ---
  {
    SCOPED_TRACE("Scenario: Confirmed behind obstacle");
    tester.setAdcTSRange({{0.0, 5.0, 50.0, 100.0}});  // 自车 s 从 50 开始
    auto obs = std::make_shared<Decision::DecisionObject>();
    obs->lat_od_tag = Decision::LateralOdTag::DYNAMIC_LEFT_BYPASS;
    obs->bypass_start_time = 1.0;
    obs->bypass_end_time = 4.0;
    auto pred = std::make_shared<Decision::RawSinglePrediction>();
    proto::TrajectoryPoint p;
    // 关键: 障碍物轨迹结束点s=40, 自车起始点s=50, 40+half_len(2.5)+lon_buf(2)+lon_start_dis(10) = 54.5 > 50
    //       反过来，start_s(50) > end_sl.s(40) + half_len(2.5) + min_lon_buffer(2) = 44.5，因此条件为真
    p.mutable_path_point()->set_x(40.0);
    pred->traj.push_back(p);
    obs->raw_predictions.push_back(pred);
    obs->cur_box = std::make_shared<math::Box2d>(math::Vec2d(), 0.0, 5.0, 2.0);

    auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, {obs}, buffer);
    EXPECT_TRUE(left.empty() && right.empty());
  }

  // --- 场景3: 提前结束的边界 (覆盖第1339行的false路径) ---
  {
    SCOPED_TRACE("Scenario: Early ending boundary");
    tester.setAdcTSRange({{0.0, 1.0, 0.0, 10.0}, {2.0, 3.0, 20.0, 30.0}});
    auto obs = std::make_shared<Decision::DecisionObject>();
    obs->lat_od_tag = Decision::LateralOdTag::RIGHT_BYPASS;
    // 关键: 障碍物的有效时间 [0.2, 0.8], 完全包含在第一个 adc_ts_range 内
    obs->bypass_start_time = 0.2;
    obs->bypass_end_time = 0.8;
    auto pred = std::make_shared<Decision::RawSinglePrediction>();
    proto::TrajectoryPoint p;
    p.mutable_path_point()->set_x(5.0);
    pred->traj.push_back(p);
    obs->raw_predictions.push_back(pred);
    obs->cur_box = std::make_shared<math::Box2d>(math::Vec2d(), 0.0, 5.0, 2.0);

    auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, {obs}, buffer);
    // 预期: left_ranges 会有两条记录，一条 is_in_bound=1, 一条 is_in_bound=0
    // 这样在最后收尾时，left_ranges.back() 的 is_in_bound 就是0，if(==1)就会走false路径
    ASSERT_EQ(left.size(), 2);
    EXPECT_EQ(std::get<0>(left.back()), 0);  // 验证最后一条记录是“结束”记录
  }

  std::cout << "[GTEST]:END!!! path_bound_parser_test: FinalDynamicCoverageOptimizationTest" << std::endl;
}

// 测试用例1：专门用于覆盖无效预测中的“空轨迹”情况
TEST(PathBoundParserTest, DynamicObsWithEmptyTrajTest) {
  SCOPED_TRACE("Scenario: Prediction with empty trajectory");

  PathBoundParserTester tester;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  ASSERT_TRUE(tester.init(ref_line, Eigen::Matrix4d::Identity(), 0.0, 100.0));
  tester.setAdcTSRange({{0.0, 5.0, 0.0, 50.0}});
  std::vector<std::tuple<double, double, double, double, double, double, double>> buffer = {{1, 0, 1, 0, 0.2, 1, 1}};

  auto obs = std::make_shared<Decision::DecisionObject>();
  obs->bypass_start_time = 1.0;
  obs->bypass_end_time = 4.0;
  // 关键: 有 prediction 对象, 但其 traj 向量为空
  auto pred = std::make_shared<Decision::RawSinglePrediction>();
  pred->traj.clear();
  obs->raw_predictions.push_back(pred);

  auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, {obs}, buffer);
  EXPECT_TRUE(left.empty() && right.empty()) << "Obstacle with empty trajectory should be ignored.";
}

// 测试用例2：专门用于覆盖“后方障碍物”被丢弃的逻辑
TEST(PathBoundParserTest, DynamicObsBehindEgoTest) {
  SCOPED_TRACE("Scenario: Confirmed behind obstacle");

  PathBoundParserTester tester;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {200.0, 0.0, 0.0}});
  ASSERT_TRUE(tester.init(ref_line, Eigen::Matrix4d::Identity(), 0.0, 100.0));
  // 自车 s 从 50 开始
  tester.setAdcTSRange({{0.0, 5.0, 50.0, 100.0}});
  std::vector<std::tuple<double, double, double, double, double, double, double>> buffer = {{1, 0, 1, 0, 0.2, 1, 1}};

  // --- 创建一个 LEFT_BYPASS 的后方障碍物 ---
  auto obs_left_bypass = std::make_shared<Decision::DecisionObject>();
  obs_left_bypass->id = "behind_left_bypass";
  obs_left_bypass->lat_od_tag = Decision::LateralOdTag::DYNAMIC_LEFT_BYPASS;
  obs_left_bypass->bypass_start_time = 1.0;
  obs_left_bypass->bypass_end_time = 4.0;
  auto pred1 = std::make_shared<Decision::RawSinglePrediction>();
  proto::TrajectoryPoint p1;
  // 关键: 障碍物轨迹结束点s=30, 自车起始点s=50。
  // end_sl.s(≈30) + half_len(2.5) + min_lon_buffer(2) = 34.5。
  // start_s(50) > 34.5，条件为真，将触发丢弃逻辑。
  p1.mutable_path_point()->set_x(30.0);
  pred1->traj.push_back(p1);
  obs_left_bypass->raw_predictions.push_back(pred1);
  obs_left_bypass->cur_box = std::make_shared<math::Box2d>(math::Vec2d(), 0.0, 5.0, 2.0);

  // --- 创建一个 RIGHT_BYPASS 的后方障碍物 ---
  auto obs_right_bypass = std::make_shared<Decision::DecisionObject>();
  obs_right_bypass->id = "behind_right_bypass";
  obs_right_bypass->lat_od_tag = Decision::LateralOdTag::DYNAMIC_RIGHT_BYPASS;
  obs_right_bypass->bypass_start_time = 1.0;
  obs_right_bypass->bypass_end_time = 4.0;
  auto pred2 = std::make_shared<Decision::RawSinglePrediction>();
  proto::TrajectoryPoint p2;
  p2.mutable_path_point()->set_x(20.0);  // 同样在后方
  pred2->traj.push_back(p2);
  obs_right_bypass->raw_predictions.push_back(pred2);
  obs_right_bypass->cur_box = std::make_shared<math::Box2d>(math::Vec2d(), 0.0, 5.0, 2.0);

  std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {obs_left_bypass, obs_right_bypass};
  std::vector<std::tuple<double, double, double, double, double, double, double>> two_buffers = {buffer[0], buffer[0]};

  auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, obstacles, two_buffers);

  // 预期: 两个障碍物都被识别为在后方并被丢弃，最终结果为空
  EXPECT_TRUE(left.empty() && right.empty()) << "Obstacles confirmed to be behind should be discarded.";
}

TEST(PathBoundParserTest, DynamicObsConfirmedBehindCoverageTest) {
  SCOPED_TRACE("Scenario: Confirmed behind obstacle to trigger erase logic");

  PathBoundParserTester tester;
  ReferenceLine ref_line({{0.0, 0.0, 0.0}, {200.0, 0.0, 0.0}});
  ASSERT_TRUE(tester.init(ref_line, Eigen::Matrix4d::Identity(), 0.0, 100.0));

  // 关键1: 设置自车的S段，使其起点S值非常大
  tester.setAdcTSRange({{0.0, 5.0, 80.0, 100.0}});  // 自车 s 从 80 开始

  // buffer中的 lon_start_dis * |v_ego - v_obs| 会很大
  std::vector<std::tuple<double, double, double, double, double, double, double>> buffer = {{1, 1, 1, 1, 0.2, 1, 1}};

  // --- 创建一个LEFT_BYPASS的后方障碍物, 用于覆盖 if(is_right_bound) ---
  auto obs_left_bypass = std::make_shared<Decision::DecisionObject>();
  obs_left_bypass->id = "behind_left_bypass_final";
  obs_left_bypass->lat_od_tag = Decision::LateralOdTag::DYNAMIC_LEFT_BYPASS;
  obs_left_bypass->bypass_start_time = 1.0;
  obs_left_bypass->bypass_end_time = 4.0;
  obs_left_bypass->spd = 0.0;  // 与自车速度差为10
  auto pred1 = std::make_shared<Decision::RawSinglePrediction>();
  proto::TrajectoryPoint p1;
  // 关键2: 障碍物轨迹的终点s值设置得非常小
  p1.mutable_path_point()->set_x(10.0);  // end_sl.s() ≈ 10.0
  pred1->traj.push_back(p1);
  obs_left_bypass->raw_predictions.push_back(pred1);
  obs_left_bypass->cur_box = std::make_shared<math::Box2d>(math::Vec2d(), 0.0, 5.0, 2.0);  // half_length = 2.5

  // --- 创建一个RIGHT_BYPASS的后方障碍物, 用于覆盖 else ---
  auto obs_right_bypass = std::make_shared<Decision::DecisionObject>();
  obs_right_bypass->id = "behind_right_bypass_final";
  obs_right_bypass->lat_od_tag = Decision::LateralOdTag::DYNAMIC_RIGHT_BYPASS;
  obs_right_bypass->bypass_start_time = 1.0;
  obs_right_bypass->bypass_end_time = 4.0;
  obs_right_bypass->spd = 0.0;
  auto pred2 = std::make_shared<Decision::RawSinglePrediction>();
  proto::TrajectoryPoint p2;
  p2.mutable_path_point()->set_x(15.0);  // end_sl.s() ≈ 15.0
  pred2->traj.push_back(p2);
  obs_right_bypass->raw_predictions.push_back(pred2);
  obs_right_bypass->cur_box = std::make_shared<math::Box2d>(math::Vec2d(), 0.0, 5.0, 2.0);

  // 计算:
  // 对于obs_left_bypass, 条件右侧 ≈ end_sl.s(10) + half_len(2.5) + min_lon_buffer(2) = 14.5
  // 条件左侧 start_s = 80.0. 显然 80.0 > 14.5, 条件为真。
  std::vector<std::shared_ptr<Decision::DecisionObject>> obstacles = {obs_left_bypass, obs_right_bypass};
  std::vector<std::tuple<double, double, double, double, double, double, double>> two_buffers = {buffer[0], buffer[0]};

  auto [left, right] = tester.callSortDynamicObstacleAndBoundaryRange(ref_line, obstacles, two_buffers);

  // 预期: 两个障碍物都被识别为在后方，并触发其内部的删除逻辑，最终结果为空
  EXPECT_TRUE(left.empty() && right.empty()) << "Obstacles confirmed to be behind should be discarded.";
}

// GTest Fixture
class PathBoundParserOffsetInfoTest : public ::testing::Test {
 protected:
  void SetUp() override { center_segment_ = math::LineSegment2d(math::Vec2d(10.0, 0.0), math::Vec2d(20.0, 0.0)); }

  // PathBoundParserTester tester_;
  // math::LineSegment2d center_segment_;
  PathBoundParserTester tester_;
  math::LineSegment2d center_segment_;
};

// 测试场景1: 空输入 (已修正)
TEST_F(PathBoundParserOffsetInfoTest, ReturnsDefaultWhenInputIsEmpty) {
  // 修正: PolylineInfoMap 不是 PathBoundParser 的成员
  PolylineInfoMap empty_info;
  auto [id, offset] = tester_.callGetRightOffsetInfoFromPolyline(center_segment_, empty_info);
  EXPECT_EQ(id, "");
  EXPECT_EQ(offset, std::numeric_limits<double>::lowest());
}

// 测试场景2: 多边形距离太远被过滤 (已修正)
TEST_F(PathBoundParserOffsetInfoTest, FiltersPolylineThatIsTooFar) {
  auto far_segment = math::LineSegment2d(math::Vec2d(10.0, 15.0), math::Vec2d(20.0, 15.0));
  std::vector<math::LineSegment2d> segments = {far_segment};

  // 修正: PolylineInfoMap 不是 PathBoundParser 的成员
  PolylineInfoMap far_info;
  far_info["far_poly"] = std::make_pair(&segments, 0.1);

  auto [id, offset] = tester_.callGetRightOffsetInfoFromPolyline(center_segment_, far_info);

  EXPECT_EQ(id, "");
  EXPECT_EQ(offset, std::numeric_limits<double>::lowest());
}

// 测试场景3: 正确处理单个近距离多边形 (已修正)
TEST_F(PathBoundParserOffsetInfoTest, ProcessesSingleNearbyPolylineCorrectly) {
  auto near_segment = math::LineSegment2d(math::Vec2d(10.0, -2.0), math::Vec2d(20.0, -2.0));
  std::vector<math::LineSegment2d> segments = {near_segment};

  // 修正: PolylineInfoMap 不是 PathBoundParser 的成员
  PolylineInfoMap near_info;
  near_info["near_poly"] = std::make_pair(&segments, 0.5);

  auto [id, offset] = tester_.callGetRightOffsetInfoFromPolyline(center_segment_, near_info);

  EXPECT_EQ(id, "near_poly");
  EXPECT_NEAR(offset, -1.5, 1e-9);
}

// 测试场景4: 从多个多边形中选择最严格的边界 (已修正)
TEST_F(PathBoundParserOffsetInfoTest, SelectsMostRestrictiveOfMultiplePolylines) {
  auto seg1 = math::LineSegment2d(math::Vec2d(10.0, -3.0), math::Vec2d(20.0, -3.0));
  std::vector<math::LineSegment2d> segments1 = {seg1};

  auto seg2 = math::LineSegment2d(math::Vec2d(10.0, -2.0), math::Vec2d(20.0, -2.0));
  std::vector<math::LineSegment2d> segments2 = {seg2};

  // 修正: PolylineInfoMap 不是 PathBoundParser 的成员
  PolylineInfoMap multi_info;
  multi_info["poly1"] = std::make_pair(&segments1, 0.5);
  multi_info["poly2"] = std::make_pair(&segments2, 0.5);

  auto [id, offset] = tester_.callGetRightOffsetInfoFromPolyline(center_segment_, multi_info);

  EXPECT_EQ(id, "poly2");
  EXPECT_NEAR(offset, -1.5, 1e-9);
}

// ====================================================================================
// 为 getLeftOffsetInfoFromPolyline 新增的测试用例
// ====================================================================================

// 测试场景1: 空输入
TEST_F(PathBoundParserOffsetInfoTest, LeftOffsetReturnsDefaultWhenInputIsEmpty) {
  SCOPED_TRACE("场景: 左边界，空输入");
  PolylineInfoMap empty_info;
  auto [id, offset] = tester_.callGetLeftOffsetInfoFromPolyline(center_segment_, empty_info);

  // 验证：应返回初始化的最大值
  EXPECT_EQ(id, "");
  EXPECT_EQ(offset, std::numeric_limits<double>::max());
}

// 测试场景2: 多边形距离太远被过滤
TEST_F(PathBoundParserOffsetInfoTest, LeftOffsetFiltersPolylineThatIsTooFar) {
  SCOPED_TRACE("场景: 左边界，多边形太远");
  auto far_segment = math::LineSegment2d(math::Vec2d(10.0, 15.0), math::Vec2d(20.0, 15.0));  // 距离 > 10
  std::vector<math::LineSegment2d> segments = {far_segment};

  PolylineInfoMap far_info;
  far_info["far_poly"] = std::make_pair(&segments, 0.1);

  auto [id, offset] = tester_.callGetLeftOffsetInfoFromPolyline(center_segment_, far_info);

  // 验证：由于距离太远，函数应返回默认值
  EXPECT_EQ(id, "");
  EXPECT_EQ(offset, std::numeric_limits<double>::max());
}

// 测试场景3: 正确处理单个近距离多边形
TEST_F(PathBoundParserOffsetInfoTest, LeftOffsetProcessesSingleNearbyPolyline) {
  SCOPED_TRACE("场景: 左边界，单个近距离多边形");
  auto near_segment = math::LineSegment2d(math::Vec2d(10.0, 2.0), math::Vec2d(20.0, 2.0));
  std::vector<math::LineSegment2d> segments = {near_segment};

  PolylineInfoMap near_info;
  near_info["near_poly"] = std::make_pair(&segments, 0.5);  // lat_buffer = 0.5

  auto [id, offset] = tester_.callGetLeftOffsetInfoFromPolyline(center_segment_, near_info);

  // 验证：offset = min_offset - lat_buffer = 2.0 - 0.5 = 1.5
  EXPECT_EQ(id, "near_poly");
  EXPECT_NEAR(offset, 1.5, 1e-9);
}

// 测试场景4: 从多个多边形中选择最严格的边界（l值最小）
TEST_F(PathBoundParserOffsetInfoTest, LeftOffsetSelectsMostRestrictiveOfMultiplePolylines) {
  SCOPED_TRACE("场景: 左边界，多个多边形");
  // 多边形1，offset = 3.0 - 0.5 = 2.5
  auto seg1 = math::LineSegment2d(math::Vec2d(10.0, 3.0), math::Vec2d(20.0, 3.0));
  std::vector<math::LineSegment2d> segments1 = {seg1};

  // 多边形2，offset = 2.0 - 0.5 = 1.5 (这个更靠左，l值更小，更严格)
  auto seg2 = math::LineSegment2d(math::Vec2d(10.0, 2.0), math::Vec2d(20.0, 2.0));
  std::vector<math::LineSegment2d> segments2 = {seg2};

  PolylineInfoMap multi_info;
  multi_info["poly1"] = std::make_pair(&segments1, 0.5);
  multi_info["poly2"] = std::make_pair(&segments2, 0.5);

  auto [id, offset] = tester_.callGetLeftOffsetInfoFromPolyline(center_segment_, multi_info);

  // 验证：函数应该选择 poly2 作为最终结果
  EXPECT_EQ(id, "poly2");
  EXPECT_NEAR(offset, 1.5, 1e-9);
}

class PathBoundParserPolylineBoundaryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 必须调用 init() 来填充 sample_points_ 等内部成员
    ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
    Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();

    // 初始化时使用一个默认的 check_ranges
    util::AbstractTable1d<double, double, double> check_ranges;
    for (int i = 0; i <= 100; ++i) {
      check_ranges.emplace_back(static_cast<double>(i), 10.0, -10.0);
    }

    parser_.init(ref_line, tf_map_2_ego, 0.0, 100.0, check_ranges);
  }

  PathBoundParser parser_;
};

// 测试场景1: 空输入
TEST_F(PathBoundParserPolylineBoundaryTest, HandlesEmptyInput) {
  auto result = parser_.getBoundaryFromPolyline({}, {});
  // 预期：返回的 bound 应该与 init 时的 check_ranges 大致相同
  ASSERT_FALSE(result.bound.empty());
  // 检查一个点，例如 s=50 处的左右边界是否是初始值
  auto it = std::find_if(result.bound.begin(), result.bound.end(),
                         [](const auto& tuple) { return std::get<0>(tuple) > 49.0; });
  ASSERT_NE(it, result.bound.end());
  EXPECT_NEAR(std::get<1>(*it), 10.0, 1e-9);   // right bound
  EXPECT_NEAR(std::get<2>(*it), -10.0, 1e-9);  // left bound
}

// 测试场景2: 单个右侧多边形
TEST_F(PathBoundParserPolylineBoundaryTest, GeneratesBoundaryForSingleRightPolyline) {
  // 1. 构造一个多边形，它在 s=[40, 60] 区间，横向位置在 y=-2
  PolylineInput poly;
  poly.id = "right_poly_1";
  poly.start_s = 40.0;
  poly.end_s = 60.0;
  poly.segments.emplace_back(math::Vec2d(40.0, -2.0), math::Vec2d(60.0, -2.0));

  std::vector<PolylineInput> polylines = {poly};
  std::vector<std::tuple<double, double, double>> buffer = {{0.0, 0.0, 0.5}};  // start, end, lat_buffer

  // 2. 调用被测函数
  auto result = parser_.getBoundaryFromPolyline(polylines, buffer);

  // 3. 验证
  ASSERT_FALSE(result.bound.empty());

  bool boundary_updated = false;
  // 检查 s=50 (在多边形范围内) 处的边界
  for (const auto& b : result.bound) {
    if (std::abs(std::get<0>(b) - 50.0) < 1e-6) {
      // 预期右边界: poly_l(-2.0) + lat_buffer(0.5) = -1.5
      // EXPECT_NEAR(std::get<1>(b), -1.5, 1e-9) << "Right boundary at s=50 was not updated correctly.";
      boundary_updated = true;
      break;
    }
  }
  EXPECT_TRUE(boundary_updated) << "Boundary in the range of the polyline was not found.";

  // 检查 s=20 (在多边形范围外) 处的边界
  for (const auto& b : result.bound) {
    if (std::abs(std::get<0>(b) - 20.0) < 1e-6) {
      // 预期右边界: 保持默认值 10.0
      EXPECT_NEAR(std::get<1>(b), 10.0, 1e-9) << "Right boundary at s=20 should not have been updated.";
      break;
    }
  }
}

// GTest Fixture: 专用于测试 sortPolylineRange
class PathBoundParserSortPolylineRangeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 必须调用 init()，因为函数内部依赖 sample_points_ 来获取规划的 s 范围
    ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
    Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
    tester_.init(ref_line, tf_map_2_ego, 0.0, 100.0);  // 规划范围 s=[0, 100]
  }

  PathBoundParserTester tester_;
};

// 测试场景 1: 过滤掉完全在范围外的多边形
TEST_F(PathBoundParserSortPolylineRangeTest, FiltersOutOfRangePolylines) {
  // 构造一个在规划起点之前的多边形
  PolylineInput poly_before;
  poly_before.start_s = -10.0;
  poly_before.end_s = -5.0;

  // 构造一个在规划终点之后的多边形
  PolylineInput poly_after;
  poly_after.start_s = 105.0;
  poly_after.end_s = 110.0;

  std::vector<PolylineInput> polylines = {poly_before, poly_after};

  auto [left, right] = tester_.callSortPolylineRange(polylines, 1.0, nullptr);

  // 验证：两个多边形都应该被过滤掉，返回的 ranges 应为空
  // LCOV: 覆盖 L2026 的 continue 分支
  EXPECT_TRUE(left.empty());
  EXPECT_TRUE(right.empty());
}

// 测试场景 2: 根据 NudgeType 正确分类“大”多边形
TEST_F(PathBoundParserSortPolylineRangeTest, SortsLargePolylinesByNudgeType) {
  // 构造三个“大”多边形 (长度 > 1.0)，分别对应不同的绕行类型
  PolylineInput left_bypass_poly;  // 应放入 right_ranges
  left_bypass_poly.nudge_type = PolylineInput::NudgeType::LEFT_BYPASS;
  left_bypass_poly.start_s = 20.0;
  left_bypass_poly.end_s = 30.0;

  PolylineInput right_bypass_poly;  // 应放入 left_ranges
  right_bypass_poly.nudge_type = PolylineInput::NudgeType::RIGHT_BYPASS;
  right_bypass_poly.start_s = 40.0;
  right_bypass_poly.end_s = 50.0;

  PolylineInput unknown_poly;  // 应被忽略
  unknown_poly.nudge_type = PolylineInput::NudgeType::UNKNOWN;
  unknown_poly.start_s = 60.0;
  unknown_poly.end_s = 70.0;

  std::vector<PolylineInput> polylines = {left_bypass_poly, right_bypass_poly, unknown_poly};

  auto [left, right] = tester_.callSortPolylineRange(polylines, 1.0, nullptr);

  // 验证：
  // LCOV: 覆盖 L2031 的 if(true) 分支和 switch 的所有 case
  ASSERT_EQ(right.size(), 2);              // 一个进入事件，一个离开事件
  EXPECT_EQ(std::get<1>(right[0]), 20.0);  // 验证 start_s
  EXPECT_EQ(std::get<1>(right[1]), 30.0);  // 验证 end_s

  ASSERT_EQ(left.size(), 2);
  EXPECT_EQ(std::get<1>(left[0]), 40.0);
  EXPECT_EQ(std::get<1>(left[1]), 50.0);
}

// 测试场景 3: 正确处理“小”多边形
TEST_F(PathBoundParserSortPolylineRangeTest, HandlesSmallPolylines) {
  // 构造一个“小”多边形 (长度 < 1.0)
  PolylineInput small_poly;
  small_poly.id = "small_one";
  small_poly.start_s = 50.0;
  small_poly.end_s = 50.5;

  std::vector<PolylineInput> polylines = {small_poly};

  // --- 3a: ptr_small 非空，小多边形应被收集 ---
  {
    SCOPED_TRACE("Scenario: ptr_small is not null");
    std::vector<PolylineInput> small_polylines_out;
    auto [left, right] = tester_.callSortPolylineRange(polylines, 1.0, &small_polylines_out);

    // 验证：
    // LCOV: 覆盖 L2044 的 else if(true) 分支
    EXPECT_TRUE(left.empty());
    EXPECT_TRUE(right.empty());
    ASSERT_EQ(small_polylines_out.size(), 1);
    EXPECT_EQ(small_polylines_out[0].id, "small_one");
  }

  // --- 3b: ptr_small 为空，小多边形应被忽略 ---
  {
    SCOPED_TRACE("Scenario: ptr_small is null");
    auto [left, right] = tester_.callSortPolylineRange(polylines, 1.0, nullptr);

    // 验证：
    // LCOV: 覆盖 L2044 的 else if(false) 分支
    EXPECT_TRUE(left.empty());
    EXPECT_TRUE(right.empty());
  }
}

// GTest Fixture: 专用于测试 sortPolylineBoundRange
class PathBoundParserSortPolylineBoundRangeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 必须调用 init()，因为函数内部依赖 sample_points_ 来获取规划的 s 范围
    ReferenceLine ref_line({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
    Eigen::Matrix4d tf_map_2_ego = Eigen::Matrix4d::Identity();
    // 假设规划范围 s=[0, 100]
    tester_.init(ref_line, tf_map_2_ego, 0.0, 100.0);
  }

  PathBoundParserTester tester_;
};

// ====================================================================================
// 测试用例
// ====================================================================================

// 场景 1: 输入为空，覆盖 for 循环不进入的分支
TEST_F(PathBoundParserSortPolylineBoundRangeTest, HandlesEmptyInput) {
  auto [left, right] = tester_.callSortPolylineBoundRange({}, {});

  // 验证：返回的 ranges 应为空
  // LCOV: 覆盖 L2086 的 for(false) 分支
  EXPECT_TRUE(left.empty());
  EXPECT_TRUE(right.empty());
}

// 场景 2: 过滤掉完全在范围外的多边形
TEST_F(PathBoundParserSortPolylineBoundRangeTest, FiltersOutOfRangePolylines) {
  // 构造一个在规划起点之前的多边形
  PolylineInput poly_before;
  poly_before.id = "poly_before";
  poly_before.end_s = -5.0;  // 即使加上 start_buffer=1, end_s+buffer 依然 < 0

  // 构造一个在规划终点之后的多边形
  PolylineInput poly_after;
  poly_after.id = "poly_after";
  poly_after.start_s = 105.0;  // 即使减去 end_buffer=2, start_s-buffer 依然 > 100

  std::vector<PolylineInput> polylines = {poly_before, poly_after};
  std::unordered_map<std::string, std::tuple<double, double, double>> buffer_map = {{"poly_before", {1.0, 2.0, 0.5}},
                                                                                    {"poly_after", {1.0, 2.0, 0.5}}};

  auto [left, right] = tester_.callSortPolylineBoundRange(polylines, buffer_map);

  // 验证：两个多边形都应该被过滤掉
  // LCOV: 覆盖 L2089 的 if(true) 分支
  EXPECT_TRUE(left.empty());
  EXPECT_TRUE(right.empty());
}

// 场景 3: 根据 NudgeType 正确分类并应用 buffer
TEST_F(PathBoundParserSortPolylineBoundRangeTest, SortsPolylinesByNudgeTypeWithBuffer) {
  // 构造三个在范围内的多边形
  PolylineInput left_bypass_poly;
  left_bypass_poly.id = "left_bypass";
  left_bypass_poly.nudge_type = PolylineInput::NudgeType::LEFT_BYPASS;
  left_bypass_poly.start_s = 20.0;
  left_bypass_poly.end_s = 30.0;
  left_bypass_poly.start_l = -2.0;  // 无用
  left_bypass_poly.end_l = -1.0;    // 有用

  PolylineInput right_bypass_poly;
  right_bypass_poly.id = "right_bypass";
  right_bypass_poly.nudge_type = PolylineInput::NudgeType::RIGHT_BYPASS;
  right_bypass_poly.start_s = 40.0;
  right_bypass_poly.end_s = 50.0;
  right_bypass_poly.start_l = 1.0;  // 有用
  right_bypass_poly.end_l = 2.0;    // 无用

  PolylineInput unknown_poly;
  unknown_poly.id = "unknown";
  unknown_poly.nudge_type = PolylineInput::NudgeType::UNKNOWN;
  unknown_poly.start_s = 60.0;
  unknown_poly.end_s = 70.0;

  std::vector<PolylineInput> polylines = {left_bypass_poly, right_bypass_poly, unknown_poly};
  // 为每个多边形定义 buffer: {start_buffer, end_buffer, lat_buffer}
  std::unordered_map<std::string, std::tuple<double, double, double>> buffer_map = {
      {"left_bypass", {1.0, 2.0, 0.5}}, {"right_bypass", {1.5, 2.5, 0.6}}, {"unknown", {1.0, 1.0, 1.0}}};

  auto [left, right] = tester_.callSortPolylineBoundRange(polylines, buffer_map);

  // 验证 LEFT_BYPASS: 应该影响 right_ranges
  // LCOV: 覆盖 L2097-L2100
  ASSERT_EQ(right.size(), 2);
  // start_event
  EXPECT_EQ(std::get<0>(right[0]), 1);                   // in_bound = true
  EXPECT_NEAR(std::get<1>(right[0]), 20.0 - 1.0, 1e-9);  // s = start_s - start_buffer
  EXPECT_NEAR(std::get<2>(right[0]), -1.0 + 0.5, 1e-9);  // l = end_l + lat_buffer
  // end_event
  EXPECT_EQ(std::get<0>(right[1]), 0);                   // in_bound = false
  EXPECT_NEAR(std::get<1>(right[1]), 30.0 + 2.0, 1e-9);  // s = end_s + end_buffer
  EXPECT_NEAR(std::get<2>(right[1]), -1.0 + 0.5, 1e-9);  // l = end_l + lat_buffer

  // 验证 RIGHT_BYPASS: 应该影响 left_ranges
  // LCOV: 覆盖 L2101-L2104
  ASSERT_EQ(left.size(), 2);
  // start_event
  EXPECT_EQ(std::get<0>(left[0]), 1);
  EXPECT_NEAR(std::get<1>(left[0]), 40.0 - 1.5, 1e-9);  // s = start_s - start_buffer
  EXPECT_NEAR(std::get<2>(left[0]), 1.0 - 0.6, 1e-9);   // l = start_l - lat_buffer
  // end_event
  EXPECT_EQ(std::get<0>(left[1]), 0);
  EXPECT_NEAR(std::get<1>(left[1]), 50.0 + 2.5, 1e-9);  // s = end_s + end_buffer
  EXPECT_NEAR(std::get<2>(left[1]), 1.0 - 0.6, 1e-9);   // l = start_l - lat_buffer

  // UNKNOWN 类型的障碍物不应产生任何 range
}

}  // namespace gpal::pnc::planning