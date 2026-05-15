#include <gtest/gtest.h>
#include <any>

#include "config_manager/config_manager.h"
#include "config/speed_planner/speed_preprocessor.pb.h"

#define private public
#include "speed_preprocessor/speed_wall.h"


namespace gpal::pnc::planning {
class SpeedWallProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 初始化配置
        speed_preprocessor_config_.set_enable_u_turn_stop(true);
        speed_preprocessor_config_.set_allow_exceed_stop_line(true);
        
        // 初始化车辆参数
        vehicle_param_.set_front_edge_to_ego(1.0);
        
        // 初始化时间范围
        time_horizon_ = 5.0;
    }

    SpeedWallProcessor processor_;
    SpeedPreProcessorConfig speed_preprocessor_config_;
    VehicleParam vehicle_param_;
    double time_horizon_;
};

TEST_F(SpeedWallProcessorTest, NullReferenceLineInfo) {
    std::vector<SpeedWall> speed_walls;
    LocalView local_view;
    DecisionResult decision_result;
    
    // 测试nullptr情况
    processor_.addTflSpeedWall(local_view, nullptr, decision_result, false, &speed_walls);
    EXPECT_TRUE(speed_walls.empty());
    
    // 测试无效reference line
    ReferenceLineInfo invalid_ref_line_info;
    processor_.addTflSpeedWall(local_view, &invalid_ref_line_info, decision_result, false, &speed_walls);
    EXPECT_TRUE(speed_walls.empty());
}

TEST_F(SpeedWallProcessorTest, NoStopLines) {
    std::vector<SpeedWall> speed_walls;
    LocalView local_view;
    DecisionResult decision_result;
    ReferenceLineInfo ref_line_info;
    
    // 设置有效的reference line
    ReferenceLine ref_line;
    ref_line_info.ref_line_ = ref_line;
    
    // 测试无停止线情况
    processor_.addTflSpeedWall(local_view, &ref_line_info, decision_result, false, &speed_walls);
    EXPECT_TRUE(speed_walls.empty());
}

TEST_F(SpeedWallProcessorTest, TurnAroundScenario) {
    std::vector<SpeedWall> speed_walls;
    LocalView local_view;
    DecisionResult decision_result;
    ReferenceLineInfo ref_line_info;
    
    // 设置有效的reference line和停止线
    vector<ReferencePoint> ref_points;
    for(int i = 0 ;i < 50; i++){
      ref_points.emplace_back(math::Vec3d(i*1.0, 0.0,0.0),0.0,0.0,0.0001,0.0);
      ref_points.back().local_s_ = i*1.0;
    }
    ReferenceLine ref_line(ref_points);
    ref_line_info.ref_line_ = ref_line;
    std::vector<StopLine> stop_lines = {StopLine{10, 0, 0, 10, DrivingDirection::kDirectionUTurnOnly}};
    ref_line_info.ref_line_.stop_lines_ = stop_lines ;

    // 设置车辆位置和速度
    auto localization = std::make_shared<Localization>();
    localization->vehicle_align_pose_point_ = MapPoint();
    local_view.getMutableLocalizationPtr() = (localization);

    // 设置交通灯决策
    decision_result.traffic_light_info_.is_need_stop = true;
    
    // 设置车辆速度
    auto chassis = std::make_shared<Chassis>();
    chassis->set_Speed(0.0f); // 移动状态
    local_view.chassis_= chassis;
    
    // 测试掉头场景
    processor_.speed_preprocessor_config_.set_allow_exceed_stop_line(true);

    for(int i = 0;i<15;i++){
          processor_.addTflSpeedWall(local_view, &ref_line_info, decision_result, true, &speed_walls);
    }
    
    
    // 验证速度墙是否添加
    EXPECT_FALSE(speed_walls.empty());
}

TEST_F(SpeedWallProcessorTest, TrafficLightStop) {
    std::vector<SpeedWall> speed_walls;
    LocalView local_view;
    DecisionResult decision_result;
    ReferenceLineInfo ref_line_info;
    
    // 设置有效的reference line和停止线
    vector<ReferencePoint> ref_points;
    for(int i = 0 ;i < 50; i++){
      ref_points.emplace_back(math::Vec3d(i*1.0, 0.0,0.0),0.0,0.0,0.0001,0.0);
      ref_points.back().local_s_ = i*1.0;
    }
    ReferenceLine ref_line(ref_points);
    ref_line_info.ref_line_ = ref_line;
    std::vector<StopLine> stop_lines = {StopLine{0.0, 0, 0, 10, DrivingDirection::kDirectionForwardOnly}};
    ref_line_info.ref_line_.stop_lines_ = stop_lines ;

    
    // 设置交通灯决策
    decision_result.traffic_light_info_.is_need_stop = true;
    
    // 设置车辆位置和速度
    auto localization = std::make_shared<Localization>();
    localization->vehicle_align_pose_point_ = MapPoint();
    local_view.getMutableLocalizationPtr() = (localization);
    
    auto chassis = std::make_shared<Chassis>();
    chassis->set_Speed(10.0) ; // 高速移动
    local_view.chassis_ = (chassis);
    
    // 测试交通灯停止场景
    processor_.speed_preprocessor_config_.set_allow_exceed_stop_line(true);
    processor_.addTflSpeedWall(local_view, &ref_line_info, decision_result, false, &speed_walls);
    
    // 验证速度墙是否添加
    EXPECT_FALSE(speed_walls.empty());
}

TEST_F(SpeedWallProcessorTest, InvalidDestinationWall) {
    PathData path_data;
    path_data.remain_dis_info_= make_pair(false, 1.0);
    std::vector<SpeedWall> speed_walls;
    processor_.addDestinationSpeedWall(path_data, &speed_walls);
    EXPECT_TRUE(speed_walls.empty());
}
TEST_F(SpeedWallProcessorTest, ValidDestinationWall) {
    PathData path_data;
    path_data.remain_dis_info_= make_pair(true, 1.0);
    std::vector<SpeedWall> speed_walls;
    processor_.addDestinationSpeedWall(path_data, &speed_walls);
    EXPECT_FALSE(speed_walls.empty());
}

TEST_F(SpeedWallProcessorTest, EmptyBlockFSInfo) {
    PathData path_data;
    BehaviorState behavior_state;
    std::vector<SpeedWall> speed_walls;
    processor_.addFreespaceSpeedWall(path_data, behavior_state, &speed_walls);
    EXPECT_TRUE(speed_walls.empty());
}

TEST_F(SpeedWallProcessorTest, SingleBlockFSInfo) {
    PathData path_data;
    BehaviorState behavior_state;
    std::vector<SpeedWall> speed_walls;

    // Add a single block FS info
    PathData::BlockFSInfo block_fs;
    block_fs.s = 5.0;
    path_data.block_fs_info_.push_back(block_fs);

    processor_.addFreespaceSpeedWall(path_data, behavior_state, &speed_walls);

    ASSERT_EQ(speed_walls.size(), 1);
}


TEST_F(SpeedWallProcessorTest, ParkInState) {
    PathData path_data;
    BehaviorState behavior_state;
    behavior_state.park_in_state_ = true;
    std::vector<SpeedWall> speed_walls;
    PathData::BlockFSInfo block_fs;
    block_fs.s = 5.0;
    path_data.block_fs_info_.push_back(block_fs);

    processor_.addFreespaceSpeedWall(path_data, behavior_state, &speed_walls);

    ASSERT_EQ(speed_walls.size(), 1);
    EXPECT_DOUBLE_EQ(speed_walls[0].stop_distance, 0.5);
}

TEST_F(SpeedWallProcessorTest, NullRef) {
    std::vector<SpeedWall> speed_walls;
    LocalView local_view;
    DecisionResult decision_result;
    
    // 测试nullptr情况
    processor_.addDecisionStopSpeedWall(local_view,decision_result, nullptr, &speed_walls);
    EXPECT_TRUE(speed_walls.empty());
    
    // 测试无效reference line
    ReferenceLineInfo invalid_ref_line_info;
    processor_.addDecisionStopSpeedWall(local_view, decision_result, &invalid_ref_line_info, &speed_walls);
    EXPECT_TRUE(speed_walls.empty());
}

TEST_F(SpeedWallProcessorTest, ValidConstraint) {
    std::vector<SpeedWall> speed_walls;
    LocalView local_view;
    DecisionResult decision_result;
    ReferenceLineInfo ref_line_info;
    
    // 设置有效的reference line和停止线
    vector<ReferencePoint> ref_points;
    for(int i = 0 ;i < 50; i++){
      ref_points.emplace_back(math::Vec3d(i*1.0, 0.0,0.0),0.0,0.0,0.0001,0.0);
      ref_points.back().local_s_ = i*1.0;
    }
    ReferenceLine ref_line(ref_points);
    ref_line_info.ref_line_ = ref_line;
    std::vector<StopLine> stop_lines = {StopLine{10, 0, 0, 10, DrivingDirection::kDirectionUTurnOnly}};
    ref_line_info.ref_line_.stop_lines_ = stop_lines ;
    // 设置车辆位置和速度
    auto localization = std::make_shared<Localization>();
    localization->vehicle_align_pose_point_ = MapPoint();
    local_view.getMutableLocalizationPtr() = (localization);
    WallConstraint wall_straint;
    wall_straint.s = 10.0;
    wall_straint.type = WallType::LONG_JUNCTION_STOP;
    wall_straint.v = 0.0;
    decision_result.longitudinal_bound_decision_.push_back(wall_straint);
    // 设置车辆速度
    auto chassis = std::make_shared<Chassis>();
    chassis->set_Speed(0.5f); // 移动状态
    local_view.chassis_= chassis;
    // 测试掉头场景
    processor_.addDecisionStopSpeedWall(local_view, decision_result, &ref_line_info,  &speed_walls);
    // 验证速度墙是否添加
    EXPECT_FALSE(speed_walls.empty());
}
}
