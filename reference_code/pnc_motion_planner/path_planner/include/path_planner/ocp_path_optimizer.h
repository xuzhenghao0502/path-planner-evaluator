/**
 * @file ocp_path_optimizer.h
 * @brief OCP路径优化器实现文件
 * @details 该文件实现了基于最优控制问题（OCP）的路径优化器，用于在路径规划过程中对路径进行平滑和优化，确保路径的连续性和可行性。
 */

#pragma once
#include <memory>
#include <atomic>
#include "util/task_handler.h"
#include "util/abstract_factory.h"
#include "point/path_pt.h"
#include "proto/vehicle_state/vehicle_state.pb.h"
#include "config/path_planner/ocp_path_optimizer_config.pb.h"
#include "ocp/ocp_model.h"
#include "math/math_utils.h"
#include "base/status.h"
#include "path_planner/path_optimizer.h"

namespace gpal::pnc::planning {

/**
 * @brief OCP路径优化器类,实现了基于最优控制问题（OCP）的路径，用于在路径规划过程中对路径进行平滑和优化，确保路径的连续性和可行性。
 */
class OcpPathOptimizer : public PathOptimizer {
  public:
    enum AsyncStatus
    {
      ASYNC_SOLVED = 0, ///< 异步求解成功
      ASYNC_UNDERLOCKING = 1,  ///< 异步求解中
      ASYNC_ERROR_SOLVED = 2, ///< 异步求解错误
      ASYNC_FAILED = -1, ///< 异步求解失败
      INFEASIBLED = -2, ///< 求解不可行
      PREPROCESS_FAILED = 3, ///< 预处理失败
      POST_CHECK_INVALID = 4  ///< 后校验无效
    };

    struct SolverData
    {
      std::atomic<bool> is_finished = false; ///< 是否完成
      std::mutex mutex; ///< 互斥锁
      std::condition_variable_any cond;  ///< 条件变量
      std::string ref_id = ""; ///< 参考ID
      std::string profile = ""; ///< 配置
      SolveStatus status = SolveStatus::SOLVER_UNINIT; ///< 求解状态
      std::shared_ptr<OptimalControlProblem> model = nullptr; ///< 最优控制问题模型
      std::vector<PathData::DebugStatusType> debug_status; ///< 调试状态
    };

    enum UpdateParamsMethod {
      UNKNOWN = -1, ///< 未知更新方法
      LATERAL_GENERAL = 0, ///< 横向通用模型参数
      LATERAL_UNCONSTRAINED = 1, ///< 横向无约束模型参数
      LATERAL_LANE_KEEP = 2, ///< 横向车道保持模型参数
    };

    struct ModelInfo {
      std::string ref_id = ""; ///< 参考ID
      std::string profile = ""; ///< 配置
      UpdateParamsMethod update_method = UpdateParamsMethod::UNKNOWN; ///< 更新方法
      std::shared_ptr<OptimalControlProblem> model = nullptr; ///< 最优控制问题模型
      SolveStatus status = SolveStatus::SOLVER_UNINIT; ///< 求解状态
    };
    
    using BoundRes = std::pair<std::tuple<double, double, double>, bool>;  ///< s, lmin, lmax, isvalid

  public:
    virtual bool init() override;

  /**
   * @brief 获取最优控制路径优化器类型标识
   * @return 返回优化器类型名称字符串 "OcpPathOptimizer"
   */
    virtual std::string name() const { return "OcpPathOptimizer"; }
    bool hasAsyncProcess() const;
    void dropAsyncProcess();
    virtual Status proc(const ReferenceLine &reference_line, const TrajectoryPt &start_point,
                        const PathBoundary &boundary, PathData *const path_data) override;
    AsyncStatus asyncProc(const std::chrono::milliseconds &timeout, const ReferenceLine &reference_line,
                          const TrajectoryPt &start_point, const PathBoundary &boundary, PathData *const path_data);
    /**
     * @brief 获取异步规划器调试信息
     * @return 返回异步规划器调试信息
     */
    std::string getAsyncPlannerDebugInfo() { return async_planner_debug_info_; }

    // only for gtest
    /**
     * @brief 设置求解器信息（仅用于测试）
     * @param solver_info 求解器信息
     */
    void setSolverInfo(const SolveInfo& solver_info) { gtest_solver_info_ = solver_info; }

    /**
     * @brief 获取求解器信息（仅用于测试）
     * @return 返回求解器信息
     */
    const SolveInfo& getSolverInfo() const { return gtest_solver_info_; }
    const SolveInfo& solverInfo() const { return solver_info_; }

    Status unconstrainedProc(const ReferenceLine &reference_line, const TrajectoryPt &start_point,
                             const PathBoundary &boundary, PathData *const path_data);
    DiscretizedPath generateProtectPath(const VehicleState &curr_state,
                                        const ReferenceLine& reference_line,
                                        const SpeedData &prev_speed_data,
                                        const DiscretizedPath &path, const int64_t curr_stamp,
                                        const double max_length = 40.0);
    /**
     * @brief 获取fallback结果
     * @return 返回fallback结果
     */
    shared_ptr<PathData> getFallbackResult() { return fallback_result_; }
    void setSpecialSceneRange(std::vector<std::tuple<std::string, float, float>> special_scene_range);
    void setRefOffsetsInfo(const std::vector<std::pair<double, double>> &ref_offsets_info);
    void setIsLaneChange(bool is_lane_change) { is_lane_change_ = is_lane_change; }

    /**
     * @brief 确保 curr_profile_type_ 已初始化为有效配置键
     * @details 用于在跳过 proc()（约束 OCP）的场景（如无约束横向模式）下，
     *          直接调用 unconstrainedProc() 前需保证 profile() 不崩溃。
     *          优先使用 preferred_label，若不存在则回退到 "regular"，
     *          再不行则取 profiles 中第一个可用 key。
     * @param preferred_label 期望使用的 profile 标签（通常为 "regular"）
     * @return true 表示成功初始化，false 表示 profiles 为空
     */
    bool ensureProfileType(const std::string& preferred_label = "regular");
   protected:
    bool reset();
    bool preProcess(const ReferenceLine &reference_line, const TrajectoryPt &start_point, const PathBoundary &boundary, const DiscretizedPath &prev_path);
    std::shared_ptr<OptimalControlProblem> initModel(const OcpPathOptimizerProfile &profile,
                                                     const DiscretizedPath &prev_path);
    bool initLateralGeneral(std::shared_ptr<OptimalControlProblem> model);
    bool updateLateralGeneral(std::shared_ptr<OptimalControlProblem> model, const DiscretizedPath &prev_path);
    bool applyBicycleLateralCtrlPolycy(std::shared_ptr<OptimalControlProblem> model, const size_t idx);
    bool tranStatesToPathData(std::shared_ptr<OptimalControlProblem> model, const ReferenceLine &reference_line,
                              const PathBoundary &boundary, PathData *const path_data);
    bool tranStatesToPathData(std::shared_ptr<OptimalControlProblem> model, const ReferenceLine &reference_line,
                              shared_ptr<PathData> fallback_result);
    void checkValid(std::shared_ptr<SolverData> data);
    void fillOcpPathData(std::shared_ptr<OptimalControlProblem> model, const PathBoundary &boundary,
                         PathData *const path_data);

    std::shared_ptr<OptimalControlProblem> initUnconstrainModel(const OcpPathOptimizerProfile &profile,
                                                                const DiscretizedPath &prev_path);
    bool initUnconstrainedLateralGeneral(std::shared_ptr<OptimalControlProblem> model);

    bool getSpeedPointsFromPrevSpeedData(const SpeedData &prev_speed_data, const int64_t curr_stamp, const double min_v,
                                         const double t0, const double t1, const double dt, const double max_s,
                                         std::vector<gpal::pnc::SpeedPoint> *speed_points);
    bool getSpeedPointsFromCVModel(const double v0, const double t0, const double t1, const double dt, const double max_s,
                                   std::vector<gpal::pnc::SpeedPoint> *speed_points);
    std::shared_ptr<OptimalControlProblem> initProtectPath(const std::string &profile,
                                                           const std::vector<gpal::pnc::SpeedPoint> &speed_points,
                                                           const DiscretizedPath &path,
                                                           const VehicleState &curr_state);
    bool initBicycleTrajectoryTracker(std::shared_ptr<OptimalControlProblem> model,
                                      const std::vector<gpal::pnc::SpeedPoint> &speed_points, const DiscretizedPath &path,
                                      const VehicleState &curr_state, const bool warm_start = false);
    bool applyBicycleTrajectoryTrackerCtrlPolycy(std::shared_ptr<OptimalControlProblem> model, const size_t& idx, const double& ref_local_s);

  private:
    void considerVehicleWidth(const double width, OcpPathOptimizer::BoundRes *bound_res);
    bool hasValidPrevPath();
    bool hasValidPrevModel();
    const OcpPathOptimizerProfile &profile() const;
    const ReferencePoint getInitPoint(const double s);
    const ReferencePoint getInitPoint(const double x, const double y);
    const DrivingDirection getInitPointDrivingDirection(const double s);
    double getUnifySpaceHeading(const double heading_base, const double heading);
    std::pair<std::tuple<double, double, double>, bool> getBarrierCenterBound(const ReferencePoint &rpt);
    std::pair<std::tuple<double, double, double>, bool> getBarrierEdgeBound(const ReferencePoint &rpt);
    std::pair<std::tuple<double, double, double>, bool> getSoftCenterBound(const double s, const bool allow_cross = false);
    std::pair<std::tuple<double, double, double>, bool> getSoftEdgeBound(const double s, const bool allow_cross = false);
    std::pair<std::tuple<double, double>, bool> getRefOffset(const ReferencePoint& rpt);
    double getMaxKappaBound(const double curr_length, const double init_v, DrivingDirection ref_direction) const;
    double getMaxDKappaBound(const double curr_length, const double init_v, DrivingDirection ref_direction) const;
    double getMaxSteerAngleBound(const double curr_length, const double init_v, DrivingDirection ref_direction) const;
    double getMaxDSteerAngleBound(const double curr_length, const double init_v, DrivingDirection ref_direction) const;
    const std::vector<std::tuple<std::string, float, float>> &getSpecialSceneRange() const;
    double getSpeedLimit(const double curr_s, const double ref_kappa);

  private:
    TaskHandler<1> task_handler_;  ///< 任务处理线程管理
    std::shared_ptr<SolverData> async_data_ = nullptr; ///< 异步数据
    OcpPathOptimizerConfig optimizer_config_; ///< 优化器配置
    std::string curr_profile_type_ = ""; ///< 当前配置
    UpdateParamsMethod curr_update_params_method_ = UpdateParamsMethod::UNKNOWN; ///< 当前横向模型参数更新方法
    double ds_ = 0; ///< 步长
    size_t N_ = 0;  ///< 步数
    ModelInfo prev_model_info_;
    std::shared_ptr<OptimalControlProblem> protect_path_model_ = nullptr;  ///< 兜底路径模型
    std::tuple<bool, double, PathPt> prev_path_info_;  ///< 历史路径信息  prev_path_valid, prev_ref_s, prev_start_pt 
    std::unique_ptr<ReferenceLine> target_ref_line_ = nullptr; ///< 目标参考线
    std::vector<double> accumulated_s_; ///< 累积的s值 
    std::pair<std::array<double, 3>, std::array<double, 3>> sl_planning_start_point_; ///< SL规划起始点
    TrajectoryPt xy_planning_start_point_; ///< XY规划起始点
    double lane_keep_start_s_ = 0.0; ///< 车道保持起始s值
    util::AbstractTable1d<double, double, double> decision_path_boundary_; ///< 决策路径边界
    util::AbstractTable1d<double, double, double> barrier_path_boundary_; ///< 障碍物路径边界
    util::AbstractTable1d<double, double, double> soft_path_boundary_; ///< 软路径边界
    util::AbstractTable1d<double, double> ref_offsets_info_; ///< 参考偏移信息
    std::vector<std::tuple<std::string, float, float>> special_scene_range_; ///< 特殊场景范围
    std::unordered_map<std::string, int32_t> priority_name_map_; ///< 优先级名称映射
    std::shared_ptr<PathData> fallback_result_ = nullptr; ///< fallback结果
    std::string async_planner_debug_info_ = ""; ///< 异步规划器调试信息
    std::vector<double> kappa_table_ = {0.001, 0.002, 0.004, 0.01, 0.02, 0.05, 0.1}; ///< 曲率表
    std::vector<double> lat_a_limit_; ///< 横向加速度限制
    double starting_acc_ = 1.0; ///< 起始加速度
    
    // only for gtest
    SolveInfo gtest_solver_info_; ///< 测试用求解信息
    double cost_weight_max_decay_length_ = 100.0; ///< 用于优化cost的最大衰减长度
    bool is_lane_change_ = false; ///< 是否变道

    SolveInfo solver_info_; ///< 求解信息
  };
}