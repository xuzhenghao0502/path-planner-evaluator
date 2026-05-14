/**
 * @file local_path_optimizer.h
 * @brief 局部路径优化器实现文件
 * @details 该文件实现了局部路径优化器的核心功能，包括路径优化、速度点生成、模型初始化等。局部路径优化器用于在路径规划过程中对局部路径进行优化，确保路径的连续性和可行性。
 */

#pragma once
#include <memory>
#include <atomic>
#include "path_planner_base.h"
#include "path_optimizer.h"
#include "util/task_handler.h"
#include "util/abstract_factory.h"
#include "point/path_pt.h"
#include "config/path_planner/local_path_optimizer_config.pb.h"

#include "ocp/ocp_model.h"
#include "math/math_utils.h"
#include "base/status.h"

namespace gpal::pnc::planning {

/**
 * @brief 局部路径优化器类,实现了局部路径优化器的核心功能。
 */
class LocalPathOptimizer : public PathOptimizer {
 public:
  /**
   * @brief 局部路径生成状态
   */
  enum LocalPathStatus {
    SOLVED = 0,  ///< 路径优化成功
    SOLVE_FAILD, ///< 路径优化失败
  };

 public:
  virtual bool init() override;

  /**
   * @brief 获取路径优化器类型标识
   * @return 返回优化器类型名称字符串 "LocalPathOptimizer"
   */
  virtual std::string name() const { return "LocalPathOptimizer"; }
  Status proc(const ReferenceLine& reference_line, const TrajectoryPt& start_point, const PathBoundary& boundary,
              PathData* const final_path_data) { return Status::OK(); }
  
  bool preProcess(const TrajectoryPt& start_point, const DiscretizedPath& prev_path, const std::string& profile_type);
  LocalPathStatus generateLocalPathProc(const VehicleState& curr_state, const SpeedData& prev_speed_data,
                                       const DiscretizedPath& path, const int64_t curr_stamp,
                                       const double max_length = 50.0);
  DiscretizedPath GenerateFallBackLocalPath(int direction, const VehicleState& curr_state, float cur_speed, float cur_s);

  /**
   * @brief 设置配置文件类型
   * @param profile 配置文件类型
   */
  void setProfile(std::string profile) { profile_type_ = profile; };

  /**
   * @brief 获取配置文件类型
   * @return 返回配置文件类型
   */
  std::string getProfile() { return profile_type_; };

  /**
   * @brief 获取局部路径结果
   * @return 返回局部路径结果
   */
  DiscretizedPath getLocalPathResult() { return local_path_res_; };

  /**
   * @brief 获取局部路径初始猜测
   * @return 返回局部路径初始猜测
   */
  DiscretizedPath getLocalPathInitGuess() { return local_path_guess_; };

   /**
   * @brief 获取是否处于倒车规划模式
   * @return 返回是否处于倒车规划模式
   */
  bool getReverseMode(){return is_reverse_plan_;};
  
 protected:
  std::shared_ptr<OptimalControlProblem> initLocalPathModel(const LocalPathOptimizerProfile& profile,
                                                            const std::vector<gpal::pnc::SpeedPoint>& speed_points,
                                                            const DiscretizedPath& path,
                                                            const VehicleState& curr_state);
  bool initBicycleTrajectoryTracker(std::shared_ptr<OptimalControlProblem> model,
                                    const std::vector<gpal::pnc::SpeedPoint>& speed_points, const DiscretizedPath& path,
                                    const VehicleState& curr_state, const bool warm_start = false);
  bool applyBicycleTrajectoryTrackerCtrlPolycy(std::shared_ptr<OptimalControlProblem> model, const size_t& idx);
  bool getSpeedPointsFromPrevSpeedData(const SpeedData& prev_speed_data, const int64_t curr_stamp, const double min_v,
                                       const double t0, const double t1, const double dt, const double max_s,
                                       std::vector<gpal::pnc::SpeedPoint>* speed_points);
  bool getSpeedPointsFromCVModel(const double v0, const double t0, const double t1, const double dt, const double max_s,
                                 std::vector<gpal::pnc::SpeedPoint>* speed_points);

  double getUnifySpaceHeading(const double heading_base, const double heading);
  double getMaxKappaBound(const double init_v) const;
  double getMaxDKappaBound(const double init_v) const;
  const LocalPathOptimizerProfile& profile() const;


 private:
  /**
   * @brief 局部路径优化求解数据
   */
  struct SolverData {
    std::atomic<bool> is_finished = false; ///< 是否完成
    std::mutex mutex; ///< 互斥锁
    std::condition_variable_any cond; ///< 条件变量
    std::string ref_id = ""; ///< 参考ID
    std::string profile = ""; ///< 配置
    SolveStatus status; ///< 求解状态
    std::shared_ptr<OptimalControlProblem> model = nullptr; ///< 最优控制问题模型
  };
  LocalPathOptimizerConfig optimizer_config_; ///< 局部路径优化器配置
  std::string profile_type_ = "";  ///< 配置文件类型
  TrajectoryPt xy_planning_start_point_; ///< 规划起始点
  std::shared_ptr<OptimalControlProblem> local_path_model_ = nullptr; ///< 局部路径优化求解模型
  DiscretizedPath local_path_res_; ///< 局部路径规划结果
  DiscretizedPath local_path_guess_; ///< 局部路径规划初始猜测
  bool is_reverse_plan_ = false; ///< 是否处于倒车规划模式
};

}
