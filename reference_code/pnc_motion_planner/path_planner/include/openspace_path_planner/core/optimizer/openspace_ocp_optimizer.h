#pragma once
#include <atomic>
#include <memory>

#include "point/path_pt.h"
#include "basic_algorithm_lib/basic_algorithm_lib.h"
#include "config/path_planner/ocp_path_optimizer_config.pb.h"
#include "base/status.h"
#include "math/math_utils.h"
#include "ocp/ocp_model.h"
#include "openspace_path_planner/core/optimizer/openspace_ocp_bound_parser.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "util/task_handler.h"
namespace gpal::pnc::planning {

class OpenspaceOcpOptimizer {
 public:
  enum AsyncStatus { ASYNC_SOLVED = 0, ASYNC_UNDERLOCKING = 1, ASYNC_ERROR_SOLVED = 2, ASYNC_FAILED = -1 };
  using BoundRes = std::pair<std::tuple<double, double, double>, bool>;  // s, lmin, lmax, isvalid
  using SlPointType = std::pair<std::array<double, 3>, std::array<double, 3>>;
  using DirectinMark = vector<std::tuple<double, double, PathPt::Direction>>;

 public:
  OpenspaceOcpOptimizer() = default;
  ~OpenspaceOcpOptimizer() = default;

  bool init();
  bool reset();
  std::string name() { return "OpenspaceOcpOptimizer"; }
  AsyncStatus asyncProc(const DiscretizedPath& orin_path, const PathPt& init_point, const PathBoundary& boundary,
                        DiscretizedPath& final_path);
  void getOCPTrailerPath(vector<PathPt>& trailer_path) { trailer_path_ = trailer_path; };
  std::string getDebugInfo() { return debug_info_; };

 protected:
  bool preProcess(const DiscretizedPath& orin_path, const PathPt& init_point, const PathBoundary& boundary);
  std::shared_ptr<OptimalControlProblem> initModel(const OcpPathOptimizerProfile& profile);
  bool initParkingGeneral(std::shared_ptr<OptimalControlProblem> model);
  bool initFallback(std::shared_ptr<OptimalControlProblem> model);

  bool tranStatesToDiscretizedPath(const TrajectoryPt& init_point, std::shared_ptr<OptimalControlProblem> model,
                                   DiscretizedPath& final_path);
  double getUnifySpaceHeading(const double heading_base, const double heading);
  const OcpPathOptimizerProfile& profile() const;
  const PathPt getInitPoint(const double s);
  void considerVehicleWidth(const double width, OpenspaceOcpOptimizer::BoundRes* bound_res);

  bool updateParkingGeneral(std::shared_ptr<OptimalControlProblem> model);

  bool hasAsyncProcess() const;
  void dropAsyncProcess();
  std::pair<double, double> parseVelocity(const double& curr_s);

 private:
  vector<PathPt> trailer_path_;
  bool has_inited_ = false;
  OcpPathOptimizerConfig config_;
  std::shared_ptr<VehicleParam> vehicle_param_;
  std::shared_ptr<VehicleConfig> vehicle_config_;
  std::string curr_profile_type_ = "";
  double ds_ = 0;
  size_t N_ = 0;

  const DiscretizedPath* target_ref_path_;
  PathPt xy_planning_start_point_;
  double planning_start_s_ = 0.0;
  std::vector<double> accumulated_s_;
  double min_v_square_ = 1.0;

  util::AbstractTable1d<double, double, double> decision_path_boundary_;
  util::AbstractTable1d<double, double, double> barrier_path_boundary_;
  util::AbstractTable1d<double, double, double> soft_path_boundary_;

  std::chrono::time_point<std::chrono::steady_clock> start_stamp_;
  std::chrono::time_point<std::chrono::steady_clock> end_stamp_;
  std::string debug_info_ = "";

 private:
  struct SolverData {
    std::atomic<bool> is_finished = false;
    std::mutex mutex;
    std::condition_variable_any cond;
    std::string ref_id = "";
    std::string profile = "";
    SolveStatus status = SolveStatus::SOLVER_UNINIT;
    std::shared_ptr<OptimalControlProblem> model = nullptr;
  };
  std::shared_ptr<SolverData> async_data_ = nullptr;
  TaskHandler<1> task_handler_;

 public:
  void setScenarioTags(std::vector<std::tuple<std::string, float, float, bool>> scenario_tags);

 private:
  const std::vector<std::tuple<std::string, float, float, bool>>& scenarioTags() const;
  void addScenarioTag(std::tuple<std::string, float, float, bool> scenario_tag);

  std::vector<std::tuple<std::string, float, float, bool>> scenario_tags_;
  std::unordered_map<std::string, int32_t> priority_map_;
};

}  // namespace gpal::pnc::planning