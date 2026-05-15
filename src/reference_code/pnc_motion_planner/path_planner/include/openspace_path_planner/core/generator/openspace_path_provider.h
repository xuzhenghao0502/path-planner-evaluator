#pragma once

#include <chrono>
#include <signal.h>
#include <sys/time.h>
#include <time.h>

#include "config_manager/config_manager.h"
#include "local_view/local_view.h"
#include "openspace_path_planner/core/generator/a_star_search.h"
#include "openspace_path_planner/core/generator/openspace_roi_decider.h"
#include "openspace_path_planner/core/manager/openspace_search_data.h"
#include "openspace_path_planner/core/openspace_path_planner.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "util/task_handler.h"
#include "util/timer.h"

namespace gpal::pnc::planning {
enum AsyncSearchStatus { ASYNC_SEARCHED = 0, ASYNC_UNDERLOCKING = 1, ASYNC_ERROR_SEARCH = 2, ASYNC_FAILED = -1 };
template<typename Model_t, typename NodeHash = std::hash<typename Model_t::NodeKey_t>>
class PathProviderBaseHAStar : public BaseOpenspacePathPlanner {
 public:
  using PathNode = typename Model_t::Node_t;
  PathProviderBaseHAStar() {};
  ~PathProviderBaseHAStar() {};
  void init() { clear(); };
  void clear();
  OpenspaceStatus run(std::any& data) override;

 private:
  AsyncSearchStatus asyncPlanOriginalPath(std::shared_ptr<OpenspaceSearchData> search_data);
  void setStartPose(const PathPt& start);
  void setGoalPose(const PathPt& end);
  bool hasAsyncProcess() const;
  void dropAsyncProcess();

 private:
  struct SearchData {
    std::atomic<bool> is_finished = false;
    std::mutex mutex;
    std::condition_variable_any cond;
    Model_t model;
    AStarSearch<Model_t, NodeHash> a_star;
    typename AStarSearch<Model_t, NodeHash>::AStarSearchResult result;
  };

  PathNode start_node_;
  PathNode goal_node_;
  std::shared_ptr<SearchData> async_search_data_ = nullptr;
  TaskHandler<1> task_handler_;
};

template<typename Model_t, typename NodeHash>
bool PathProviderBaseHAStar<Model_t, NodeHash>::hasAsyncProcess() const {
  return async_search_data_ != nullptr;
}

template<typename Model_t, typename NodeHash>
void PathProviderBaseHAStar<Model_t, NodeHash>::dropAsyncProcess() {
  if (hasAsyncProcess()) {
    async_search_data_.reset();
  }
}

template<typename Model_t, typename NodeHash>
typename PathProviderBaseHAStar<Model_t, NodeHash>::OpenspaceStatus PathProviderBaseHAStar<Model_t, NodeHash>::run(
    std::any& data) {
  auto search_data = std::any_cast<std::shared_ptr<OpenspaceSearchData>>(data);
  auto search_status = asyncPlanOriginalPath(search_data);
  if (search_status == AsyncSearchStatus::ASYNC_FAILED) {
    return OpenspaceStatus::FAILED;
  }
  if (search_status == AsyncSearchStatus::ASYNC_SEARCHED) {
    return OpenspaceStatus::FINISH;
  } else {
    return OpenspaceStatus::WAITING;
  }
}

template<typename Model_t, typename NodeHash>
AsyncSearchStatus PathProviderBaseHAStar<Model_t, NodeHash>::asyncPlanOriginalPath(
    std::shared_ptr<OpenspaceSearchData> search_data) {
  util::TimerLogger<std::milli> timer("asyncSearchPlan", [](const std::string& str) {
    OPENSPACE_LOG(I, "[PathProviderBaseHAStar][asyncPlanOriginalPath]", str);
  });
  AsyncSearchStatus status = AsyncSearchStatus::ASYNC_SEARCHED;
  search_data->debug_info_ = "search: ";
  if (async_search_data_ == nullptr) {
    std::chrono::time_point<std::chrono::steady_clock> start_stamp_ = std::chrono::steady_clock::now();
    auto async_timer = timer.tap("search");
    auto async_data = std::make_shared<SearchData>();
    {
      auto tap_timer = async_timer.tap("initModel");
      async_data->model.init(search_data->roi_, *search_data->freespace_ptr_, search_data->search_config_name_,
                             search_data->scenario_tags_);
      async_data->a_star.Clear();
      search_data->search_path_.clear();
      setStartPose(search_data->start_pose_);
      setGoalPose(search_data->end_pose_);

      if (search_data->is_region_search_) {
        typename Model_t::TargetRegion tr;
        tr.target_region = search_data->goal_region_;
        tr.target_theta = search_data->goal_region_heading_;
        tr.theta_tolerance = search_data->goal_region_heading_tolerance_ * M_PI / 180.0;
        tr.target_direction = PathNode::DIRECTION::FORWARD;

        async_data->model.setTargetRegion(tr, goal_node_);
      } else {
        async_data->model.setEnd(goal_node_);
      }

      // 重规划需要定义起点方向
      if (std::any_of(search_data->scenario_tags_.begin(), search_data->scenario_tags_.end(),
                      [](const std::string& tag) { return tag == "replan"; })) {
        start_node_.direction = search_data->start_direction_ == PathPt::Direction::FORWARD
                                    ? PathNode::DIRECTION::FORWARD
                                    : PathNode::DIRECTION::BACKWARD;
      }
      OPENSPACE_LOG(
          I, "[PathProviderBaseHAStar][asyncPlanOriginalPath]start_node_ direction : ", (int)start_node_.direction);
    }
    {
      auto tap_timer = async_timer.tap("importTask");
      task_handler_.importTask([=] {
        async_data->result = async_data->a_star.plan(async_data->model, start_node_);
        async_data->is_finished = true;
        async_data->cond.notify_all();
      });
      async_search_data_ = async_data;
    }
  }
  {
    auto tap_timer = timer.tap("underlockSearch");
    std::unique_lock<std::mutex> lck(async_search_data_->mutex);
    int timeout = 10;
    async_search_data_->cond.wait_for(lck, std::chrono::milliseconds(timeout), [=]() {
      bool is_finished = async_search_data_->is_finished;
      return is_finished;
    });
  }
  auto tap_timer = timer.tap("postProcess");
  if (async_search_data_->is_finished) {
    if (async_search_data_->result.status) {
      search_data->search_path_ = async_search_data_->model.getPathResult();
    } else {
      status = AsyncSearchStatus::ASYNC_FAILED;
    }
    search_data->debug_info_ +=
        fmt::format("{} {} {:.3f}\n", static_cast<int>(async_search_data_->result.status),
                    async_search_data_->result.expanded_nodes_num, async_search_data_->result.search_time);
    async_search_data_.reset();
  } else {
    status = AsyncSearchStatus::ASYNC_UNDERLOCKING;
  }
  OPENSPACE_LOG(I, "[PathProviderBaseHAStar][asyncPlanOriginalPath]: " + search_data->debug_info_);
  OPENSPACE_LOG(I, "[PathProviderBaseHAStar][asyncPlanOriginalPath]AsyncSearchStatus : ", (int)status);
  return status;
}

template<typename Model_t, typename NodeHash>
void PathProviderBaseHAStar<Model_t, NodeHash>::clear() {
  start_node_ = PathNode();
  goal_node_ = PathNode();
  dropAsyncProcess();
}

template<typename Model_t, typename NodeHash>
void PathProviderBaseHAStar<Model_t, NodeHash>::setStartPose(const PathPt& start) {
  start_node_.x = start.x();
  start_node_.y = start.y();
  start_node_.theta = start.theta();
  OPENSPACE_LOG(I, "[PathProviderBaseHAStar][setStartPose] : setStartPose : x = ", start_node_.x,
                " y = ", start_node_.y, " theta_deg = ", start_node_.theta * RAD2ANG);
}

template<typename Model_t, typename NodeHash>
void PathProviderBaseHAStar<Model_t, NodeHash>::setGoalPose(const PathPt& end) {
  goal_node_.x = end.x();
  goal_node_.y = end.y();
  goal_node_.theta = end.theta();
  OPENSPACE_LOG(I, "[PathProviderBaseHAStar][setGoalPose] : setGoalPose : x = ", goal_node_.x, " y = ", goal_node_.y,
                " theta_deg = ", goal_node_.theta * RAD2ANG);
}

}  // namespace gpal::pnc::planning