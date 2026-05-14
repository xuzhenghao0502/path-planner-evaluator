#pragma once

#include <iostream>
#include <queue>
#include <set>

#include "base/log.h"
#include "openspace_path_planner/utils/openspace_common.h"
namespace gpal::pnc::planning {

template <typename Model_t, typename NodeHash = std::hash<typename Model_t::NodeKey_t>>
class AStarSearch {
 public:
  enum AStarSearchStatus : int32_t { FAILED = 0, SUCCESS = 1 };
  struct AStarSearchResult {
    AStarSearchStatus status = AStarSearchStatus::FAILED;  // 使用原来的枚举值
    double search_time = 0.0;                              // 搜索时间（秒）
    uint32_t expanded_nodes_num = 0;                       // 扩展节点数

    AStarSearchResult() : status(AStarSearchStatus::FAILED), search_time(0.0), expanded_nodes_num(0) {}
    AStarSearchResult(AStarSearchStatus _status, double _time, uint32_t _nodes)
        : status(_status), search_time(_time), expanded_nodes_num(_nodes) {}
  };

 public:
  using PathNode = typename Model_t::Node_t;
  using PathNodeControl = typename Model_t::NodeControl_t;
  using PathNodeKey = typename Model_t::NodeKey_t;
  struct SearchNode {
    enum NodeStatus : uint8_t { NONE, OPEN, CLOSED };
    PathNodeKey key;
    double heuristic_cost = 0;
    double move_cost = 0;
    NodeStatus status = NodeStatus::NONE;
    std::shared_ptr<PathNode> path_node;
    std::weak_ptr<SearchNode> predecessor;

    explicit SearchNode(PathNodeKey _key = PathNodeKey()) : key(_key) {}
    double cost() const { return heuristic_cost + move_cost; };
    bool operator==(const SearchNode& rhs) const { return cost() == rhs.cost(); }
    bool operator>(const SearchNode& rhs) const { return cost() > rhs.cost(); }
    bool operator<(const SearchNode& rhs) const { return cost() < rhs.cost(); }
    bool operator>=(const SearchNode& rhs) const { return cost() > rhs.cost() || cost() == rhs.cost(); }
    bool operator<=(const SearchNode& rhs) const { return cost() < rhs.cost() || cost() == rhs.cost(); }
  };

  using SearchNodeMap = std::unordered_map<PathNodeKey, std::shared_ptr<SearchNode>, NodeHash>;

 public:
  AStarSearch() = default;
  virtual ~AStarSearch() = default;
  void Clear() {
    node_map_.clear();
    open_list_ = OpenList();
    search_time_ = 0.0;
    search_count_ = 0;
  }
  
  AStarSearchResult plan(Model_t& model, const PathNode& start) {
    std::vector<PathNode> starts;
    model.setStart(start);
    starts.emplace_back(start);
    return plan(model, starts);
  }
  
  AStarSearchResult plan(Model_t& model, const std::vector<PathNode>& starts) {
    initOpenList(model, starts);
    search_count_ = 0;
    auto start_stamp = std::chrono::steady_clock::now();
    while (!open_list_.empty()) {
      search_count_++;
      // Pop minimum cost node from openlist
      // auto [curr_node, open_cost] = open_list_.top();
      auto curr_node = open_list_.top().first;
      auto open_cost = open_list_.top().second;
      open_list_.pop();
      if (curr_node->status == SearchNode::CLOSED) {
        continue;
      }
      curr_node->status = SearchNode::CLOSED;
      auto curr_stamp = std::chrono::steady_clock::now();
      search_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(curr_stamp - start_stamp).count();
      if (search_time_ > model.searchTimeLimit()) {
        OPENSPACE_LOG(W, "[AStarSearch][plan] AStar search failed, spend ms: ", search_time_,
                      "search count is: ", search_count_);
        return AStarSearchResult(AStarSearchStatus::FAILED, search_time_, search_count_);
      }
      auto successors = generateSuccessors(model, *curr_node);

      if (model.debugPrintSwitch()) {
        for (auto& successors_pair : successors) {
          OPENSPACE_LOG(D, "[AStarSearch][plan]successor_x= ", successors_pair.first.x, " y= ", successors_pair.first.y,
                        " theta= ", successors_pair.first.theta * RAD2ANG,
                        " ctrl= ", successors_pair.second.steer_grade,
                        " direction= ", successors_pair.second.target_direction);
        }
      }

      for (auto& successors_pair : successors) {
        // [successor, ctrl]
        auto successor = successors_pair.first;
        auto ctrl = successors_pair.second;
        
        if (model.isReachTarget(successor, 
                                [&](std::vector<PathNode>* path) { retrievePathNodes(curr_node, successor, path); })) {
          OPENSPACE_LOG(I, "[AStarSearch][plan] success to find path after expand nodes: ", search_count_);
          return AStarSearchResult(AStarSearchStatus::SUCCESS, search_time_, search_count_);
        }
        auto successor_node = getSearchNode(model, successor);
        
        double heuristic_cost = model.calcHeuristicCost(successor);
        double edge_cost = model.calcEdgeCost(*curr_node->path_node, heuristic_cost, successor, ctrl);
        if (updateSearchNode(successor_node, curr_node, successor, edge_cost, heuristic_cost)) {
          open_list_.emplace(successor_node, successor_node->cost());
          successor_node->status = SearchNode::OPEN;
        }
      }
    }
    OPENSPACE_LOG(E, "[AStarSearch][plan] Failed to find path after expand nodes: ", search_count_);
    return AStarSearchResult(AStarSearchStatus::FAILED, search_time_, search_count_);
  }

 private:
  void initOpenList(Model_t& model, const std::vector<PathNode>& starts) {
    std::set<PathNodeKey> key_set;
    for (auto& start : starts) {
      auto successor_node = getSearchNode(model, start);
      auto [iter, success] = key_set.emplace(successor_node->key);
      if (success) {
        open_list_.emplace(successor_node, successor_node->cost());
      }
    }
  }
  std::vector<std::pair<PathNode, PathNodeControl>> generateSuccessors(Model_t& model, const SearchNode& node) {
    std::vector<std::pair<PathNode, PathNodeControl>> successors;
    auto control_space = model.generateFeasibleControlSpace(*node.path_node);
    std::set<PathNodeKey> key_set;
    key_set.emplace(node.key);
    if (node.predecessor.lock() != nullptr) {
      key_set.emplace(node.predecessor.lock()->key);
    }
    for (auto& control : control_space) {
      int step_num = 1;
      while (step_num <= model.moveNums()) {
        auto [flag, child_node] = model.generateSuccessor(*node.path_node, control, step_num);
        if (!flag) break;
        auto child_key = model.generateNodeKey(child_node);
        auto [iter, success] = key_set.emplace(child_key);
        if (success) {
          successors.emplace_back(std::move(child_node), control);
          break;
        }
        step_num++;
      }
    }
    return successors;
  }
  std::shared_ptr<SearchNode> getSearchNode(Model_t& model, const PathNode& path_node) {
    std::shared_ptr<SearchNode> ptr;                     // 智能指针定义一个地址
    auto search_key = model.generateNodeKey(path_node);  // 传入起点返回一个key,包括x,y,theta，和方向前进or后退
    auto [iter, success] = node_map_.emplace(search_key, std::make_shared<SearchNode>(search_key));
    if (success) {
      iter->second->path_node = std::make_shared<PathNode>(path_node);
    }
    return iter->second;
  }
  bool updateSearchNode(std::shared_ptr<SearchNode> node, std::shared_ptr<SearchNode> predecessor_node,
                        const PathNode& path_node, const double edge_cost, const double heurestic_cost) {
    if ((node->status == SearchNode::NONE) ||
        (node->status == SearchNode::OPEN && node->cost() > predecessor_node->move_cost + edge_cost + heurestic_cost)) {
      node->move_cost = predecessor_node->move_cost + edge_cost;
      node->heuristic_cost = heurestic_cost;
      node->path_node = std::make_shared<PathNode>(path_node);
      node->predecessor = predecessor_node;
      return true;
    }
    return false;
  }
  void retrievePathNodes(std::shared_ptr<SearchNode> predecessor_node, const PathNode& path_node,
                         std::vector<PathNode>* path_nodes) {
    std::vector<PathNode> res;
    res.emplace_back(path_node);
    std::shared_ptr<SearchNode> curr = predecessor_node;
    while (curr != nullptr) {
      res.emplace_back(*curr->path_node);
      curr = curr->predecessor.lock();
    }
    path_nodes->insert(path_nodes->end(), res.rbegin(), res.rend());
  }

 private:
  struct SearchNodeCompare {
    bool operator()(std::pair<std::shared_ptr<SearchNode>, double>& lhs,
                    std::pair<std::shared_ptr<SearchNode>, double>& rhs) {
      return lhs.second > rhs.second;
    }
  };
  using OpenList = std::priority_queue<std::pair<std::shared_ptr<SearchNode>, double>,
                                       std::vector<std::pair<std::shared_ptr<SearchNode>, double>>, SearchNodeCompare>;
  SearchNodeMap node_map_;
  OpenList open_list_;
  double search_time_ = 0.0;
  uint32_t search_count_ = 0;
};

}  // namespace gpal::pnc::planning