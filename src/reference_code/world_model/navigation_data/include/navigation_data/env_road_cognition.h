#pragma once

#include "local_router.h"

namespace gpal::pnc::planning {

using namespace std;
using gpal::pnc::planning::Status;

/**
 * @brief EnvRoadCognition类
 * @details 用于表示道路结构认知信息，继承自StampedBase
 */
class EnvRoadCognition : public StampedBase {
 public:
  friend class EnvRoadCognitionAdapter;
  explicit EnvRoadCognition();
  ~EnvRoadCognition();

  void clear();

  const proto::road_cognition::EnvRoadCognition::MapProviderType& getMapProvider() const { return map_provider_; }
  void setMapProvider(const proto::road_cognition::EnvRoadCognition::MapProviderType& map_provider) {
    map_provider_ = map_provider;
  }

  const proto::road_cognition::StatusInfo& getStatusInfo() const { return status_info_; }
  void setStatusInfo(const proto::road_cognition::StatusInfo& status_info) { status_info_ = status_info; }

  /**
   * @brief 获取参考线列表
   * @details 返回参考线列表的常量引用，不允许外部修改
   *
   * @return 参考线列表的常量引用
   */
  const std::vector<proto::road_cognition::ReferenceLine>& getReferenceLines() const { return reference_lines_; }
  /**
   * @brief 获取可修改的参考线列表
   * @details 返回参考线列表的指针，允许外部修改
   *
   * @return 参考线列表的指针
   */
  std::vector<proto::road_cognition::ReferenceLine>* getMutableReferenceLines() { return &reference_lines_; }
  void setReferenceLines(const std::vector<proto::road_cognition::ReferenceLine>& reference_lines) {
    reference_lines_ = reference_lines;
  }

  /**
   * @brief 获取区域列表
   * @details 返回区域列表的常量引用，不允许外部修改
   *
   * @return 区域列表的常量引用
   */
  const std::vector<PerceptionArea>& getAreas() const { return areas_; }
  /**
   * @brief 获取可修改的区域列表
   * @details 返回区域列表的指针，允许外部修改
   *
   * @return 区域列表的指针
   */
  std::vector<PerceptionArea>* mutableAreas() { return &areas_; }
  void setAreas(const std::vector<PerceptionArea>& areas) { areas_ = areas; }

  const std::vector<RiskBoundary>& getRiskBoundaries() const { return risk_boundaries_; }
  std::vector<RiskBoundary>* getMutableRiskBoundaries() { return &risk_boundaries_; }
  void setRiskBoundaries(const std::vector<RiskBoundary>& risk_boundaries) { risk_boundaries_ = risk_boundaries; }

  const std::map<std::string, EnvTrafficLight>& getTrafficLightMap() const { return traffic_lights_; }
  std::map<std::string, EnvTrafficLight>* getMutableTrafficLightMap() { return &traffic_lights_; }

  const std::vector<EnvRoad>& getRoads() const { return roads_; }
  std::vector<EnvRoad>* getMutableRoads() { return &roads_; }
  void setRoads(const std::vector<EnvRoad>& roads) { roads_ = roads; }

  const proto::road_cognition::ScenarioInfo& getScenarioInfo() const { return scenario_info_; }
  void setScenarioInfo(const proto::road_cognition::ScenarioInfo& scenario_info) { scenario_info_ = scenario_info; }

  const proto::HpaRoutingInfo::DestinationInfo& getDestinationInfo() const { return dest_info_; }
  void setDestinationInfo(const proto::HpaRoutingInfo::DestinationInfo& dest_info) { dest_info_ = dest_info; }

  const std::vector<EnvLaneMarking>& getLaneMarkings() const { return lane_markings_; }
  std::vector<EnvLaneMarking>* getMutableLaneMarkings() { return &lane_markings_; }

 protected:
  proto::road_cognition::EnvRoadCognition::MapProviderType map_provider_ =
      proto::road_cognition::EnvRoadCognition_MapProviderType_kMapProviderAmapSd;
  proto::road_cognition::StatusInfo status_info_;
  std::vector<proto::road_cognition::ReferenceLine> reference_lines_;  ///< 参考线列表
  std::vector<PerceptionArea> areas_;                                  ///< 区域列表
  std::vector<RiskBoundary> risk_boundaries_;                          ///< 风险边界数组
  std::map<std::string, EnvTrafficLight> traffic_lights_;                        //< 交通灯数组
  std::vector<EnvRoad> roads_;                                         //< 道路信息数组
  proto::road_cognition::ScenarioInfo scenario_info_;                  //< 场景信息
  proto::HpaRoutingInfo::DestinationInfo dest_info_;                   //< 导航终点信息
  std::vector<EnvLaneMarking> lane_markings_;                  //< 指定范围内（基于下游需求）的车道边界线(含道路边界线）信息
};

}  // namespace gpal::pnc::planning
