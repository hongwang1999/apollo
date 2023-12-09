/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/planner/public_road/public_road_planner.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::TrajectoryPoint;

Status PublicRoadPlanner::Init(const PlanningConfig& config) {
  config_ = config;
  //场景注册
  //ScenarioManager负责实际的场景注册，而这个场景管理器读取/modules/planning/config/scenario目录下的配置文件
  //具体在代码中是通过ScenarioManager::RegisterScenario这个函数对场景配置文件进行读取
  //这里注意的是在ScenarioManager::Init()初始化的时候，也就是刚开始执行planning场景时会定义一个默认的场景LANE_FOLLOW
  scenario_manager_.Init(config);
  return Status::OK();
}
//https://blog.csdn.net/sinat_52032317/article/details/128456949
Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
  scenario_manager_.Update(planning_start_point, *frame);
  scenario_ = scenario_manager_.mutable_scenario();
  auto result = scenario_->Process(planning_start_point, frame);

  if (FLAGS_enable_record_debug) {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();
    scenario_debug->set_scenario_type(scenario_->scenario_type());
    scenario_debug->set_stage_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }

  if (result == scenario::Scenario::STATUS_DONE) {
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, *frame);
  } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
    return Status(common::PLANNING_ERROR, "scenario returned unknown");
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
