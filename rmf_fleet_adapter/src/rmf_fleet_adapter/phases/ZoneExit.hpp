/*
 * Copyright (C) 2026 Centre for Healthcare Assistive & Robotics
 * Technology (CHART)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__ZONEEXIT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__ZONEEXIT_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <rmf_zone_msgs/msg/zone_request.hpp>
#include <rmf_zone_msgs/msg/zone_state.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
class ZoneExit
{
public:
  class PendingPhase;
  class ActivePhase;
};

//==============================================================================
class ZoneExit::PendingPhase : public LegacyTask::PendingPhase
{
public:

  PendingPhase(
    agv::RobotContextPtr context,
    std::string zone_name,
    rmf_traffic::Time expected_finish,
    std::shared_ptr<rmf_traffic::PlanId> plan_id);

  std::shared_ptr<LegacyTask::ActivePhase> begin() override;

  rmf_traffic::Duration estimate_phase_duration() const override;

  const std::string& description() const override;

private:

  agv::RobotContextPtr _context;
  std::string _zone_name;
  rmf_traffic::Time _expected_finish;
  std::shared_ptr<rmf_traffic::PlanId> _plan_id;
  std::string _description;
};

//==============================================================================
class ZoneExit::ActivePhase
: public LegacyTask::ActivePhase,
  public std::enable_shared_from_this<ActivePhase>
{
public:

  static std::shared_ptr<ActivePhase> make(
    agv::RobotContextPtr context,
    std::string zone_name,
    rmf_traffic::Time expected_finish,
    std::shared_ptr<rmf_traffic::PlanId> plan_id);

  const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

  rmf_traffic::Duration estimate_remaining_time() const override;

  void emergency_alarm(bool on) override;

  void cancel() override;

  const std::string& description() const override;

private:

  agv::RobotContextPtr _context;
  std::string _zone_name;
  std::string _assigned_waypoint;
  rxcpp::observable<LegacyTask::StatusMsg> _obs;

  rclcpp::Subscription<rmf_zone_msgs::msg::ZoneState>::SharedPtr _state_sub;
  rclcpp::Publisher<rmf_zone_msgs::msg::ZoneRequest>::SharedPtr _request_pub;

  rclcpp::TimerBase::SharedPtr _timeout_timer;
  rclcpp::TimerBase::SharedPtr _delay_timer;
  rmf_traffic::Time _expected_finish;
  std::shared_ptr<rmf_traffic::PlanId> _plan_id;
  std::string _description_text;

  ActivePhase(
    agv::RobotContextPtr context,
    std::string zone_name,
    rmf_traffic::Time expected_finish,
    std::shared_ptr<rmf_traffic::PlanId> plan_id);
  void _init_obs();
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__ZONEEXIT_HPP
