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

#ifndef SRC__RMF_FLEET_ADAPTER__TASKS__ZONE_HPP
#define SRC__RMF_FLEET_ADAPTER__TASKS__ZONE_HPP

#include <rmf_task_sequence/Phase.hpp>

#include "../agv/internal_FleetUpdateHandle.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
void add_zone(
  agv::TaskDeserialization& deserialization);

} // namespace tasks
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKS__ZONE_HPP
