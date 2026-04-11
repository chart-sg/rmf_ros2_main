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

#include "Node.hpp"

#include <rclcpp/executors.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rmf_fleet_adapter::zone_supervisor::Node>();

  // MultiThreadedExecutor is required for future synchronous sensor service
  // calls. Using it from here ensures no executor change is
  // needed when sensor support is added. The MutuallyExclusive callback group
  // on zone request processing still ensures zone_log serialization.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
