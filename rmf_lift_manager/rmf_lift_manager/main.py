# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS node for the lift manager."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rmf_lift_msgs.msg import LiftRequest
from rmf_lift_msgs.msg import LiftState

from .lift_manager_core import LiftManagerCore

# Same names as rmf_fleet_adapter/StandardNames.hpp
FINAL_LIFT_REQUEST_TOPIC = 'lift_requests'
ADAPTER_LIFT_REQUEST_TOPIC = 'adapter_lift_requests'
LIFT_STATE_TOPIC = 'lift_states'


class LiftManagerNode(Node):
    """Wire the lift manager core to its topics."""

    def __init__(self):
        super().__init__('rmf_lift_manager')

        # Same QoS as lift_supervisor (SystemDefaultsQoS().keep_last(10)):
        # reliability and durability are left to the middleware. On
        # CycloneDDS this resolves to reliable/volatile for both nodes.
        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.SYSTEM_DEFAULT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT)
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._lift_request_pub = self.create_publisher(
            LiftRequest, FINAL_LIFT_REQUEST_TOPIC, transient_qos)

        self._core = LiftManagerCore(
            publish=self._lift_request_pub.publish,
            now=lambda: self.get_clock().now().to_msg(),
            log_info=self.get_logger().info)

        self._adapter_lift_request_sub = self.create_subscription(
            LiftRequest, ADAPTER_LIFT_REQUEST_TOPIC,
            self._core.on_adapter_lift_request, transient_qos)
        self._lift_state_sub = self.create_subscription(
            LiftState, LIFT_STATE_TOPIC,
            self._core.on_lift_state, default_qos)


def main(args=None):
    """Run the lift manager node."""
    rclpy.init(args=args)
    node = LiftManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
