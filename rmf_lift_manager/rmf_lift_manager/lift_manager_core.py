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

"""Lift manager logic, kept free of ROS node code so it can be tested."""

import copy

from rmf_lift_msgs.msg import LiftRequest


class LiftManagerCore:
    """
    Forward adapter lift requests and keep lifts in line with them.

    Port of rmf_fleet_adapter's lift_supervisor. Holds at most one active
    request per lift.
    """

    def __init__(self, publish, now, log_info):
        """
        Create the core.

        publish: callable taking a LiftRequest, sends it on lift_requests.
        now: callable returning a builtin_interfaces/Time.
        log_info: callable taking a string.
        """
        self._publish = publish
        self._now = now
        self._log_info = log_info
        # lift_name -> active LiftRequest
        self._active_sessions = {}

    def on_adapter_lift_request(self, msg):
        """Handle a LiftRequest from a fleet adapter."""
        current = self._active_sessions.get(msg.lift_name)
        is_end = msg.request_type == LiftRequest.REQUEST_END_SESSION

        if current is None:
            if not is_end:
                self._log_info(
                    f'[{msg.session_id}] Received new adapter lift request '
                    f'to [{msg.destination_floor}] with request type '
                    f'[{msg.request_type}]')
                self._store_and_publish(msg)
            return

        if current.session_id != msg.session_id:
            # Another session holds this lift: ignore, as lift_supervisor.
            return

        if is_end:
            msg.request_time = self._now()
            self._publish(msg)
            self._log_info(
                f'[{msg.session_id}] Published end lift session from '
                'lift manager')
            del self._active_sessions[msg.lift_name]
            return

        # Same timestamp so it doesn't confuse the comparison.
        compared = copy.deepcopy(msg)
        compared.request_time = current.request_time
        if compared != current:
            self._log_info(
                f'[{msg.session_id}] Received updated adapter lift request '
                f'to [{msg.destination_floor}] with request type '
                f'[{msg.request_type}]')
        self._store_and_publish(msg)

    def on_lift_state(self, state):
        """Handle a LiftState: republish or release as needed."""
        request = self._active_sessions.get(state.lift_name)

        if request is not None:
            correct_floor = (
                request.destination_floor == state.destination_floor
                or (request.destination_floor == state.current_floor
                    and not state.destination_floor))
            if (not correct_floor
                    or request.door_state != state.door_state
                    or request.session_id != state.session_id):
                request.request_time = self._now()
                self._publish(request)
        elif state.session_id:
            # The lift shows a session nobody holds: keep asking it to end
            # so that the lift gets released.
            release = LiftRequest()
            release.lift_name = state.lift_name
            release.destination_floor = state.current_floor
            release.session_id = state.session_id
            release.request_time = self._now()
            release.request_type = LiftRequest.REQUEST_END_SESSION
            self._publish(release)

    def _store_and_publish(self, msg):
        msg.request_time = self._now()
        self._active_sessions[msg.lift_name] = msg
        self._publish(msg)
