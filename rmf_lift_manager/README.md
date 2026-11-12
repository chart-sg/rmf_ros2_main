# rmf_lift_manager

Lift manager for Open-RMF. It replaces `lift_supervisor` from
`rmf_fleet_adapter`.

## What it does now
It is a port of `lift_supervisor`. It keeps at most one active `LiftRequest`
per lift:

- It forwards fleet adapter lift requests to the lifts. A request from a
  different session is ignored while the lift is held.
- If a `LiftState` doesn't match the active request (floor, door state or
  session), it republishes the request.
- If a lift shows a session that holds no active request, it keeps sending
  `END_SESSION` until the lift is released.

## Topics

| Dir | Topic | Type | QoS |
|---|---|---|---|
| sub | `adapter_lift_requests` | `rmf_lift_msgs/LiftRequest` | reliable, transient_local, depth 100 |
| pub | `lift_requests` | `rmf_lift_msgs/LiftRequest` | reliable, transient_local, depth 100 |
| sub | `lift_states` | `rmf_lift_msgs/LiftState` | system default, depth 10 |

## Run

```
ros2 run rmf_lift_manager lift_manager
```

In `rmf_demos`, `common.launch.xml` runs it in place of `lift_supervisor`
when `use_lift_manager:=true`. The clinic and hotel demos set this.

## Planned
- A queue that serves one lift selection at a time, using
  `LiftSelectRequest` and `LiftSelectState`.
- Relaying selections to a lift group adapter.
- Per-robot lane setting.

## Tests

```
colcon test --packages-select rmf_lift_manager
colcon test-result --verbose
```
