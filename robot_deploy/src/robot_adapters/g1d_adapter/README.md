# g1d_adapter

ROS 2 adapter for the **Unitree G1-D wheel base**: camera → VLN → MPC → AGV.
Arm, torso, lift and hand control are not included.

## Dependencies and inputs

- ROS 2 Humble / Python 3.10 and the [workspace dependencies](../../../README.md#from-a-fresh-machine).
- Official [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
  and its Cyclone DDS dependencies, installed in ROS's Python environment.
- A forward-facing camera and measured `odom` → `base_link` odometry, with clocks
  synchronized to ROS. Both manual and automatic control require fresh odometry.

Use `rmw_fastrtps_cpp` for ROS. With `rmw_cyclonedds_cpp`, set `ROS_DOMAIN_ID`
different from the SDK's `dds_domain_id` to avoid domain conflicts; the SDK must
remain on the robot's DDS domain.

## Native odometry bridge

`g1d_odometry` bridges DDS `rt/agv/odom` → ROS `g1d/odom` without motion commands.
It preserves measured pose, twist and covariance, and adds `clock_offset_s` to
source timestamps. Stale, future or invalid samples are rejected.
An empty native `child_frame_id` becomes `base_frame` (default `base_link`);
nonempty mismatches fail. Coordinates must already match; transform upstream otherwise.

Run the receiver alone:

```bash
ros2 run g1d_adapter g1d_odometry --ros-args \
  -p network_interface:=eth0
```

For external ROS odometry, launch with `odom_source:=ros odom_topic:=/your/odom`.
The adapter forwards valid measurements to `odom`; `g1d/odometry_status` reports
native bridge health. See [receiver parameters](../../../../docs/CONFIGURATION.md#g1-d-native-odometry-receiver).

## Build and launch

[Build and source](../../../README.md#from-a-fresh-machine) the workspace, start
the [camera sender](../../vln_camera/README.md#standalone-sender) separately, then run:

```bash
ros2 launch vln_bringup g1d.launch.py \
  network_interface:=eth0 \
  server_url:=ws://GPU_HOST:8050
```

Replace `eth0` with the robot's DDS interface. Defaults are native odometry and
ZMQ camera input at `tcp://192.168.100.78:5556` (`head_fpv`); override `camera_endpoint`
for another host. For ROS images, set `camera_source:=ros image_topic:=/your/image`
and add `image_transport:=compressed` for `CompressedImage`.

Run the GPU server with `--task vln`, then open `http://<robot-ip>:8088`.
Control starts disabled; default speed caps are **0.3 m/s** and **0.3 rad/s**.
`READY` means fresh valid odometry, not a verified motion RPC connection.
See [configuration](../../../../docs/CONFIGURATION.md#g1-d-ros-2-launch) and
[camera timing](../../vln_camera/README.md#wire-format-and-timestamps).

## Controls and failure behavior

- `web/cmd_vel` / `mpc/cmd_vel`: `TwistStamped` in `base_link`; forward `linear.x`
  (m/s) and yaw `angular.z` (rad/s, positive counterclockwise) only.
- `control/set_manual` / `control/set_auto`: `SetBool`; hold zero until a valid command.
- `control/stop`: `Trigger`; disable, attempt a bounded zero-speed burst, then cease RPCs.
- `robot/emergency_stop` / `robot/reset_emergency_stop`: `Trigger`; latch/reset the
  wheel-base software stop. The stop does not cut hardware power; reset does not enable motion.
- `robot/stand`, `robot/walk`, `robot/sit` are unsupported.

Command timeout, odometry loss or SDK errors disable control and attempt a stop.
Wait for `STOPPING` to finish before re-enabling. After `FAULT`, resolve the cause,
restore odometry, press **STOP**, then **Take control**. For `STOPPED`, reset the
software latch first. Release manual control and wait for `READY` before **Start VLN**.
Stopping cannot guarantee physical braking on communication loss or remote-control hand-back.

## Protocol references

Uses Unitree's [`agv` API](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/agv/g1_agv_api.hpp)
version `1.0.0.1`, movement API `1001` with `{vx, vy: 0.0, vyaw}`.

## Tests

From the repository root:

```bash
python -m pytest -q tests/test_g1d_adapter.py tests/test_g1d_node.py \
  tests/test_g1d_odometry.py tests/test_g1d_web.py tests/test_g1d_camera_launch.py
```
