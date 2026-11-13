# vln_camera

Standalone camera sender, desktop viewer and ROS 2 receiver:
`camera → XRAW v2 JPEG over ZMQ → sensor_msgs/CompressedImage`.
Start the sender separately; the viewer and receiver only subscribe to its stream.

## Standalone sender

On the Ubuntu camera host, install Python 3.10+, OpenCV and pyzmq, then run from
this repository's root. The sender needs no ROS or `OrinVideoSender`.

```bash
sudo apt-get install python3-opencv python3-zmq
python3 robot_deploy/src/vln_camera/scripts/mjpg_zmq_sender.py \
  --camera head_fpv=/dev/video0 \
  --bind head_fpv=tcp://0.0.0.0:5556 \
  --width 640 --height 480 --fps 30 \
  --encoding jpeg --jpeg-quality 80
```

Receivers connect to the camera host's IP, e.g. `tcp://192.168.100.78:5556`.
See [sender options](../../../docs/CONFIGURATION.md#standalone-zmq-camera-sender).
The [script](scripts/mjpg_zmq_sender.py) retains its XR Robotics MIT notice and
source revision.

## Standalone viewer

Preview the stream on a desktop with Python 3.10+, without ROS.
From the repository root:

```bash
python3 -m venv .venv-camera-viewer
source .venv-camera-viewer/bin/activate
python -m pip install opencv-python numpy pyzmq
python robot_deploy/src/vln_camera/scripts/zmq_camera_viewer.py tcp://PI_IP:5556
```

Replace `PI_IP` with the sender's IP. Keep GUI-enabled `opencv-python` in this
separate environment; `opencv-python-headless` cannot display windows.
Press **Q** or **Esc** to exit. See [viewer options](../../../docs/CONFIGURATION.md#standalone-zmq-camera-viewer).

## Receiver only

Install ROS dependencies, build and source the workspace as described in the
[deployment guide](../../README.md#from-a-fresh-machine), then run:

```bash
ros2 run vln_camera zmq_camera --ros-args \
  -p endpoint:=tcp://192.168.100.78:5556
```

- `camera/color/image_raw/compressed`: original JPEG bytes as `CompressedImage`.
- `camera/status`: JSON with frame freshness, accepted/rejected counts and errors.

Set both `vln_client` and `vln_web` to
`image_topic:=camera/color/image_raw/compressed image_transport:=compressed`.
The receiver forwards JPEG without reencoding; the VLN client resizes to 480×270.

## G1-D integration

The G1-D launch starts the receiver by default at `tcp://192.168.100.78:5556`,
role `head_fpv`, and configures both image consumers. With the sender running:

```bash
ros2 launch vln_bringup g1d.launch.py \
  network_interface:=eth0 \
  server_url:=ws://GPU_HOST:8050
```

This also starts navigation, the robot adapter and native odometry bridge.
Override `camera_endpoint` for another host; use `camera_source:=ros` for an
existing ROS image publisher. See the
[G1-D guide](../robot_adapters/g1d_adapter/README.md) for SDK, odometry and frame setup.

## Wire format and timestamps

The receiver accepts single-part XRAW v2 JPEG packets: an 88-byte header followed
by JPEG bytes. See [the parser](vln_camera/xraw.py) for the layout and validation.

The sender records time after capture and JPEG encoding. The corrected image
stamp estimates capture time:

```text
stamp = sender_wall + clock_offset_s - capture_delay_s
```

Synchronize sender, ROS and odometry clocks. Both corrections default to zero
and require measurement; they are not calculated automatically. The receiver
keeps the latest packet and rejects stale (default >0.5 s), future, duplicate or
out-of-order timestamps. Receipt time never replaces the source timestamp.
See [receiver parameters](../../../docs/CONFIGURATION.md#zmq-head-camera-receiver).

## Tests

From the repository root, without ROS or camera hardware:

```bash
python -m pytest -q tests/test_mjpg_zmq_sender.py tests/test_zmq_camera_viewer.py tests/test_xraw_camera.py tests/test_g1d_camera_launch.py
```
