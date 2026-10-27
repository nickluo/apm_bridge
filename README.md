# apm_bridge

Hardware and AirSim SITL use **the same `CustomLinkVehicleNode` executable** and
the same **custom Betaflight protocol**. No MSP is used by the SITL implementation.
The existing hardware launch/link interfaces are preserved; actuated gimbals now
share the AIM/MISSION/HORIZON state machine described below.

## Build and launch

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select apm_bridge --cmake-args \
  -DAPM_BRIDGE_AIRSIM=ON -DAIRSIM_ROOT=/path/to/cosys_airsim
source install/setup.bash
ros2 launch apm_bridge sitl.launch.py
```

The CMake option builds **only the camera RPC dependency** from the Cosys-AirSim
rpclib sources with the ROS compiler. It does not link Unreal's incompatible
prebuilt libc++ libraries. Without this option, hardware and custom-link SITL
still build; disable the optional camera with `gimbal_enabled:=false`.

The SITL parameters set mass to **2.0 kg**, turn off voltage/pressure
compensation, and disable the real gimbal serial port. Build/start Betaflight
SITL with `USE_CUSTOM_LINK`, then start the bridge before starting AirSim.
Use the calibrated 2 kg `BetaFlight` vehicle model and EEPROM described in
Cosys-AirSim's `docs/betaflight_sitl.md`.

### Custom-link connection topology

```text
AirSim BetaFlight adapter -- custom link --> node relay :5764
                                             |
ROS control / arm ---- CustomLinkVehicleNode -+-- custom link --> BF :5763
ROS IMU / RC / state <------------------------+<-- telemetry ----+
AirSim rotors <--------- mixer frame 0x13 ----+
AirSim camera API :41451 <--- camera pose control / orientation ---> /fpv/gimbal
```

In AirSim's `settings.json`, set the BetaFlight vehicle's **`TcpPort` to 5764**,
not 5763. Restart the Play session after changing settings:

```json
{
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Copter": {
      "VehicleType": "BetaFlight",
      "UdpIp": "127.0.0.1",
      "TcpPort": 5764,
      "RC": {"RemoteControlID": -1, "AllowAPIWhenDisconnected": true}
    }
  }
}
```

Do not enable AirSim API flight control, run another control publisher, or
run an external UDP FDM injector. The relay forwards AirSim's simulation
input frames **0x21/0x22/0x23**, forwards FC telemetry and **0x13** motor output
back to AirSim, and reserves **0x20** exclusively for the existing ROS
custom-link client. Frames are CRC-validated and assembled before forwarding,
so fragmented TCP input cannot interleave with ROS control frames.

Launch arguments: `tcp_host`, `tcp_port` select the FC; `airsim_link_port`
selects the relay listener; `airsim_host`, `airsim_port`, `vehicle_name`,
`camera_name` select the optional camera API. `gimbal_enabled` and `parameters`
select the camera/configuration.

## Gimbal abstraction

`apm_bridge::Gimbal` defines the shared `start()`, `stop()` and
`updateVehicleState(imu, gravity)`, `setMode(mode)` and `updateTargetLos(target)`
interface, with an orientation callback.
`xfrobot::GimbalControl` implements the serial backend and
`custom_link_bridge::AirsimGimbal` implements the camera RPC backend.
`apm_bridge::StrapdownGimbal` implements a camera with a fixed body-relative pitch.
`createGimbal` selects/configures the backend from the existing parameters.
`CustomLinkVehicleNode` holds only `std::unique_ptr<Gimbal>` and publishes the
callback message; it does not depend on concrete drivers, protocol status,
fusion flags, polling timers or heading-composition details.

The input is vehicle FLU-to-ENU orientation and body specific acceleration.
All callbacks preserve camera FLU-to-ENU orientation, camera FLU rates and the
legacy heading-relative Euler slots. Serial communication stays above 50 Hz;
AirSim polls with a 20 ms minimum delay plus RPC latency. Serial status logging and optional
FRD/NEU UAV-data conversion live in the serial backend. Lifecycle stop is
idempotent and cancels callbacks/joins workers before the node is destroyed.
The hardware `run()`, `getOrientationFLU()` and command APIs remain available
for existing standalone callers and protocol tests.

Only an explicitly disabled AirSim gimbal returns no object. Invalid backend configuration or missing
AirSim SDK is an explicit error; serial-open failures are logged without
publishing success-shaped gimbal data.

### Shared C20S / C40D / C200T pose reconstruction

Serial and AirSim use `gimbalCameraToWorld` for all three models. With FRD
encoder angles `(roll, pitch, yaw)`, the camera FLU-to-ENU orientation is
`q_body * q_z(-yaw) * q_x(roll) * q_y(-pitch)`: complete vehicle attitude,
followed by the yaw-roll-pitch joint chain. Positive vendor pitch is nose-up;
positive vendor yaw is clockwise. Only the available axes differ:

| Model | Roll | Pitch | Yaw |
| --- | --- | --- | --- |
| C20S | absent | present | absent |
| C40D | present | present | absent |
| C200T | present | present | present |

Absent encoder slots are ignored, not interpreted as zero world rotation;
those degrees of freedom remain coupled to the vehicle. All models assume
aligned carrier/camera frames at encoder zero and no additional mounting
rotation. No mounting flip is inferred from `inv_flag`; a different mounting
requires an explicit calibrated extrinsic transform.

The serial backend reorders `mtr_angle` from **pitch/roll/yaw** into the joint
chain; it never mixes absolute `cam_angle` roll/pitch with vehicle heading.
`cam_angle` remains available to the existing stabilization/protection control
policies. Vehicle attitude is snapshotted with each valid encoder packet, so
repeated publications of one timestamp have one pose. Missing, invalid or
older-than-200-ms vehicle attitude suppresses pose publication for **all models**,
even when `send_uav_data=false`; that switch only disables the vendor's optional
onboard fusion input. Samples use the latest received vehicle attitude, not
hardware-synchronized or interpolated measurements. Angular rates are reordered
from pitch/roll/yaw and converted FRD-to-FLU as `(roll, -pitch, -yaw)` before
filtering; absent motors do not imply zero camera gyro rates.

AirSim derives virtual encoders from the body-relative camera rotation with
axis-aware yaw-roll-pitch decomposition, then uses the same reconstruction for
pose publication, control feedback and angular-rate estimation. Pitch crossing
+/-90 degrees is preserved. At a three-axis roll singularity, equivalent pitch
and yaw values preserve the rotation, not unique physical encoder readings.
Legacy Euler slots for every model are derived from the reconstructed
heading-relative rotation; `getOrientationFLU()` returns that relative
orientation, whereas the callback publishes the complete ENU orientation.

`gimbal_interface_test` checks all three models using serial PTYs and AirSim RPC
simulation against independent rotation-matrix ground truth: mixed vehicle and
joint attitudes, ignored absent-axis slots, roll singularities, pitch beyond
+/-90 degrees, gyro signs, freshness and stable timestamps. Existing
AIM/MISSION/HORIZON targets and model-specific protection policies are retained;
AirSim HORIZON now applies the same yaw-roll-pitch joint constraints as MISSION,
rather than projecting through a separate ZYX rotation order.
With `APM_BRIDGE_AIRSIM=ON`, opt in to `APM_BRIDGE_AIRSIM_LIVE_TEST=ON` to register
`gimbal_airsim_live_test`. It reads 20 poses from an already running AirSim
instance (default `127.0.0.1:41451`, first vehicle, camera `0`, C20S), injects
virtual encoders and vehicle attitude into a PTY serial backend, and requires
less than 0.1-degree reconstruction error against the simulator's camera pose.
It never sends AirSim camera, vehicle or flight-control commands. Optional
`APM_BRIDGE_AIRSIM_HOST`, `APM_BRIDGE_AIRSIM_VEHICLE`, `APM_BRIDGE_AIRSIM_CAMERA`
and `APM_BRIDGE_AIRSIM_MODEL` environment variables select the endpoint/model.
The live test requires a matching aligned camera and sufficiently slow motion
because the two RPC pose reads are not simultaneous; it is not a dynamic flight
test.

### AIM / MISSION / HORIZON

`CustomLinkVehicleNode` is the only camera controller and `/fpv/gimbal` publisher.
Do not run the former Tracker camera-control worker alongside it. The current
mode is published on `/fpv/gimbal_mode` (`std_msgs/msg/String`, transient-local).

| Mode | Selection | Actuated camera behavior |
| --- | --- | --- |
| AIM | Disarmed/startup, manual aiming before a mission, or a new tracker `ON` | Vendor command 4, FPV angle control; camera +X aims toward body ROS FLU +Z (up), offset by `pitch_aim`, within joint limits. |
| MISSION | `/fpv/mission_state=true` and FC OFFBOARD authorized | Center LOS pitch/yaw at zero using external feedback + compensated rate feedforward; command 4, LOCK + REAL_SPEED, with mechanical limits. |
| HORIZON | Mission exit, tracker `OFF`, API manual request, or pilot takeover while airborne | Keep heading aligned with the body and pitch at `pitch_hold` above horizontal. Stabilize roll on models that have a roll motor. |

HORIZON persists after takeover/exit until a new tracker `ON`, a new authorized
mission, or disarming. A stale `mission_state=true` cannot resume an interrupted
mission after a manual takeover/OFF; it must become false before a new mission.
A `MISSION` trigger frame also requests FC OFFBOARD itself (see "Tracker
trigger" below), so the authorization arrives with the mission start.
Losing a target **does not leave MISSION**: pitch/yaw tracking decelerates to
zero and holds the current stabilized orientation. Missing/invalid/stale LOS is
logged. Real hardware also stops MISSION rates and suppresses camera pose
publication after 200 ms without valid gimbal feedback.

For **both actuated backends**, `gimbal.pitch_aim` and `gimbal.pitch_hold` are
startup settings in degrees, **positive nose-up**:

- AIM requests body-relative pitch **`90 + pitch_aim`**. At offset zero, camera
  +X is parallel to body FLU +Z (up); a positive offset tilts beyond +Z toward
  the body's rear. It follows body motion and does not use `pitch_hold`.
- MISSION always requests **zero pitch/yaw LOS error**, independent of both
  offsets. Travel limits can prevent full centering; unavailable axes are not
  commanded, and hardware rates cannot push farther outward at a limit.
- HORIZON alone uses `pitch_hold`, relative to horizontal, exactly as before.

**The existing default pitch travel is [-135,+40] degrees.** Consequently the
default `pitch_aim=0` requests +90 but produces the reachable +40-degree AIM
setpoint and logs a warning. Do not enlarge real mechanical limits without
checking the device/drawing. `pitch_aim=-50` requests +40; `pitch_aim=-90`
recovers the former body-forward AIM direction. To test exact +Z alignment in
AirSim, use a simulation-only config with `pitch_max>=90` (and enough margin for
the requested offset). Equal min/max retains the existing disabled-limit convention.

AirSim AIM uses the explicit joint angles without folding pitch into ZYX's
[-90,+90] branch. MISSION constraints use the vendor's yaw-roll-pitch joint order
and choose the nearest feasible orientation; looking along body +Z no longer
causes an Euler-angle flip or false single-axis roll-protection trigger.

The fixed-mount strapdown convention below is unchanged; a physical fixed camera
cannot implement these actuated modes.

`gimbal.model` selects C20S (pitch), C40D (roll/pitch) or C200T (roll/pitch/yaw)
in hardware **and AirSim**. Absent axes remain body-coupled; a pitch-only gimbal
cannot correct a target's horizontal error. Both backends apply mechanical
limits and C20S roll-protection hysteresis. Exact horizon hold can be unreachable
at extreme body attitudes or mechanical limits. Mechanical/protection stops
take priority over smoothing.

MISSION parameters (same YAML keys on both backends):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `gimbal.tracking_gain` | 2.0 | Image-angle proportional gain, 1/s |
| `gimbal.tracking_max_rate` | 30.0 | Per-axis limit, deg/s; MISSION tracking, HORIZON leveling and AIM/HORIZON angle trajectories (absorbs the former `max_angle_speed`) |
| `gimbal.tracking_max_accel` | 90.0 | Per-axis command slew limit, deg/s² |
| `gimbal.tracking_filter_tau` | 0.15 | First-order commanded-rate filter, seconds |
| `gimbal.tracking_rate_feedforward` | 1.0 | Gain on ego-compensated LOS angular-rate feedforward; 0 retains angle feedback only for noisy rendered-camera tracking |
| `gimbal.los_timeout` | 0.3 | Maximum LOS sample age, including transport delay, seconds |

The shared controller converts the spherical LOS derivatives through the
camera-frame LOS Jacobian before filtering/slew-limiting feedback and feedforward
together. Saturation scales the actuated rate vector uniformly to preserve its
correction direction; absent axes cannot consume the rate budget. Spherical LOS
derivatives are not directly assigned to gyro axes.
Mode transitions reset the LOS controller's rate history. Rate/acceleration/filter
parameters must be finite and positive; the tracking rate must not exceed 150 deg/s.

### Mode-transition motion limits

AIM and HORIZON are **targets**, not instantaneous camera poses. AirSim now
integrates a shared joint-angle trajectory using `tracking_max_rate` (deg/s)
and `tracking_max_accel` (deg/s²), with braking before reaching an angle target.
The trajectory state/velocity is retained across simulated AIM/HORIZON changes;
MISSION continues using the existing bounded LOS controller and world-attitude
stabilization directly, then updates the trajectory state for the next angle
mode. Applying a second joint servo to MISSION would delay vehicle-motion
compensation and can lose visual lock during launch. Joint limits and roll-protection stops take priority over acceleration
continuity. Yaw wrap is handled along the shortest path when yaw limits are
disabled. Startup/reconnect seeds the trajectory from measured encoder angles.

The serial backend ramps FPV AIM angle commands from the actual joint encoders,
and C20S LOCK/HORIZON pitch commands from the actual absolute camera pitch;
these are different command reference frames. It does not send the final angle
step on mode entry. No angle trajectory advances without feedback, and stale
feedback (200 ms) stops motion with a zero real-speed command. Existing
MISSION rate-control and hardware protection semantics remain in effect.

Previously, AirSim's `simSetCameraPose` jumped directly to AIM/HORIZON targets,
so quaternion differentiation correctly reported thousands of deg/s for a
one-frame 80-degree jump. A physical gimbal cannot teleport: its reported gyro
comes directly from the protocol, not from differentiating reconstructed poses.
However, the same abrupt angle targets could still produce a real motor jerk;
the host-side angle ramps remove that step without assuming undocumented
firmware servo limits. Measured/simulated gyro data is **not clipped** to hide
motion. Rates are in the camera frame, so vehicle rotation can add to the
motor-relative rate cap; a rate above that cap alone is not proof of a servo
limit violation. Physical motor tracking, latency and acceleration still need
bench verification on the actual device.

Regression tests check both pose-derived angular speed and published gyro
through repeated AIM/MISSION/HORIZON changes, bounded serial angle commands,
settling, mechanical stops and stale-feedback behavior.

**AirSim setup:** set the controlled camera's
`"Gimbal": {"Stabilization": 0.0}` and restart the Play session. Otherwise
Unreal's built-in stabilization competes with `simSetCameraPose`, including
overriding AIM. The tracker example settings have been updated. Camera control
uses `airsim.host/port/vehicle_name/camera_name`, preserves the camera's initial
body-relative position, and converts world ENU/FLU commands to relative NED/FRD
poses. Configure the tracker with matching endpoint/camera names.

### Tracker trigger (`/fpv/tracker_trigger`)

`std_msgs/msg/Header` frames drive the vision pipeline and the mission
lifecycle; the bridge both consumes and publishes them:

| frame_id | Published by the bridge when | Bridge reaction on receipt |
| --- | --- | --- |
| `ON` | FC **arming** edge (props spin up), or an AirSim RC preview pulse while disarmed | gimbal AIM request; tracker starts |
| `OFF` | FC **disarming** edge | gimbal abort; tracker + interceptor stop; **requests MANUAL** |
| `MISSION` | RC trigger pulse while vision/radar is locked and no mission runs | **requests OFFBOARD** |
| `DISMISS` | RC trigger pulse while a mission runs | interceptor cancels control, **tracker keeps running**; **requests MANUAL** |

The interceptor cancels interception control on either `OFF` or `DISMISS`;
`OFF` additionally stops the tracker (disarm edge), while `DISMISS` is the
pilot's stop pulse - vision stays up so a later pulse can re-trigger once
still locked.

Additionally, `/fpv/mission_state` falling `true -> false` (the interceptor
terminated) requests MANUAL as well, so host authority is released even if no
`OFF` frame is sent. The OFFBOARD/MANUAL requests are the same `mode_req`
transitions as the explicit `~/offboard` topic below - entering a mission no
longer requires the pilot's OFFBOARD aux switch.

The RC trigger pulse is a `channel_trigger` channel (SITL default CH8/AUX4,
zero-based index 7) held **above 1900 for more than 0.5 s**; it is one-shot per
hold and re-arms when the channel drops below 1100. A pulse only starts a
locked, idle mission or dismisses a running one - a pulse without a lock is
ignored with a warning, and it never toggles the tracker (arming edges own
tracker start/stop). The AirSim-only exception is a pulse while disarmed:
with `airsim.enabled=true` it publishes `ON` for non-arming vision/aim checks,
even if a lock is already present. It never publishes `MISSION` or requests
OFFBOARD while disarmed. Hardware and armed-flight pulse behavior is unchanged.

### Target LOS wire contract

`/fpv/target_los` uses `geometry_msgs/msg/TwistStamped`:

| Field | Meaning |
| --- | --- |
| `header.stamp` | Same projection timestamp as the paired `/fpv/target_bbox` |
| `header.frame_id` | `camera_flu` if valid; empty on loss/invalid input |
| `twist.linear.y` | LOS pitch, rad, positive down |
| `twist.linear.z` | LOS yaw, rad, positive left |
| `twist.angular.y` | Ego-compensated pitch-angle rate, rad/s |
| `twist.angular.z` | Ego-compensated yaw-angle rate, rad/s |
| `twist.linear.x`, `twist.angular.x` | Unused, zero |

These are spherical **LOS angle derivatives**, not an IMU angular-velocity
vector or translational velocity. Tracker removes camera/airframe **rotational**
ego-motion by rotating the previous ray into the current camera frame before
differentiating; vehicle translation remains part of relative target motion
(range would be needed to remove it). Rate history is re-expressed in the new
camera frame before filtering. The first observation, a KF reinitialization,
or a gap over 0.3 s restarts the rate estimate at zero.

Tracker uses the image response's capture-time camera orientation for KF
updates, and newer fresh bridge camera poses for low-latency projection at
the actual pose sample time. If bridge
camera feedback is unavailable, it explicitly warns and publishes capture-time
bbox/LOS rather than fabricating an identity camera pose. Image debug messages
retain their capture timestamp. Bridge rejects wrong-frame, non-finite,
expired, future-dated or out-of-order valid LOS samples.

### Fixed-pitch strapdown camera

When `airsim.enabled=false` and `gimbal_port=""`, the factory automatically
selects `StrapdownGimbal`, rather than disabling camera orientation publication.
Nonempty hardware ports still select XFrobot; AirSim selection is unchanged.

Set the shared `gimbal.pitch_hold` at startup. For strapdown it is in **degrees FLU**: positive points below
the body forward axis, negative points above it. Default is 0.0; the accepted
range is [-180,180]. For example, a camera tilted 20 degrees upward:

```bash
ros2 launch apm_bridge custom_link.launch.py pitch_hold:=-20.0
```

The hardware launch defaults to an empty gimbal port. The equivalent parameters
are `airsim.enabled: false`, `gimbal_port: ""`, and `gimbal.pitch_hold: -20.0`.
For an actuated gimbal, this parameter has the HORIZON-only meaning described
above, not a body-relative mount angle. Vendor sign conventions are unchanged.
The optional launch argument `pitch_hold` overrides this value only when set;
otherwise the YAML value is preserved. `gimbal.fixed_pitch` / `fixed_pitch`
have been removed; migrate them to `gimbal.pitch_hold` / `pitch_hold`.

Orientation uses `q_camera_ENU = q_vehicle_ENU * q_camera_body`, not addition of
Euler pitch angles. Body angular velocity/covariance are rotated into camera
FLU. The callback preserves the vehicle IMU sample timestamp and the existing
`/fpv/gimbal` frame/legacy-Euler contract. It publishes once per fresh vehicle
IMU update (normally 200 Hz), with no serial port, RPC call or extra worker.
Acceleration is marked unavailable because no camera accelerometer or
translation/lever-arm model is specified. Stopping waits for any in-flight
callback and prevents further publications until restarted.

## Preserved interfaces

For an opt-in captive three-axis rig experiment controlled by the zero-based
`tuning_channel` parameter (default 8 = CH9), see
[bench PID tuning](parameters/bench_pid_README.md) and
[the disabled-by-default parameters](parameters/bench_pid.yaml).
It never arms or saves the FC. ACRO start uses a bounded quaternion hold of the
captured starting orientation between rate-only trials. ANGLE start selects native
FC ANGLE closed-loop angle-step trials (yaw remains a rate test), with the existing
outer-loop settings fixed and recorded; it requires new firmware capability
`CMD_APPLY_PID args="angle" -> ANGLE OFFBOARD 1`. The raw ATTITUDE path encodes
roll/pitch targets in degrees and yaw in deg/s using `mode_req=3`; normal raw
BODY_RATES remains `mode_req=2`. Mode feedback distinguishes `OFFBOARD|ANGLE`.
Actual PID application still uses a short, guarded zero-output window. It cannot
support an unbalanced rig after power loss or stopping. Runtime PID application
requires the matching firmware/service extension; ordinary CLI SET readback alone
is not proof that the running PID coefficients changed.

The existing `/fpv/control_command`, `/fpv/control_command_raw`, `/fpv/arm`,
`/fpv/low_level_feedback` and MAVROS-compatible telemetry interfaces are reused.
`/fpv/offboard` (`std_msgs/msg/Bool`, private name `~/offboard`) independently
requests OFFBOARD (`true`, custom-link `0x20 mode_req=2`) or manual control
(`false`, `mode_req=0`). The same mode requests are also issued automatically
by the tracker-trigger linkage described above (MISSION enters OFFBOARD,
OFF/mission-end releases it); both paths share one mode request. A request is
sent even without a fresh control command;
subsequent control/arm frames retain the requested mode. Mode requests do not
change the host arm bit, and do not keep stale control commands alive: maintain
a fresh control stream while in OFFBOARD. The actual result is reported by
`/mavros/state`, subject to firmware safety and pilot/Rescue arbitration.
The pilot's held OFFBOARD AUX switch takes priority over an API manual request.
After a pilot takeover or a pilot mode change during host control, re-issuing
`true` is enough to enter a new API session: the bridge automatically emits a
fresh `mode_req` 0->2 edge (a pending request is not defeated by incidental
pilot dial sweeps; the latch only applies while the host is actively
controlling).

```bash
ros2 topic pub --once /fpv/offboard std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /fpv/offboard std_msgs/msg/Bool "{data: false}"
```

### CLI read/write service (`/fpv/cli`)

`quadrotor_msgs/srv/CliCommand` exposes the firmware settings table over the
custom link (`0x24` up / `0x15` down, chunked responses) through the same
engine the serial CLI and `MSP2_CLI_SETTING` use - identical name matching,
range validation and `"name = value"` response text. `CMD_GET`/`CMD_SET`
round-trip a setting (SET replies with the readback), `CMD_INFO` returns the
metadata block (`pgn`, `type`, min/max or lookup `values`, `default`)
reassembled from any number of 56-byte chunks, `CMD_BIND_RX` puts the
receiver into pairing mode (the serial CLI's `bind_rx`; CRSF/SRXL2/SPI) and
replies `"binding..."`, and `CMD_SAVE` persists the RAM config to flash
(what `MSP_EEPROM_WRITE` does) - by default followed by an FC reboot like
the serial CLI's `save` (the link drops and reconnects; pass `noreboot` to
persist without rebooting). SAVE is refused while armed. FC-side failures arrive as
`ok=true, status=ERR` with the text (`SET FAILED`, `NOT FOUND`, ...); local
failures (link down, `timeout`, `chunk gap`, `length mismatch`, bad request)
as `ok=false` with `error`. Writes are RAM-only (CLI `set` without `save`)
and revert on reboot; there is no armed guard, matching MSP - do not hold the
service responsible for what you set in flight.

With `cli.bind_rx_on_connect: true` the node sends `bind_rx` automatically
each time the link to the FC comes up (once per connection) and logs the FC's
confirmation - enable it temporarily when pairing a new transmitter/receiver,
keep it off otherwise so every power-up does not interrupt reception.

```bash
ros2 service call /fpv/cli quadrotor_msgs/srv/CliCommand \
    "{cmd: 0, args: 'failsafe_delay'}"
ros2 service call /fpv/cli quadrotor_msgs/srv/CliCommand \
    "{cmd: 1, args: 'failsafe_delay = 20'}"
```

The service uses rclcpp's deferred-response pattern: the executor callback
only queues the request and a dedicated worker thread blocks on the link
response (`cli.timeout_s`, default 3 s), so a slow or timing-out call never
stalls control/telemetry callbacks. Requests are serialized (the firmware
keeps one in-flight response stream, last request wins); more than 8 queued
requests get an immediate `busy` reply.

Normal collective thrust is mass-normalized acceleration (m/s2); raw collective
is BF normalized throttle. Arming, OFFBOARD, watchdog, pilot takeover and
time synchronization retain the real custom-link client's semantics.
`link.control_rate` sets both the latest-command cache push rate and the
custom-link control-frame transmit rate at startup. Periods use nanosecond
resolution, including for fractional frequencies; invalid rates fail explicitly.
Command timeout behavior is unchanged.
When `baro_compensation` is enabled, its factor is
`(pressure_pa / 101325.0) * (273.15 / (273.15 + temperature_c))`, using measured
FC pressure and temperature, not an ISA estimate derived from altitude.
Temperature initially defaults to 20 C. Zero pressure retains the previous
factor; temperature at or below absolute zero retains the previous temperature.
Both invalid readings are logged.

Gyro/acceleration/attitude/state/RC/GPS/battery data all come from the firmware's
custom telemetry; none are inferred through MSP or AirSim state APIs.
`/fpv/altitude` exposes the **0x11** altitude (meters) for calibration.
The RC channels and link quality are mirrored on `/mavros/rc/in`
(`mavros_msgs/msg/RCIn`, best effort) from the **0x11** stream at 100 Hz;
`rssi` carries the FC's RC link quality, **0..100** (hardware: the rx RSSI
source, e.g. CRSF LQ; simulator: 100 while RC frames are fresh, 0 after the
feeder stops). The RC trigger pulse
(`channel_trigger`) defaults to CH8/AUX4, zero-based index 7, in the SITL
parameters; see "Tracker trigger" above for its mission start/stop semantics.

The **only** AirSim RPC is `simGetCameraInfo`. Camera FRD/NED orientation is
converted to FLU/ENU; `/fpv/gimbal` preserves the hardware heading-relative
Euler slots in `orientation_covariance[0:3]`. Angular velocity is derived from
successive camera poses, and marked unavailable until two poses arrive.
Camera API failure is logged and does not stop custom-link flight telemetry.

## Motor RPM feedback

`/fpv/motor_rpm` publishes `std_msgs/msg/Float32MultiArray` at 100 Hz from the
firmware's **0x16** ESC telemetry stream. The FC converts bidirectional DShot
eRPM (Bluejay-class ESCs) to mechanical RPM itself using its `motor_poles`
setting, so the bridge passes values through unchanged: `data[i]` is motor `i`
mechanical RPM and motors without the valid bit (bidir DShot off, or no eRPM
frame yet) publish **NaN** — distinguish that from a true 0 RPM stop with
`isnan`. Requires the custom-link firmware with `0x16` support; older firmware
simply never publishes. `test/hw_custom_link_rpm_test.py` uses the same frames
directly over serial for props-off OFFBOARD control-chain verification.

## GPS local odometry

`/fpv/odom` publishes `nav_msgs/msg/Odometry` with sensor-data QoS (best effort),
driven by the firmware's new **0x14** telemetry stream at **100 Hz**. No MSP or
AirSim pose API is used for odometry. Both the FC firmware and bridge must be
updated; older peers ignore the new message ID without affecting other streams.
The payload is now 64 bytes, including ARM heading; the former 62-byte form is
rejected rather than interpreted with an invented heading.

The successful ARM edge captures both a GPS origin and the horizontal heading
for this flight. In `header.frame_id="odom"`, **X follows the initial ARM heading,
Y points left, Z points up**, and position is in meters. This is a fixed,
heading-aligned local frame, not compass ENU and not the current body frame.
The firmware keeps its ENU fusion/navigation coordinates unchanged; the bridge
rotates position, orientation and pose covariance by the inverse ARM yaw.
`child_frame_id` uses `imu.frame_id` (`base_link` by default).
ROS requires twist in the child frame, so velocity and its covariance remain
body **FLU**, with angular rates in rad/s. The captured heading is present in
every packet, allowing correct interpretation even when the bridge connects
after ARM or after the aircraft has turned.
FC Kalman position/velocity covariance accompanies each sample; unknown IMU
uncertainty is represented conservatively, not as a zero-uncertainty estimate.

Publication requires an armed FC, a fresh GPS **3D FIX**, an origin captured
with 3D FIX at ARM, and valid/fresh XY and Z fusion. For NMEA, a checksum-valid
GSA sentence with fix type 3 is required; GGA's general fix flag alone is not
treated as 3D. UBX already requires a valid FIX_3D. Sensor source settings are
respected (GPS-only, barometer, rangefinder, optical flow); no fresh fused
solution means no fabricated odometry.

Before ARM and after disarming there is no odometry. GPS loss pauses it; recovery
retains the original ARM position and heading even if the internal XY estimator
recaptures its own reference. Re-arming creates a new position/heading origin. If ARM occurred without
3D FIX, this flight remains without odometry even after GPS acquisition: re-arm
with 3D FIX to establish the requested ARM origin. No automatic TF is published.

Wire layout (64-byte payload; all fields little-endian):

| Field | Type | Units |
|---|---|---|
| `ts_us` | uint32 | FC sample time, mapped by custom-link time sync |
| `flags`, `origin_id` | uint16 each | validity bits; increments at every ARM |
| `position_cm[3]` | int32 | ARM-relative East, North, Up |
| `velocity_cms[3]` | int16 | ENU cm/s |
| `attitude_cdeg[3]` | int16 | same FLU attitude / North yaw as 0x10 |
| `gyro_dps_x10[3]` | int16 | FLU angular velocity, 0.1 deg/s |
| `position_variance[3]` | float32 | ENU diagonal, m2 |
| `velocity_variance[3]` | float32 | ENU diagonal, (m/s)2 |
| `arm_yaw_cdeg` | int16 | yaw captured at ARM, 0.01 deg CCW from North |

Flags `0x01/0x02/0x04/0x08` mean GPS 3D FIX / ARM origin / valid estimate /
armed. The bridge requires all four and rejects malformed payloads/covariance.
The stream adds about 7.1 kB/s to the custom-link downlink.

Validation (ports 5761/5763 must be free; uses an isolated EEPROM and synthetic
custom-link IMU/GPS/RC input, without flying UE or changing the real EEPROM):

```bash
# In the Betaflight checkout:
make TARGET=SITL
make EXTRA_FLAGS=-Werror test_custom_link_unittest test_position_estimator_unittest
# In this package, with the rebuilt ROS workspace sourced:
python3 test/sitl_odometry_test.py --binary /path/to/betaflight_SITL.elf
ros2 topic echo /fpv/odom --qos-reliability best_effort
```

The regression measures wire and ROS rates against 100 Hz (+/-5%), checks
ARM-heading pose/FLU twist transformations and covariance, GPS loss/recovery,
position/heading reset at re-arm, and
GPS acquisition only after ARM. SITL TCP output is drained only by the dyad
worker to avoid cross-thread stream-buffer corruption at the added data rate.

AirSim's synthetic magnetometer models 2 degrees east declination: calibrate the
simulation EEPROM with `mag_declination=20` (0.1 degree units). Otherwise the
heading frame is about 2 degrees away from true GPS ENU, creating an apparent
cross-axis displacement proportional to forward distance. This is not a
hardware compass preset. See the Betaflight checkout's
`src/test/sitl/airsim_compass.cfg` and `airsim_odom_hold_test.py` for native
POS HOLD/ALT HOLD truth-versus-odometry validation.

## Motor calibration

The analytic reference for the 2 kg F70/HQ7040 simulation is:

```text
rotor signal = 0.055 + 0.945 * BF collective
total force = 4 * 22.76123465 N * rotor signal
BF collective = C * total force + D
C = 0.011622843339487498, D = -0.058201058201058205
2 kg hover collective = 0.16976126
```

These are simulation collective coefficients, not ESC throttle/RPM.
The shipped parameters were subsequently measured with the real AirSim/UE
simulation on 2026-10-10: 42 steady-state samples over seven throttle levels,
repeated twice. The measured curve is:

```text
BF collective = 0.011891350984191815 * total force - 0.059949564628813956
2 kg hover collective = 0.17327906962943534
throttle RMSE = 0.0006232991, maximum error = 0.0016002142
```

The raw FC measurements are in
[sitl_motor_calibration.csv](parameters/sitl_motor_calibration.csv).
After changing motors, physics or PWM endpoints, run an experiment with the
SITL launch **running**, the vehicle disarmed and other ROS controllers stopped:

```bash
ros2 run apm_bridge sitl_motor_calibration \
  --parameters src/apm_bridge/parameters/sitl_parameters.yaml \
  --collect /tmp/sitl_thrust.csv --output /tmp/sitl_fitted.yaml
ros2 launch apm_bridge sitl.launch.py parameters:=/tmp/sitl_fitted.yaml
```

The script uses ROS raw commands and arming, hence **only the custom protocol**
for flight. It hovers at 4 m, applies short pulses and logs FC-timestamped
FLU specific acceleration, quaternion, altitude and throttle. Force is
`mass * acc_body_up`, already gravity-inclusive; adding another g would be
incorrect. The script rejects stale data, contact/tilt and excessive fit
residuals, then lands and requests confirmed disarming. Use an empty/open
simulation scene; it does not reset the simulator or use high-level AirSim APIs.
The hover controller fuses FC inertial acceleration and noisy raw barometric
altitude in a vertical estimator; the recorded acceleration used for fitting
is not filtered or pressure-compensated.
Pulses last 0.30 s and only the final 0.06 s is sampled, allowing the firmware's
acceleration filter and command transport to settle. The maximum accepted
throttle residual remains 0.01; transient samples are not used to fit the curve.
In SITL, landed-state detection uses a filtered height relative to the disarmed
ground reference, while `/fpv/altitude` retains the original absolute FC
barometric altitude. Hardware altitude behavior is unchanged.

Offline refitting:

```bash
ros2 run apm_bridge sitl_motor_calibration \
  --parameters src/apm_bridge/parameters/sitl_parameters.yaml \
  --fit /tmp/sitl_thrust.csv --degree 1 --output /tmp/sitl_fitted.yaml
colcon test --packages-select apm_bridge
colcon test-result --verbose
```

Degree 1 is appropriate for AirSim's thrust-linear rotor signal.
Degrees 2/3 populate B/A for supported nonlinear experimental data. The fit
checks monotonicity and a maximum throttle residual, writes a standalone ROS
YAML with `n=1` (total force) and keeps both compensations disabled.
The generated thrust limit never exceeds the highest sampled throttle. Before
arming, the script verifies `airsim.enabled=true`, checks mass/gravity against
the running node and rejects other control publishers.

## Real-physics validation (2026-10-10)

The AirSim/UE custom-link chain passed calibrated mass-normalized takeoff,
2 m hover, roll/pitch/yaw rate tracking, landing and confirmed disarming.
Measured hover height was 2.063 m with 0.062 m/s vertical speed.
Roll/pitch response gains were 0.986/0.987; positive/negative yaw gains were
0.975/0.992. Armed zero-throttle correctly reported ON_GROUND, flight reported
IN_AIR, and landing returned ON_GROUND.

Motor fitting achieved R2=0.9999806 with a maximum throttle residual of 0.0016002.
Both AirSim-enabled and hardware-only builds passed; `colcon test` reported
9 tests with zero failures. Flight control used only ROS/custom-link messages;
AirSim ground-truth APIs were used solely for independent observation.

## Moving-flight gimbal validation (2026-10-12, historical behavior)

The measurements below predate the body-+Z AIM / zero-centered MISSION revision:
they tested the former body-forward AIM and MISSION `pitch_hold` image offset.
They are retained as historical evidence, not claimed as live validation of the
revised defaults. The script now verifies the limited `90 + pitch_aim` AIM
reference and zero-centered MISSION; its `--pitch-aim` defaults to **-90**
explicitly so the existing ground target remains visible. For zero-offset
upward AIM, supply an airborne/visible target; the script will not fabricate LOS
for a target behind the camera.

`test/airsim_gimbal_flight.py` owns a temporary SITL/bridge and clones the tuned
EEPROM rather than changing the original. The live UE/Cosys-AirSim vehicle flies
a continuous horizontal trajectory at approximately 3.5 m, using ROS body-rate
and collective commands over custom-link only. No high-level AirSim flight
control, synthetic FDM/IMU data, or vehicle pose teleportation is used.

Prerequisites: an idle running BetaFlight Play session, camera 0 at 1536x960 /
72-degree scene FOV with built-in stabilization disabled, a sourced ROS
workspace, the Cosys-AirSim Python SDK, and free TCP ports 5761/5763/5764.
The script uses isolated ROS domain **173** by default, waits for real 1-g
sensor data before initializing its flight estimator, then lands and confirms
disarming and restoration of its configured ground AIM reference before stopping
its processes.

```bash
# From the apm_bridge package directory:
source /opt/ros/jazzy/setup.bash
source ../../install/setup.bash
python3 test/airsim_gimbal_flight.py \
  --binary /path/to/betaflight/obj/main/betaflight_SITL.elf \
  --eeprom /path/to/betaflight/eeprom.bin \
  --model C20S --pitch-aim -90.0 --pitch-hold 10.0 --output /tmp/gimbal-c20s-run
# Repeat with --model C200T and a DIFFERENT output directory.
```

Both models passed the declared pose/tracking criteria while continuously moving
(100% of evaluated samples above 0.2 m/s; median horizontal speeds 0.78-0.94 m/s;
minimum height 3.29 m in the latest reported runs):

| Measurement (P95, degrees) | C20S | C200T |
| --- | ---: | ---: |
| AIM camera/body orientation error | 0.184 | 0.075 |
| MISSION LOS pitch error relative to +10-degree offset | 0.689 | 0.756 |
| MISSION LOS yaw error | Not actuated | 1.833 |
| HORIZON world pitch error relative to +10 degrees | 0.044 | 0.026 |

Whole-stage MISSION camera pitch/yaw rate P99 values were 10.65/4.76 deg/s
(C20S) and 16.85/21.83 deg/s (C200T). The controller's 30 deg/s per-axis command
limit is covered by regression tests, but it is not an absolute bound on
individual RPC pose-difference samples: the latest C200T run included one
42.94 deg/s pitch sample over 27 ms. The same event averaged 30.00 deg/s over
54 ms and 25.61 deg/s over 82 ms, consistent with discrete pose updates and
asynchronous observation. Strict instantaneous rendered-motion limiting is
therefore not certified by this test. `result.json`'s `passed` flag refers to
the declared P95 pose/LOS and P99 rate criteria, not an absolute sample maximum.

This exercise caught and fixed a missing spherical-LOS-to-body rate conversion.
The off-center/roll-coupled Jacobian and absent-axis rate-budget cases now have
regression tests. Both AirSim-enabled and hardware-only builds passed; the
bridge's six test groups and the tracker's seven tests passed.

The LOS source is the actual scene's **TargetDrone actor-origin reference
point**, projected using live camera poses and the tracker's
`TargetLosEstimator`. It is not the visible mesh's bbox centroid, and this test
does **not** certify YOLO/NanoTrack acquisition or physical RC takeover. HORIZON
is exercised by exiting MISSION without stopping aircraft motion.

Each output directory contains `result.json`, timestamped `flight.csv`, cloned
EEPROM/configuration, process logs, and actual `AIM.png`, `MISSION.png` and
`HORIZON.png` frames. Yellow marks the image center; green marks the projected
geometric reference (which need not coincide with the visible mesh centroid).
The live-flight script is manual and is not run by `colcon test`.
