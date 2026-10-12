# apm_bridge

Hardware and AirSim SITL use **the same `CustomLinkVehicleNode` executable** and
the same **custom Betaflight protocol**. No MSP is used by the SITL implementation.
The existing hardware launch/configuration are unchanged.

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
AirSim camera API :41451 ---- orientation only --> /fpv/gimbal
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

## Preserved interfaces

The existing `/fpv/control_command`, `/fpv/control_command_raw`, `/fpv/arm`,
`/fpv/low_level_feedback` and MAVROS-compatible telemetry interfaces are reused.
`/fpv/offboard` (`std_msgs/msg/Bool`, private name `~/offboard`) independently
requests OFFBOARD (`true`, custom-link `0x20 mode_req=2`) or manual control
(`false`, `mode_req=0`). A request is sent even without a fresh control command;
subsequent control/arm frames retain the requested mode. Mode requests do not
change the host arm bit, and do not keep stale control commands alive: maintain
a fresh control stream while in OFFBOARD. The actual result is reported by
`/mavros/state`, subject to firmware safety and pilot/Rescue arbitration.
The pilot's held OFFBOARD AUX switch takes priority over an API manual request.
After pilot takeover, deliberately request `false`, then `true` to enter a
new API session. Existing automatic OFFBOARD behavior is unchanged until this
topic is used.

```bash
ros2 topic pub --once /fpv/offboard std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /fpv/offboard std_msgs/msg/Bool "{data: false}"
```

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
RC trigger defaults to CH8/AUX4, zero-based index 7.

The **only** AirSim RPC is `simGetCameraInfo`. Camera FRD/NED orientation is
converted to FLU/ENU; `/fpv/gimbal` preserves the hardware heading-relative
Euler slots in `orientation_covariance[0:3]`. Angular velocity is derived from
successive camera poses, and marked unavailable until two poses arrive.
Camera API failure is logged and does not stop custom-link flight telemetry.

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
