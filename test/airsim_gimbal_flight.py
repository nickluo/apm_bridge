#!/usr/bin/env python3
"""Verify moving-flight gimbal modes with real UE physics and ROS/custom-link control.

AirSim APIs only observe the aircraft/camera/target. No pose teleportation,
synthetic flight sensors, MSP flight control or AirSim flight-control API is used.
"""

import argparse
from contextlib import ExitStack
import csv
import json
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import threading
import time

import cosysairsim as airsim
import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TwistStamped
from ament_index_python.packages import get_package_prefix
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import ControlCommand
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, Header, String
from vision_msgs.msg import Detection2D

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from sitl_support import MotorModel, VerticalEstimator, euler, enu_flu
from airsim_tracker.target_los import TargetLosEstimator


def vector(value):
    return np.array([value.x_val, value.y_val, value.z_val])


def rotation(value):
    return Rotation.from_quat([value.x_val, value.y_val, value.z_val, value.w_val])


def read_kinematics(client):
    value = client.simGetGroundTruthKinematics("Copter")
    if not isinstance(value, airsim.KinematicsState):
        raise RuntimeError("AirSim returned invalid aircraft kinematics")
    return value


def stop_process(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


class FlightObserver(Node):
    def __init__(self, model, pitch_hold):
        super().__init__("airsim_gimbal_flight")
        self.model = model
        self.pitch_hold = pitch_hold
        self.state: State | None = None
        self.imu: Imu | None = None
        self.odom: Odometry | None = None
        self.extended: ExtendedState | None = None
        self.mode: str | None = None
        self.received = {}
        self.vertical = VerticalEstimator()
        self.imu_stamp = None
        self.ground = 0.0
        self.last_altitude = 0.0
        self.last_throttle = 0.0
        self.yaw_origin = 0.0
        self.enabled = self.flying = self.moving = self.mission = False
        self.motion_start = 0.0
        self.height_goal = 0.0
        self.error = None
        self.lock = threading.RLock()
        self.command_pub = self.create_publisher(ControlCommand, "/fpv/control_command_raw", qos_profile_sensor_data)
        self.arm_pub = self.create_publisher(Bool, "/fpv/arm", 10)
        self.mission_pub = self.create_publisher(Bool, "/fpv/mission_state", 10)
        self.trigger_pub = self.create_publisher(Header, "/fpv/tracker_trigger", 10)
        self.los_pub = self.create_publisher(TwistStamped, "/fpv/target_los", 10)
        self.bbox_pub = self.create_publisher(Detection2D, "/fpv/target_bbox", 10)
        self.create_subscription(State, "/mavros/state", lambda m: self.receive("state", m), 10)
        self.create_subscription(Odometry, "/fpv/odom", lambda m: self.receive("odom", m), qos_profile_sensor_data)
        self.create_subscription(ExtendedState, "/mavros/extended_state", lambda m: self.receive("extended", m), 10)
        self.create_subscription(Imu, "/mavros/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Float64, "/fpv/altitude", self.on_altitude, qos_profile_sensor_data)
        self.create_subscription(String, "/fpv/gimbal_mode", self.on_mode,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.01, self.control)
        self.create_timer(.1, lambda: self.mission_pub.publish(Bool(data=self.mission)))

    def receive(self, key, message):
        with self.lock:
            setattr(self, key, message)
            self.received[key] = time.monotonic()

    def on_mode(self, message):
        self.mode = message.data

    def on_altitude(self, message):
        with self.lock:
            self.vertical.update(message.data)
            self.last_altitude = message.data
            self.received["altitude"] = time.monotonic()

    def on_imu(self, message):
        with self.lock:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if self.vertical.initialized and self.imu_stamp is not None:
                dt = stamp - self.imu_stamp
                if 0 < dt <= .2:
                    q, a = message.orientation, message.linear_acceleration
                    acceleration = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([a.x, a.y, a.z])
                    self.vertical.predict(acceleration[2] - 9.80665, dt)
                else:
                    self.get_logger().warning("Discontinuous flight IMU timestamp", throttle_duration_sec=2.0)
            self.imu_stamp = stamp
            self.receive("imu", message)

    def fail(self, message):
        self.error = message
        self.enabled = self.flying = False
        self.get_logger().error(message)
        self.arm_pub.publish(Bool(data=False))
        self.command_pub.publish(ControlCommand(armed=False, control_mode=ControlCommand.BODY_RATES,
                                               collective_thrust=0.0))

    def control(self):
        with self.lock:
            if not self.enabled or not self.flying:
                self.command_pub.publish(ControlCommand(armed=self.enabled, control_mode=ControlCommand.BODY_RATES,
                                                       collective_thrust=0.0))
                return
            now = time.monotonic()
            if self.state is None or self.imu is None or not self.state.armed or self.state.mode != "OFFBOARD":
                self.fail("Aircraft left armed OFFBOARD")
                return
            if any(now - self.received.get(key, 0) > timeout for key, timeout in
                   (("state", .5), ("imu", .3), ("altitude", .3))):
                self.fail("Flight telemetry became stale")
                return
            q = self.imu.orientation
            roll, pitch, yaw = euler((q.x, q.y, q.z, q.w))
            height, velocity = self.vertical.height - self.ground, self.vertical.velocity
            if height > 8 or abs(velocity) > 5 or max(abs(roll), abs(pitch)) > math.radians(25):
                self.fail(f"Flight envelope exceeded: h={height:.2f}, vz={velocity:.2f}, r/p={roll:.2f}/{pitch:.2f}")
                return
            roll_goal = pitch_goal = 0.0
            yaw_goal = self.yaw_origin
            if self.odom is not None:
                p, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
                oq = self.odom.pose.pose.orientation
                body_to_odom = Rotation.from_quat([oq.x, oq.y, oq.z, oq.w])
                velocity_odom = body_to_odom.apply([v.x, v.y, v.z])
                reference = np.zeros(2)
                reference_velocity = np.zeros(2)
                if self.moving:
                    if now - self.received.get("odom", 0) > .3:
                        self.fail("Motion test requires fresh FC odometry")
                        return
                    t, radius, frequency = now - self.motion_start, 1.5, .45
                    reference = radius * np.array([math.sin(frequency * t), 1 - math.cos(frequency * t)])
                    reference_velocity = radius * frequency * np.array([math.cos(frequency * t), math.sin(frequency * t)])
                    yaw_goal += math.radians(8) * math.sin(.4 * t)
                acceleration = np.clip(
                    1.5 * (reference_velocity + .8 * (reference - [p.x, p.y]) - velocity_odom[:2]), -1.5, 1.5)
                body_acceleration = body_to_odom.inv().apply([*acceleration, 0])
                roll_goal = -math.atan2(body_acceleration[1], 9.80665)
                pitch_goal = math.atan2(body_acceleration[0], 9.80665)
                if math.hypot(p.x, p.y) > 8:
                    self.fail("Aircraft left the 8 m horizontal test envelope")
                    return
            rates = np.clip([4 * (roll_goal - roll), 4 * (pitch_goal - pitch),
                             3 * math.atan2(math.sin(yaw_goal - yaw), math.cos(yaw_goal - yaw))], -.5, .5)
            rotor_hover = .055 + .945 * self.model.throttle(2.0 * 9.80665)
            tilt = max(.7, math.cos(roll) * math.cos(pitch))
            throttle = (rotor_hover / tilt - .055) / .945 + rotor_hover / .587 * (
                .05 * (self.height_goal - height) - .10 * velocity)
            message = ControlCommand(armed=True, control_mode=ControlCommand.BODY_RATES,
                                     collective_thrust=float(np.clip(throttle, 0, .5)))
            message.bodyrates.x, message.bodyrates.y, message.bodyrates.z = map(float, rates)
            self.last_throttle = message.collective_thrust
            self.command_pub.publish(message)


def wait_until(predicate, observer, timeout, label, allow_flight_error=False):
    deadline = time.monotonic() + timeout
    while not predicate():
        if observer.error and not allow_flight_error:
            raise RuntimeError(observer.error)
        if time.monotonic() >= deadline:
            raise TimeoutError(label)
        time.sleep(.02)


def run_flight(client, node, output, args):
    report = {"passed": False, "model": args.model, "pitch_hold_deg": args.pitch_hold,
              "pitch_aim_deg": args.pitch_aim, "requested_aim_pitch_deg": 90 + args.pitch_aim,
              "ros_domain_id": args.ros_domain_id,
              "control": "ROS/custom-link only", "physics": "live UE/Cosys-AirSim",
              "los_source": "TargetDrone actor-origin geometric reference, using TargetLosEstimator; not a visual bbox centroid",
              "thresholds": {"aim_relative_error_p95_deg": 1.0, "horizon_pitch_error_p95_deg": 1.5,
                             "mission_pitch_error_p95_deg": 2.0, "mission_yaw_error_p95_deg": 2.0,
                             "minimum_airborne_height_m": 2.0, "minimum_motion_fraction": .8,
                             "mission_camera_pitch_rate_p99_dps": 35.0}}
    rows = []
    previous: tuple[float, Rotation] | None = None
    aim_rotation = Rotation.identity()
    focal = 768 / math.tan(math.radians(72) / 2)
    estimator = TargetLosEstimator(focal, (768, 480))
    target_global = vector(client.simGetObjectPose("TargetDrone").position)
    origin = vector(read_kinematics(client).position)
    local_offset = vector(client.simGetObjectPose("Copter").position) - origin
    target = target_global - local_offset
    report["target_ned_m"] = target.tolist()
    start = time.monotonic()
    fields = ("stage", "elapsed_s", "height_m", "north_m", "east_m", "speed_xy_mps", "body_roll_deg",
              "body_pitch_up_deg", "body_heading_deg", "camera_pitch_up_deg", "relative_rotation_error_deg",
              "relative_roll_deg", "relative_yaw_deg", "los_pitch_down_deg", "los_yaw_left_deg",
              "camera_pitch_rate_dps", "camera_yaw_rate_dps", "mode", "landed_state")
    fields += ("fc_height_est_m", "baro_altitude_m", "specific_force_z_mps2", "collective")
    fields += ("camera_roll_deg", "camera_heading_deg", "los_pitch_rate_dps", "los_yaw_rate_dps")
    fields += ("aim_reference_error_deg",)
    stream = (output / "flight.csv").open("x", newline="")
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()

    def sample(stage, publish_los=False):
        nonlocal previous
        if node.error:
            raise RuntimeError(node.error)
        kinematics = read_kinematics(client)
        camera = client.simGetCameraInfo("0", "Copter").pose
        stamp = node.get_clock().now()
        body_rotation, camera_rotation = rotation(kinematics.orientation), rotation(camera.orientation)
        relative = body_rotation.inv() * camera_rotation
        roll, pitch, yaw = body_rotation.as_euler("ZYX")[::-1]
        cr, cp, cy = camera_rotation.as_euler("ZYX")[::-1]
        rr, _, ry = relative.as_euler("ZYX")[::-1]
        ray = camera_rotation.inv().apply(target - vector(camera.position))
        if ray[0] <= 0 and publish_los:
            raise RuntimeError("Target moved behind the camera")
        angles = np.array([math.atan2(ray[2], math.hypot(ray[0], ray[1])), -math.atan2(ray[1], ray[0])])
        published_rates = np.zeros(2)
        if publish_los:
            center = (768 + focal * ray[1] / ray[0], 480 + focal * ray[2] / ray[0])
            camera_flu_to_enu = Rotation.from_quat(enu_flu(camera.orientation)).as_matrix()
            published_angles, rates = estimator.update(center, stamp.nanoseconds * 1e-9, camera_flu_to_enu)
            published_rates = rates
            if np.max(np.abs(published_angles - angles)) > 1e-6:
                raise AssertionError("Tracker LOS axes disagree with independent FRD ground-truth projection")
            los = TwistStamped()
            los.header.stamp = stamp.to_msg()
            los.header.frame_id = "camera_flu"
            los.twist.linear.y, los.twist.linear.z = map(float, published_angles)
            los.twist.angular.y, los.twist.angular.z = map(float, rates)
            bbox = Detection2D()
            bbox.header.stamp = los.header.stamp
            bbox.header.frame_id = "camera_frame"
            bbox.bbox.center.position.x, bbox.bbox.center.position.y = map(float, center)
            bbox.bbox.size_x = bbox.bbox.size_y = 30.0
            node.bbox_pub.publish(bbox)
            node.los_pub.publish(los)
        now = time.monotonic()
        rates = np.zeros(3)
        if previous is not None:
            previous_time, previous_rotation = previous
            rates = (previous_rotation.inv() * camera_rotation).as_rotvec() / (now - previous_time)
        previous = (now, camera_rotation)
        height = float(origin[2] - kinematics.position.z_val)
        row = dict(zip(fields, (stage, now - start, height,
            kinematics.position.x_val, kinematics.position.y_val,
            math.hypot(kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val),
            math.degrees(roll), math.degrees(pitch), math.degrees(yaw), math.degrees(cp),
            math.degrees(relative.magnitude()), math.degrees(rr), math.degrees(ry),
            math.degrees(angles[0]), math.degrees(angles[1]), math.degrees(rates[1]), math.degrees(rates[2]),
            node.mode, node.extended.landed_state if node.extended else -1,
            node.vertical.height - node.ground, node.last_altitude,
            node.imu.linear_acceleration.z if node.imu else 0, node.last_throttle,
            math.degrees(cr), math.degrees(cy), math.degrees(published_rates[0]), math.degrees(published_rates[1]),
            math.degrees((aim_rotation.inv() * relative).magnitude()))))
        writer.writerow(row)
        stream.flush()
        rows.append(row)
        return row, height

    def stage(name, duration, desired_mode, settling=1.0, publish_los=False):
        wait_until(lambda: node.mode == desired_mode, node, 3, f"{name} mode not acknowledged")
        estimator.reset()
        begin = time.monotonic()
        measured = []
        all_stage = []
        while time.monotonic() - begin < duration:
            row, _ = sample(name, publish_los)
            all_stage.append(row)
            if row["mode"] != desired_mode:
                raise AssertionError(f"{name} changed gimbal mode while airborne")
            if time.monotonic() - begin >= settling:
                measured.append(row)
            time.sleep(.025)
        if len(measured) < 15:
            raise AssertionError(f"{name} has too few real-camera observations")
        data = lambda key: np.asarray([row[key] for row in measured])
        metric = {"samples": len(measured), "height_min_m": float(data("height_m").min()),
                  "in_air_fraction": float(np.mean(data("landed_state") == ExtendedState.LANDED_STATE_IN_AIR)),
                  "motion_fraction": float(np.mean(data("speed_xy_mps") > .2)),
                  "speed_xy_median_mps": float(np.median(data("speed_xy_mps"))),
                  "body_roll_pitch_span_deg": [float(np.ptp(data("body_roll_deg"))), float(np.ptp(data("body_pitch_up_deg")))],
                  "relative_error_p95_deg": float(np.percentile(data("relative_rotation_error_deg"), 95)),
                  "aim_reference_error_p95_deg": float(np.percentile(data("aim_reference_error_deg"), 95)),
                  "pitch_error_p95_deg": float(np.percentile(np.abs(data("camera_pitch_up_deg") - args.pitch_hold), 95)),
                  "los_pitch_error_p95_deg": float(np.percentile(np.abs(data("los_pitch_down_deg")), 95)),
                  "los_yaw_error_p95_deg": float(np.percentile(np.abs(data("los_yaw_left_deg")), 95)),
                  "pitch_rate_p99_dps": float(np.percentile(np.abs(data("camera_pitch_rate_dps")), 99)),
                  "whole_stage_pitch_rate_p99_dps": float(np.percentile(
                      [abs(row["camera_pitch_rate_dps"]) for row in all_stage], 99)),
                  "whole_stage_yaw_rate_p99_dps": float(np.percentile(
                      [abs(row["camera_yaw_rate_dps"]) for row in all_stage], 99)),
                  "relative_absent_axes_p95_deg": [float(np.percentile(np.abs(data("relative_roll_deg")), 95)),
                                                    float(np.percentile(np.abs(data("relative_yaw_deg")), 95))]}
        report[name] = metric
        print(f"{name}: {json.dumps(metric)}", flush=True)
        if metric["height_min_m"] < 2 or metric["motion_fraction"] < .8 or metric["in_air_fraction"] < .99:
            raise AssertionError(f"{name} was not a continuously moving airborne test")
        if name == "AIM" and metric["aim_reference_error_p95_deg"] > 1:
            raise AssertionError("AIM failed body +Z/offset reference after mechanical limiting")
        if name == "HORIZON" and metric["pitch_error_p95_deg"] > 1.5:
            raise AssertionError("HORIZON failed world pitch hold")
        if name == "MISSION":
            if metric["los_pitch_error_p95_deg"] > 2:
                raise AssertionError("MISSION failed zero-pitch target centering")
            if args.model == "C200T" and metric["los_yaw_error_p95_deg"] > 2:
                raise AssertionError("MISSION failed horizontal target centering")
            if max(metric["whole_stage_pitch_rate_p99_dps"], metric["whole_stage_yaw_rate_p99_dps"]) > 35:
                raise AssertionError("MISSION camera motion exceeded the 30 deg/s limit plus 5 deg/s observation allowance")
        response = client.simGetImages(
            [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Copter")[0]
        if not response.image_data_uint8:
            sample(name, publish_los)
            time.sleep(.05)
            response = client.simGetImages(
                [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Copter")[0]
        image = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        if image.size == response.width * response.height and image.size > 0:
            image = cv2.cvtColor(image.reshape(response.height, response.width), cv2.COLOR_GRAY2BGR)
        elif image.size == response.width * response.height * 3 and image.size > 0:
            image = image.reshape(response.height, response.width, 3).copy()
        else:
            raise RuntimeError(f"{name} returned an invalid live camera image")
        capture_ray = rotation(response.camera_orientation).inv().apply(target - vector(response.camera_position))
        if capture_ray[0] > 0:
            pixel = (round(response.width / 2 + focal * capture_ray[1] / capture_ray[0]),
                     round(response.height / 2 + focal * capture_ray[2] / capture_ray[0]))
            cv2.drawMarker(image, pixel, (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
        cv2.drawMarker(image, (response.width // 2, response.height // 2), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.putText(image, f"{args.model} {name} | aim offset {args.pitch_aim:g}, horizon {args.pitch_hold:g} deg",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
        if not cv2.imwrite(str(output / f"{name}.png"), image):
            raise RuntimeError(f"Failed to save {name} live camera evidence")

    try:
        wait_until(lambda: node.state is not None and node.imu is not None and node.vertical.initialized,
                   node, 20, "custom-link telemetry unavailable")
        wait_until(lambda: node.imu is not None and 8 < node.imu.linear_acceleration.z < 11.5,
                   node, 15, "Live physics accelerometer did not become ready")
        time.sleep(5)
        parameters = AsyncParameterClient(node, "custom_link_bridge")
        if not parameters.wait_for_services(timeout_sec=5):
            raise RuntimeError("Bridge parameter service unavailable")
        future = parameters.get_parameters(["airsim.enabled", "link.transport", "gimbal_port", "gimbal.model",
                                             "gimbal.pitch_hold", "mass", "gravity_const",
                                             "gimbal.pitch_aim", "gimbal.pitch_travel_min", "gimbal.pitch_travel_max"])
        wait_until(future.done, node, 5, "Bridge parameter query timed out")
        result = future.result()
        if result is None or len(result.values) != 10:
            raise RuntimeError("Bridge returned an invalid parameter response")
        values = result.values
        if not values[0].bool_value or values[1].string_value != "tcp" or values[2].string_value:
            raise RuntimeError("Test requires an AirSim TCP bridge, never a real serial link")
        if values[3].string_value != args.model or values[4].double_value != args.pitch_hold:
            raise RuntimeError("Running gimbal model/pitch_hold differs from the test")
        if not math.isclose(values[7].double_value, args.pitch_aim, abs_tol=1e-5):
            raise RuntimeError("Running pitch_aim differs from the test")
        pitch_min, pitch_max = values[8].double_value, values[9].double_value  # 机械行程(编码器域)
        effective_aim = 90 + args.pitch_aim if pitch_min == pitch_max else float(np.clip(90 + args.pitch_aim, pitch_min, pitch_max))
        report["effective_aim_pitch_deg"] = effective_aim
        aim_rotation = Rotation.from_rotvec([0, -math.radians(effective_aim), 0])
        if not math.isclose(values[5].double_value, 2) or not math.isclose(values[6].double_value, 9.80665):
            raise RuntimeError("Flight harness requires the calibrated 2 kg SITL model")
        if node.count_publishers("/fpv/control_command_raw") != 1 or node.count_publishers("/fpv/control_command") != 0:
            raise RuntimeError("Another ROS flight controller is publishing: " +
                repr([(info.node_name, info.topic_type) for topic in ("/fpv/control_command_raw", "/fpv/control_command")
                      for info in node.get_publishers_info_by_topic(topic)]))
        if node.count_publishers("/fpv/target_los") != 1:
            raise RuntimeError("Another target LOS publisher is running")
        if node.state.armed:
            raise RuntimeError("Test requires an initially disarmed aircraft")
        with node.lock:
            node.vertical = VerticalEstimator()
            node.vertical.update(node.last_altitude)
            node.ground = node.last_altitude
            node.imu_stamp = None
        print(f"Ground sensors ready: altitude={node.ground:.3f}, acc_z={node.imu.linear_acceleration.z:.3f}", flush=True)
        q = node.imu.orientation
        node.yaw_origin = euler((q.x, q.y, q.z, q.w))[2]
        node.enabled = True
        node.arm_pub.publish(Bool(data=True))
        wait_until(lambda: node.state.armed and node.state.mode == "OFFBOARD", node, 15, "SITL arming refused")
        node.height_goal = 3.5
        node.flying = True
        deadline = time.monotonic() + 25
        while not (abs(node.vertical.height - node.ground - 3.5) < .4 and
                   abs(node.vertical.velocity) < .4 and node.odom is not None):
            _, height = sample("TAKEOFF")
            if height > 8:
                node.fail("Ground truth takeoff exceeded 8 m")
                raise RuntimeError(node.error)
            if time.monotonic() >= deadline:
                raise TimeoutError("Moving-flight takeoff did not settle")
            time.sleep(.05)
        print("Airborne OFFBOARD established at 3.5 m", flush=True)
        node.motion_start = time.monotonic()
        node.moving = True
        node.trigger_pub.publish(Header(frame_id="ON"))
        stage("AIM", 10, "AIM", settling=2)
        node.mission = True
        stage("MISSION", 18, "MISSION", settling=6, publish_los=True)
        node.mission = False
        stage("HORIZON", 10, "HORIZON", settling=2)
        report["passed"] = True
    except (RuntimeError, TimeoutError, AssertionError) as error:
        report["error"] = str(error)
        raise
    finally:
        try:
            node.mission = False
            node.moving = False
            if node.enabled and node.state is not None and node.state.armed:
                node.height_goal = -1.0
                wait_until(lambda: node.vertical.height - node.ground < .2 and abs(node.vertical.velocity) < .3,
                           node, 20, "Landing did not settle")
            node.enabled = node.flying = False
            node.arm_pub.publish(Bool(data=False))
            wait_until(lambda: node.state is not None and not node.state.armed, node, 5, "Disarm was not confirmed",
                       allow_flight_error=True)
            wait_until(lambda: node.mode == "AIM", node, 3, "Ground AIM mode was not acknowledged",
                       allow_flight_error=True)
            deadline = time.monotonic() + 10
            while True:
                final = read_kinematics(client)
                camera = client.simGetCameraInfo("0", "Copter")
                if not isinstance(camera, airsim.CameraInfo):
                    raise RuntimeError("AirSim returned invalid cleanup camera information")
                alignment = math.degrees((aim_rotation.inv() * rotation(final.orientation).inv() *
                                          rotation(camera.pose.orientation)).magnitude())
                if (np.linalg.norm(vector(final.linear_velocity)) < .25 and
                        abs(origin[2] - final.position.z_val) < .3 and alignment < 1):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("Aircraft is disarmed but ground-truth touchdown is not confirmed")
                time.sleep(.1)
            report["final_speed_mps"] = float(np.linalg.norm(vector(final.linear_velocity)))
            state = node.state
            if state is None:
                raise RuntimeError("Disarm telemetry disappeared during cleanup")
            report["final_armed"] = bool(state.armed)
            report["final_gimbal_mode"] = node.mode
            report["final_camera_aim_error_deg"] = alignment
            print(f"Landing/disarm confirmed; speed={report['final_speed_mps']:.4f} m/s", flush=True)
        except (RuntimeError, TimeoutError) as error:
            report["passed"] = False
            report["cleanup_error"] = str(error)
            raise
        finally:
            stream.close()
            (output / "result.json").write_text(json.dumps(report, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--eeprom", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", choices=("C20S", "C200T"), default="C20S")
    parser.add_argument("--pitch-hold", type=float, default=10.0)
    parser.add_argument("--pitch-aim", type=float, default=-90.0,
                        help="AIM offset (deg, nose-up); -90 keeps the existing ground target visible")
    parser.add_argument("--ros-domain-id", type=int, default=173,
                        help="isolated DDS domain for this simulation experiment")
    args = parser.parse_args()
    args.binary, args.eeprom = args.binary.resolve(), args.eeprom.resolve()
    args.output = args.output.resolve()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("ROS domain must be in [0,232]")
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    if (not args.binary.is_file() or not args.eeprom.is_file() or not math.isfinite(args.pitch_hold) or
            not math.isfinite(args.pitch_aim) or abs(args.pitch_aim) > 90):
        parser.error("SITL binary/EEPROM must exist and pitch hold must be finite")
    for port in (5761, 5763, 5764):
        with socket.socket() as probe:
            probe.settimeout(.3)
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                raise RuntimeError(f"Port {port} is occupied; will not replace another simulator/controller")
    client = airsim.MultirotorClient(timeout_value=3)
    settings_text = client.getSettingsString()
    if not isinstance(settings_text, str):
        raise RuntimeError("AirSim did not return its runtime settings")
    settings = json.loads(settings_text)
    vehicle = settings["Vehicles"]["Copter"]
    if vehicle["VehicleType"] != "BetaFlight" or vehicle["TcpPort"] != 5764:
        raise RuntimeError("Live AirSim must use BetaFlight via bridge relay 5764")
    if vehicle["Cameras"]["0"]["Gimbal"]["Stabilization"] != 0:
        raise RuntimeError("Disable built-in camera stabilization and restart Play")
    if client.isApiControlEnabled("Copter") or np.linalg.norm(vector(read_kinematics(client).linear_velocity)) > .3:
        raise RuntimeError("AirSim is not idle; do not take over another flight")
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(args.eeprom, args.output / "eeprom.bin")
    sys.path.insert(0, str(args.binary.parents[2] / "src/test/sitl"))
    from sitl_joystick import CLI_CONFIG_LINES
    lines = [*CLI_CONFIG_LINES, "feature GPS", "set gps_provider = VIRTUAL", "set trust_mag = ON",
             "set mag_declination = 20", "set small_angle = 45", "set ap_hover_throttle = 1211",
             "set ap_throttle_min = 1050", "set airmode_start_throttle_percent = 15",
             "set custom_link_motors_stream = ON"]
    config = args.output / "flight.cfg"
    config.write_text("\n".join(lines) + "\n")
    provision = subprocess.run([str(args.binary), "--config", str(config)], cwd=args.output,
                               capture_output=True, text=True, timeout=60)
    (args.output / "provision.log").write_text(provision.stdout + provision.stderr)
    if provision.returncode or "###ERROR" in provision.stdout + provision.stderr:
        raise RuntimeError("Failed to provision the isolated EEPROM; inspect provision.log")
    with ExitStack() as stack:
        sitl_log = stack.enter_context((args.output / "sitl.log").open("w"))
        sitl = subprocess.Popen([str(args.binary)], cwd=args.output, stdout=sitl_log, stderr=sitl_log)
        stack.callback(stop_process, sitl)
        bridge_log = stack.enter_context((args.output / "bridge.log").open("w"))
        bridge = subprocess.Popen(
            [str(Path(get_package_prefix("apm_bridge")) / "lib/apm_bridge/custom_link_bridge_node"), "--ros-args",
             "--params-file", str(Path(__file__).resolve().parents[1] / "parameters/sitl_parameters.yaml"),
             "-p", f"gimbal.model:={args.model}", "-p", f"gimbal.pitch_hold:={args.pitch_hold}",
             "-p", f"gimbal.pitch_aim:={args.pitch_aim}",
             "-r", "~/control_command_raw:=/fpv/control_command_raw", "-r", "~/control_command:=/fpv/control_command",
             "-r", "~/arm:=/fpv/arm", "-r", "~/offboard:=/fpv/offboard",
             "-r", "~/low_level_feedback:=/fpv/low_level_feedback"], stdout=bridge_log, stderr=bridge_log)
        stack.callback(stop_process, bridge)
        model = MotorModel(0, 0, .011891350984191815, -.059949564628813956, thrust_limit=.5)
        rclpy.init()
        node = FlightObserver(model, args.pitch_hold)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            run_flight(client, node, args.output, args)
        finally:
            executor.shutdown()
            thread.join(timeout=3)
            node.destroy_node()
            rclpy.shutdown()
    print(f"PASS: real moving-flight {args.model} AIM/MISSION/HORIZON; evidence: {args.output}", flush=True)


if __name__ == "__main__":
    main()
