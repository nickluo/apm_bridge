#pragma once

#include <array>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace apm_bridge
{
    using GimbalJointAngles = std::array<double, 3>; // FRD roll, pitch, yaw in radians.
    using GimbalAxes = std::array<bool, 3>;

    // Aligned frames at encoder zero. The ZXY chain (yaw * roll * pitch) matches
    // both the physical joint order and the vendor camera-frame euler order
    // Z -> Y -> X from protocol appendix 2: phi, theta and psi are right-hand
    // rotations about the LFD camera axes X (left, pitch), Y (front, roll) and
    // Z (down, yaw) respectively, and the same chain expressed in the carrier
    // FRD basis is Rz(yaw)*Rx(roll)*Ry(pitch)
    // (theta -> +roll, -phi -> pitch-up, psi -> yaw). See
    // doc/gimbal_protocol_verified.md section 3.1 (2026-11-08 revision).
    //
    // mount: 云台基座在载机 FRD 下的固定安装旋转 (参数 gimbal.mount_rotation,
    // 行主序 3x3, 默认单位阵): camera_to_body = mount * (关节链)。
    inline tf2::Quaternion gimbalCameraToWorld(const tf2::Quaternion &body_to_world,
                                              const GimbalJointAngles &joints, const GimbalAxes &axes,
                                              const tf2::Quaternion &mount = tf2::Quaternion(0, 0, 0, 1))
    {
        const auto yaw = tf2::Quaternion(tf2::Vector3(0, 0, 1), axes[2] ? -joints[2] : 0);
        const auto roll = tf2::Quaternion(tf2::Vector3(1, 0, 0), axes[0] ? joints[0] : 0);
        const auto pitch = tf2::Quaternion(tf2::Vector3(0, 1, 0), axes[1] ? -joints[1] : 0);
        return (body_to_world * mount * yaw * roll * pitch).normalized();
    }

    // ZXY virtual encoders avoid the +/-90-degree ZYX pitch fold (ZXY is also
    // the vendor euler order, see gimbalCameraToWorld above).
    // mount 语义同上: 先除掉安装旋转 (mount^-1 * camera_to_body) 再分解关节角。
    inline GimbalJointAngles gimbalJointAngles(const tf2::Quaternion &camera_to_body, const GimbalAxes &axes,
                                               const tf2::Quaternion &mount = tf2::Quaternion(0, 0, 0, 1))
    {
        const tf2::Matrix3x3 rotation((mount.inverse() * camera_to_body).normalized());
        if (!axes[2])
            return {axes[0] ? std::atan2(rotation[2][1], rotation[1][1]) : 0,
                    axes[1] ? std::atan2(-rotation[0][2], rotation[0][0]) : 0, 0};
        const double cos_roll = std::hypot(rotation[2][0], rotation[2][2]);
        const bool singular = cos_roll < 1e-6;
        const double roll = std::atan2(rotation[2][1], cos_roll);
        const double pitch = singular ? 0 : std::atan2(rotation[2][0], rotation[2][2]);
        const double yaw = singular ? -std::atan2(rotation[1][0], rotation[0][0]) :
            std::atan2(rotation[0][1], rotation[1][1]);
        return {axes[0] ? roll : 0, axes[1] ? pitch : 0, axes[2] ? yaw : 0};
    }

    // Forward-propagate a body attitude by dt with body-frame angular rates omega (rad/s):
    // q(t+dt) = q(t) * exp(omega*dt). The RIGHT product applies the increment in the body
    // frame (strapdown gyro integration), so an attitude sampled at t1 can be re-epoched to
    // t2 using the gyro from the same message — residual error is gyro bias * dt only.
    // Inputs must be finite; near-zero omega returns q unchanged. 2026-11-09: used to
    // time-align the FC attitude to the gimbal-encoder sample before composing the camera
    // pose (epoch mismatch rotates the composed pose by omega*skew — 36 deg at 180 deg/s
    // over the old 200 ms gate — which under bank becomes the j*sin(roll) false azimuth).
    inline tf2::Quaternion propagateBodyRates(const tf2::Quaternion &q, const tf2::Vector3 &omega, double dt)
    {
        const double speed = omega.length();
        if (speed < 1e-12)
            return q;
        tf2::Quaternion out = q * tf2::Quaternion(omega / speed, speed * dt);
        out.normalize();
        return out;
    }
}
