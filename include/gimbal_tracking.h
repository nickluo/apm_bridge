#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace apm_bridge
{
    enum class GimbalMode { AIM, MISSION, HORIZON };

    inline const char *gimbalModeName(GimbalMode mode)
    {
        switch (mode)
        {
        case GimbalMode::AIM: return "AIM";
        case GimbalMode::MISSION: return "MISSION";
        case GimbalMode::HORIZON: return "HORIZON";
        }
        throw std::invalid_argument("Invalid gimbal mode");
    }

    struct TargetLos
    {
        bool valid = false;
        double pitch = 0, yaw = 0; // Camera FLU: pitch down, yaw left; rad.
        double pitch_rate = 0, yaw_rate = 0; // Ego-compensated, rad/s.
        double age = 0; // Sample age at reception, seconds.
    };

    struct GimbalTrackingConfig
    {
        double gain = 2.0;
        double rate_feedforward = 1.0;
        double max_rate_dps = 30.0;
        double max_accel_dps2 = 90.0;
        double filter_tau = 0.15;
        double los_timeout = 0.3;

        void validate() const
        {
            if (!std::isfinite(gain) || gain <= 0 ||
                !std::isfinite(rate_feedforward) || rate_feedforward < 0 ||
                !std::isfinite(max_rate_dps) || max_rate_dps <= 0 || max_rate_dps > 150 ||
                !std::isfinite(max_accel_dps2) || max_accel_dps2 <= 0 ||
                !std::isfinite(filter_tau) || filter_tau <= 0 ||
                !std::isfinite(los_timeout) || los_timeout <= 0)
                throw std::invalid_argument("Invalid gimbal tracking gain, rate, acceleration, filter or timeout");
        }
    };

    class GimbalAngleTrajectory
    {
    public:
        explicit GimbalAngleTrajectory(const GimbalTrackingConfig &config) : cfg_(config) { cfg_.validate(); }

        void reset(const std::array<double, 3> &degrees, const std::array<double, 3> &rates = {})
        {
            position_ = degrees;
            for (size_t i = 0; i < rate_.size(); ++i)
                rate_[i] = std::clamp(rates[i], -cfg_.max_rate_dps, cfg_.max_rate_dps);
        }

        std::array<double, 3> step(const std::array<double, 3> &target, double dt,
                                   const std::array<bool, 3> &axes)
        {
            if (!std::isfinite(dt) || dt <= 0)
                throw std::invalid_argument("Invalid gimbal angle trajectory step");
            for (size_t i = 0; i < position_.size(); ++i)
            {
                if (!axes[i])
                {
                    position_[i] = rate_[i] = 0;
                    continue;
                }
                const double error = target[i] - position_[i];
                const double brake_step = cfg_.max_accel_dps2 * dt;
                const double braking = std::sqrt(brake_step * brake_step +
                    2 * cfg_.max_accel_dps2 * std::abs(error)) - brake_step;
                const double desired = std::copysign(std::min({cfg_.max_rate_dps, braking, 8 * std::abs(error)}), error);
                const double previous = rate_[i];
                rate_[i] += std::clamp(desired - rate_[i], -cfg_.max_accel_dps2 * dt, cfg_.max_accel_dps2 * dt);
                position_[i] += (previous + rate_[i]) * .5 * dt;
            }
            return position_;
        }

        void constrain(size_t axis, double lower, double upper)
        {
            if (lower >= upper)
                return;
            position_[axis] = std::clamp(position_[axis], lower, upper);
            if ((position_[axis] <= lower && rate_[axis] < 0) ||
                (position_[axis] >= upper && rate_[axis] > 0))
                rate_[axis] = 0; // Mechanical stops override acceleration continuity.
        }

        const std::array<double, 3> &rates() const { return rate_; }
        const std::array<double, 3> &positions() const { return position_; }

    private:
        GimbalTrackingConfig cfg_;
        std::array<double, 3> position_{};
        std::array<double, 3> rate_{};
    };

    class GimbalTrackingController
    {
    public:
        using Clock = std::chrono::steady_clock;
        explicit GimbalTrackingController(const GimbalTrackingConfig &config) : cfg_(config)
        {
            cfg_.validate();
        }

        void update(const TargetLos &target, Clock::time_point now = Clock::now())
        {
            if (!std::isfinite(target.pitch) || !std::isfinite(target.yaw) ||
                !std::isfinite(target.pitch_rate) || !std::isfinite(target.yaw_rate) ||
                !std::isfinite(target.age) || target.age < 0)
                throw std::invalid_argument("Non-finite target LOS");
            if (target.valid && (std::abs(target.pitch) >= M_PI_2 - 1e-6 ||
                                 std::abs(target.yaw) >= M_PI_2 - 1e-6))
                throw std::invalid_argument("Target LOS must be inside the forward camera hemisphere");
            std::lock_guard<std::mutex> lock(mutex_);
            target_ = target;
            stamp_ = now - std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(target.age));
        }

        void reset()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            rate_ = {};
        }

        bool fresh(Clock::time_point now = Clock::now()) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return freshLocked(now);
        }

        std::array<double, 3> step(double roll_error, double dt, Clock::time_point now = Clock::now(),
                                  std::array<bool, 3> axes = {true, true, true})
        {
            if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(roll_error))
                throw std::invalid_argument("Invalid gimbal controller step");
            std::lock_guard<std::mutex> lock(mutex_);
            const bool valid = freshLocked(now);
            std::array<double, 3> desired = {
                axes[0] ? cfg_.gain * roll_error : 0.0,
                valid ? cfg_.rate_feedforward * target_.pitch_rate + cfg_.gain * target_.pitch : 0.0,
                valid ? cfg_.rate_feedforward * target_.yaw_rate + cfg_.gain * target_.yaw : 0.0};
            const double limit = cfg_.max_rate_dps * M_PI / 180.0;
            if (valid)
            {
                // Invert the spherical LOS Jacobian, with roll supplied by the leveling loop.
                const double cy = std::cos(target_.yaw), sy = std::sin(target_.yaw);
                desired[1] = (desired[1] + desired[0] * sy) / cy;
                desired[2] -= std::tan(target_.pitch) * (desired[0] * cy + desired[1] * sy);
            }
            for (size_t i = 0; i < desired.size(); ++i)
                if (!axes[i])
                    desired[i] = 0;
            const double peak = std::max({std::abs(desired[0]), std::abs(desired[1]), std::abs(desired[2])});
            if (peak > limit)
                for (auto &value : desired)
                    value *= limit / peak; // Preserve the projected LOS correction direction when saturated.
            const double slew = cfg_.max_accel_dps2 * M_PI / 180.0 * dt;
            const double alpha = -std::expm1(-dt / cfg_.filter_tau);
            for (size_t i = 0; i < rate_.size(); ++i)
            {
                const double filtered = rate_[i] + alpha * (desired[i] - rate_[i]);
                rate_[i] += std::clamp(filtered - rate_[i], -slew, slew);
            }
            return rate_;
        }

    private:
        bool freshLocked(Clock::time_point now) const
        {
            const double age = std::chrono::duration<double>(now - stamp_).count();
            return target_.valid && age >= 0 && age <= cfg_.los_timeout;
        }
        GimbalTrackingConfig cfg_;
        mutable std::mutex mutex_;
        TargetLos target_;
        Clock::time_point stamp_{};
        std::array<double, 3> rate_{};
    };

    class GimbalModeSelector
    {
    public:
        GimbalMode update(bool armed, bool airborne, bool mission, bool manual,
                          bool aim_requested = false, bool abort_requested = false)
        {
            if (!mission)
                mission_blocked_ = false;
            if (!armed)
            {
                mode_ = GimbalMode::AIM;
                mission_blocked_ = mission;
            }
            else if (abort_requested || (mission && manual && mode_ == GimbalMode::MISSION))
            {
                mission_blocked_ = mission;
                mode_ = airborne ? GimbalMode::HORIZON : GimbalMode::AIM;
            }
            else if (mission && !manual && !mission_blocked_)
                mode_ = GimbalMode::MISSION;
            else if (mode_ == GimbalMode::MISSION || (manual && !was_manual_ && airborne))
                mode_ = airborne ? GimbalMode::HORIZON : GimbalMode::AIM;
            else if (aim_requested)
                mode_ = GimbalMode::AIM;
            was_manual_ = manual;
            return mode_;
        }

    private:
        GimbalMode mode_ = GimbalMode::AIM;
        bool was_manual_ = true;
        bool mission_blocked_ = false;
    };
}
