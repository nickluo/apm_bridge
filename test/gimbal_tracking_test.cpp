#include "gimbal_tracking.h"

#include <iostream>

namespace
{
    void require(bool condition, const char *message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    void modes()
    {
        using apm_bridge::GimbalMode;
        apm_bridge::GimbalModeSelector selector;
        require(selector.update(false, false, false, true) == GimbalMode::AIM, "disarmed AIM");
        require(selector.update(true, true, false, true) == GimbalMode::AIM, "manual aiming in flight");
        require(selector.update(true, true, true, false) == GimbalMode::MISSION, "authorized mission");
        require(selector.update(true, true, false, false) == GimbalMode::HORIZON, "mission exit horizon");
        require(selector.update(true, true, false, true) == GimbalMode::HORIZON, "manual hold horizon");
        require(selector.update(true, true, false, true, true) == GimbalMode::AIM, "new ON re-enters AIM");
        require(selector.update(true, true, true, false) == GimbalMode::MISSION, "new mission");
        require(selector.update(true, true, true, true) == GimbalMode::HORIZON, "takeover preempts stale mission");
        require(selector.update(true, true, true, false) == GimbalMode::HORIZON, "stale mission cannot resume");
        selector.update(true, true, false, false);
        require(selector.update(true, true, true, false) == GimbalMode::MISSION, "new mission edge resumes");
        require(selector.update(true, true, true, false, false, true) == GimbalMode::HORIZON, "OFF preempts mission");
        require(selector.update(true, true, true, false) == GimbalMode::HORIZON, "OFF latch persists");
        require(selector.update(false, false, true, false) == GimbalMode::AIM, "disarm resets");
        // DISARM 从每个状态回 AIM (任务未复位时锁存, 复位后可重入)
        selector.update(true, true, false, false); // 清除 disarm 时置位的任务锁存
        require(selector.update(true, true, true, false) == GimbalMode::MISSION, "mission again");
        require(selector.update(false, true, true, false) == GimbalMode::AIM, "disarm during MISSION -> AIM");
        require(selector.update(true, true, true, false) == GimbalMode::AIM,
                "re-arm with stale mission stays AIM (disarm reset persists)");
        require(selector.update(true, true, false, false) == GimbalMode::AIM, "mission reset clears latch (AIM holds)");
        require(selector.update(true, true, true, false) == GimbalMode::MISSION, "fresh mission resumes");
        require(selector.update(true, true, false, false) == GimbalMode::HORIZON, "mission ends airborne -> HORIZON");
        require(selector.update(false, true, false, true) == GimbalMode::AIM, "disarm from HORIZON -> AIM");
        require(selector.update(true, true, false, true) == GimbalMode::AIM,
                "re-armed manual stays AIM (no HORIZON regression)");
        selector.update(false, false, true, false); // 恢复 disarm+mission 锁存, 供下条既有断言
        require(selector.update(true, false, true, false) == GimbalMode::AIM, "stale mission after rearm");
    }

    void rates()
    {
        apm_bridge::GimbalTrackingConfig config;
        apm_bridge::GimbalTrackingController controller(config);
        const auto start = apm_bridge::GimbalTrackingController::Clock::now();
        constexpr double dt = 0.02;
        constexpr double radians = M_PI / 180.0;
        const apm_bridge::TargetLos centered{true, 0, 0, 0, 0};
        controller.update(centered, start);
        auto output = controller.step(0, dt, start);
        require(output[1] == 0 && output[2] == 0, "MISSION must center LOS at zero");
        std::array<double, 3> previous{};
        for (int i = 0; i < 150; ++i)
        {
            const auto now = start + std::chrono::milliseconds(20 * i);
            controller.update({true, 1.0, -1.0, 1.0, -1.0}, now);
            output = controller.step(1.0, dt, now);
            for (size_t axis = 0; axis < 3; ++axis)
            {
                require(std::abs(output[axis]) <= config.max_rate_dps * radians + 1e-10, "rate limit");
                require(std::abs(output[axis] - previous[axis]) <= config.max_accel_dps2 * radians * dt + 1e-10,
                        "acceleration limit");
            }
            previous = output;
        }
        require(output[1] > 0 && output[2] < 0, "FLU target control signs");
        auto stale = start + std::chrono::seconds(4);
        require(!controller.fresh(stale), "stale LOS accepted");
        for (int i = 0; i < 100; ++i)
            output = controller.step(0, dt, stale);
        require(std::abs(output[1]) < 1e-5 && std::abs(output[2]) < 1e-5, "stale LOS did not stop");
        controller.reset();
        controller.update({true, 0, 0, .1, -.2}, stale);
        for (int i = 0; i < 100; ++i)
            output = controller.step(0, dt, stale);
        require(std::abs(output[1] - .1) < 1e-5 && std::abs(output[2] + .2) < 1e-5, "LOS rate feedforward");
        controller.update({}, stale);
        require(!controller.fresh(stale), "lost target accepted");
        controller.update({true, 0, 0, 0, 0, .25}, stale);
        require(controller.fresh(stale), "fresh delayed sample rejected");
        require(!controller.fresh(stale + std::chrono::milliseconds(60)), "transport delay extended LOS timeout");
        bool rejected = false;
        config.filter_tau = 0;
        try { apm_bridge::GimbalTrackingController invalid(config); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "invalid filter configuration accepted");
    }

    void sphericalRateMapping()
    {
        apm_bridge::GimbalTrackingConfig config;
        apm_bridge::GimbalTrackingController controller(config);
        const auto now = apm_bridge::GimbalTrackingController::Clock::now();
        const double pitch = .35, yaw = .4;
        controller.update({true, pitch, yaw, .1 - config.gain * pitch, .2 - config.gain * yaw}, now);
        std::array<double, 3> rate{};
        for (int i = 0; i < 200; ++i)
            rate = controller.step(.025, .02, now);
        const double pitch_derivative = rate[1] * std::cos(yaw) - rate[0] * std::sin(yaw);
        const double yaw_derivative = rate[2] + std::tan(pitch) *
            (rate[0] * std::cos(yaw) + rate[1] * std::sin(yaw));
        require(std::abs(pitch_derivative - .1) < 1e-8 &&
                std::abs(yaw_derivative - .2) < 1e-8,
                "spherical LOS derivatives were treated as camera body angular velocity");
        controller.reset();
        controller.update({true, .2, .55, 0, 0}, now);
        for (int i = 0; i < 200; ++i)
            rate = controller.step(0, .02, now, {false, true, false});
        require(rate[0] == 0 && rate[2] == 0 && std::abs(rate[1] * std::cos(.55) - .4) < 1e-8,
                "absent yaw motor incorrectly limited single-axis pitch tracking");
    }

    void angleTrajectories()
    {
        apm_bridge::GimbalTrackingConfig config;
        config.max_rate_dps = 140;
        config.max_accel_dps2 = 600;
        apm_bridge::GimbalAngleTrajectory trajectory(config);
        const std::array<bool, 3> axes{false, true, false};
        trajectory.reset({0, 0, 0});
        for (double target : {80.0, 0.0, -110.0, 110.0, 0.0})
        {
            for (int i = 0; i < 400; ++i)
            {
                const double dt = i % 2 ? .017 : .027;
                const auto before = trajectory.positions();
                const auto rate_before = trajectory.rates();
                const auto after = trajectory.step({40, target, 60}, dt, axes);
                require(after[0] == 0 && after[2] == 0, "angle trajectory moved an absent axis");
                require(std::abs(after[1] - before[1]) <= config.max_rate_dps * dt + 1e-10,
                        "angle setpoint teleported");
                require(std::abs(trajectory.rates()[1] - rate_before[1]) <= config.max_accel_dps2 * dt + 1e-10,
                        "angle trajectory acceleration discontinuity");
            }
            require(std::abs(trajectory.positions()[1] - target) < .001, "angle trajectory did not settle");
        }
        trajectory.reset({0, 10, 0}, {0, 140, 0});
        const auto moving = trajectory.rates()[1];
        trajectory.step({0, -80, 0}, .02, axes);
        require(std::abs(trajectory.rates()[1] - moving) <= 12.000001, "moving rate-to-angle transition reset velocity");
        trajectory.reset({0, 119.99, 0}, {0, 140, 0});
        trajectory.step({0, 120, 0}, .02, axes);
        trajectory.constrain(1, -120, 120);
        require(trajectory.positions()[1] <= 120 && trajectory.rates()[1] == 0, "angle trajectory ignored mechanical stop");
    }

    void noisyRateFeedforward()
    {
        apm_bridge::GimbalTrackingConfig config;
        config.rate_feedforward = 0;
        apm_bridge::GimbalTrackingController controller(config);
        const auto now = apm_bridge::GimbalTrackingController::Clock::now();
        controller.update({true, 0, 0, 10, -10}, now);
        auto output = controller.step(0, .02, now);
        require(output[1] == 0 && output[2] == 0, "disabled noisy rate feedforward moved a centered camera");
        controller.update({true, .1, -.1, -10, 10}, now);
        for (int i = 0; i < 200; ++i)
            output = controller.step(0, .02, now);
        require(output[1] > 0 && output[2] < 0, "disabling feedforward removed angle tracking");
        config.rate_feedforward = -1;
        bool rejected = false;
        try { apm_bridge::GimbalTrackingController invalid(config); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "negative rate feedforward accepted");
    }
}

int main()
{
    try
    {
        modes();
        rates();
        sphericalRateMapping();
        noisyRateFeedforward();
        angleTrajectories();
        std::cout << "PASS: gimbal states, zero-centered MISSION, feedforward, timeout, filtering and rate limits\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
