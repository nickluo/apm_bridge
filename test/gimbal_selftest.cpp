// 云台协议自检 harness: 在串口/pty 上运行 GimbalControl 驱动, 按时间表调用各类
// API, 供 test/gimbal_pysim_test.py 模拟云台固件做协议级验证。
//
// 用法: gimbal_selftest <port> <model C20S|C40D|C200T> <duration_sec> [roll_hold_enter] [roll_hold_exit]
// 状态回调输出到 stderr ("[status] ..."), 供仿真测试断言接收路径。
// roll_hold_enter/exit 可选覆盖 C-20S 大滚转阈值 (实机调试用)。

#include "gimbal.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace xfrobot;

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        fprintf(stderr, "usage: %s <port> <model C20S|C40D|C200T> <duration_sec>\n", argv[0]);
        return 2;
    }
    const std::string port = argv[1];
    const std::string model_name = argv[2];
    const float duration = static_cast<float>(std::atof(argv[3]));

    GimbalModel model;
    try
    {
        model = gimbalModelFromString(model_name);
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "%s\n", e.what());
        return 2;
    }

    GimbalConfig cfg = GimbalConfig::defaultsFor(model);
    // GIMBAL_HW=1: 实机模式 —— 不发送伪造载机数据 (协议警告错误数据劣化姿态
    // 解算)、不执行事件表 (陀螺校准/命令5/6), 只运行控制律
    const bool hw_mode = std::getenv("GIMBAL_HW") != nullptr;
    cfg.send_uav_data = !hw_mode;
    if (model == GimbalModel::C20S)
        cfg.pitch_hold_deg = -5.0f; // 非零期望角: 编码验证 + 实机上角度保持可见
    if (argc >= 6) // 可选: 覆盖 C-20S 大滚转阈值
    {
        cfg.roll_hold_enter_deg = static_cast<float>(std::atof(argv[4]));
        cfg.roll_hold_exit_deg = static_cast<float>(std::atof(argv[5]));
        fprintf(stderr, "roll_hold override: enter=%.1f exit=%.1f\n",
                cfg.roll_hold_enter_deg, cfg.roll_hold_exit_deg);
    }

    GimbalControl gimbal(port, cfg);
    gimbal.setStatusCallback([](const GimbalStatus &s) {
        fprintf(stderr, "[status] fw=%u hw_err=0x%02X stat=%u cmd=%u(%u) tca=%d inv=%d\n",
                s.fw_ver, s.hw_err, s.gbc_stat, s.cmd_value, s.cmd_stat,
                s.tca_ready ? 1 : 0, s.inv_flag ? 1 : 0);
    });
    gimbal.run();
    if (!gimbal.is_running())
    {
        fprintf(stderr, "failed to open %s\n", port.c_str());
        return 1;
    }
    fprintf(stderr, "selftest started: model=%s duration=%.1fs\n", model_name.c_str(), duration);

    const auto t0 = std::chrono::steady_clock::now();
    const auto elapsed = [&] {
        return std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
    };

    // 事件表 (秒), 与 test/gimbal_pysim_test.py 的时间轴对应
    enum Event
    {
        EV_GO_ZERO,      // 3.0s: 全轴回中 (缺失轴由驱动自动掩掉)
        EV_GYRO_CALIB,   // 4.0s: 陀螺仪校准 (一次性命令)
        EV_POINT_SHIFT,  // 5.0s: 指点平移 (一次性命令 + target_angle)
        EV_TRACK_ON,     // 6.0s: 云台内置目标跟踪开
        EV_UAV_PAUSE,    // 7.0s: 停止载机数据喂入 (测过期门控)
        EV_UAV_RESUME,   // 7.5s: 恢复载机数据
        EV_TRACK_OFF,    // 8.0s: 目标跟踪关, 回手动控制
        EV_COUNT
    };
    const float event_time[EV_COUNT] = {3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 7.5f, 8.0f};
    bool fired[EV_COUNT] = {false};
    bool uav_paused = false;

    while (elapsed() < duration)
    {
        const float t = elapsed();
        for (int i = 0; !hw_mode && i < EV_COUNT; i++)
        {
            if (!fired[i] && t >= event_time[i])
            {
                fired[i] = true;
                switch (i)
                {
                case EV_GO_ZERO:     gimbal.requestGoZero(0x07); break;
                case EV_GYRO_CALIB:  gimbal.sendCommand(1); break;
                case EV_POINT_SHIFT: gimbal.sendPointShift(2.5f, -1.5f); break;
                case EV_TRACK_ON:    gimbal.setTargetTracking(true, 3.0f, 2.0f); break;
                case EV_UAV_PAUSE:   uav_paused = true; break;
                case EV_UAV_RESUME:  uav_paused = false; break;
                case EV_TRACK_OFF:   gimbal.setTargetTracking(false, 0.0f, 0.0f); break;
                }
            }
        }

        // 载机数据 10Hz 喂入 (驱动侧过期阈值 200ms); 2.0s 后才开始,
        // 让仿真测试先观察到 valid=0 的无数据阶段。实机模式不喂 (仅仿真)。
        if (!hw_mode && t >= 2.0f && !uav_paused && static_cast<int>(t * 10) != static_cast<int>((t - 0.005f) * 10))
        {
            const int16_t ang[3] = {100, -200, 300}; // 1.00 / -2.00 / 3.00 deg
            const int16_t acc[3] = {10, -20, 980};   // 0.10 / -0.20 / 9.80 m/s2
            gimbal.setUavData(true, ang, acc);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    fprintf(stderr, "selftest done\n");
    return 0;
}
