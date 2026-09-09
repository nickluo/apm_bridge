#pragma once

//
// 自定义链路主机端客户端：传输无关 (串口/TCP)、读取线程 + 发送线程、
// 四时间戳软同步。遥测回调在读取线程上下文调用，注意不要阻塞。
//

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "custom_link/protocol.h"
#include "custom_link/transport.h"

namespace custom_link
{
    struct TimeSyncState
    {
        bool valid = false;          // 已有可用时差模型
        double rtt_ms = 0.0;         // 平滑后的往返延迟
        double offset_ms = 0.0;      // 平滑后的偏置 B = host - fc（墙钟减 FC uptime，
                                      // 绝对数值巨大属正常；看稳定性看 rtt/jitter）
        double jitter_ms = 0.0;      // 最近一次 RTT 与平滑值之差
        uint64_t last_ok_ms = 0;     // 最近一次成功对时 (主机时钟 ms)
    };

    struct ControlInput
    {
        double throttle = 0.0;       // 0..1
        double rate_dps[3] = {0, 0, 0}; // BF 体轴 deg/s
        uint8_t mode_req = 2;        // offboard
    };

    struct LinkStats
    {
        uint64_t rx_frames = 0;
        uint64_t tx_frames = 0;
        uint32_t rx_per_msg[8] = {0}; // msgId 低 3 位索引
        uint64_t reconnects = 0;
    };

    /// 主机墙钟 (system_clock, us)。所有时间戳/T1/T4 与 ROS 节点时钟同域。
    uint64_t hostClockUs();

    class Client
    {
    public:
        /// frame: 解析出的遥测帧；fcUs: 帧 ts_us 原值；hostUs: 换算到主机时基的采样时刻。
        using TelemetryCb = std::function<void(const protocol::Frame &, uint32_t fcUs, int64_t hostUs)>;
        using LinkStateCb = std::function<void(bool up)>;

        Client(std::unique_ptr<Transport> transport);
        ~Client();

        Client(const Client &) = delete;
        Client &operator=(const Client &) = delete;

        void setTelemetryCallback(TelemetryCb cb) { telemetry_cb_ = std::move(cb); }
        void setLinkStateCallback(LinkStateCb cb) { link_cb_ = std::move(cb); }

        void start();
        void stop();
        bool isUp() const { return up_.load(); }

        /// 控制量（ROS 回调线程写，发送线程读）。valid=false 后停止发送
        /// 0x20，FC 看门狗超时自动退回 RC。
        void setControl(const ControlInput &input, bool valid);
        void clearControl() { setControl({}, false); }

        /// 独立于控制流的解锁请求：置位期间发送线程持续携带 arm=1 的帧
        /// （控制量无效时发中性值），已解锁且每秒未成功则重发 0->1 边沿。
        void setArmRequest(bool arm) { arm_request_ = arm; }
        void setArmedFeedback(bool armed) { armed_feedback_ = armed; }

        /// 主动触发一次对时（另有周期性自动对时）。
        void requestTimeSync();

        /// 把 FC 时间戳 (us, u32 回绕) 换算到主机墙钟 (us)。
        /// 时差模型无效时返回 hostClockUs()。
        int64_t fcToHostUs(uint32_t fcUs);

        TimeSyncState timeSync() const;
        LinkStats stats() const;

        void setRates(double control_rate_hz, double sync_rate_hz)
        {
            control_period_ms_ = std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_hz));
            sync_period_ms_ = std::chrono::milliseconds(static_cast<int>(1000.0 / sync_rate_hz));
        }

    private:
        void readerLoop();
        void senderLoop();
        void handleFrame(const protocol::Frame &frame, uint64_t completeUs);
        bool ensureOpen();
        void sendControlFrame(bool arm, const ControlInput &input);
        void sendTimesync();

        static void parserTrampoline(const protocol::Frame &frame, uint64_t completeUs, void *ctx)
        {
            static_cast<Client *>(ctx)->handleFrame(frame, completeUs);
        }

        std::unique_ptr<Transport> transport_;
        protocol::Parser parser_{20000};

        TelemetryCb telemetry_cb_;
        LinkStateCb link_cb_;

        std::thread reader_thread_;
        std::thread sender_thread_;
        std::atomic_bool running_{false};
        std::atomic_bool up_{false};

        // 控制量（ROS 回调线程写，发送线程读）
        std::mutex cmd_mtx_;
        ControlInput cmd_;
        bool cmd_valid_ = false;
        std::atomic_bool arm_request_{false};
        std::atomic_bool armed_feedback_{false};

        // 发送侧序列
        uint16_t cmd_seq_ = 0;
        std::chrono::milliseconds control_period_ms_{10};
        std::chrono::milliseconds sync_period_ms_{1000};

        // 时间同步（读取线程写，任意线程读；互斥保护）
        mutable std::mutex sync_mtx_;
        TimeSyncState sync_;
        int64_t fc_offset_us_ = 0;    // FC = host + offset
        uint64_t fc_ref_us_ = 0;      // 最近一次对时的 FC 侧 u64 参考点
        std::atomic<uint64_t> pending_t1_{0}; // 在途请求的 T1

        // 统计
        mutable std::mutex stats_mtx_;
        LinkStats stats_;
    };
} // namespace custom_link
