#pragma once

//
// 自定义链路主机端客户端：传输无关 (串口/TCP)、读取线程 + 发送线程、
// 四时间戳软同步（内部单调钟域，偏置+漂移率联合模型，输出映射回墙钟）。
// 遥测回调在读取线程上下文调用，注意不要阻塞。
//

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "custom_link/protocol.h"
#include "custom_link/transport.h"

namespace custom_link
{
    struct TimeSyncState
    {
        bool valid = false;          // 已有可用时差模型
        double rtt_ms = 0.0;         // 平滑后的最小往返延迟（延迟最优估计）
        double offset_ms = 0.0;      // 偏置 B = wall - fc（墙钟减 FC uptime，
                                      // 绝对数值巨大属正常；看稳定性看 rtt/jitter/skew）
        double jitter_ms = 0.0;      // RTT 相对下限的超额量（平滑，>=0）
        double skew_ppm = 0.0;       // FC 相对主机的时钟漂移率（实测可达 ±7000 ppm）
        uint64_t last_ok_ms = 0;     // 最近一次成功对时 (主机单调钟 ms)
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

    /// 主机墙钟 (system_clock, us)。与 ROS 节点时钟同域，仅用于输出侧。
    uint64_t hostClockUs();
    /// 主机墙钟 (system_clock, us)。
    uint64_t wallClockUs();
    /// 主机单调钟 (steady_clock, us)。对时模型的内部时基，免疫 NTP 调整。
    uint64_t monoClockUs();

    inline std::chrono::nanoseconds periodFromRate(double rate_hz)
    {
        if (!std::isfinite(rate_hz) || rate_hz <= 0.0)
            throw std::invalid_argument("Link rate must be finite and positive");
        const long double period_ns = 1.0e9L / rate_hz;
        if (period_ns < 1.0L ||
            period_ns >= static_cast<long double>(std::chrono::nanoseconds::max().count()))
            throw std::invalid_argument("Link rate period must be representable in nanoseconds");
        return std::chrono::nanoseconds(static_cast<std::chrono::nanoseconds::rep>(period_ns));
    }

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

        /// 显式模式请求：发送一次 0x20（即使无控制量），随后控制帧沿用该模式。
        /// 不改变 arm 请求，也不替代持续控制流/FC 看门狗。
        /// 重复 setOffboardRequest(true) 会自动产生一次 mode_req 0->2 新边沿
        ///（先发一帧 0 再回 2），等价于重新请求——用于解锁 FC 侧 apiBlocked。
        void setOffboardRequest(bool offboard);

        /// 主动触发一次对时（另有周期性自动对时）。
        void requestTimeSync();

        /// ---- CLI 设置读写 (0x24 上行 / 0x15 下行) ----
        /// 阻塞直至响应分块收齐或超时；分块在读取线程重组。FC 侧在途响应
        /// 仅一条（新请求顶替旧的），并发调用由内部互斥串行化。
        struct CliResult
        {
            bool ok = false;          // 收到完整响应（含 FC 报错，看 status）
            uint8_t status = 0;       // 协议 CLI_STATUS_*
            std::string text;         // 重组后的响应文本
            std::string error;        // ok=false 时的本地原因
        };
        CliResult cliCommand(uint8_t cmd, const std::string &args, double timeout_s);

        /// 把 FC 时间戳 (us, u32 回绕) 换算到主机墙钟 (us)。
        /// 时差模型无效时返回 hostClockUs()。
        int64_t fcToHostUs(uint32_t fcUs);

        TimeSyncState timeSync() const;
        LinkStats stats() const;

        void setRates(double control_rate_hz, double sync_rate_hz)
        {
            const auto control_period = periodFromRate(control_rate_hz);
            const auto sync_period = periodFromRate(sync_rate_hz);
            control_period_ = control_period;
            sync_period_ = sync_period;
        }

    private:
        void readerLoop();
        void senderLoop();
        void handleFrame(const protocol::Frame &frame, uint64_t completeUs);
        void handleCliFrame(const protocol::Frame &frame);
        void failCliPendingLocked(const char *reason);
        bool ensureOpen();
        bool sendControlFrame(bool arm, const ControlInput &input);
        void sendTimesync();

        // ---- 对时模型（均持 sync_mtx_ 调用） ----
        // 偏置 B = mono - fc 并非常数：两侧时钟速率不同（实测 FC 快 ~6800 ppm），
        // B 以固定斜率漂移。故模型 = 质量过滤后的样本环 + 最小二乘拟合
        // B(t) = b0 + c1*(t-t0)，查表时外推，天然跟踪任意漂移率。
        struct SyncSample
        {
            uint64_t tUs;  // T4 时刻（单调钟）
            int64_t bUs;   // NTP 对称差偏置
        };
        void resetSyncLocked();                        // 清空模型（重连/FC重启/重锁定）
        void refitLocked();                            // 环内样本 LSQ 拟合
        int64_t offsetAtLocked(uint64_t tUs) const;    // t 时刻偏置（拟合/退化模式）

        static constexpr int kSyncRing = 16;
        SyncSample ring_[kSyncRing]{};
        int ring_count_ = 0;
        int ring_head_ = 0;
        bool fit_valid_ = false;
        double fit_t0_ = 0.0;       // 拟合中心点（数值稳定性）
        double fit_b0_ = 0.0;
        double fit_c1_ = 0.0;       // 漂移率（钳位 ±2%）
        int64_t rtt_floor_us_ = -1; // RTT 下限（向下立即、向上按速率老化）
        uint64_t last_sample_us_ = 0;
        uint64_t prev_f3_us_ = 0;   // 上一响应的 FC 侧参考（跳变检测）
        int outlier_streak_ = 0;    // 连续野值计数（→重锁定）
        uint64_t last_ext_ts_us_ = 0; // 遥测时间线回退检测（FC 重启）

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
        std::optional<uint8_t> mode_request_;
        bool rearm_pending_ = false;      // 持 cmd_mtx_：重复 true 的 0->2 边沿在途
        uint64_t mode_request_revision_ = 0;
        uint64_t mode_request_sent_revision_ = 0;
        std::atomic_bool arm_request_{false};
        std::atomic_bool armed_feedback_{false};

        // 发送侧序列
        uint16_t cmd_seq_ = 0;
        std::chrono::nanoseconds control_period_{std::chrono::milliseconds(10)};
        std::chrono::nanoseconds sync_period_{std::chrono::milliseconds(1000)};

        // 所有 transport 写（发送线程 0x20/0x30 与调用线程 0x24）串行化
        std::mutex write_mtx_;

        // CLI 请求/响应：cli_mtx_ 串行化并发调用；cli_state_mtx_ + cv 在
        // 调用线程（等待）与读取线程（收块）之间交接单条在途响应
        std::mutex cli_mtx_;
        std::mutex cli_state_mtx_;
        std::condition_variable cli_cv_;
        uint16_t cli_req_id_ = 0;       // 持 cli_mtx_ 递增，0 永不分配
        struct
        {
            uint16_t reqId = 0;         // 0 = 无在途请求（迟到的旧块被忽略）
            std::string text;
            uint16_t total = 0;
            uint8_t status = 0;
            bool done = false;
            std::string error;
        } cli_pending_;                 // 持 cli_state_mtx_

        // 时间同步（读取线程写，任意线程读；互斥保护）
        mutable std::mutex sync_mtx_;
        TimeSyncState sync_;
        uint64_t fc_ref_us_ = 0;      // 最近一次对时的 FC 侧 u64 参考点
        std::atomic<uint64_t> pending_t1_{0}; // 在途请求的 T1

        // 统计
        mutable std::mutex stats_mtx_;
        LinkStats stats_;
    };
} // namespace custom_link
