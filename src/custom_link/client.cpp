#include "custom_link/client.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>

namespace custom_link
{
    using namespace std::chrono_literals;

    uint64_t hostClockUs()
    {
        // 与 rclcpp 默认节点时钟同域（use_sim_time=false 时为墙钟），
        // 保证遥测时间戳可与 this->now() 直接比较。
        return wallClockUs();
    }

    uint64_t wallClockUs()
    {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
    }

    uint64_t monoClockUs()
    {
        // 单调钟：对时模型的内部时基，免疫 NTP 对墙钟的阶跃/渐变调整。
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    namespace
    {
        /// 把 FC 的 u32 us 时间戳按参考点扩展成 u64（最近回绕原则）。
        uint64_t extend32(uint32_t fc32, uint64_t refUs)
        {
            return refUs + static_cast<int64_t>(static_cast<int32_t>(fc32 - static_cast<uint32_t>(refUs)));
        }

        /// 所有遥测 payload 均以 u32 ts_us 开头。
        uint32_t extractTs(const protocol::Frame &frame)
        {
            if (frame.len < 4)
                return 0;
            uint32_t v;
            std::memcpy(&v, frame.payload.data(), sizeof(v));
            return v;
        }

        // ---- 对时模型参数（实机标定见 test/timesync_drift_test.py） ----
        constexpr double kSkewClamp = 20000e-6;    // 漂移率钳位 ±2%（实测 FC 达 ±7000ppm）
        constexpr int64_t kFcJumpResetUs = 2000000;   // FC 时间线跳变阈值（重启/回退）
        constexpr int64_t kOutlierBandUs = 20000;     // 相对拟合线的野值带
        constexpr int kResetStreak = 3;               // 连续野值次数 → 重锁定
        constexpr double kFloorRiseUsPerSec = 200.0;  // RTT 下限老化速率
        constexpr int64_t kRttSanityUs = 200000;      // RTT 合理性上界
    } // namespace

    Client::Client(std::unique_ptr<Transport> transport)
        : transport_(std::move(transport))
    {
    }

    Client::~Client()
    {
        stop();
    }

    void Client::start()
    {
        if (running_.exchange(true))
            return;
        reader_thread_ = std::thread(&Client::readerLoop, this);
        sender_thread_ = std::thread(&Client::senderLoop, this);
    }

    void Client::stop()
    {
        running_ = false;
        if (reader_thread_.joinable())
            reader_thread_.join();
        if (sender_thread_.joinable())
            sender_thread_.join();
        transport_->close();
        up_ = false;
    }

    void Client::setControl(const ControlInput &input, bool valid)
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        cmd_ = input;
        cmd_valid_ = valid;
    }

    TimeSyncState Client::timeSync() const
    {
        std::lock_guard<std::mutex> lk(sync_mtx_);
        return sync_;
    }

    LinkStats Client::stats() const
    {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        return stats_;
    }

    void Client::requestTimeSync()
    {
        if (up_.load())
            sendTimesync();
    }

    bool Client::ensureOpen()
    {
        if (transport_->isOpen())
        {
            // 传输层可能在构造时已打开（WJ Wood 串口构造即 open），这条
            // 快速路径同样要把链路标记为 up 并通知回调。
            if (!up_.exchange(true) && link_cb_)
                link_cb_(true);
            return true;
        }
        up_ = false;
        if (!transport_->open())
            return false;
        {
            std::lock_guard<std::mutex> lk(stats_mtx_);
            ++stats_.reconnects;
        }
        // 重连后 FC 时钟参考点失效，强制重新对时
        {
            std::lock_guard<std::mutex> lk(sync_mtx_);
            resetSyncLocked();
        }
        up_ = true;
        if (link_cb_)
            link_cb_(true);
        return true;
    }

    void Client::readerLoop()
    {
        uint8_t buf[4096];
        while (running_.load())
        {
            if (!ensureOpen())
            {
                std::this_thread::sleep_for(500ms);
                continue;
            }

            const int got = transport_->read(buf, sizeof(buf), 50ms);
            if (got < 0)
            {
                transport_->close();
                if (up_.exchange(false) && link_cb_)
                    link_cb_(false);
                continue;
            }
            if (got == 0)
                continue;

            const uint64_t nowUs = monoClockUs();
            for (int i = 0; i < got; ++i)
            {
                parser_.processByte(buf[i], nowUs, &Client::parserTrampoline, this);
            }
        }
    }

    /// 清空对时模型（链路重连 / FC 时钟跳变 / 持续野值后重锁定）。持 sync_mtx_ 调用。
    void Client::resetSyncLocked()
    {
        ring_count_ = 0;
        ring_head_ = 0;
        fit_valid_ = false;
        rtt_floor_us_ = -1;
        last_sample_us_ = 0;
        prev_f3_us_ = 0;
        outlier_streak_ = 0;
        last_ext_ts_us_ = 0;
        sync_.valid = false;
        sync_.skew_ppm = 0.0;
    }

    /// 环内样本的最小二乘拟合 b(t) = b0 + c1*(t - t0)，持 sync_mtx_ 调用。
    void Client::refitLocked()
    {
        fit_valid_ = false;
        if (ring_count_ < 3)
            return; // 样本不足：offsetAtLocked 退化为取最新样本
        // 样本占据槽位 (head-count) .. (head-1)，head 为下一个写入槽
        const int base = (ring_head_ - ring_count_ + kSyncRing) % kSyncRing;
        double mt = 0.0, mb = 0.0;
        for (int i = 0; i < ring_count_; ++i)
        {
            const auto &s = ring_[(base + i) % kSyncRing];
            mt += static_cast<double>(s.tUs);
            mb += static_cast<double>(s.bUs);
        }
        mt /= ring_count_;
        mb /= ring_count_;
        double den = 0.0, num = 0.0;
        for (int i = 0; i < ring_count_; ++i)
        {
            const auto &s = ring_[(base + i) % kSyncRing];
            const double dt = static_cast<double>(s.tUs) - mt;
            den += dt * dt;
            num += dt * (static_cast<double>(s.bUs) - mb);
        }
        if (den <= 0.0)
            return;
        double c1 = num / den;
        c1 = std::clamp(c1, -kSkewClamp, kSkewClamp);
        fit_t0_ = mt;
        fit_b0_ = mb;
        fit_c1_ = c1;
        fit_valid_ = true;
    }

    /// 单调钟 t 时刻的偏置 B = mono - fc（µs），持 sync_mtx_ 调用。
    int64_t Client::offsetAtLocked(uint64_t tUs) const
    {
        if (fit_valid_)
        {
            const double dt = static_cast<double>(tUs) - fit_t0_;
            return fit_b0_ + static_cast<int64_t>(fit_c1_ * dt);
        }
        if (ring_count_ > 0)
        {
            // 收敛初期：退化为最新样本（无斜率项，误差 ≤ 漂移率×样本间隔）
            const SyncSample &s = ring_[(ring_head_ + kSyncRing - 1) % kSyncRing];
            return s.bUs;
        }
        return 0;
    }

    void Client::handleFrame(const protocol::Frame &frame, uint64_t completeUs)
    {
        {
            std::lock_guard<std::mutex> lk(stats_mtx_);
            ++stats_.rx_frames;
            ++stats_.rx_per_msg[frame.msgId & 7u]; // 0x10/0x11/0x12 -> 0/1/2
        }

        if (frame.msgId == protocol::MSG_FC_TIMESYNC)
        {
            protocol::PayloadTimesyncResp resp;
            if (!frame.as(resp) || resp.t1 != pending_t1_.load())
                return; // 过期或重复的应答

            // FC 时钟是开机 uptime (u32 us)，与主机零点无关，不能直接 T2-T1。
            // 把 FC 时间戳解回绕到连续 u64 时间线后估计偏置 B = mono - fc
            //（NTP 对称差：B2 = T1-F2 高估上行延迟，B4 = T4-F3 低估下行延迟，
            // 取中抵消不对称）。内部全部走单调钟域，免疫墙钟 NTP 调整。
            const uint64_t f2 = extend32(resp.t2_isr_us, fc_ref_us_);
            const uint64_t f3 = extend32(resp.t3_tx_us, f2);

            const int64_t rtt = (static_cast<int64_t>(completeUs - resp.t1)) -
                                static_cast<int64_t>(f3 - f2);
            if (rtt < 0 || rtt > kRttSanityUs)
                return; // 异常样本（丢帧补发/调度突发），丢弃

            const int64_t b = ((static_cast<int64_t>(resp.t1) - static_cast<int64_t>(f2)) +
                               (static_cast<int64_t>(completeUs) - static_cast<int64_t>(f3))) / 2;

            std::lock_guard<std::mutex> lk(sync_mtx_);

            // FC 时间线大幅跳变 = 飞控重启（uptime 归零）：整体重置后重新锚定
            if (prev_f3_us_ != 0 &&
                (f2 + kFcJumpResetUs < prev_f3_us_ || f2 > prev_f3_us_ + kFcJumpResetUs))
                resetSyncLocked();
            prev_f3_us_ = f3;
            fc_ref_us_ = f3;

            // RTT 下限：立即向下跟随，向上以固定速率老化（链路劣化后可抬升）
            if (rtt_floor_us_ < 0 || last_sample_us_ == 0)
                rtt_floor_us_ = rtt;
            else
            {
                const double rise = kFloorRiseUsPerSec *
                                    static_cast<double>(completeUs - last_sample_us_) / 1e6;
                rtt_floor_us_ = std::min<int64_t>(static_cast<int64_t>(rtt_floor_us_ + rise), rtt);
            }
            last_sample_us_ = completeUs;

            sync_.rtt_ms = sync_.rtt_ms == 0.0 ? static_cast<double>(rtt_floor_us_) / 1000.0
                                               : sync_.rtt_ms + 0.3 * (static_cast<double>(rtt_floor_us_) / 1000.0 - sync_.rtt_ms);
            sync_.jitter_ms = 0.7 * sync_.jitter_ms +
                              0.3 * static_cast<double>(rtt - rtt_floor_us_) / 1000.0;

            // 质量门限：延迟显著劣化的样本偏置易被排队不对称污染，不入环
            const int64_t qgate = rtt_floor_us_ + std::max<int64_t>(2000, 2 * rtt_floor_us_);
            if (rtt <= qgate)
            {
                if (fit_valid_ && std::llabs(b - offsetAtLocked(completeUs)) > kOutlierBandUs)
                {
                    // 持续偏离拟合线 → 时钟真变了，重锁定；单次偏离 → 跳过
                    if (++outlier_streak_ >= kResetStreak)
                    {
                        resetSyncLocked();
                        prev_f3_us_ = f3;
                        fc_ref_us_ = f3;
                    }
                }
                else
                {
                    outlier_streak_ = 0;
                    ring_[ring_head_] = {completeUs, b};
                    ring_head_ = (ring_head_ + 1) % kSyncRing;
                    ring_count_ = std::min(ring_count_ + 1, kSyncRing);
                    refitLocked();
                }
            }

            sync_.valid = ring_count_ > 0;
            if (sync_.valid)
            {
                sync_.last_ok_ms = monoClockUs() / 1000;
                sync_.skew_ppm = fit_valid_ ? fit_c1_ * 1e6 : 0.0;
                // 输出域偏置 = (mono-fc) + (wall-mono)，绝对数值巨大属正常
                const int64_t wm = static_cast<int64_t>(wallClockUs()) -
                                   static_cast<int64_t>(monoClockUs());
                sync_.offset_ms = static_cast<double>(offsetAtLocked(completeUs) + wm) / 1000.0;
            }
            return;
        }

        // 遥测帧：ts_us 换算到主机墙钟后上抛；模型未就绪时退化为到达时刻
        int64_t hostUs = static_cast<int64_t>(wallClockUs());
        const uint32_t fcTs = extractTs(frame);
        {
            std::lock_guard<std::mutex> lk(sync_mtx_);
            if (sync_.valid)
            {
                const uint64_t ext = extend32(fcTs, fc_ref_us_);
                if (last_ext_ts_us_ != 0 && ext + 1000000 < last_ext_ts_us_)
                {
                    // 遥测时间线大幅回退 = FC 已重启（尚无对时样本发现）：
                    // 立即作废模型，退回到达时刻，等下一次对时重新锚定
                    resetSyncLocked();
                }
                else
                {
                    last_ext_ts_us_ = ext;
                    // fc→mono 用模型外推，mono→wall 用即时读数映射
                    //（墙钟步进会被输出侧如实跟随，而不污染模型）
                    const int64_t monoUs = static_cast<int64_t>(ext) + offsetAtLocked(completeUs);
                    hostUs = monoUs + static_cast<int64_t>(wallClockUs()) -
                             static_cast<int64_t>(completeUs);
                }
            }
        }
        if (telemetry_cb_)
            telemetry_cb_(frame, fcTs, hostUs);
    }

    void Client::senderLoop()
    {
        auto nextControl = std::chrono::steady_clock::now();
        auto nextSync = std::chrono::steady_clock::now();

        while (running_.load())
        {
            const auto now = std::chrono::steady_clock::now();

            if (now >= nextSync)
            {
                nextSync += sync_period_ms_;
                if (up_.load())
                    sendTimesync();
            }

            if (now >= nextControl)
            {
                if (now >= nextControl + control_period_ms_)
                    nextControl = now + control_period_ms_; // 补偿睡眠超时
                else
                    nextControl += control_period_ms_;

                if (up_.load())
                {
                    ControlInput input;
                    bool valid = false;
                    {
                        std::lock_guard<std::mutex> lk(cmd_mtx_);
                        input = cmd_;
                        valid = cmd_valid_;
                    }
                    const bool arm = arm_request_.load();

                    // 发帧条件：控制量新鲜，或仍有解锁请求。arm 为电平语义：
                    // FC 侧在 arm=1 且未解锁时自行限率重试 tryArm()，主机不
                    // 做边沿脉冲（脉冲会在解锁反馈延迟时误触 DISARM 边沿）。
                    // 全部失效即停发，FC 侧看门狗超时后退回 RC。
                    if (valid || arm)
                    {
                        sendControlFrame(arm, input);
                    }
                }
            }

            std::this_thread::sleep_until(std::min(nextControl, nextSync));
        }
    }

    void Client::sendControlFrame(bool arm, const ControlInput &input)
    {
        protocol::PayloadControl p{};
        p.host_ts_us = monoClockUs(); // 协议定义为主机单调时钟
        p.cmd_seq = cmd_seq_++;
        p.arm = arm ? 1 : 0;
        p.mode_req = input.mode_req;
        p.throttle = static_cast<uint16_t>(1000.0 + input.throttle * 1000.0 + 0.5);
        p.throttle = std::min<uint16_t>(std::max<uint16_t>(p.throttle, 1000), 2000);
        for (int i = 0; i < 3; ++i)
        {
            const double v = input.rate_dps[i] * 10.0;
            p.rate_x10[i] = static_cast<int16_t>(std::clamp(v, -32768.0, 32767.0));
        }

        uint8_t buf[protocol::kFrameMax];
        const uint8_t len = protocol::encode(protocol::MSG_HOST_CONTROL, 0, &p, sizeof(p), buf, sizeof(buf));
        if (len == 0)
            return;
        if (!transport_->write(buf, len))
        {
            transport_->close();
            up_ = false;
            if (link_cb_)
                link_cb_(false);
            return;
        }
        std::lock_guard<std::mutex> lk(stats_mtx_);
        ++stats_.tx_frames;
    }

    void Client::sendTimesync()
    {
        protocol::PayloadTimesyncReq p;
        p.t1 = monoClockUs();
        pending_t1_.store(p.t1);

        uint8_t buf[protocol::kFrameMax];
        const uint8_t len = protocol::encode(protocol::MSG_HOST_TIMESYNC, 0, &p, sizeof(p), buf, sizeof(buf));
        if (len == 0)
            return;
        if (!transport_->write(buf, len))
        {
            transport_->close();
            up_ = false;
        }
    }

    int64_t Client::fcToHostUs(uint32_t fcUs)
    {
        std::lock_guard<std::mutex> lk(sync_mtx_);
        if (!sync_.valid)
            return static_cast<int64_t>(wallClockUs());
        const int64_t monoUs = static_cast<int64_t>(extend32(fcUs, fc_ref_us_)) +
                               offsetAtLocked(monoClockUs());
        return monoUs + static_cast<int64_t>(wallClockUs()) -
               static_cast<int64_t>(monoClockUs());
    }

} // namespace custom_link
