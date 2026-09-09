#include "custom_link/client.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace custom_link
{
    using namespace std::chrono_literals;

    uint64_t hostClockUs()
    {
        // 与 rclcpp 默认节点时钟同域（use_sim_time=false 时为墙钟），
        // 保证遥测时间戳可与 this->now() 直接比较。
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
    }

    namespace
    {
        uint64_t hostMonoMs()
        {
            return hostClockUs() / 1000;
        }

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
            sync_.valid = false;
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

            const uint64_t nowUs = hostClockUs();
            for (int i = 0; i < got; ++i)
            {
                parser_.processByte(buf[i], nowUs, &Client::parserTrampoline, this);
            }
        }
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

            // FC 时钟是开机 uptime (u32 us)，主机是墙钟 (u64 us)，两者零点无关，
            // 不能直接 T2-T1。把 FC 时间戳解回绕到连续 u64 时间线后估计
            // 偏置 B = host - fc（NTP 对称差）：B2 = T1-F2 高估上行延迟，
            // B4 = T4-F3 低估下行延迟，取中抵消不对称。
            uint64_t f2 = extend32(resp.t2_isr_us, fc_ref_us_);
            const uint64_t f3 = extend32(resp.t3_tx_us, f2);

            const double rtt_us = static_cast<double>(static_cast<int64_t>(completeUs - resp.t1)) -
                                  static_cast<double>(static_cast<int64_t>(f3 - f2));
            if (rtt_us < 0.0 || rtt_us > 200000.0)
                return; // 异常样本（丢帧补发/调度突发），丢弃

            const int64_t b = ((static_cast<int64_t>(resp.t1) - static_cast<int64_t>(f2)) +
                               (static_cast<int64_t>(completeUs) - static_cast<int64_t>(f3))) / 2;

            std::lock_guard<std::mutex> lk(sync_mtx_);
            if (!sync_.valid)
            {
                sync_.valid = true;
                fc_offset_us_ = b;
            }
            else
            {
                // 野值抑制：偏置突变超过数倍 RTT 多半是样本撞上调度突发
                const int64_t limit = static_cast<int64_t>(rtt_us) * 4 + 2000;
                if (b - fc_offset_us_ > limit || fc_offset_us_ - b > limit)
                {
                    fc_ref_us_ = f3;
                    return;
                }
                // RTT 明显劣化时降低权重
                const double a = (rtt_us / 1000.0 > 2.0 * sync_.rtt_ms + 2.0) ? 0.1 : 0.3;
                fc_offset_us_ += static_cast<int64_t>(a * static_cast<double>(b - fc_offset_us_));
            }
            sync_.rtt_ms = sync_.rtt_ms == 0.0 ? rtt_us / 1000.0
                                               : sync_.rtt_ms + 0.3 * (rtt_us / 1000.0 - sync_.rtt_ms);
            sync_.jitter_ms = rtt_us / 1000.0 - sync_.rtt_ms;
            sync_.last_ok_ms = hostMonoMs();
            sync_.offset_ms = static_cast<double>(fc_offset_us_) / 1000.0; // 绝对偏置 (墙钟-uptime)，数值巨大属正常
            fc_ref_us_ = f3;
            return;
        }

        // 遥测帧：ts_us 换算到主机时基后上抛；模型未就绪时退化为到达时刻
        int64_t hostUs = static_cast<int64_t>(completeUs);
        const uint32_t fcTs = extractTs(frame);
        {
            std::lock_guard<std::mutex> lk(sync_mtx_);
            if (sync_.valid)
                hostUs = static_cast<int64_t>(extend32(fcTs, fc_ref_us_)) + fc_offset_us_;
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
        p.host_ts_us = hostClockUs();
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
        p.t1 = hostClockUs();
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
            return static_cast<int64_t>(hostClockUs());
        return static_cast<int64_t>(extend32(fcUs, fc_ref_us_)) + fc_offset_us_;
    }

} // namespace custom_link
