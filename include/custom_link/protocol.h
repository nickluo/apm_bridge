#pragma once

//
// Betaflight 自定义高频双向链路协议 — 主机端镜像。
//
// 与固件侧 telemetry/custom_link_protocol.h 严格对应：
//   0xEB 0x90 | MsgID u8 | Len u8 | Seq u8 | Payload(N<=64) | CRC16 lo hi
//   CRC-16/XMODEM (poly 0x1021, MSB-first, init 0)，覆盖 MsgID+Len+Seq+Payload，
//   小端追加。所有多字节字段小端（x86/ARM 主机天然小端）。
//

#include <array>
#include <cstdint>
#include <cstring>

namespace custom_link
{
    namespace protocol
    {
        constexpr uint8_t kSync1 = 0xEB;
        constexpr uint8_t kSync2 = 0x90;
        constexpr uint8_t kFrameOverhead = 7;                       // sync(2)+id+len+seq+crc(2)
        constexpr uint8_t kMaxPayload = 64;
        constexpr uint8_t kFrameMax = kFrameOverhead + kMaxPayload;

        enum MsgId : uint8_t
        {
            MSG_FC_FAST = 0x10,        // 200 Hz gyro/acc/attitude
            MSG_FC_MEDIUM = 0x11,      // 100 Hz baro/temp/rc
            MSG_FC_SLOW = 0x12,        // 10 Hz battery/gps/modes/status
            MSG_HOST_CONTROL = 0x20,   // host -> FC control setpoints
            MSG_HOST_TIMESYNC = 0x30,  // host -> FC time sync request (u64 T1)
            MSG_FC_TIMESYNC = 0x31,    // FC -> host time sync response
        };

#pragma pack(push, 1)
        struct PayloadFast
        {
            uint32_t ts_us;            // FC micros() 时间戳
            int16_t gyro[3];           // 0.1 deg/s (BF 体轴: x前 y右 z上)
            int16_t acc[3];            // 1 mg
            int16_t attitude[3];       // roll/pitch/yaw, 0.01 deg，固件原生欧拉角，
                                       // 即 FLU 右手系 (REP-103)：roll 左滚为正、
                                       // pitch 抬头为负；yaw 为右手数学角（绕 +Z
                                       // 逆时针为正，0=磁北，±180° 折返）
        };

        struct PayloadMedium
        {
            uint32_t ts_us;
            uint32_t baro_pa;          // Pa
            int32_t baro_alt_cm;       // cm
            int16_t temp_cdeg;     // 0.01 degC，融合温度：气压计 > IMU >
                                       // ISA 估计（海平面 20 degC，每升 100 m 降 0.6 degC）
            uint16_t rc[16];           // us
        };

        struct PayloadSlow
        {
            uint32_t ts_us;
            uint16_t vbat_mv;
            int32_t current_ma;
            int16_t mah;
            uint8_t fix;               // 0/1
            uint8_t sats;
            int32_t lat_e7;            // deg*1e7
            int32_t lon_e7;
            int32_t alt_msl_cm;
            uint16_t gspeed_cms;
            uint16_t course_cdeg;      // 0.01 deg
            uint32_t mode_flags;       // Betaflight flightModeFlags
            uint16_t status;           // bit0 armed, bit1 failsafe
        };

        struct PayloadControl
        {
            uint64_t host_ts_us;       // 主机单调时钟 us
            uint16_t cmd_seq;
            uint8_t arm;
            uint8_t mode_req;          // 0 manual, 1 angle, 2 offboard
            uint16_t throttle;         // 1000..2000
            int16_t rate_x10[3];       // 0.1 deg/s (BF 体轴)
        };

        struct PayloadTimesyncReq
        {
            uint64_t t1;
        };

        struct PayloadTimesyncResp
        {
            uint64_t t1;
            uint32_t t2_isr_us;        // FC RX ISR 时间戳
            uint32_t t3_tx_us;         // FC 发送前时间戳
        };
#pragma pack(pop)

        static_assert(sizeof(PayloadFast) == 22, "fast payload size");
        static_assert(sizeof(PayloadMedium) == 46, "medium payload size");
        static_assert(sizeof(PayloadSlow) == 36, "slow payload size");
        static_assert(sizeof(PayloadControl) == 20, "control payload size");
        static_assert(sizeof(PayloadTimesyncReq) == 8, "timesync req size");
        static_assert(sizeof(PayloadTimesyncResp) == 16, "timesync resp size");

        constexpr uint16_t STATUS_ARMED = 0x0001;
        constexpr uint16_t STATUS_FAILSAFE = 0x0002;

        // flightModeFlags 位（与固件 runtime_config.h 一致）
        namespace mode
        {
            constexpr uint32_t ANGLE = 1u << 0;
            constexpr uint32_t HORIZON = 1u << 1;
            constexpr uint32_t ALT_HOLD = 1u << 3;
            constexpr uint32_t POS_HOLD = 1u << 5;
            constexpr uint32_t HEADFREE = 1u << 6;
            constexpr uint32_t PASSTHRU = 1u << 8;
            constexpr uint32_t FAILSAFE = 1u << 10;
            constexpr uint32_t GPS_RESCUE = 1u << 11;
            constexpr uint32_t AUTOPILOT = 1u << 12;
            constexpr uint32_t OFFBOARD = 1u << 13;
        }

        /// CRC-16/XMODEM，可带种子（链式覆盖多段数据）。
        inline uint16_t crc16Seeded(uint16_t seed, const uint8_t *data, size_t len)
        {
            uint16_t crc = seed;
            for (size_t i = 0; i < len; ++i)
            {
                crc ^= static_cast<uint16_t>(data[i]) << 8;
                for (int b = 0; b < 8; ++b)
                {
                    crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                         : static_cast<uint16_t>(crc << 1);
                }
            }
            return crc;
        }

        struct Frame
        {
            uint8_t msgId = 0;
            uint8_t len = 0;
            uint8_t seq = 0;
            std::array<uint8_t, kMaxPayload> payload{};

            template <typename T>
            bool as(T &out) const
            {
                if (len != sizeof(T))
                    return false;
                std::memcpy(&out, payload.data(), sizeof(T));
                return true;
            }
        };

        inline uint16_t frameCrc(const Frame &frame)
        {
            // msgId/len/seq 在 Frame 头部连续
            const uint16_t head = crc16Seeded(0, &frame.msgId, 3);
            return crc16Seeded(head, frame.payload.data(), frame.len);
        }

        /// 序列化一帧到 out，返回帧长；参数非法返回 0。
        inline uint8_t encode(uint8_t msgId, uint8_t seq, const void *payload, uint8_t payloadLen,
                              uint8_t *out, uint8_t outSize)
        {
            if (payloadLen == 0 || payloadLen > kMaxPayload || payload == nullptr)
                return 0;
            const uint8_t frameSize = static_cast<uint8_t>(kFrameOverhead + payloadLen);
            if (out == nullptr || outSize < frameSize)
                return 0;

            Frame frame;
            frame.msgId = msgId;
            frame.len = payloadLen;
            frame.seq = seq;
            std::memcpy(frame.payload.data(), payload, payloadLen);
            const uint16_t crc = frameCrc(frame);

            out[0] = kSync1;
            out[1] = kSync2;
            out[2] = msgId;
            out[3] = payloadLen;
            out[4] = seq;
            std::memcpy(&out[5], payload, payloadLen);
            out[frameSize - 2] = static_cast<uint8_t>(crc & 0xFF);
            out[frameSize - 1] = static_cast<uint8_t>(crc >> 8);
            return frameSize;
        }

        /// 逐字节喂入的解析状态机，镜像固件侧行为（帧头重扫描/长度校验/
        /// 字节间超时/CRC 校验）。frameCompleteUs 为完成字节的到达时刻
        /// （主机单调时钟 us）。
        class Parser
        {
        public:
            using Handler = void (*)(const Frame &, uint64_t frameCompleteUs, void *ctx);

            explicit Parser(uint64_t interByteTimeoutUs = 100000)
                : interByteTimeoutUs_(interByteTimeoutUs) {}

            void processByte(uint8_t c, uint64_t nowUs, Handler handler, void *ctx)
            {
                if (state_ != SYNC1 &&
                    static_cast<int64_t>(nowUs - lastByteUs_) > static_cast<int64_t>(interByteTimeoutUs_))
                {
                    reset();
                }
                lastByteUs_ = nowUs;

                switch (state_)
                {
                case SYNC1:
                    if (c == kSync1)
                        state_ = SYNC2;
                    break;

                case SYNC2:
                    if (c == kSync2)
                    {
                        hdrPos_ = 0;
                        state_ = HDR;
                    }
                    else if (c != kSync1)
                    {
                        state_ = SYNC1;
                    }
                    break;

                case HDR:
                    (&frame_.msgId)[hdrPos_++] = c;
                    if (hdrPos_ >= 3)
                    {
                        if (frame_.len == 0 || frame_.len > kMaxPayload)
                        {
                            reset();
                        }
                        else
                        {
                            payloadPos_ = 0;
                            state_ = PAYLOAD;
                        }
                    }
                    break;

                case PAYLOAD:
                    frame_.payload[payloadPos_++] = c;
                    if (payloadPos_ >= frame_.len)
                    {
                        crc_ = frameCrc(frame_);
                        state_ = CRC_LO;
                    }
                    break;

                case CRC_LO:
                    if (c == static_cast<uint8_t>(crc_ & 0xFF))
                    {
                        state_ = CRC_HI;
                    }
                    else
                    {
                        reset();
                    }
                    break;

                case CRC_HI:
                {
                    const bool ok = (c == static_cast<uint8_t>(crc_ >> 8));
                    const Frame completed = frame_;
                    reset();
                    if (ok && handler)
                        handler(completed, nowUs, ctx);
                    break;
                }
                }
            }

        private:
            enum State { SYNC1, SYNC2, HDR, PAYLOAD, CRC_LO, CRC_HI };

            void reset()
            {
                state_ = SYNC1;
                hdrPos_ = 0;
                payloadPos_ = 0;
            }

            State state_ = SYNC1;
            uint8_t hdrPos_ = 0;
            uint8_t payloadPos_ = 0;
            uint64_t lastByteUs_ = 0;
            uint64_t interByteTimeoutUs_;
            uint16_t crc_ = 0;
            Frame frame_;
        };
    } // namespace protocol
} // namespace custom_link
