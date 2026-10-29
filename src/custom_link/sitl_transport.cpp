#include "custom_link/sitl_transport.h"
#include "custom_link/client.h"

#include <array>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace custom_link
{
    SitlTransport::SitlTransport(std::string fc_host, uint16_t fc_port, std::string bind_host,
                                 uint16_t bind_port, rclcpp::Logger logger)
        : fc_(std::move(fc_host), fc_port), bind_host_(std::move(bind_host)),
          bind_port_(bind_port), logger_(std::move(logger)) {}

    SitlTransport::~SitlTransport() { close(); }

    std::string SitlTransport::description() const
    {
        return "SITL relay " + bind_host_ + ":" + std::to_string(bind_port_) + " -> " + fc_.description();
    }

    bool SitlTransport::open()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closeLocked();
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo *address = nullptr;
        const int error = ::getaddrinfo(bind_host_.c_str(), std::to_string(bind_port_).c_str(), &hints, &address);
        if (error != 0)
        {
            RCLCPP_ERROR(logger_, "SITL relay address: %s", gai_strerror(error));
            return false;
        }
        listener_ = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        const int one = 1;
        if (listener_ >= 0)
            ::setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        const bool listening = listener_ >= 0 &&
            ::bind(listener_, address->ai_addr, address->ai_addrlen) == 0 && ::listen(listener_, 1) == 0;
        ::freeaddrinfo(address);
        if (!listening || ::fcntl(listener_, F_SETFL, O_NONBLOCK) < 0)
        {
            RCLCPP_ERROR(logger_, "Cannot listen on SITL relay %s:%u: %s",
                         bind_host_.c_str(), bind_port_, std::strerror(errno));
            closeLocked();
            return false;
        }
        if (!fc_.open())
        {
            RCLCPP_WARN(logger_, "SITL relay waiting for FC at %s", fc_.description().c_str());
            closeLocked();
            return false;
        }
        RCLCPP_INFO(logger_, "%s; point AirSim TcpPort to %u", description().c_str(), bind_port_);
        return true;
    }

    void SitlTransport::closeLocked()
    {
        fc_.close();
        if (peer_ >= 0)
            ::close(peer_);
        if (listener_ >= 0)
            ::close(listener_);
        peer_ = listener_ = -1;
        parser_ = protocol::Parser{};
        forwarding_ok_ = true;
    }

    void SitlTransport::close()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closeLocked();
    }

    bool SitlTransport::isOpen() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return fc_.isOpen();
    }

    void SitlTransport::forwardFrame(const protocol::Frame &frame, uint64_t, void *context)
    {
        auto &self = *static_cast<SitlTransport *>(context);
        // AirSim is only the physics provider. ROS remains the sole 0x20 control owner.
        if (frame.msgId != 0x21 && frame.msgId != 0x22 && frame.msgId != 0x23)
        {
            RCLCPP_WARN_ONCE(self.logger_, "Ignored AirSim host frame 0x%02x; disable AirSim API control", frame.msgId);
            return;
        }
        // Relay observability: forwarded-frame counters per type plus FC write
        // failures, surfaced at 1 Hz. Localizes a stalled feeder (the plugin
        // stops sending) vs an FC-side RX loss (counters rise, FC sees none).
        switch (frame.msgId)
        {
        case 0x21: self.relayImu_++; break;
        case 0x22: self.relayEnv_++; break;
        default:   self.relayRc_++; break;
        }
        const uint64_t diagNow = monoClockUs();
        if (diagNow - self.lastRelayDiagUs_ >= 1000000)
        {
            self.lastRelayDiagUs_ = diagNow;
            RCLCPP_INFO(self.logger_, "AirSim relay: imu=%llu env=%llu rc=%llu fcWriteFail=%llu",
                        static_cast<unsigned long long>(self.relayImu_),
                        static_cast<unsigned long long>(self.relayEnv_),
                        static_cast<unsigned long long>(self.relayRc_),
                        static_cast<unsigned long long>(self.relayWriteFails_));
        }
        std::array<uint8_t, protocol::kFrameMax> bytes{};
        const auto size = protocol::encode(frame.msgId, frame.seq, frame.payload.data(),
                                           frame.len, bytes.data(), bytes.size());
        const bool ok = self.fc_.write(bytes.data(), size);
        if (!ok)
        {
            self.relayWriteFails_++;
        }
        self.forwarding_ok_ = self.forwarding_ok_ && ok;
    }

    bool SitlTransport::servicePeer()
    {
        const int accepted = ::accept(listener_, nullptr, nullptr);
        if (accepted >= 0)
        {
            if (peer_ >= 0)
            {
                RCLCPP_WARN(logger_, "Rejected a second AirSim relay connection");
                ::close(accepted);
            }
            else
            {
                peer_ = accepted;
                const int one = 1;
                ::setsockopt(peer_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                RCLCPP_INFO(logger_, "AirSim custom-link physics peer connected");
            }
        }
        else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        {
            RCLCPP_ERROR(logger_, "SITL relay accept failed: %s", std::strerror(errno));
            return false;
        }
        if (peer_ < 0)
            return true;
        std::array<uint8_t, 4096> bytes{};
        const auto count = ::recv(peer_, bytes.data(), bytes.size(), MSG_DONTWAIT);
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
            return true;
        if (count <= 0)
        {
            RCLCPP_WARN(logger_, "AirSim physics peer disconnected");
            return false;
        }
        const auto now = monoClockUs();
        for (ssize_t i = 0; i < count; ++i)
            parser_.processByte(bytes[i], now, &SitlTransport::forwardFrame, this);
        if (!forwarding_ok_)
            RCLCPP_ERROR(logger_, "Failed to forward AirSim simulation frames to FC");
        return forwarding_ok_;
    }

    int SitlTransport::read(uint8_t *buf, int n, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        do
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!fc_.isOpen() || !servicePeer())
                return -1;
            const int count = fc_.read(buf, n, std::chrono::milliseconds(0));
            if (count <= 0)
            {
                if (count < 0)
                    return -1;
                // Do not hold/reacquire the IO lock while waiting; the control sender needs it too.
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (peer_ >= 0)
            {
                int sent = 0;
                while (sent < count)
                {
                    const auto put = ::send(peer_, buf + sent, count - sent, MSG_NOSIGNAL | MSG_DONTWAIT);
                    if (put <= 0)
                    {
                        RCLCPP_ERROR(logger_, "AirSim relay write failed/backpressured: %s", std::strerror(errno));
                        return -1;
                    }
                    sent += static_cast<int>(put);
                }
            }
            return count;
        } while (std::chrono::steady_clock::now() < deadline);
        return 0;
    }

    bool SitlTransport::write(const uint8_t *buf, int n)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (n >= 3 && buf[2] == protocol::MSG_HOST_CONTROL && peer_ < 0)
        {
            RCLCPP_WARN_ONCE(logger_, "SITL control is blocked until the AirSim physics peer connects");
            return false;
        }
        return fc_.write(buf, n);
    }
}
