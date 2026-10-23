#include "custom_link/sitl_transport.h"
#include "custom_link/client.h"

#include <array>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
    class Socket
    {
    public:
        explicit Socket(int fd = -1) : fd_(fd) {}
        ~Socket() { if (fd_ >= 0) ::close(fd_); }
        int get() const { return fd_; }
    private:
        int fd_;
    };

    void require(bool condition, const char *message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    uint16_t listen(int fd)
    {
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        require(::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0, "bind");
        require(::listen(fd, 1) == 0, "listen");
        socklen_t length = sizeof(address);
        require(::getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length) == 0, "getsockname");
        return ntohs(address.sin_port);
    }

    void connect(int fd, uint16_t port)
    {
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(port);
        require(::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0, "connect");
    }

    std::vector<uint8_t> frame(uint8_t id)
    {
        std::array<uint8_t, custom_link::protocol::kFrameMax> bytes{};
        const uint8_t payload[] = {1, 2, 3, 4};
        const int size = custom_link::protocol::encode(id, 9, payload, sizeof(payload), bytes.data(), bytes.size());
        return {bytes.begin(), bytes.begin() + size};
    }

    void send(int fd, const std::vector<uint8_t> &data)
    {
        require(::send(fd, data.data(), data.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(data.size()), "send");
    }

    std::vector<uint8_t> receive(int fd, size_t size)
    {
        const timeval timeout{1, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        pollfd descriptor{fd, POLLIN, 0};
        require(::poll(&descriptor, 1, 1000) == 1, "receive timeout");
        std::vector<uint8_t> data(size);
        require(::recv(fd, data.data(), size, MSG_WAITALL) == static_cast<ssize_t>(size), "receive");
        return data;
    }

    void noData(int fd)
    {
        pollfd descriptor{fd, POLLIN, 0};
        require(::poll(&descriptor, 1, 0) == 0, "unexpected FC input");
    }

    class CaptureTransport final : public custom_link::Transport
    {
    public:
        bool open() override { return true; }
        void close() override {}
        bool isOpen() const override { return true; }
        std::string description() const override { return "capture"; }
        int read(uint8_t *, int, std::chrono::milliseconds timeout) override
        {
            std::this_thread::sleep_for(timeout);
            return 0;
        }
        bool write(const uint8_t *bytes, int size) override
        {
            if (size >= custom_link::protocol::kFrameOverhead &&
                bytes[2] == custom_link::protocol::MSG_HOST_CONTROL)
            {
                custom_link::protocol::PayloadControl control{};
                require(bytes[3] == sizeof(control), "control payload size");
                std::memcpy(&control, bytes + 5, sizeof(control));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    controls_.push_back(control);
                }
                changed_.notify_all();
            }
            return true;
        }
        size_t count()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return controls_.size();
        }
        custom_link::protocol::PayloadControl waitFor(
            size_t start, std::function<bool(const custom_link::protocol::PayloadControl &)> predicate)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            size_t matched = start;
            const bool found = changed_.wait_for(lock, std::chrono::seconds(1), [&]() {
                for (; matched < controls_.size(); ++matched)
                    if (predicate(controls_[matched]))
                        return true;
                return false;
            });
            require(found, "mode request was not transmitted");
            return controls_[matched];
        }

    private:
        std::mutex mutex_;
        std::condition_variable changed_;
        std::vector<custom_link::protocol::PayloadControl> controls_;
    };

    void verifyModeRequests()
    {
        auto transport = std::make_unique<CaptureTransport>();
        auto &capture = *transport;
        custom_link::Client client(std::move(transport));
        client.setRates(100.0, 10.0);
        client.start();
        custom_link::ControlInput command;
        command.throttle = 0.3;
        command.rate_dps[0] = 12.5;
        client.setControl(command, true);
        capture.waitFor(0, [](const auto &frame) { return frame.mode_req == 2 && frame.throttle == 1300; });

        client.clearControl();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        size_t start = capture.count();
        client.setOffboardRequest(false);
        auto manual = capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 0; });
        require(manual.arm == 0 && manual.throttle == 1000 && manual.rate_x10[0] == 0, "idle manual request");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        require(capture.count() == start + 1, "mode-only request bypassed control watchdog");

        start = capture.count();
        client.setControl(command, true);
        capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 0 && frame.throttle == 1300; });
        start = capture.count();
        client.setOffboardRequest(true);
        auto offboard = capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 2; });
        require(offboard.arm == 0 && offboard.throttle == 1300 && offboard.rate_x10[0] == 125, "OFFBOARD altered controls/arm");

        start = capture.count();
        command.angle = true;
        command.rate_dps[0] = 2.0;
        client.setControl(command, true);
        auto angle = capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 3; });
        require(angle.rate_x10[0] == 20 && angle.arm == 0, "ANGLE target/arm transport");
        start = capture.count();
        client.setOffboardRequest(false);
        capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 0; });
        command.angle = false;
        command.rate_dps[0] = 12.5;
        client.setControl(command, true);
        client.setOffboardRequest(true);
        capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 2; });

        start = capture.count();
        client.setArmRequest(true);
        capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 2 && frame.arm == 1; });
        start = capture.count();
        client.setOffboardRequest(false);
        manual = capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 0; });
        require(manual.arm == 1, "manual request changed the arm bit");
        client.clearControl();
        client.setArmRequest(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        start = capture.count();
        client.setOffboardRequest(true);
        offboard = capture.waitFor(start, [](const auto &frame) { return frame.mode_req == 2; });
        require(offboard.arm == 0 && offboard.throttle == 1000, "idle OFFBOARD request");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        require(capture.count() == start + 1, "idle OFFBOARD request kept controls alive");
        client.stop();
    }
}

int main()
{
    try
    {
        verifyModeRequests();
        require(custom_link::periodFromRate(25.0).count() == 40000000, "25 Hz period");
        require(custom_link::periodFromRate(100.0).count() == 10000000, "100 Hz period");
        require(custom_link::periodFromRate(125.5).count() == 7968127, "fractional rate precision");
        require(custom_link::periodFromRate(2000.0).count() == 500000, "sub-millisecond period");
        for (const double rate : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::quiet_NaN(), 1.0e-12, 1.0e12})
        {
            bool rejected = false;
            try { custom_link::periodFromRate(rate); }
            catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "invalid link rate was accepted");
        }
        Socket server(::socket(AF_INET, SOCK_STREAM, 0));
        const auto fc_port = listen(server.get());
        uint16_t relay_port;
        {
            Socket reserve(::socket(AF_INET, SOCK_STREAM, 0));
            relay_port = listen(reserve.get());
        }
        custom_link::SitlTransport relay("127.0.0.1", fc_port, "127.0.0.1", relay_port,
                                         rclcpp::get_logger("sitl_relay_test"));
        require(relay.open(), "relay open");
        Socket fc(::accept(server.get(), nullptr, nullptr));
        Socket peer(::socket(AF_INET, SOCK_STREAM, 0));
        connect(peer.get(), relay_port);
        std::array<uint8_t, 512> buffer{};
        relay.read(buffer.data(), buffer.size(), std::chrono::milliseconds(5));

        const auto imu = frame(0x21);
        send(peer.get(), {imu.begin(), imu.begin() + 3});
        relay.read(buffer.data(), buffer.size(), std::chrono::milliseconds(5));
        noData(fc.get());
        const auto control = frame(0x20);
        require(relay.write(control.data(), control.size()), "ROS control write");
        require(receive(fc.get(), control.size()) == control, "ROS control interleaved");
        send(peer.get(), {imu.begin() + 3, imu.end()});
        relay.read(buffer.data(), buffer.size(), std::chrono::milliseconds(5));
        require(receive(fc.get(), imu.size()) == imu, "fragmented physics frame");

        for (const uint8_t id : {0x22, 0x23})
        {
            const auto input = frame(id);
            send(peer.get(), input);
            relay.read(buffer.data(), buffer.size(), std::chrono::milliseconds(5));
            require(receive(fc.get(), input.size()) == input, "simulation input");
        }
        for (const uint8_t id : {0x10, 0x11, 0x12, 0x13, 0x14, 0x31})
        {
            const auto output = frame(id);
            send(fc.get(), output);
            const int count = relay.read(buffer.data(), buffer.size(), std::chrono::milliseconds(20));
            require(count == static_cast<int>(output.size()), "FC telemetry read");
            require(std::vector<uint8_t>(buffer.begin(), buffer.begin() + count) == output, "ROS telemetry");
            require(receive(peer.get(), output.size()) == output, "AirSim telemetry/motors");
        }
        send(peer.get(), control);
        relay.read(buffer.data(), buffer.size(), std::chrono::milliseconds(5));
        noData(fc.get());
        auto bad_crc = imu;
        bad_crc.back() ^= 1;
        send(peer.get(), bad_crc);
        relay.read(buffer.data(), buffer.size(), std::chrono::milliseconds(5));
        noData(fc.get());
        ::shutdown(peer.get(), SHUT_RDWR);
        require(relay.read(buffer.data(), buffer.size(), std::chrono::milliseconds(20)) < 0, "peer loss");
        relay.close();
        require(!relay.isOpen(), "close");
        std::cout << "PASS: independent OFFBOARD/manual requests, idle requests/watchdog, custom-link relay, CRC and disconnect\n";
        return 0;
    }
    catch (const std::runtime_error &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
