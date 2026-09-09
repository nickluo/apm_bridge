#include "custom_link/transport.h"

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <thread>

namespace custom_link
{
    // ------------------------------------------------------------------
    // SerialTransport
    // ------------------------------------------------------------------

    bool SerialTransport::open()
    {
        close();
        try
        {
            serial_.setPort(port_);
            serial_.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(1);
            serial_.setTimeout(to);
            serial_.open();
            return serial_.isOpen();
        }
        catch (const serial::IOException &)
        {
            return false;
        }
    }

    void SerialTransport::close()
    {
        try
        {
            if (serial_.isOpen())
                serial_.close();
        }
        catch (const serial::IOException &)
        {
        }
    }

    bool SerialTransport::isOpen() const
    {
        try
        {
            return serial_.isOpen();
        }
        catch (const serial::IOException &)
        {
            return false;
        }
    }

    int SerialTransport::read(uint8_t *buf, int n, std::chrono::milliseconds timeout)
    {
        try
        {
            // WJ Wood 的 read(buf, size) 会等到读满或超时，直接读大块会引入
            // 整段超时的时延；先查可读量再精确读取，无数据时短暂等待。
            size_t avail = serial_.available();
            if (avail == 0)
            {
                const auto ms = std::min<int64_t>(timeout.count(), 5);
                std::this_thread::sleep_for(std::chrono::milliseconds(ms));
                avail = serial_.available();
                if (avail == 0)
                    return 0;
            }
            const size_t want = std::min(avail, static_cast<size_t>(n));
            const size_t got = serial_.read(buf, want);
            return static_cast<int>(got);
        }
        catch (const serial::IOException &)
        {
            return -1;
        }
    }

    bool SerialTransport::write(const uint8_t *buf, int n)
    {
        try
        {
            const size_t put = serial_.write(buf, static_cast<size_t>(n));
            return put == static_cast<size_t>(n);
        }
        catch (const serial::IOException &)
        {
            return false;
        }
    }

    // ------------------------------------------------------------------
    // TcpTransport
    // ------------------------------------------------------------------

    bool TcpTransport::open()
    {
        close();

        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo *res = nullptr;
        if (::getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || res == nullptr)
            return false;

        int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (fd < 0)
        {
            ::freeaddrinfo(res);
            return false;
        }

        // SITL 固件可能在主机进程之后才起来：用短超时避免阻塞调用方线程过久
        timeval tv{1, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        if (::connect(fd, res->ai_addr, res->ai_addrlen) != 0)
        {
            ::freeaddrinfo(res);
            ::close(fd);
            return false;
        }
        ::freeaddrinfo(res);

        const int one = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        fd_ = fd;
        return true;
    }

    void TcpTransport::close()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    int TcpTransport::read(uint8_t *buf, int n, std::chrono::milliseconds timeout)
    {
        if (fd_ < 0)
            return -1;
        pollfd pfd{fd_, POLLIN, 0};
        const int ms = static_cast<int>(timeout.count());
        const int pr = ::poll(&pfd, 1, ms);
        if (pr < 0)
            return -1;
        if (pr == 0 || !(pfd.revents & POLLIN))
            return 0;
        const ssize_t got = ::recv(fd_, buf, static_cast<size_t>(n), 0);
        if (got <= 0)
            return -1; // 对端关闭或错误
        return static_cast<int>(got);
    }

    bool TcpTransport::write(const uint8_t *buf, int n)
    {
        if (fd_ < 0)
            return false;
        size_t sent = 0;
        while (sent < static_cast<size_t>(n))
        {
            const ssize_t put = ::send(fd_, buf + sent, static_cast<size_t>(n) - sent, MSG_NOSIGNAL);
            if (put <= 0)
                return false;
            sent += static_cast<size_t>(put);
        }
        return true;
    }
} // namespace custom_link
