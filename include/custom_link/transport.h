#pragma once

//
// 自定义链路传输层：真机走串口 (WJ Wood serial)，SITL 仿真走 TCP
// (Betaflight SITL 把 UART 映射为 TCP 服务器，UART3 = tcp/5763)。
//

#include <chrono>
#include <cstdint>
#include <string>

#include "serial/serial.h"

namespace custom_link
{
    class Transport
    {
    public:
        virtual ~Transport() = default;

        virtual bool open() = 0;
        virtual void close() = 0;
        virtual bool isOpen() const = 0;
        virtual std::string description() const = 0;

        /// 阻塞读，最多 timeout；返回读到的字节数，0 = 超时，<0 = 错误（需重连）。
        virtual int read(uint8_t *buf, int n, std::chrono::milliseconds timeout) = 0;
        /// 全量写出；返回 false = 错误（需重连）。
        virtual bool write(const uint8_t *buf, int n) = 0;
    };

    class SerialTransport final : public Transport
    {
    public:
        SerialTransport(std::string port, uint32_t baudrate)
            : port_(std::move(port)), baudrate_(baudrate) {}

        bool open() override;
        void close() override;
        bool isOpen() const override;
        std::string description() const override { return port_; }
        int read(uint8_t *buf, int n, std::chrono::milliseconds timeout) override;
        bool write(const uint8_t *buf, int n) override;

    private:
        std::string port_;
        uint32_t baudrate_;
        serial::Serial serial_{port_, baudrate_, serial::Timeout::simpleTimeout(1)};
    };

    class TcpTransport final : public Transport
    {
    public:
        TcpTransport(std::string host, uint16_t port)
            : host_(std::move(host)), port_(port) {}

        ~TcpTransport() override { close(); }

        bool open() override;
        void close() override;
        bool isOpen() const override { return fd_ >= 0; }
        std::string description() const override { return host_ + ":" + std::to_string(port_); }
        int read(uint8_t *buf, int n, std::chrono::milliseconds timeout) override;
        bool write(const uint8_t *buf, int n) override;

    private:
        std::string host_;
        uint16_t port_;
        int fd_ = -1;
    };
} // namespace custom_link
