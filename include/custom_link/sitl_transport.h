#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "custom_link/protocol.h"
#include "custom_link/transport.h"

namespace custom_link
{
    // One FC connection shared by the ROS client and AirSim's custom-link physics adapter.
    class SitlTransport final : public Transport
    {
    public:
        SitlTransport(std::string fc_host, uint16_t fc_port, std::string bind_host,
                      uint16_t bind_port, rclcpp::Logger logger);
        ~SitlTransport() override;
        bool open() override;
        void close() override;
        bool isOpen() const override;
        std::string description() const override;
        int read(uint8_t *buf, int n, std::chrono::milliseconds timeout) override;
        bool write(const uint8_t *buf, int n) override;

    private:
        static void forwardFrame(const protocol::Frame &frame, uint64_t now, void *context);
        bool servicePeer();
        void closeLocked();
        TcpTransport fc_;
        std::string bind_host_;
        uint16_t bind_port_;
        rclcpp::Logger logger_;
        mutable std::mutex mutex_;
        int listener_ = -1;
        int peer_ = -1;
        bool forwarding_ok_ = true;
        protocol::Parser parser_;
    };
}
