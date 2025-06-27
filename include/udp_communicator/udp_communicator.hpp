#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>

using json = nlohmann::json;
using namespace std::chrono_literals;

class UdpCommNode : public rclcpp::Node
{
public:
    UdpCommNode();
    ~UdpCommNode();

private:
    void init_udp();
    void send_request();
    void receive_loop();

    // UDP
    int sock_;
    sockaddr_in dest_addr_;
    std::thread recv_thread_;
    std::atomic<bool> running_{true};

    // ROS
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

