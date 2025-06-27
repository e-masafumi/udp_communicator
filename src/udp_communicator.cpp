#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <nlohmann/json.hpp>  // ← JSONライブラリ（後述）
#include "udp_communicator/udp_communicator.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

UdpCommNode::UdpCommNode() : Node("udp_communicator")
{
    init_udp();

    publisher_ = this->create_publisher<std_msgs::msg::String>("sensor_json", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&UdpCommNode::send_request, this));

    recv_thread_ = std::thread([this]() { receive_loop(); });

    RCLCPP_INFO(this->get_logger(), "UdpCommNode started");
}

UdpCommNode::~UdpCommNode()
{
    running_ = false;
    if (recv_thread_.joinable())
        recv_thread_.join();
    close(sock_);
}

void UdpCommNode::init_udp()
{
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0)
        throw std::runtime_error("Failed to create socket");

    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(5000);
    inet_pton(AF_INET, "192.168.10.2", &dest_addr_.sin_addr);

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(8001);
    local_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
        throw std::runtime_error("Failed to bind UDP socket");
}

void UdpCommNode::send_request()
{
    uint8_t request_id = 0x01;
    sendto(sock_, &request_id, sizeof(request_id), 0,
           (struct sockaddr *)&dest_addr_, sizeof(dest_addr_));
}

void UdpCommNode::receive_loop()
{
    char buffer[1024];
    while (running_)
    {
        sockaddr_in sender_addr;
        socklen_t addr_len = sizeof(sender_addr);
        ssize_t len = recvfrom(sock_, buffer, sizeof(buffer) - 1, 0,
                               (struct sockaddr *)&sender_addr, &addr_len);
        if (len > 0)
        {
            buffer[len] = '\0';
            std::string raw_data(buffer);
            
						try {
                json partial = json::parse(raw_data);  // 受信JSONを解析

                // timestamp追加
                partial["timestamp"] = this->now().seconds();

                std_msgs::msg::String msg;
                msg.data = partial.dump();  // フィールド追加済みJSONを文字列化

                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());

            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Invalid JSON received: %s", e.what());
            }

//            auto now = this->now();

 //           json j;
 //           j["timestamp"] = now.seconds();
 //           j["type"] = "sensor_data";
 //           j["value"] = raw_data;

 //           std_msgs::msg::String msg;
 //           msg.data = j.dump();
 //           publisher_->publish(msg);

 //           RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());
        }
    }
}
