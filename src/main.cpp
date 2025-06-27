#include "udp_communicator/udp_communicator.hpp"  // クラス定義を含むヘッダをinclude

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
