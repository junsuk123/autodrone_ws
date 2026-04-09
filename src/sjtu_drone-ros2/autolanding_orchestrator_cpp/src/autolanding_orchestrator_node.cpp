#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AutoLandingOrchestratorNode : public rclcpp::Node {
public:
  AutoLandingOrchestratorNode()
  : Node("autolanding_orchestrator_node") {
    status_pub_ = create_publisher<std_msgs::msg::String>("/autolanding/orchestrator/status", 10);
    timer_ = create_wall_timer(1s, std::bind(&AutoLandingOrchestratorNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "autolanding_orchestrator_node started");
  }

private:
  void on_timer() {
    std_msgs::msg::String msg;
    msg.data = "python_orchestrator_active";
    status_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoLandingOrchestratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
