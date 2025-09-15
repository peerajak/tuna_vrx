#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

class ThrusterRelayNode : public rclcpp::Node
{
public:
    ThrusterRelayNode()
    : Node("thruster_relay_node")
    {
        // Publisher: Float64 to left thruster
        publisher_l_ = this->create_publisher<std_msgs::msg::Float64>(
            "/tuna/thrusters/tuna_left/thrust", 10);

        // Subscriber: Float64MultiArray from left thruster controller
        subscription_l_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tuna/left_thruster_controller/commands",
            10,
            std::bind(&ThrusterRelayNode::left_topic_callback, this, std::placeholders::_1));

        // Publisher: Float64 to left thruster
        publisher_r_ = this->create_publisher<std_msgs::msg::Float64>(
            "/tuna/thrusters/tuna_right/thrust", 10);

        // Subscriber: Float64MultiArray from left thruster controller
        subscription_r_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tuna/right_thruster_controller/commands",
            10,
            std::bind(&ThrusterRelayNode::right_topic_callback, this, std::placeholders::_1));


    }

private:
    void left_topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Left Received empty Float64MultiArray");
            return;
        }

        auto out_msg = std_msgs::msg::Float64();
        out_msg.data = msg->data[0];  // Take the first element
        publisher_l_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(), "Relayed value: %f", out_msg.data);
    }
    void right_topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Right Received empty Float64MultiArray");
            return;
        }

        auto out_msg = std_msgs::msg::Float64();
        out_msg.data = msg->data[0];  // Take the first element
        publisher_r_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(), "Relayed value: %f", out_msg.data);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_l_,publisher_r_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_l_,subscription_r_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterRelayNode>());
    rclcpp::shutdown();
    return 0;
}
