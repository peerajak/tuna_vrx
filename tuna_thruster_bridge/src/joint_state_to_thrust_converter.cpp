#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

class JointStateToThrustConverter : public rclcpp::Node
{
public:
  JointStateToThrustConverter()
  : Node("joint_state_to_thrust_converter")
  {
    // Create subscribers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/tuna/joint_states",
      10,
      std::bind(&JointStateToThrustConverter::joint_state_callback, this, std::placeholders::_1));

    // Create publishers
    left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/tuna/thrusters/tuna_left/thrust",
      10);
    
    right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/tuna/thrusters/tuna_right/thrust",
      10);

    // Declare parameters for joint names (with defaults)
    this->declare_parameter("left_wheel_joint_name", "left_wheel_joint");
    this->declare_parameter("right_wheel_joint_name", "right_wheel_joint");

    // Get parameter values
    left_wheel_joint_name_ = this->get_parameter("left_wheel_joint_name").as_string();
    right_wheel_joint_name_ = this->get_parameter("right_wheel_joint_name").as_string();

    RCLCPP_INFO(this->get_logger(), "Listening for joint states...");
    RCLCPP_INFO(this->get_logger(), "Looking for left wheel: '%s'", left_wheel_joint_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Looking for right wheel: '%s'", right_wheel_joint_name_.c_str());
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Find indices of the left and right wheel joints
    int left_index = -1;
    int right_index = -1;
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == left_wheel_joint_name_) {
        left_index = i;
      } else if (msg->name[i] == right_wheel_joint_name_) {
        right_index = i;
      }
    }

    // Check if we found both joints
    if (left_index == -1 || right_index == -1) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000, // 5 seconds throttle
        "Could not find joint names in message. Left: %d, Right: %d",
        left_index, right_index
      );
      return;
    }

    // Check if velocity data is available for both joints
    if (msg->velocity.size() <= static_cast<size_t>(left_index) || 
        msg->velocity.size() <= static_cast<size_t>(right_index)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000, // 5 seconds throttle
        "Velocity data not available for all joints"
      );
      return;
    }

    // Extract velocities
    double left_velocity = msg->velocity[left_index];
    double right_velocity = msg->velocity[right_index];

    // Create and publish thrust messages
    std_msgs::msg::Float64 left_thrust_msg;
    left_thrust_msg.data = left_velocity;
    left_thrust_pub_->publish(left_thrust_msg);

    std_msgs::msg::Float64 right_thrust_msg;
    right_thrust_msg.data = right_velocity;
    right_thrust_pub_->publish(right_thrust_msg);

    // Optional: Log for debugging (throttled to avoid spam)
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000, // 1 second throttle
      "Left thrust: %.2f, Right thrust: %.2f",
      left_velocity, right_velocity
    );
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
  
  std::string left_wheel_joint_name_;
  std::string right_wheel_joint_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateToThrustConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}