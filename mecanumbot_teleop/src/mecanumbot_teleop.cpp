
#include <memory>
#include "mecanumbot_teleop/mecanumbot_teleop.hpp"

using namespace debict::mecanumbot::teleop;

MecanumbotTeleop::MecanumbotTeleop(const std::string & name)
    : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // Setup the parameters
    declare_parameter<std::string>("joy_topic", "joy");
    declare_parameter<std::string>("twist_topic", "cmd_vel");
    declare_parameter<int>("move.forward.axis", 1);
    declare_parameter<double>("move.forward.scale", 1.0);
    declare_parameter<double>("move.forward.offset", 0.0);
    declare_parameter<double>("move.forward.deadzone", 0.0);
    declare_parameter<int>("move.reverse.axis", 1);
    declare_parameter<double>("move.reverse.scale", 1.0);
    declare_parameter<double>("move.reverse.offset", 0.0);
    declare_parameter<double>("move.reverse.deadzone", 0.0);
    declare_parameter<int>("move.left.axis", 0);
    declare_parameter<double>("move.left.scale", 1.0);
    declare_parameter<double>("move.left.offset", 0.0);
    declare_parameter<double>("move.left.deadzone", 0.0);
    declare_parameter<int>("move.right.axis", 0);
    declare_parameter<double>("move.right.scale", 1.0);
    declare_parameter<double>("move.right.offset", 0.0);
    declare_parameter<double>("move.right.deadzone", 0.0);
    declare_parameter<int>("turn.left.axis", 5);
    declare_parameter<double>("turn.left.scale", 1.0);
    declare_parameter<double>("turn.left.offset", 0.0);
    declare_parameter<double>("turn.left.deadzone", 0.0);
    declare_parameter<int>("turn.right.axis", 4);
    declare_parameter<double>("turn.right.scale", 1.0);
    declare_parameter<double>("turn.right.offset", 0.0);
    declare_parameter<double>("turn.right.deadzone", 0.0);

    // Load the parameters
    move_forward_config_.axis = (uint32_t)get_parameter("move.forward.axis").as_int();
    move_forward_config_.scale = get_parameter("move.forward.scale").as_double();
    move_forward_config_.offset = get_parameter("move.forward.offset").as_double();
    move_forward_config_.deadzone = get_parameter("move.forward.deadzone").as_double();
    move_reverse_config_.axis = (uint32_t)get_parameter("move.reverse.axis").as_int();
    move_reverse_config_.scale = get_parameter("move.reverse.scale").as_double();
    move_reverse_config_.offset = get_parameter("move.reverse.offset").as_double();
    move_reverse_config_.deadzone = get_parameter("move.reverse.deadzone").as_double();
    move_left_config_.axis = (uint32_t)get_parameter("move.left.axis").as_int();
    move_left_config_.scale = get_parameter("move.left.scale").as_double();
    move_left_config_.offset = get_parameter("move.left.offset").as_double();
    move_left_config_.deadzone = get_parameter("move.left.deadzone").as_double();
    move_right_config_.axis = (uint32_t)get_parameter("move.right.axis").as_int();
    move_right_config_.scale = get_parameter("move.right.scale").as_double();
    move_right_config_.offset = get_parameter("move.right.offset").as_double();
    move_right_config_.deadzone = get_parameter("move.right.deadzone").as_double();
    turn_left_config_.axis = (uint32_t)get_parameter("turn.left.axis").as_int();
    turn_left_config_.scale = get_parameter("turn.left.scale").as_double();
    turn_left_config_.offset = get_parameter("turn.left.offset").as_double();
    turn_left_config_.deadzone = get_parameter("turn.left.deadzone").as_double();
    turn_right_config_.axis = (uint32_t)get_parameter("turn.right.axis").as_int();
    turn_right_config_.scale = get_parameter("turn.right.scale").as_double();
    turn_right_config_.offset = get_parameter("turn.right.offset").as_double();
    turn_right_config_.deadzone = get_parameter("turn.right.deadzone").as_double();

    // Setup the velocity command publisher
    std::string twist_topic = get_parameter("twist_topic").as_string();
    twist_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, 10);

    // Setup the joystick message subscriber
    std::string joy_topic = get_parameter("joy_topic").as_string();
    joy_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(joy_topic, 10, std::bind(&MecanumbotTeleop::on_joy_message, this, std::placeholders::_1));
}

void MecanumbotTeleop::on_joy_message(std::unique_ptr<sensor_msgs::msg::Joy> msg)
{
    geometry_msgs::msg::TwistStamped::UniquePtr twist_message(new geometry_msgs::msg::TwistStamped());
    twist_message->header.frame_id = "twist";
    twist_message->header.stamp = now();
    twist_message->twist.linear.x = get_axis_value(msg, move_forward_config_) + get_axis_value(msg, move_reverse_config_);
    twist_message->twist.linear.y = get_axis_value(msg, move_left_config_) + get_axis_value(msg, move_right_config_);
    twist_message->twist.linear.z = 0.0;
    twist_message->twist.angular.x = 0.0;
    twist_message->twist.angular.y = 0.0;
    twist_message->twist.angular.z = get_axis_value(msg, turn_left_config_) + get_axis_value(msg, turn_right_config_);
    twist_publisher_->publish(std::move(twist_message));
}

double MecanumbotTeleop::get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy> & msg, AxisConfig& config)
{
    if (msg->axes.size() >= (size_t)config.axis) {
        double value = ((double)msg->axes[config.axis] - config.offset) * config.scale;
        if (value > config.deadzone) {
            value -= config.deadzone;
        }
        else if (value < -config.deadzone) {
            value += config.deadzone;
        }
        else {
            value = 0.0;
        }
        return value;
    }
    return 0.0;
}