
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include "mecanumbot_teleop/mecanumbot_joystick.hpp"

using namespace debict::mecanumbot::teleop;

#define MECANUMBOT_JOYSTICK_UNSCALED_MAX    32767.0

MecanumbotJoystick::MecanumbotJoystick(const std::string & name)
    : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // Setup the parameters
    declare_parameter<std::string>("dev", "/dev/input/js0");
    declare_parameter<std::string>("joy_topic", "joy");
    declare_parameter<double>("deadzone", 0.5f);

    // Setup the joystick message publisher
    std::string joy_topic = get_parameter("joy_topic").as_string();
    joy_publisher_ = create_publisher<sensor_msgs::msg::Joy>(joy_topic, 10);

    // Initialize the update
    update_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MecanumbotJoystick::update, this));

    // Reset the device handle
    device_handle_ = -1;
}

MecanumbotJoystick::~MecanumbotJoystick()
{
    // Close the joystick
    if (device_handle_ >= 0) {
        ::close(device_handle_);
    }
}

void MecanumbotJoystick::update()
{
    if (device_handle_ < 0) {
        std::string device = get_parameter("dev").as_string();
        device_handle_ = ::open(device.c_str(), O_RDONLY | O_NONBLOCK);
        if (device_handle_ < 0) {
            return;
        }

        deadzone_ = get_parameter("deadzone").as_double();
        if (deadzone_ > 1.0) {
            deadzone_ /= MECANUMBOT_JOYSTICK_UNSCALED_MAX;
        }
        if (deadzone_ < 0.0) {
            deadzone_ = 0.0;
        }
        scale_ = -1.0f / (1.0f - deadzone_) / MECANUMBOT_JOYSTICK_UNSCALED_MAX;
        unscaled_deadzone_ = MECANUMBOT_JOYSTICK_UNSCALED_MAX * deadzone_;
    }

    js_event event;
    bool has_event = false;
    while (true) {
        auto num_bytes = ::read(device_handle_, &event, sizeof(js_event));
        if (num_bytes < (ssize_t)sizeof(js_event)) {
            break;
        }

        if (event.type == JS_EVENT_AXIS || event.type == JS_EVENT_AXIS | JS_EVENT_INIT) {
            if (event.number >= axes_.size()) {
                size_t old_size = axes_.size();
                axes_.resize(event.number + 1);
                for (size_t i = old_size; i < axes_.size(); i++) {
                    axes_[i] = 0.0f;
                }
            }
            double value = (double)event.value;
            if (value > unscaled_deadzone_) {
                value -= unscaled_deadzone_;
            }
            else if (value < -unscaled_deadzone_) {
                value += unscaled_deadzone_;
            }
            else {
                value = 0.0;
            }
            axes_[event.number] = (float)(value * scale_);
            has_event = true;
        }
        else if (event.type == JS_EVENT_BUTTON || event.type == JS_EVENT_BUTTON | JS_EVENT_INIT) {
            if (event.number >= buttons_.size()) {
                size_t old_size = buttons_.size();
                buttons_.resize(event.number + 1);
                for (size_t i = old_size; i < buttons_.size(); i++) {
                    buttons_[i] = 0;
                }
            }
            buttons_[event.number] = event.value ? 1 : 0;
            has_event = true;
        }
    }

    if (has_event == true) {
        sensor_msgs::msg::Joy::UniquePtr joy_msg(new sensor_msgs::msg::Joy());
        joy_msg->header.frame_id = "joy";
        joy_msg->header.stamp = now();
        joy_msg->axes.resize(axes_.size());
        joy_msg->buttons.resize(buttons_.size());
        for (size_t i = 0; i < axes_.size(); i++) {
            joy_msg->axes[i] = axes_[i];
        }
        for (size_t i = 0; i < buttons_.size(); i++) {
            joy_msg->buttons[i] = buttons_[i];
        }
        joy_publisher_->publish(std::move(joy_msg));
    }
}