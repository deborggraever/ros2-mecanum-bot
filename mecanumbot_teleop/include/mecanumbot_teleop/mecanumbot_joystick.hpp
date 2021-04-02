
#ifndef __DEBICT_MECANUMBOT_TELEOP__MECANUMBOT_JOYSTICK_H__
#define __DEBICT_MECANUMBOT_TELEOP__MECANUMBOT_JOYSTICK_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>

namespace debict
{
    namespace mecanumbot
    {
        namespace teleop
        {
            class MecanumbotJoystick
                : public rclcpp::Node
            {
            public:
                MecanumbotJoystick(const std::string & name);
                ~MecanumbotJoystick();

            private:
                void update();

            private:
                rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
                rclcpp::TimerBase::SharedPtr update_timer_;
                std::vector<int> buttons_;
                std::vector<double> axes_;
                int device_handle_;
                double deadzone_;
                double scale_;
                double unscaled_deadzone_;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_TELEOP__MECANUMBOT_JOYSTICK_H__