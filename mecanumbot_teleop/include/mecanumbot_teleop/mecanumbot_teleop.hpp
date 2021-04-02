
#ifndef __DEBICT_MECANUMBOT_TELEOP__MECANUMBOT_TELEOP_H__
#define __DEBICT_MECANUMBOT_TELEOP__MECANUMBOT_TELEOP_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace debict
{
    namespace mecanumbot
    {
        namespace teleop
        {
            class MecanumbotTeleop
                : public rclcpp::Node
            {
            public:
                struct AxisConfig
                {
                    uint32_t axis;
                    double scale;
                    double offset;
                    double deadzone;
                };
            public:
                MecanumbotTeleop(const std::string & name);

            private:
                void on_joy_message(std::unique_ptr<sensor_msgs::msg::Joy> msg);
                double get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy> & msg, AxisConfig& config);

            private:
                rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
                AxisConfig move_forward_config_;
                AxisConfig move_reverse_config_;
                AxisConfig move_left_config_;
                AxisConfig move_right_config_;
                AxisConfig turn_left_config_;
                AxisConfig turn_right_config_;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_TELEOP__MECANUMBOT_TELEOP_H__
