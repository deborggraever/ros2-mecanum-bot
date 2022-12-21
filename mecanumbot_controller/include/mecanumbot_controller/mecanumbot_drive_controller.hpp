
#ifndef __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_DRIVE_CONTROLLER_H__
#define __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_DRIVE_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>

#include "mecanumbot_controller/mecanumbot_wheel.hpp"
#include "mecanumbot_controller/mecanumbot_controller_compiler.h"

namespace debict
{
    namespace mecanumbot
    {
        namespace controller
        {
            using Twist = geometry_msgs::msg::TwistStamped;

            class MecanumbotDriveController
                : public controller_interface::ControllerInterface
            {
            public:
                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                MecanumbotDriveController();
                
                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::InterfaceConfiguration command_interface_configuration() const override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::InterfaceConfiguration state_interface_configuration() const override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_init() override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
            
            protected:
                std::shared_ptr<MecanumbotWheel> get_wheel(const std::string & wheel_joint_name);

            protected:
                rclcpp::Subscription<Twist>::SharedPtr velocity_command_subsciption_;
                realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;
                std::shared_ptr<MecanumbotWheel> fl_wheel_;
                std::shared_ptr<MecanumbotWheel> fr_wheel_;
                std::shared_ptr<MecanumbotWheel> rl_wheel_;
                std::shared_ptr<MecanumbotWheel> rr_wheel_;
                std::string fl_wheel_joint_name_;
                std::string fr_wheel_joint_name_;
                std::string rl_wheel_joint_name_;
                std::string rr_wheel_joint_name_;
                double linear_scale_;
                double radial_scale_;
                double wheel_radius_;
                double wheel_distance_width_;
                double wheel_distance_length_;
                double wheel_separation_width_;
                double wheel_separation_length_;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_DRIVE_CONTROLLER_H__
