
#ifndef __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_HARDWARE_H__
#define __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_HARDWARE_H__

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <vector>

#include "mecanumbot_hardware/mecanumbot_serial_port.hpp"
#include "mecanumbot_hardware/mecanumbot_hardware_compiler.h"

namespace debict
{
    namespace mecanumbot
    {
        namespace hardware
        {
            class MecanumbotHardware
                : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
            {
            public:
                RCLCPP_SHARED_PTR_DEFINITIONS(MecanumbotHardware)

                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info) override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type start() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type stop() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type read() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type write() override;

            private:
                std::vector<double> position_states_;
                std::vector<double> velocity_states_;
                std::vector<double> velocity_commands_;
                std::vector<double> velocity_commands_saved_;
                std::shared_ptr<MecanumbotSerialPort> serial_port_;
                std::string serial_port_name_;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_HARDWARE_H__