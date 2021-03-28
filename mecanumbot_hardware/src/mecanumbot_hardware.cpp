
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "mecanumbot_hardware/mecanumbot_hardware.hpp"

PLUGINLIB_EXPORT_CLASS(
    debict::mecanumbot::hardware::MecanumbotHardware,
    hardware_interface::SystemInterface
)

using namespace debict::mecanumbot::hardware;

hardware_interface::return_type MecanumbotHardware::configure(const hardware_interface::HardwareInfo & system_info)
{
    if (configure_default(system_info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }

    serial_port_name_ = info_.hardware_parameters["serial_port"];
    position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of state interfaces (expected: 2)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint state interface 0 type (expected: position)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::return_type::ERROR;
        }
    }

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MecanumbotHardware::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]
            )
        );
    }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]
            )
        );
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumbotHardware::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]
            )
        );
    }
    return command_interfaces;
}

hardware_interface::return_type MecanumbotHardware::start()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware starting ...");

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (std::isnan(position_states_[i])) {
            position_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_states_[i])) {
            velocity_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_commands_[i])) {
            velocity_commands_[i] = 0.0f;
        }
        velocity_commands_saved_[i] = velocity_commands_[i];
    }

    serial_port_ = std::make_shared<MecanumbotSerialPort>();
    if (serial_port_->open(serial_port_name_) != return_type::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware failed to open serial port");
        return hardware_interface::return_type::ERROR;
    }

    status_ = hardware_interface::status::STARTED;

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware started");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumbotHardware::stop()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopping ...");

    if (serial_port_->is_open()) {
        serial_port_->close();
        serial_port_.reset();
    }

    status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopped");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumbotHardware::read()
{
    for (size_t i = 0; i < info_.joints.size(); i++) {
        //RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Got position %.5f, velocity %.5f for joint %d!", position_states_[i], velocity_states_[i], i);
    }
    position_states_[0] = 1.1f;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumbotHardware::write()
{
    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (velocity_commands_[i] != velocity_commands_saved_[i]) {
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Got command %.5f for joint %d!", velocity_commands_[i], i);

            //TODO: Send motor command over usb to device

            velocity_commands_saved_[i] = velocity_commands_[i];
        }
    }
    return hardware_interface::return_type::OK;
}
