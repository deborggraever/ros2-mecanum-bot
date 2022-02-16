
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mecanumbot_controller/mecanumbot_drive_controller.hpp"

PLUGINLIB_EXPORT_CLASS(
    debict::mecanumbot::controller::MecanumbotDriveController,
    controller_interface::ControllerInterface
)

using namespace debict::mecanumbot::controller;

MecanumbotDriveController::MecanumbotDriveController()
    : controller_interface::ControllerInterface()
    , velocity_command_subsciption_(nullptr)
    , velocity_command_ptr_(nullptr)
{

}

controller_interface::InterfaceConfiguration MecanumbotDriveController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MecanumbotDriveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return state_interfaces_config;
}

controller_interface::return_type MecanumbotDriveController::init(const std::string & controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }

    try {
        auto node = get_node();
        node->declare_parameter<std::string>("fl_wheel_joint_name", "");
        node->declare_parameter<std::string>("fr_wheel_joint_name", "");
        node->declare_parameter<std::string>("rl_wheel_joint_name", "");
        node->declare_parameter<std::string>("rr_wheel_joint_name", "");
        node->declare_parameter<double>("wheel_radius", 0.0);
        node->declare_parameter<double>("wheel_distance.width", 0.0);
        node->declare_parameter<double>("wheel_distance.length", 0.0);
    }
    catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type MecanumbotDriveController::update()
{
    // Get the last velocity command
    auto velocity_command = velocity_command_ptr_.readFromRT();
    if (!velocity_command || !(*velocity_command)) {
        return controller_interface::return_type::OK;
    }

    // Calculate the wheel velocity
    // See: http://robotsforroboticists.com/drive-kinematics/
    const auto twist = (*velocity_command)->twist;
    double fl_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double fr_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double rl_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double rr_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);

    fl_wheel_->set_velocity(fl_wheel_velocity);
    fr_wheel_->set_velocity(fr_wheel_velocity);
    rl_wheel_->set_velocity(rl_wheel_velocity);
    rr_wheel_->set_velocity(rr_wheel_velocity);

    return controller_interface::return_type::OK;
}

CallbackReturn MecanumbotDriveController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Configure MecanumbotDriverController");

    fl_wheel_joint_name_ = get_node()->get_parameter("fl_wheel_joint_name").as_string();
    fr_wheel_joint_name_ = get_node()->get_parameter("fr_wheel_joint_name").as_string();
    rl_wheel_joint_name_ = get_node()->get_parameter("rl_wheel_joint_name").as_string();
    rr_wheel_joint_name_ = get_node()->get_parameter("rr_wheel_joint_name").as_string();
    if (fl_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'fl_wheel_joint_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (fr_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'fr_wheel_joint_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (rl_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'rl_wheel_joint_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (rr_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'rr_wheel_joint_name' parameter was empty");
        return CallbackReturn::ERROR;
    }

    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    wheel_distance_width_ = get_node()->get_parameter("wheel_distance.width").as_double();
    wheel_distance_length_ = get_node()->get_parameter("wheel_distance.length").as_double();
    if (wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_radius' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }
    if (wheel_distance_width_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_distance.width' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }
    if (wheel_distance_length_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_distance.length' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }
    wheel_separation_width_ = wheel_distance_width_ / 2;
    wheel_separation_length_ = wheel_distance_length_ / 2;

    velocity_command_subsciption_ = get_node()->create_subscription<Twist>("/cmd_vel", 10, [this](const Twist::SharedPtr twist)
    {
        velocity_command_ptr_.writeFromNonRT(twist);
    });

    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumbotDriveController::on_activate(const rclcpp_lifecycle::State &)
{
    // Initialize the wheels
    fl_wheel_ = get_wheel(fl_wheel_joint_name_);
    fr_wheel_ = get_wheel(fr_wheel_joint_name_);
    rl_wheel_ = get_wheel(rl_wheel_joint_name_);
    rr_wheel_ = get_wheel(rr_wheel_joint_name_);
    if (!fl_wheel_ || !fr_wheel_ || !rl_wheel_ || !rr_wheel_) {
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumbotDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumbotDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
    // Uninitialize the wheels
    fl_wheel_.reset();
    fr_wheel_.reset();
    rl_wheel_.reset();
    rr_wheel_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumbotDriveController::on_error(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumbotDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

std::shared_ptr<MecanumbotWheel> MecanumbotDriveController::get_wheel(const std::string & wheel_joint_name)
{
    // Lookup the position state interface
    const auto position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity command interface
    const auto velocity_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity command interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Create the wheel instance
    return std::make_shared<MecanumbotWheel>(
        std::ref(*position_state),
        std::ref(*velocity_state),
        std::ref(*velocity_command)
        );
}
