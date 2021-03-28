
#include "mecanumbot_controller/mecanumbot_wheel.hpp"

using namespace debict::mecanumbot::controller;

MecanumbotWheel::MecanumbotWheel(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command
    )
    : position_state_(position_state)
    , velocity_state_(velocity_state)
    , velocity_command_(velocity_command)
{

}

void MecanumbotWheel::set_velocity(double value)
{
    velocity_command_.get().set_value(value);
}