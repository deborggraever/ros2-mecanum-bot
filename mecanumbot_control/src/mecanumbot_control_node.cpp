
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <realtime_tools/thread_priority.hpp>

int const kSchedPriority = 50;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Setup the controller manager node
    std::string controller_manager_node_name = "controller_manager";
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto controller_manager_node = std::make_shared<controller_manager::ControllerManager>(executor, controller_manager_node_name);
    RCLCPP_INFO(controller_manager_node->get_logger(), "update rate is %d Hz", controller_manager_node->get_update_rate());

    std::thread cm_thread([controller_manager_node]() {
      if (realtime_tools::has_realtime_kernel()) {
        if (!realtime_tools::configure_sched_fifo(kSchedPriority)) {
          RCLCPP_WARN(controller_manager_node->get_logger(), "Could not enable FIFO RT scheduling policy");
        }
      }
      else {
        RCLCPP_INFO(controller_manager_node->get_logger(), "RT kernel is recommended for better performance");
      }

      // for calculating sleep time
      auto const period = std::chrono::nanoseconds(1'000'000'000 / controller_manager_node->get_update_rate());
      auto const cm_now = std::chrono::nanoseconds(controller_manager_node->now().nanoseconds());
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{cm_now};

      // for calculating the measured period of the loop
      rclcpp::Time previous_time = controller_manager_node->now();

      while (rclcpp::ok()) {
        // calculate measured period
        auto const current_time = controller_manager_node->now();
        auto const measured_period = current_time - previous_time;
        previous_time = current_time;
        
        controller_manager_node->read(controller_manager_node->now(), measured_period);
        controller_manager_node->update(controller_manager_node->now(), measured_period);
        controller_manager_node->write(controller_manager_node->now(), measured_period);

        next_iteration_time += period;
        std::this_thread::sleep_until(next_iteration_time);
      }
    });

    // Load the controllers
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    controller_manager_node->load_controller("joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster");
    controller_manager_node->load_controller("mecanumbot_drive_controller", "mecanumbot_controller/MecanumbotDriveController");
    controller_manager_node->configure_controller("joint_state_broadcaster");
    controller_manager_node->configure_controller("mecanumbot_drive_controller");

    start_controllers.push_back("joint_state_broadcaster");
    start_controllers.push_back("mecanumbot_drive_controller");
    controller_manager_node->switch_controller(start_controllers, stop_controllers, 1, controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);

    // Run the node(s)
    executor->add_node(controller_manager_node);
    executor->spin();

    // Exit
    rclcpp::shutdown();
    return 0;
}