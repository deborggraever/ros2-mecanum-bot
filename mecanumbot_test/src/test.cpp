#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() 
    : Node("minimal_publisher")
    {
        _joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        _timer = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        auto now = get_clock()->now();

        auto joint_states = std::make_unique<sensor_msgs::msg::JointState>();
        joint_states->header.frame_id = "base_link";
        joint_states->header.stamp = now;

        joint_states->name.push_back("fl_wheel_joint");
        joint_states->position.push_back(0);
        joint_states->velocity.push_back(0);
        joint_states->effort.push_back(0);

        joint_states->name.push_back("fr_wheel_joint");
        joint_states->position.push_back(0);
        joint_states->velocity.push_back(0);
        joint_states->effort.push_back(0);

        _joint_state_publisher->publish(std::move(joint_states));
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_state_publisher;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
