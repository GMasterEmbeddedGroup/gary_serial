#include "rclcpp/rclcpp.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <libserial/SerialPort.h>

using namespace std::chrono_literals;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace gary_contact{
    class AutoAIMContact : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit AutoAIMContact(const rclcpp::NodeOptions & options);

    private:
        union {
                float aim_pitch = 0.0;
                uint8_t pitch_raw[4];
                };

        union {
                float aim_yaw = 0.0;
                uint8_t yaw_raw[4];
                };

        uint8_t packet[8] = {};
    
        //callback
        void data_send();
        void auto_aim_callback(gary_msgs::msg::AutoAIM msg);

        rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr vision_subscription_;
        rclcpp::TimerBase::SharedPtr aim_timer_;

        std::string serial_port;
        std::shared_ptr<LibSerial::SerialPort> serial;

    };

}