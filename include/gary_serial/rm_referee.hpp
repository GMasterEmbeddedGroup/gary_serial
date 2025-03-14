#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "utils/protocols/msg_handler_base.hpp"
#include "gary_msgs/msg/interactive_data_send.hpp"

#include <libserial/SerialPort.h>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace gary_serial {

    enum decode_stage{
        STAGE_LOOKING_SOF = 0,
        STAGE_READING_HEADER,
        STAGE_READING_PACKET
    };

    class RMReferee : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit RMReferee(const rclcpp::NodeOptions & options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        //callback group
        rclcpp::CallbackGroup::SharedPtr cb_group;

        //callback
        void update();
        void interactive_data_callback(gary_msgs::msg::InteractiveDataSend::SharedPtr msg);

        //params
        double update_freq;
        std::string serial_port;

        //subscribers
        rclcpp::Subscription<gary_msgs::msg::InteractiveDataSend>::SharedPtr interactive_data_sub;

        //timers
        rclcpp::TimerBase::SharedPtr timer_update;

        //serial port
        std::shared_ptr<LibSerial::SerialPort> serial;

        decode_stage stage{};

        //buffer
        uint16_t remaining_byte{};
        uint8_t rx_buffer[255]{};
        uint8_t tx_buffer[255]{};

        uint16_t data_length{};
        uint16_t cmd_id{};

        std::map<uint16_t, std::shared_ptr<MsgHandlerBase>> msg_handlers;
        std::vector<gary_msgs::msg::InteractiveDataSend> interactive_data_buffer;

    };
}