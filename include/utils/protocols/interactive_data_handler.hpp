#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/interactive_data_recv.hpp"

using namespace gary_serial;

class InteractiveDataHandler : public MsgHandlerBase {

public:
    explicit InteractiveDataHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::InteractiveDataRecv>("/referee/interactive_data_recv",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.data_cmd_id = buf[1] << 8 | buf[0];
        this->msg.sender_id = buf[3] << 8 | buf[2];
        this->msg.receiver_id = buf[5] << 8 | buf[4];

        this->msg.data.clear();
        
        for (int i = 0; i < len - 6; ++i) {
            this->msg.data.push_back(buf[6] + i);
        }
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::InteractiveDataRecv msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::InteractiveDataRecv>::SharedPtr publisher;
};
