#pragma once

#include "msg_handler_base.hpp"
#include "gary_msgs/msg/robot_hurt.hpp"

using namespace gary_serial;

class RobotHurtHandler : public MsgHandlerBase {

public:
    explicit RobotHurtHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::RobotHurt>("/referee/robot_hurt",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.armor_id = buf[0] & 0x0f;
        this->msg.hurt_type = buf[0] >> 4;
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::RobotHurt msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotHurt>::SharedPtr publisher;
};
