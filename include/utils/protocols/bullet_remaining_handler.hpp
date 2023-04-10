#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/bullet_remaining.hpp"

using namespace gary_serial;

class BulletRemainingHandler : public MsgHandlerBase {

public:
    explicit BulletRemainingHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::BulletRemaining>("/referee/bullet_remaining",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.remaining_17mm_num = buf[1] << 8 | buf[0];
        this->msg.remaining_42mm_num = buf[3] << 8 | buf[2];
        this->msg.remaining_coin_num = buf[5] << 8 | buf[4];
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::BulletRemaining msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::BulletRemaining>::SharedPtr publisher;
};
