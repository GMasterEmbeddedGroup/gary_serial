#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/shoot_data.hpp"

using namespace gary_serial;

class ShootDataHandler : public MsgHandlerBase {

public:
    explicit ShootDataHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::ShootData>("/referee/shoot_data",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.bullet_type = buf[0];
        this->msg.shooter_id = buf[1];
        this->msg.bullet_freq = buf[2];
        uint2float_u uint2Float = {{buf[3],buf[4],buf[5],buf[6]}};
        this->msg.bullet_speed = uint2Float.f;
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::ShootData msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::ShootData>::SharedPtr publisher;
};
