#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/robot_buff.hpp"

using namespace gary_serial;

class RobotBuffHandler : public MsgHandlerBase {

public:
    explicit RobotBuffHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::RobotBuff>("/referee/robot_buff",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.robot_replenishing_blood = (buf[0] >> 0) & 0x01;
        this->msg.shooter_cooling_acceleration = (buf[0] >> 1) & 0x01;
        this->msg.robot_defense_bonus = (buf[0] >> 2) & 0x01;
        this->msg.robot_attack_bonus = (buf[0] >> 3) & 0x01;
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::RobotBuff msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotBuff>::SharedPtr publisher;
};
