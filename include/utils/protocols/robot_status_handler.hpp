#pragma once

#include "msg_handler_base.hpp"
#include "gary_msgs/msg/robot_status.hpp"

using namespace gary_serial;

class RobotStatusHandler : public MsgHandlerBase {

public:
    explicit RobotStatusHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::RobotStatus>("/referee/robot_status",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.robot_id = buf[0];
        this->msg.robot_level = buf[1];
        this->msg.remain_hp = buf[3] << 8 | buf[2];
        this->msg.max_hp = buf[5] << 8 | buf[4];
        this->msg.shooter_17mm_id1_cooling_rate = buf[7] << 8 | buf[6];
        this->msg.shooter_17mm_id1_cooling_limit = buf[9] << 8 | buf[8];
        this->msg.shooter_17mm_id1_speed_limit = buf[11] << 8 | buf[10];
        this->msg.shooter_17mm_id2_cooling_rate = buf[13] << 8 | buf[12];
        this->msg.shooter_17mm_id2_cooling_limit = buf[15] << 8 | buf[14];
        this->msg.shooter_17mm_id2_speed_limit = buf[17] << 8 | buf[16];
        this->msg.shooter_42mm_id1_cooling_rate = buf[19] << 8 | buf[18];
        this->msg.shooter_42mm_id1_cooling_limit = buf[21] << 8 | buf[20];
        this->msg.shooter_42mm_id1_speed_limit = buf[23] << 8 | buf[22];
        this->msg.chassis_power_limit = buf[25] << 8 | buf[24];
        this->msg.gimbal_power_output = (buf[26] >> 0) & 0x01;
        this->msg.chassis_power_output = (buf[26] >> 1) & 0x01;
        this->msg.shooter_power_output = (buf[26] >> 2) & 0x01;
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::RobotStatus msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotStatus>::SharedPtr publisher;
};
