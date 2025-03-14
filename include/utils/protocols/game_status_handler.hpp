#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/game_status.hpp"

using namespace gary_serial;

class GameStatusHandler : public MsgHandlerBase {

public:
    explicit GameStatusHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::GameStatus>("/referee/game_status",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.game_type = buf[0] & 0x0f;
        this->msg.game_progress = buf[0] >> 4;
        this->msg.stage_remain_time = buf[2] << 8 | buf[1];
        this->msg.sync_time_stamp = (uint64_t)buf[10] << 56 | (uint64_t)buf[9] << 48 |
                                (uint64_t)buf[8] << 40 | (uint64_t)buf[7] << 32 |
                                (uint64_t)buf[6] << 24 | (uint64_t)buf[5] << 16 |
                                (uint64_t)buf[4] << 8 | buf[3];
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::GameStatus msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::GameStatus>::SharedPtr publisher;
};
