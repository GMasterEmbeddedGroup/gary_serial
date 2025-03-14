#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/game_result.hpp"

using namespace gary_serial;

class GameResultHandler : public MsgHandlerBase {

public:
    explicit GameResultHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::GameResult>("/referee/game_result",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.winner = buf[0];
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::GameResult msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::GameResult>::SharedPtr publisher;
};
