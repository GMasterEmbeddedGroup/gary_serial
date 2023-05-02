#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/client_command.hpp"

using namespace gary_serial;

class ClientCommandHandler : public MsgHandlerBase {

public:
    explicit ClientCommandHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::ClientCommand>("/referee/bullet_remaining",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        ::uint2float_u uint2float = {{buf[0],buf[1],buf[2],buf[3]}};
        this->msg.target_position_x = uint2float.f;
        uint2float = {{buf[4],buf[5],buf[6],buf[7]}};
        this->msg.target_position_y = uint2float.f;
        uint2float = {{buf[8],buf[9],buf[10],buf[11]}};
        this->msg.target_position_z = uint2float.f;
        this->msg.keyboard_key_pressed = buf[12];
        this->msg.target_robot_id = buf[14] << 8 | buf[13];
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::ClientCommand msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::ClientCommand>::SharedPtr publisher;
};
