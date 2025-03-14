#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/robot_position.hpp"

using namespace gary_serial;

class RobotPositionHandler : public MsgHandlerBase {

public:
    explicit RobotPositionHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::RobotPosition>("/referee/robot_position",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        uint2float_u uint2float;
        uint2float.u[0] = buf[0];
        uint2float.u[1] = buf[1];
        uint2float.u[2] = buf[2];
        uint2float.u[3] = buf[3];
        this->msg.x = uint2float.f;
        uint2float.u[0] = buf[4];
        uint2float.u[1] = buf[5];
        uint2float.u[2] = buf[6];
        uint2float.u[3] = buf[7];
        this->msg.y = uint2float.f;
        uint2float.u[0] = buf[8];
        uint2float.u[1] = buf[9];
        uint2float.u[2] = buf[10];
        uint2float.u[3] = buf[11];
        this->msg.z = uint2float.f;
        uint2float.u[0] = buf[12];
        uint2float.u[1] = buf[13];
        uint2float.u[2] = buf[14];
        uint2float.u[3] = buf[15];
        this->msg.yaw = uint2float.f;
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::RobotPosition msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotPosition>::SharedPtr publisher;
};
