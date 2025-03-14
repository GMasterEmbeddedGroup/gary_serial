#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/power_heat.hpp"

using namespace gary_serial;

class PowerHeatHandler : public MsgHandlerBase {

public:
    explicit PowerHeatHandler(RMReferee *nodeptr) {
        this->publisher = nodeptr->create_publisher<gary_msgs::msg::PowerHeat>("/referee/power_heat",
                                                                                 rclcpp::SystemDefaultsQoS());
        this->publisher->on_activate();
    }

    bool decode(uint8_t* buf, uint16_t len) override {
        this->msg.chassis_volt = ((float) (buf[1] << 8 | buf[0])) / 1000.0f;
        this->msg.chassis_current = ((float) (buf[3] << 8 | buf[2])) / 1000.0f;
        uint2float_u uint2float;
        uint2float.u[0] = buf[4];
        uint2float.u[1] = buf[5];
        uint2float.u[2] = buf[6];
        uint2float.u[3] = buf[7];
        this->msg.chassis_power = uint2float.f;
        this->msg.chassis_power_buffer = buf[9] << 8 | buf[8];
        this->msg.shooter_17mm_id1_heat = buf[11] << 8 | buf[10];
        this->msg.shooter_17mm_id2_heat = buf[13] << 8 | buf[12];
        this->msg.shooter_42mm_id1_heat = buf[15] << 8 | buf[14];
        rclcpp::Clock clock;
        this->msg.header.stamp = clock.now();
        return true;
    }

    void publish() override{
        this->publisher->publish(this->msg);
    }
private:
    gary_msgs::msg::PowerHeat msg;
    rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::PowerHeat>::SharedPtr publisher;
};
