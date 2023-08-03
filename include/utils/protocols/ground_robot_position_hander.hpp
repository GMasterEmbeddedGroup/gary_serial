#pragma once

#include "msg_handler_base.hpp"
#include "utils/uint2float.hpp"
#include "gary_msgs/msg/ground_robot_position.hpp"

using namespace gary_serial;

class GroundRobotPositionHandler : public MsgHandlerBase{
    public:
        explicit GroundRobotPositionHandler(RMReferee* nodeptr){
            this->publisher = nodeptr->create_publisher<gary_msgs::msg::GroundRobotPosition>("/referee/ground_robot_position",
                                                                                rclcpp::SystemDefaultsQoS());
            this->publisher->on_activate();
        }

        bool decode(uint8_t* buf, uint16_t len) override{
            ::uint2float_u uint2float;
            uint2float = {{buf[0],buf[1],buf[2],buf[3]}};
            this->msg.hero_x = uint2float.f;
            uint2float = {{buf[4],buf[5],buf[6],buf[7]}};
            this->msg.hero_y = uint2float.f;
            uint2float = {{buf[8],buf[9],buf[10],buf[11]}};
            this->msg.engineer_x = uint2float.f;
            uint2float = {{buf[12],buf[13],buf[14],buf[15]}};
            this->msg.engineer_y = uint2float.f;
            uint2float = {{buf[16],buf[17],buf[18],buf[19]}};
            this->msg.standard_3_x = uint2float.f;
            uint2float = {{buf[20],buf[21],buf[22],buf[32]}};
            this->msg.standard_3_y = uint2float.f;
            uint2float = {{buf[24],buf[25],buf[26],buf[27]}};
            this->msg.standard_4_x = uint2float.f;
            uint2float = {{buf[28],buf[29],buf[30],buf[31]}};
            this->msg.standard_4_y = uint2float.f;
            uint2float = {{buf[32],buf[33],buf[34],buf[35]}};
            this->msg.standard_5_x = uint2float.f;
            uint2float = {{buf[36],buf[37],buf[38],buf[39]}};
            this->msg.standard_5_y = uint2float.f;

            rclcpp::Clock clock;
            this->msg.header.stamp = clock.now();
            return true;
        }

        void publish() override{
            this->publisher->publish(this->msg);
        }


    private:    
        gary_msgs::msg::GroundRobotPosition msg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::GroundRobotPosition>::SharedPtr publisher;
};
