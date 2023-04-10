#pragma once

#include <utility>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace gary_serial{
    class MsgHandlerBase {
    public:
        virtual bool decode(uint8_t* buf, uint16_t len) = 0;
        virtual void publish() = 0;
        virtual ~MsgHandlerBase() {}
    };
}
