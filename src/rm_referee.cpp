#include "gary_serial/rm_referee.hpp"
#include "utils/crc8_crc16.hpp"

#include "utils/protocols/robot_status_handler.hpp"
#include "utils/protocols/power_heat_handler.hpp"
#include "utils/protocols/robot_position_handler.hpp"
#include "utils/protocols/robot_hurt_handler.hpp"
#include "utils/protocols/game_status_handler.hpp"
#include "utils/protocols/bullet_remaining_handler.hpp"
#include "utils/protocols/client_command_handler.hpp"
#include "utils/protocols/shoot_data_handler.hpp"
#include "utils/protocols/game_result_handler.hpp"
#include "utils/protocols/robot_buff_handler.hpp"
#include "utils/protocols/interactive_data_handler.hpp"
#include "utils/protocols/ground_robot_position_hander.hpp"

using namespace std::chrono_literals;
using namespace gary_serial;


RMReferee::RMReferee(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "rm_referee", options) {

    this->declare_parameter("update_freq", 100.0f);
    this->declare_parameter("serial_port", "/dev/ttyREFEREE0");

    this->update_freq = 100.0f;
}

CallbackReturn RMReferee::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create callback group
    this->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    //get update_freq
    this->update_freq = this->get_parameter("update_freq").as_double();

    //get serial_port
    this->serial_port = this->get_parameter("serial_port").as_string();

    this->serial = std::make_shared<LibSerial::SerialPort>(this->serial_port,
                                                           LibSerial::BaudRate::BAUD_115200,
                                                           LibSerial::CharacterSize::CHAR_SIZE_8,
                                                           LibSerial::FlowControl::FLOW_CONTROL_NONE,
                                                           LibSerial::Parity::PARITY_NONE,
                                                           LibSerial::StopBits::STOP_BITS_1
                                                           );

    this->msg_handlers.emplace(0x001, std::make_shared<GameStatusHandler>(this));
    this->msg_handlers.emplace(0x201, std::make_shared<RobotStatusHandler>(this));
    this->msg_handlers.emplace(0x202, std::make_shared<PowerHeatHandler>(this));
    this->msg_handlers.emplace(0x203, std::make_shared<RobotPositionHandler>(this));
    this->msg_handlers.emplace(0x206, std::make_shared<RobotHurtHandler>(this));
    this->msg_handlers.emplace(0x207, std::make_shared<ShootDataHandler>(this));
    this->msg_handlers.emplace(0x208, std::make_shared<BulletRemainingHandler>(this));
    this->msg_handlers.emplace(0x303, std::make_shared<ClientCommandHandler>(this));
    this->msg_handlers.emplace(0x002, std::make_shared<GameResultHandler>(this));
    this->msg_handlers.emplace(0x204, std::make_shared<RobotBuffHandler>(this));
    this->msg_handlers.emplace(0x20B, std::make_shared<GroundRobotPositionHandler>(this));
    this->msg_handlers.emplace(0x301, std::make_shared<InteractiveDataHandler>(this));

    this->interactive_data_sub = this->create_subscription<gary_msgs::msg::InteractiveDataSend>("/referee/interactive_data_send",
                                                                                                rclcpp::SystemDefaultsQoS(),
                                                                                                std::bind(&RMReferee::interactive_data_callback, this, std::placeholders::_1),
                                                                                                sub_options);

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RMReferee::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    this->serial.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RMReferee::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create timer
    this->timer_update = this->create_wall_timer(1000ms / this->update_freq, [this] { update(); }, this->cb_group);

    this->stage = STAGE_LOOKING_SOF;
    this->remaining_byte = 1;

    RCLCPP_INFO(this->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RMReferee::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete timer
    this->timer_update.reset();

    RCLCPP_INFO(this->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RMReferee::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->timer_update.get() != nullptr) this->timer_update.reset();
    if (this->serial != nullptr) this->serial.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RMReferee::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->timer_update.get() != nullptr) this->timer_update.reset();
    if (this->serial != nullptr) this->serial.reset();

    RCLCPP_ERROR(this->get_logger(), "Error happened");
    return CallbackReturn::SUCCESS;
}


void RMReferee::interactive_data_callback(gary_msgs::msg::InteractiveDataSend::SharedPtr msg) {
    this->interactive_data_buffer.emplace_back(*msg);
    RCLCPP_DEBUG(this->get_logger(), "interactive data recv callback");
}

void RMReferee::update() {
    RCLCPP_DEBUG(this->get_logger(), "update");

    try {

        if (!this->serial->IsOpen()) {
            this->serial->Open(this->serial_port);
            RCLCPP_DEBUG(this->get_logger(), "serial not open, trying to open");
            return;
        }

        //RX
        RCLCPP_DEBUG(this->get_logger(), "now available %d bytes, %d bytes in need",
                     this->serial->GetNumberOfBytesAvailable(),
                     this->remaining_byte);

        while (this->serial->GetNumberOfBytesAvailable() >= this->remaining_byte) {
            RCLCPP_DEBUG(this->get_logger(), "stage %d", this->stage);
            switch (this->stage) {

                case STAGE_LOOKING_SOF: {
                    char ch;
                    this->serial->ReadByte(ch);
                    if ((uint8_t) ch == 0xa5) {
                        this->stage = STAGE_READING_HEADER;
                        this->remaining_byte = 4;
                        this->rx_buffer[0] = (uint8_t) ch;
                        RCLCPP_DEBUG(this->get_logger(), "found SOF");
                    } else {
                        this->stage = STAGE_LOOKING_SOF;
                        this->remaining_byte = 1;
                        RCLCPP_DEBUG(this->get_logger(), "SOF mismatch, found 0x%x", ch);
                    }
                    break;
                }

                case STAGE_READING_HEADER:
                    for (int i = 1; i < 5; ++i) {
                        char ch;
                        this->serial->ReadByte(ch);
                        this->rx_buffer[i] = (uint8_t) ch;
                    }
                    if (verify_CRC8_check_sum(this->rx_buffer, 5) == 1) {
                        this->stage = STAGE_READING_PACKET;
                        this->data_length = this->rx_buffer[2] << 8 | this->rx_buffer[1];
                        this->remaining_byte = 2 + this->data_length + 2;
                        RCLCPP_DEBUG(this->get_logger(), "found header, packet length %d", this->data_length);
                        break;
                    } else {
                        this->stage = STAGE_LOOKING_SOF;
                        this->remaining_byte = 1;
                        RCLCPP_WARN(this->get_logger(), "found header, but crc8 checksum mismatch");
                    }
                    break;
                case STAGE_READING_PACKET:
                    for (int i = 5; i < 5 + 2 + this->data_length + 2; ++i) {
                        char ch;
                        this->serial->ReadByte(ch);
                        this->rx_buffer[i] = (uint8_t) ch;
                    }
                    if (verify_CRC16_check_sum(this->rx_buffer, 5 + 2 + this->data_length + 2) == 1) {

                        this->cmd_id = this->rx_buffer[6] << 8 | this->rx_buffer[5];
                        RCLCPP_DEBUG(this->get_logger(), "found packet, cmd 0x%x", this->cmd_id);

                        if (this->msg_handlers.count(this->cmd_id) == 1) {
                            this->msg_handlers[this->cmd_id]->decode(this->rx_buffer + 5 + 2, this->data_length);
                            this->msg_handlers[this->cmd_id]->publish();
                        } else {
                            RCLCPP_WARN(this->get_logger(), "no matching handler for 0x%x", this->cmd_id);
                        }

                        this->stage = STAGE_LOOKING_SOF;
                        this->remaining_byte = 1;
                        break;
                    } else {
                        this->stage = STAGE_LOOKING_SOF;
                        this->remaining_byte = 1;
                        RCLCPP_WARN(this->get_logger(), "found packet 0x%x, but crc16 checksum mismatch",
                                    this->rx_buffer[6] << 8 | this->rx_buffer[5]);
                    }
                    break;
            }
        }

        //TX
        if (this->interactive_data_buffer.empty()) return;

        for (auto i = this->interactive_data_buffer.begin(); i < this->interactive_data_buffer.end(); ++i) {
            rclcpp::Time msg_time(i->header.stamp.sec, i->header.stamp.nanosec, RCL_ROS_TIME);
            rclcpp::Time time_now = this->get_clock()->now();
            if ((time_now - msg_time).seconds() > i->valid_time) {
                this->interactive_data_buffer.erase(i);
                RCLCPP_WARN(this->get_logger(), "interactive data invalid after %f seconds", (time_now - msg_time).seconds());
            }
        }

        if (this->interactive_data_buffer.empty()) return;

        auto interactive_itr = this->interactive_data_buffer.begin();
        for (auto i = this->interactive_data_buffer.begin(); i < this->interactive_data_buffer.end(); ++i) {
            if (i->priority < interactive_itr->priority) interactive_itr = i;
        }

        gary_msgs::msg::InteractiveDataSend data_send = *interactive_itr;
        this->interactive_data_buffer.erase(interactive_itr);

        //SOF
        this->tx_buffer[0] = 0xa5;
        //DATA_LENGTH_LOW
        this->tx_buffer[1] = 6 + data_send.data.size();
        //DATA_LENGTH_HIGH
        this->tx_buffer[2] = 0x00;
        //SEQ
        this->tx_buffer[3]++;
        //CRC8
        append_CRC8_check_sum(this->tx_buffer, 5);

        //CMD_ID_LOW
        this->tx_buffer[5] = 0x0301 & 0xff;
        //CMD_ID_HIGH
        this->tx_buffer[6] = 0x0301 >> 8;
        //DATA_CMD_ID_LOW
        this->tx_buffer[7] = data_send.data_cmd_id & 0xff;
        //DATA_CMD_ID_HIGH
        this->tx_buffer[8] = data_send.data_cmd_id >> 8;
        //SENDER_ID_LOW
        this->tx_buffer[9] = data_send.sender_id & 0xff;
        //SENDER_ID_HIGH
        this->tx_buffer[10] = data_send.sender_id >> 8;
        //RECEIVER_ID_LOW
        this->tx_buffer[11] = data_send.receiver_id & 0xff;
        //RECEIVER_ID_HIGH
        this->tx_buffer[12] = data_send.receiver_id >> 8;

        //DATA
        for (unsigned long i = 0; i < data_send.data.size(); ++i) {
            this->tx_buffer[13 + i] = data_send.data[i];
        }

        //CRC16
        append_CRC16_check_sum(this->tx_buffer, 5 + 2 + 6 + data_send.data.size() + 2);

        //SEND DATA
        for (unsigned long i = 0; i < 5 + 2 + 6 + data_send.data.size() + 2; ++i) {
            this->serial->WriteByte(this->tx_buffer[i]);
        }

    } catch (std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Caught Error: %s", e.what());
    }
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<RMReferee> rm_referee = std::make_shared<RMReferee>(rclcpp::NodeOptions());

    exe.add_node(rm_referee->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_serial::RMReferee)