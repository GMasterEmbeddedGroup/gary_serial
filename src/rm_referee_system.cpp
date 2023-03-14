#include "gary_serial/rm_referee_system.hpp"
#include "utils/rm_referee.hpp"
#include <cstring>
#include <chrono>
#include <regex>
#include <stdexcept>
#include <utility>
//#include <sys/ioctl.h>



using namespace std::chrono_literals;
using namespace gary_serial;


RMRefereeSystem::RMRefereeSystem(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "rm_referee_system", options) {

//    this->declare_parameter("send_topic", "/remote_control");
    this->declare_parameter("diagnostic_topic", "/diagnostics");
    this->declare_parameter("update_freq", 100.0f);
    this->declare_parameter("publisher_freq", 50.0f);
    this->declare_parameter("diag_freq", 10.0f);
    this->declare_parameter("serial_port", "/dev/ttyUSB114514");
//    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("override_diag_device_name", "");

    this->update_freq = 100.0f;
//    this->publisher_freq = 66.0f;
    this->publisher_freq = 0.0f;
    this->diag_freq = 10.0f;
    this->baudrate = 115200;
    this->fd = 0;
    this->is_opened = false;
    this->available_len = 0;
    this->decode_fail_cnt = 0;
    this->flag_transmission_jammed = false;
    this->data_len = 0;

    std::memset(this->frame_header, 0, sizeof(frame_header));
    std::memset(this->tail, 0, sizeof(tail));
    std::memset(&this->cmd_id, 0, sizeof(cmd_id));

    this->rmReferee = std::make_shared<utils::RMReferee>();
}

CallbackReturn RMRefereeSystem::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

//    check and create data publisher
    this->AerialRobotEnergyPublisher = this->create_publisher<gary_msgs::msg::AerialRobotEnergy>(
            "RefereeAerialRobotEnergyMSG", rclcpp::SystemDefaultsQoS());
    this->BulletRemainingPublisher = this->create_publisher<gary_msgs::msg::BulletRemaining>(
            "RefereeBulletRemainingMSG", rclcpp::SystemDefaultsQoS());
    this->ClientCommandPublisher = this->create_publisher<gary_msgs::msg::ClientCommand>(
            "RefereeClientCommandMSG", rclcpp::SystemDefaultsQoS());
    this->ClientReceivePublisher = this->create_publisher<gary_msgs::msg::ClientReceive>(
            "RefereeClientReceiveMSG", rclcpp::SystemDefaultsQoS());
    this->CustomControllerPublisher = this->create_publisher<gary_msgs::msg::CustomController>(
            "RefereeCustomControllerMSG", rclcpp::SystemDefaultsQoS());
    this->DartClientCmdPublisher = this->create_publisher<gary_msgs::msg::DartClientCmd>(
            "RefereeDartClientCmdMSG", rclcpp::SystemDefaultsQoS());
    this->DartRemainingTimePublisher = this->create_publisher<gary_msgs::msg::DartRemainingTime>(
            "RefereeDartRemainingTimeMSG", rclcpp::SystemDefaultsQoS());
    this->FieldEventsPublisher = this->create_publisher<gary_msgs::msg::FieldEvents>(
            "RefereeFieldEventsMSG", rclcpp::SystemDefaultsQoS());
    this->GameResultPublisher = this->create_publisher<gary_msgs::msg::GameResult>(
            "RefereeGameResultMSG", rclcpp::SystemDefaultsQoS());
    this->GameStatusPublisher = this->create_publisher<gary_msgs::msg::GameStatus>(
            "RefereeGameStatusMSG", rclcpp::SystemDefaultsQoS());
    this->ICRABuffDebuffZoneAndLurkStatusPublisher = this->create_publisher<gary_msgs::msg::ICRABuffDebuffZoneAndLurkStatus>(
            "RefereeICRABuffDebuffZoneAndLurkStatusMSG", rclcpp::SystemDefaultsQoS());
    this->ImageTransmitterPublisher = this->create_publisher<gary_msgs::msg::ImageTransmitter>(
            "RefereeImageTransmitterMSG", rclcpp::SystemDefaultsQoS());
    this->InteractiveDataRecvPublisher = this->create_publisher<gary_msgs::msg::InteractiveDataRecv>(
            "RefereeInteractiveDataRecvMSG", rclcpp::SystemDefaultsQoS());
    this->PowerHeatPublisher = this->create_publisher<gary_msgs::msg::PowerHeat>(
            "RefereePowerHeatMSG", rclcpp::SystemDefaultsQoS());
    this->RefereeWarningPublisher = this->create_publisher<gary_msgs::msg::RefereeWarning>(
            "RefereeRefereeWarningMSG", rclcpp::SystemDefaultsQoS());
    this->RFIDStatusPublisher = this->create_publisher<gary_msgs::msg::RFIDStatus>(
            "RefereeRFIDStatusMSG", rclcpp::SystemDefaultsQoS());
    this->RobotBuffPublisher = this->create_publisher<gary_msgs::msg::RobotBuff>(
            "RefereeRobotBuffMSG", rclcpp::SystemDefaultsQoS());
    this->RobotHPPublisher = this->create_publisher<gary_msgs::msg::RobotHP>(
            "RefereeRobotHPMSG", rclcpp::SystemDefaultsQoS());
    this->RobotHurtPublisher = this->create_publisher<gary_msgs::msg::RobotHurt>(
            "RefereeRobotHurtMSG", rclcpp::SystemDefaultsQoS());
    this->RobotPositionPublisher = this->create_publisher<gary_msgs::msg::RobotPosition>(
            "RefereeRobotPositionMSG", rclcpp::SystemDefaultsQoS());
    this->RobotStatusPublisher = this->create_publisher<gary_msgs::msg::RobotStatus>(
            "RefereeRobotStatusMSG", rclcpp::SystemDefaultsQoS());
    this->ShootDataPublisher = this->create_publisher<gary_msgs::msg::ShootData>(
            "RefereeShootDataMSG", rclcpp::SystemDefaultsQoS());
    this->SupplyProjectileActionPublisher = this->create_publisher<gary_msgs::msg::SupplyProjectileAction>(
            "RefereeSupplyProjectileActionMSG", rclcpp::SystemDefaultsQoS());
    this->SupplyProjectileRequestPublisher = this->create_publisher<gary_msgs::msg::SupplyProjectileRequest>(
            "RefereeSupplyProjectileRequestMSG", rclcpp::SystemDefaultsQoS());

    //subscriber
    this->InteractiveDataSendSubscription = this->create_subscription<gary_msgs::msg::InteractiveDataSend>("/RefereeInteractiveDataSend",10,
                                                                                     std::bind(&RMRefereeSystem::topic_callback,
                                                                                               this,std::placeholders::_1));

    //check and create diagnostic publisher
    if (this->get_parameter("diagnostic_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "diagnostic_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->diagnostic_topic = this->get_parameter("diagnostic_topic").as_string();
    this->diag_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            this->diagnostic_topic, rclcpp::SystemDefaultsQoS());

    //get update_freq
    if (this->get_parameter("update_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "update_freq type must be double");
        return CallbackReturn::FAILURE;
    }
    this->update_freq = this->get_parameter("update_freq").as_double();

    if (this->get_parameter("publisher_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "update_freq type must be double");
        return CallbackReturn::FAILURE;
    }
    this->publisher_freq = this->get_parameter("publisher_freq").as_double();

    //get diag_freq
    if (this->get_parameter("diag_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "diag_freq type must be double");
        return CallbackReturn::FAILURE;
    }
    this->diag_freq = this->get_parameter("diag_freq").as_double();

    //get serial_port
    if (this->get_parameter("serial_port").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "serial_port type must be string");
        return CallbackReturn::FAILURE;
    }
    this->serial_port = this->get_parameter("serial_port").as_string();

    //get baudrate
//    if (this->get_parameter("baudrate").get_type() != rclcpp::PARAMETER_INTEGER) {
//        RCLCPP_ERROR(this->get_logger(), "baudrate type must be integer");
//        return CallbackReturn::FAILURE;
//    }
//    this->baudrate = this->get_parameter("baudrate").as_int();

    //get override_diag_device_name
    if (this->get_parameter("override_diag_device_name").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "override_diag_device_name type must be string");
        return CallbackReturn::FAILURE;
    }
    this->override_diag_device_name = this->get_parameter("override_diag_device_name").as_string();

//    this->dr16_msg.header.frame_id = "";
    this->diag_msg.header.frame_id = "";

    diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
    if (this->override_diag_device_name.empty()) {
        //default device name, without /dev/ prefix
        diagnostic_status.hardware_id = this->serial_port.substr(5);
        diagnostic_status.name = this->serial_port.substr(5);
    } else {
        //custom device name
        diagnostic_status.hardware_id = this->override_diag_device_name;
        diagnostic_status.name = this->override_diag_device_name;
    }
    this->diag_msg.status.emplace_back(diagnostic_status);

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RMRefereeSystem::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete publisher
    this->diag_publisher.reset();
    this->timer_detect.reset();
    this->AerialRobotEnergyPublisher.reset();
    this->BulletRemainingPublisher.reset();
    this->ClientCommandPublisher.reset();
    this->ClientReceivePublisher.reset();
    this->CustomControllerPublisher.reset();
    this->DartClientCmdPublisher.reset();
    this->DartRemainingTimePublisher.reset();
    this->FieldEventsPublisher.reset();
    this->GameResultPublisher.reset();
    this->GameStatusPublisher.reset();
    this->ICRABuffDebuffZoneAndLurkStatusPublisher.reset();
    this->ImageTransmitterPublisher.reset();
    this->InteractiveDataRecvPublisher.reset();
    this->PowerHeatPublisher.reset();
    this->RefereeWarningPublisher.reset();
    this->RFIDStatusPublisher.reset();
    this->RobotBuffPublisher.reset();
    this->RobotHPPublisher.reset();
    this->RobotHurtPublisher.reset();
    this->RobotPositionPublisher.reset();
    this->RobotStatusPublisher.reset();
    this->ShootDataPublisher.reset();
    this->SupplyProjectileActionPublisher.reset();
    this->SupplyProjectileRequestPublisher.reset();


    //clear diag msg
    this->diag_msg.status.clear();

    RCLCPP_INFO(this->get_logger(), "cleaning up");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RMRefereeSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create timer
    this->timer_update = this->create_wall_timer(1000ms / this->update_freq, [this] { update(); });
    this->timer_publisher = this->create_wall_timer(1000ms / this->publisher_freq, [this] { publish_data(); });
    this->timer_diag = this->create_wall_timer(1000ms / this->diag_freq, [this] { publish_diag(); });
    this->timer_detect = this->create_wall_timer(1000ms, [this] { detect_jammed(); });
    //activate publisher
    this->diag_publisher->on_activate();
    this->AerialRobotEnergyPublisher->on_activate();
    this->BulletRemainingPublisher->on_activate();
    this->ClientCommandPublisher->on_activate();
    this->ClientReceivePublisher->on_activate();
    this->CustomControllerPublisher->on_activate();
    this->DartClientCmdPublisher->on_activate();
    this->DartRemainingTimePublisher->on_activate();
    this->FieldEventsPublisher->on_activate();
    this->GameResultPublisher->on_activate();
    this->GameStatusPublisher->on_activate();
    this->ICRABuffDebuffZoneAndLurkStatusPublisher->on_activate();
    this->ImageTransmitterPublisher->on_activate();
    this->InteractiveDataRecvPublisher->on_activate();
    this->PowerHeatPublisher->on_activate();
    this->RefereeWarningPublisher->on_activate();
    this->RFIDStatusPublisher->on_activate();
    this->RobotBuffPublisher->on_activate();
    this->RobotHPPublisher->on_activate();
    this->RobotHurtPublisher->on_activate();
    this->RobotPositionPublisher->on_activate();
    this->RobotStatusPublisher->on_activate();
    this->ShootDataPublisher->on_activate();
    this->SupplyProjectileActionPublisher->on_activate();
    this->SupplyProjectileRequestPublisher->on_activate();

    //open serial
    this->open();
    //update timestamp
    this->last_update_timestamp = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RMRefereeSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete timer
    this->timer_update.reset();
    this->timer_publisher.reset();
    this->timer_diag.reset();
    this->timer_detect.reset();
    //deactivate publisher
    this->diag_publisher->on_deactivate();
    this->AerialRobotEnergyPublisher->on_deactivate();
    this->BulletRemainingPublisher->on_deactivate();
    this->ClientCommandPublisher->on_deactivate();
    this->ClientReceivePublisher->on_deactivate();
    this->CustomControllerPublisher->on_deactivate();
    this->DartClientCmdPublisher->on_deactivate();
    this->DartRemainingTimePublisher->on_deactivate();
    this->FieldEventsPublisher->on_deactivate();
    this->GameResultPublisher->on_deactivate();
    this->GameStatusPublisher->on_deactivate();
    this->ICRABuffDebuffZoneAndLurkStatusPublisher->on_deactivate();
    this->ImageTransmitterPublisher->on_deactivate();
    this->InteractiveDataRecvPublisher->on_deactivate();
    this->PowerHeatPublisher->on_deactivate();
    this->RefereeWarningPublisher->on_deactivate();
    this->RFIDStatusPublisher->on_deactivate();
    this->RobotBuffPublisher->on_deactivate();
    this->RobotHPPublisher->on_deactivate();
    this->RobotHurtPublisher->on_deactivate();
    this->RobotPositionPublisher->on_deactivate();
    this->RobotStatusPublisher->on_deactivate();
    this->ShootDataPublisher->on_deactivate();
    this->SupplyProjectileActionPublisher->on_deactivate();
    this->SupplyProjectileRequestPublisher->on_deactivate();
    //close serial
    this->close();

    RCLCPP_INFO(this->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RMRefereeSystem::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->diag_publisher.get() != nullptr) this->diag_publisher.reset();
    if (this->timer_update.get() != nullptr) this->timer_update.reset();
    if (this->timer_publisher.get() != nullptr) this->timer_publisher.reset();
    if (this->timer_diag.get() != nullptr) this->timer_diag.reset();
    if (this->timer_detect.get() != nullptr) this->timer_detect.reset();
    if (this->AerialRobotEnergyPublisher.get() != nullptr) this->AerialRobotEnergyPublisher.reset();
    if (this->BulletRemainingPublisher.get() != nullptr) this->BulletRemainingPublisher.reset();
    if (this->ClientCommandPublisher.get() != nullptr) this->ClientCommandPublisher.reset();
    if (this->ClientReceivePublisher.get() != nullptr) this->ClientReceivePublisher.reset();
    if (this->CustomControllerPublisher.get() != nullptr) this->CustomControllerPublisher.reset();
    if (this->DartClientCmdPublisher.get() != nullptr) this->DartClientCmdPublisher.reset();
    if (this->DartRemainingTimePublisher.get() != nullptr) this->DartRemainingTimePublisher.reset();
    if (this->FieldEventsPublisher.get() != nullptr) this->FieldEventsPublisher.reset();
    if (this->GameResultPublisher.get() != nullptr) this->GameResultPublisher.reset();
    if (this->GameStatusPublisher.get() != nullptr) this->GameStatusPublisher.reset();
    if (this->ICRABuffDebuffZoneAndLurkStatusPublisher.get() != nullptr)
        this->ICRABuffDebuffZoneAndLurkStatusPublisher.reset();
    if (this->ImageTransmitterPublisher.get() != nullptr) this->ImageTransmitterPublisher.reset();
    if (this->InteractiveDataRecvPublisher.get() != nullptr) this->InteractiveDataRecvPublisher.reset();
    if (this->PowerHeatPublisher.get() != nullptr) this->PowerHeatPublisher.reset();
    if (this->RefereeWarningPublisher.get() != nullptr) this->RefereeWarningPublisher.reset();
    if (this->RFIDStatusPublisher.get() != nullptr) this->RFIDStatusPublisher.reset();
    if (this->RobotBuffPublisher.get() != nullptr) this->RobotBuffPublisher.reset();
    if (this->RobotHPPublisher.get() != nullptr) this->RobotHPPublisher.reset();
    if (this->RobotHurtPublisher.get() != nullptr) this->RobotHurtPublisher.reset();
    if (this->RobotPositionPublisher.get() != nullptr) this->RobotPositionPublisher.reset();
    if (this->RobotStatusPublisher.get() != nullptr) this->RobotStatusPublisher.reset();
    if (this->ShootDataPublisher.get() != nullptr) this->ShootDataPublisher.reset();
    if (this->SupplyProjectileActionPublisher.get() != nullptr) this->SupplyProjectileActionPublisher.reset();
    if (this->SupplyProjectileRequestPublisher.get() != nullptr) this->SupplyProjectileRequestPublisher.reset();

    this->close();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RMRefereeSystem::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

//    if (this->msg_publisher.get() != nullptr) this->msg_publisher.reset();
    if (this->diag_publisher.get() != nullptr) this->diag_publisher.reset();
    if (this->timer_update.get() != nullptr) this->timer_update.reset();
    if (this->timer_publisher.get() != nullptr) this->timer_publisher.reset();
    if (this->timer_diag.get() != nullptr) this->timer_diag.reset();
    if (this->timer_detect.get() != nullptr) this->timer_detect.reset();
    if (this->AerialRobotEnergyPublisher.get() != nullptr) this->AerialRobotEnergyPublisher.reset();
    if (this->BulletRemainingPublisher.get() != nullptr) this->BulletRemainingPublisher.reset();
    if (this->ClientCommandPublisher.get() != nullptr) this->ClientCommandPublisher.reset();
    if (this->ClientReceivePublisher.get() != nullptr) this->ClientReceivePublisher.reset();
    if (this->CustomControllerPublisher.get() != nullptr) this->CustomControllerPublisher.reset();
    if (this->DartClientCmdPublisher.get() != nullptr) this->DartClientCmdPublisher.reset();
    if (this->DartRemainingTimePublisher.get() != nullptr) this->DartRemainingTimePublisher.reset();
    if (this->FieldEventsPublisher.get() != nullptr) this->FieldEventsPublisher.reset();
    if (this->GameResultPublisher.get() != nullptr) this->GameResultPublisher.reset();
    if (this->GameStatusPublisher.get() != nullptr) this->GameStatusPublisher.reset();
    if (this->ICRABuffDebuffZoneAndLurkStatusPublisher.get() != nullptr)
        this->ICRABuffDebuffZoneAndLurkStatusPublisher.reset();
    if (this->ImageTransmitterPublisher.get() != nullptr) this->ImageTransmitterPublisher.reset();
    if (this->InteractiveDataRecvPublisher.get() != nullptr) this->InteractiveDataRecvPublisher.reset();
    if (this->PowerHeatPublisher.get() != nullptr) this->PowerHeatPublisher.reset();
    if (this->RefereeWarningPublisher.get() != nullptr) this->RefereeWarningPublisher.reset();
    if (this->RFIDStatusPublisher.get() != nullptr) this->RFIDStatusPublisher.reset();
    if (this->RobotBuffPublisher.get() != nullptr) this->RobotBuffPublisher.reset();
    if (this->RobotHPPublisher.get() != nullptr) this->RobotHPPublisher.reset();
    if (this->RobotHurtPublisher.get() != nullptr) this->RobotHurtPublisher.reset();
    if (this->RobotPositionPublisher.get() != nullptr) this->RobotPositionPublisher.reset();
    if (this->RobotStatusPublisher.get() != nullptr) this->RobotStatusPublisher.reset();
    if (this->ShootDataPublisher.get() != nullptr) this->ShootDataPublisher.reset();
    if (this->SupplyProjectileActionPublisher.get() != nullptr) this->SupplyProjectileActionPublisher.reset();
    if (this->SupplyProjectileRequestPublisher.get() != nullptr) this->SupplyProjectileRequestPublisher.reset();
    this->close();

    RCLCPP_ERROR(this->get_logger(), "Error happened");
    return CallbackReturn::SUCCESS;
}

bool RMRefereeSystem::open() {
    int ret;

    if (this->is_opened) this->close();

    using namespace LibSerial;
    this->refereeSerialStream = std::make_shared<SerialStream>();
    refereeSerialStream->SetBaudRate(BaudRate::BAUD_115200);
    refereeSerialStream->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    refereeSerialStream->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    refereeSerialStream->SetStopBits(StopBits::STOP_BITS_1);
    refereeSerialStream->SetParity(Parity::PARITY_NONE);

    refereeSerialStream->Open(serial_port);

    if (!refereeSerialStream->IsOpen()) return false;
    else is_opened = true;
    RCLCPP_INFO(this->get_logger(), "Serial %s opened", serial_port.c_str());
    return true;
}

void RMRefereeSystem::close() {
    if (this->is_opened) refereeSerialStream->Close();
    this->is_opened = false;
}

bool RMRefereeSystem::get_available_len() {

    //check serial open
//    if (!is_opened) return false;
//
//    int ret;
//    int len;
//
//    //get available len
//    ret = ioctl(this->fd, FIONREAD, &len, sizeof(len));
//
//    //failed to get
//    if (ret < 0) {
//        RCLCPP_DEBUG(this->get_logger(), "failed to get buffer available length");
//        return false;
//    }
//
//    //check serial disconnect
//    if (errno == 5) {
//        RCLCPP_DEBUG(this->get_logger(), "serial port disconnect");
//        errno = 0;
//        return false;
//    }
//
//    this->available_len = len;
    return true;
}

bool RMRefereeSystem::read() {

    //check serial open
    try {
        if (!is_opened) {
            this->open();
        }
    } catch (LibSerial::AlreadyOpen::exception &e) {
        RCLCPP_DEBUG(this->get_logger(), "Serial Already Open But tried to reopen: %s;", e.what());
    } catch (LibSerial::OpenFailed::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial: %s;", e.what());
        return false;
    }

//    get available length
    try {
        if (!refereeSerialStream->IsDataAvailable()) {
            RCLCPP_DEBUG(this->get_logger(), "failed to get available length");
        }
    } catch (LibSerial::NotOpen::exception &e) {
        RCLCPP_DEBUG(this->get_logger(), "Serial Not Open: %s;", e.what());
    } catch (std::exception &e) {
        RCLCPP_DEBUG(this->get_logger(), "WHAT= %s;", e.what());
        return false;
    }

    // get header
    bool haveData = false;
    try {
        haveData = (refereeSerialStream->GetNumberOfBytesAvailable() > 0);
    } catch (LibSerial::NotOpen::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "NO DATA AVAILABLE: Serial NOT OPEN: %s;", e.what());
        return false;
    } catch (LibSerial::ReadTimeout::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "NO DATA AVAILABLE: Read Time Out: %s;", e.what());
    }

    //get all package and solve
    while (rclcpp::ok() && haveData) {
        try {
            haveData = (refereeSerialStream->GetNumberOfBytesAvailable() > 0);
        } catch (LibSerial::NotOpen::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "NO DATA AVAILABLE: Serial NOT OPEN: %s;", e.what());
            return false;
        } catch (LibSerial::ReadTimeout::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "NO DATA AVAILABLE: Read Time Out: %s;", e.what());
        }
        try {
            //look for SOF
            uint8_t next_byte = 0x00;
            uint16_t cnt = this->refereeSerialStream->GetNumberOfBytesAvailable();
            RCLCPP_DEBUG(this->get_logger(), "available length: %d", cnt);
            bool flag_SOF = false;
            if (cnt == 4095) {
                try {
                    refereeSerialStream->read(reinterpret_cast<char *>(&next_byte), 1);
                    RCLCPP_DEBUG(this->get_logger(), "ERROR 4095 Happened! SOF: %x;", next_byte);
                    this->close();
                    this->open();
                } catch (LibSerial::AlreadyOpen::exception &e) {
                    RCLCPP_DEBUG(this->get_logger(), "Serial Already Open But tried to reopen: %s;", e.what());
                } catch (LibSerial::OpenFailed::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open serial: %s;", e.what());
                    return false;
                }
                continue;
            }
//            while (refereeSerialStream->GetNumberOfBytesAvailable() > 0) {
            while (cnt--) {
                refereeSerialStream->read(reinterpret_cast<char *>(&next_byte), 1);
                RCLCPP_DEBUG(this->get_logger(), "Looking for SOF, Current: %x;", next_byte);
                if (next_byte == utils::_ns_rm_referee::HEADER_SOF) {
                    flag_SOF = true;
                    break;
                }
            }
            if (!flag_SOF) {
                RCLCPP_DEBUG(this->get_logger(), "No SOF found yet.");
                continue;
//                return false;
            } else {
                RCLCPP_DEBUG(this->get_logger(), "SOF found. Solving Data.");
            }
            this->frame_header[0] = 0xA5;
            uint16_t index = 0;

            //available length less than a header
            if (refereeSerialStream->GetNumberOfBytesAvailable() < 4) {

                //sleep 2ms and retry
                RCLCPP_DEBUG(this->get_logger(), "available length less than a header, sleep 2ms and retry");
                rclcpp::sleep_for(2ms);

                if (!refereeSerialStream->IsDataAvailable()) {
//                    this->close();
                    throw UpdateExcept("failed to get available length");
                }
                //still less than a header
                if (refereeSerialStream->GetNumberOfBytesAvailable() < 4) {
                    RCLCPP_DEBUG(this->get_logger(), "waited 2ms but still less than header");
                    return false;
                }
            }

            refereeSerialStream->read(reinterpret_cast<char *>(this->frame_header + 1), 4);
            this->data_len = this->rmReferee->solveDataLength(this->frame_header);
            std::memcpy(this->data + index, this->frame_header, utils::_ns_rm_referee::REF_PROTOCOL_HEADER_SIZE);
            index += utils::_ns_rm_referee::REF_PROTOCOL_HEADER_SIZE;

            //available length less than a packet
            if (refereeSerialStream->GetNumberOfBytesAvailable() < data_len) {

                //sleep 2ms and retry
                RCLCPP_DEBUG(this->get_logger(), "available length less than a packet, sleep 2ms and retry");
                rclcpp::sleep_for(2ms);

                if (!refereeSerialStream->IsDataAvailable()) {
//                    this->close();
                    throw UpdateExcept("failed to get available length");
                }
                //still less than a packet
                if (refereeSerialStream->GetNumberOfBytesAvailable() < data_len) {
                    RCLCPP_DEBUG(this->get_logger(), "waited 2ms but still less than packet");
                    return false;
                }
            }

            refereeSerialStream->read(reinterpret_cast<char *>(this->cmd_id.cmdid_uint8), 2);
            std::memcpy(this->data + index, this->cmd_id.cmdid_uint8, sizeof(this->cmd_id));
            index += sizeof(cmd_id);

            refereeSerialStream->read(reinterpret_cast<char *>(this->data + index), this->data_len);
            index += data_len;

            refereeSerialStream->read(reinterpret_cast<char *>(this->data + index),
                                      utils::_ns_rm_referee::REF_PROTOCOL_CRC16_SIZE);

            if (cmd_id.cmd_id_uint16 != this->rmReferee->solveData(data)) {
                RCLCPP_DEBUG(this->get_logger(), "THIS_ID=%d,THAT_ID=%d", cmd_id.cmd_id_uint16,
                             this->rmReferee->solveData(data));
                throw UpdateExcept("UNKNOWN_ERROR (GOT DIFFERENT CMD_ID WHILE CALCULATING)");
            }
            using namespace utils::_ns_rm_referee;
            using namespace gary_msgs::msg;
            switch (cmd_id.cmdid_t) {
                case AERIAL_ROBOT_ENERGY_CMD_ID: {
                    this->AerialRobotEnergyMsg.header.stamp = this->get_clock()->now();
                    this->AerialRobotEnergyMsg.attack_time = this->rmReferee->get_aerialRobotEnergy().attack_time;
//                auto msg = std::make_pair(AerialRobotEnergyPriority,
//                                          std::make_pair(cmd_id.cmdid_t, this->AerialRobotEnergyMsg));
//                topicQueue.emplace(msg);
                    AerialRobotEnergyPublisher->publish(AerialRobotEnergyMsg);
                    break;
                }
                case BULLET_REMAINING_CMD_ID: {
                    this->BulletRemainingMsg.header.stamp = this->get_clock()->now();
                    BulletRemainingMsg.remaining_17mm_num = this->rmReferee->get_bulletRemaining().bullet_remaining_num_17mm;
                    BulletRemainingMsg.remaining_42mm_num = this->rmReferee->get_bulletRemaining().bullet_remaining_num_42mm;
                    BulletRemainingMsg.remaining_coin_num = this->rmReferee->get_bulletRemaining().coin_remaining_num;
//                auto msg = std::make_pair(BulletRemainingPriority, std::make_pair(cmd_id.cmdid_t, BulletRemainingMsg));
//                topicQueue.emplace(msg);
                    BulletRemainingPublisher->publish(BulletRemainingMsg);
                    break;
                }
                case MAP_INTERACTIVE_DATA_CDM_ID: {
                    this->ClientCommandMsg.header.stamp = this->get_clock()->now();
                    ClientCommandMsg.keyboard_key_pressed = this->rmReferee->get_robotCommand().command_keyboard;
                    ClientCommandMsg.target_position_x = this->rmReferee->get_robotCommand().target_position_x;
                    ClientCommandMsg.target_position_y = this->rmReferee->get_robotCommand().target_position_y;
                    ClientCommandMsg.target_position_z = this->rmReferee->get_robotCommand().target_position_z;
                    ClientCommandMsg.target_robot_id = this->rmReferee->get_robotCommand().target_robot_ID;
//                auto msg = std::make_pair(ClientCommandPriority, std::make_pair(cmd_id.cmdid_t, ClientCommandMsg));
//                topicQueue.emplace(msg);
                    ClientCommandPublisher->publish(ClientCommandMsg);
                    break;
                }
                case MAP_RECEIVE_INFO: {
                    this->ClientReceiveMsg.header.stamp = this->get_clock()->now();
                    ClientReceiveMsg.target_robot_id = this->rmReferee->get_mapCommand().target_robot_ID;
                    ClientReceiveMsg.target_position_x = this->rmReferee->get_mapCommand().target_position_x;
                    ClientReceiveMsg.target_position_y = this->rmReferee->get_mapCommand().target_position_y;
//                auto msg = std::make_pair(ClientReceivePriority, std::make_pair(cmd_id.cmdid_t, ClientReceiveMsg));
//                topicQueue.emplace(msg);
                    ClientReceivePublisher->publish(ClientReceiveMsg);
                    break;
                }
                case CONTROLLER_INTERACTIVE_DATA_CMD_ID: {
                    this->CustomControllerMsg.header.stamp = this->get_clock()->now();
                    CustomControllerMsg.data_length = this->rmReferee->getCustomeDataLen();
                    std::vector<uint8_t> v(this->rmReferee->get_customControllerData().data,
                                           this->rmReferee->get_customControllerData().data +
                                           CustomControllerMsg.data_length);
                    CustomControllerMsg.data = v;
//                auto msg = std::make_pair(CustomControllerPriority,
//                                          std::make_pair(cmd_id.cmdid_t, CustomControllerMsg));
//                topicQueue.emplace(msg);
                    CustomControllerPublisher->publish(CustomControllerMsg);
                    break;
                }
                case DART_CLIENT_COMMAND_CMD_ID: {
                    this->DartClientCmdMsg.header.stamp = this->get_clock()->now();
                    DartClientCmdMsg.attack_target = this->rmReferee->get_dartClientCmd().dart_attack_target;
                    DartClientCmdMsg.launch_status = this->rmReferee->get_dartClientCmd().dart_launch_opening_status;
                    DartClientCmdMsg.operate_launch_cmd_time = this->rmReferee->get_dartClientCmd().operate_launch_cmd_time;
                    DartClientCmdMsg.target_change_time = this->rmReferee->get_dartClientCmd().target_change_time;
//                auto msg = std::make_pair(DartClientCmdPriority, std::make_pair(cmd_id.cmdid_t, DartClientCmdMsg));
//                topicQueue.emplace(msg);
                    DartClientCmdPublisher->publish(DartClientCmdMsg);
                    break;
                }
                case DART_COUNTDOWN_CMD_ID: {
                    this->DartRemainingTimeMsg.header.stamp = this->get_clock()->now();
                    DartRemainingTimeMsg.dart_remaining_time = this->rmReferee->get_dartRemainingTime().dart_remaining_time;
//                auto msg = std::make_pair(DartRemainingTimePriority,
//                                          std::make_pair(cmd_id.cmdid_t, DartRemainingTimeMsg));
//                topicQueue.emplace(msg);
                    DartRemainingTimePublisher->publish(DartRemainingTimeMsg);
                    break;
                }
                case FIELD_EVENTS_CMD_ID: {
                    this->FieldEventsMsg.header.stamp = this->get_clock()->now();
                    FieldEventsMsg.base_has_shield = this->rmReferee->get_eventData().base_shield_exist;
                    FieldEventsMsg.big_power_rune_activation_status = this->rmReferee->get_eventData().big_buff;
                    FieldEventsMsg.outpost_alive = this->rmReferee->get_eventData().tower_alive;
                    FieldEventsMsg.power_rune_activation_point_occupation = this->rmReferee->get_eventData().hit_point;
                    FieldEventsMsg.r2b2_ground_occupation = this->rmReferee->get_eventData().highland_2_occupied;
                    FieldEventsMsg.r3b3_ground_occupation = this->rmReferee->get_eventData().highland_3_occupied;
                    FieldEventsMsg.r4b4_ground_occupation = this->rmReferee->get_eventData().highland_4_occupied;
                    FieldEventsMsg.small_power_rune_activation_status = this->rmReferee->get_eventData().small_buff;
                    FieldEventsMsg.supplier_1_occupation = this->rmReferee->get_eventData().heal_point_1;
                    FieldEventsMsg.supplier_2_occupation = this->rmReferee->get_eventData().heal_point_2;
                    FieldEventsMsg.supplier_3_occupation = this->rmReferee->get_eventData().heal_point_3;
//                auto msg = std::make_pair(FieldEventsPriority, std::make_pair(cmd_id.cmdid_t, FieldEventsMsg));
//                topicQueue.emplace(msg);
                    FieldEventsPublisher->publish(FieldEventsMsg);
                    break;
                }
                case GAME_RESULT_CMD_ID: {
                    this->GameResultMsg.header.stamp = this->get_clock()->now();
                    GameResultMsg.winner = this->rmReferee->get_gameResult().winner;
//                auto msg = std::make_pair(GameResultPriority, std::make_pair(cmd_id.cmdid_t, GameResultMsg));
//                topicQueue.emplace(msg);
                    GameResultPublisher->publish(GameResultMsg);
                    break;
                }
                case GAME_STATE_CMD_ID: {
                    this->GameStatusMsg.header.stamp = this->get_clock()->now();
                    GameStatusMsg.game_progress = this->rmReferee->get_gameState().game_progress;
                    GameStatusMsg.game_type = this->rmReferee->get_gameState().game_type;
                    GameStatusMsg.stage_remain_time = this->rmReferee->get_gameState().stage_remain_time;
                    GameStatusMsg.sync_time_stamp = this->rmReferee->get_gameState().UNIX_time_map;
//                auto msg = std::make_pair(GameStatusPriority, std::make_pair(cmd_id.cmdid_t, GameStatusMsg));
//                topicQueue.emplace(msg);
                    GameStatusPublisher->publish(GameStatusMsg);
                    break;
                }
                case AI_STATE_CMD_ID: {
                    this->ICRABuffDebuffZoneAndLurkStatusMsg.header.stamp = this->get_clock()->now();
                    ICRABuffDebuffZoneAndLurkStatusMsg.f1_zone_buff_debuff_status = this->rmReferee->get_buffZoneAndLurkStatus().F1_zone_buff_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f1_zone_status = this->rmReferee->get_buffZoneAndLurkStatus().F1_zone_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f2_zone_buff_debuff_status = this->rmReferee->get_buffZoneAndLurkStatus().F2_zone_buff_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f2_zone_status = this->rmReferee->get_buffZoneAndLurkStatus().F2_zone_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f3_zone_buff_debuff_status = this->rmReferee->get_buffZoneAndLurkStatus().F3_zone_buff_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f3_zone_status = this->rmReferee->get_buffZoneAndLurkStatus().F3_zone_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f4_zone_buff_debuff_status = this->rmReferee->get_buffZoneAndLurkStatus().F4_zone_buff_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f4_zone_status = this->rmReferee->get_buffZoneAndLurkStatus().F4_zone_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f5_zone_buff_debuff_status = this->rmReferee->get_buffZoneAndLurkStatus().F5_zone_buff_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f5_zone_status = this->rmReferee->get_buffZoneAndLurkStatus().F5_zone_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f6_zone_buff_debuff_status = this->rmReferee->get_buffZoneAndLurkStatus().F6_zone_buff_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.f6_zone_status = this->rmReferee->get_buffZoneAndLurkStatus().F6_zone_status;
                    ICRABuffDebuffZoneAndLurkStatusMsg.lurk_mode = this->rmReferee->get_buffZoneAndLurkStatus().lurk_mode;
                    ICRABuffDebuffZoneAndLurkStatusMsg.blue1_bullet_left = this->rmReferee->get_buffZoneAndLurkStatus().blue1_bullet_left;
                    ICRABuffDebuffZoneAndLurkStatusMsg.blue2_bullet_left = this->rmReferee->get_buffZoneAndLurkStatus().blue2_bullet_left;
                    ICRABuffDebuffZoneAndLurkStatusMsg.red1_bullet_left = this->rmReferee->get_buffZoneAndLurkStatus().red1_bullet_left;
                    ICRABuffDebuffZoneAndLurkStatusMsg.red2_bullet_left = this->rmReferee->get_buffZoneAndLurkStatus().red2_bullet_left;
//                auto msg = std::make_pair(ICRABuffDebuffZoneAndLurkStatusPriority,
//                                          std::make_pair(cmd_id.cmdid_t, ICRABuffDebuffZoneAndLurkStatusMsg));
//                topicQueue.emplace(msg);
                    ICRABuffDebuffZoneAndLurkStatusPublisher->publish(ICRABuffDebuffZoneAndLurkStatusMsg);
                    break;
                }
                case KEYBOARD_AND_MOUSE_INFO_CMD_ID: {
                    this->ImageTransmitterMsg.header.stamp = this->get_clock()->now();
                    ImageTransmitterMsg.mouse_x = this->rmReferee->get_keyboardCommand().mouse_x;
                    ImageTransmitterMsg.mouse_y = this->rmReferee->get_keyboardCommand().mouse_y;
                    ImageTransmitterMsg.mouse_z = this->rmReferee->get_keyboardCommand().mouse_z;
                    ImageTransmitterMsg.mouse_press_l = this->rmReferee->get_keyboardCommand().left_button_down;
                    ImageTransmitterMsg.mouse_press_r = this->rmReferee->get_keyboardCommand().right_button_down;
                    ImageTransmitterMsg.key_w = this->rmReferee->get_keyboardCommand().key_w;
                    ImageTransmitterMsg.key_s = this->rmReferee->get_keyboardCommand().key_s;
                    ImageTransmitterMsg.key_a = this->rmReferee->get_keyboardCommand().key_a;
                    ImageTransmitterMsg.key_d = this->rmReferee->get_keyboardCommand().key_d;
                    ImageTransmitterMsg.key_shift = this->rmReferee->get_keyboardCommand().key_shift;
                    ImageTransmitterMsg.key_ctrl = this->rmReferee->get_keyboardCommand().key_ctrl;
                    ImageTransmitterMsg.key_q = this->rmReferee->get_keyboardCommand().key_q;
                    ImageTransmitterMsg.key_e = this->rmReferee->get_keyboardCommand().key_e;
                    ImageTransmitterMsg.key_r = this->rmReferee->get_keyboardCommand().key_r;
                    ImageTransmitterMsg.key_f = this->rmReferee->get_keyboardCommand().key_f;
                    ImageTransmitterMsg.key_g = this->rmReferee->get_keyboardCommand().key_g;
                    ImageTransmitterMsg.key_z = this->rmReferee->get_keyboardCommand().key_z;
                    ImageTransmitterMsg.key_x = this->rmReferee->get_keyboardCommand().key_x;
                    ImageTransmitterMsg.key_c = this->rmReferee->get_keyboardCommand().key_c;
                    ImageTransmitterMsg.key_v = this->rmReferee->get_keyboardCommand().key_v;
                    ImageTransmitterMsg.key_b = this->rmReferee->get_keyboardCommand().key_b;
//                auto msg = std::make_pair(ImageTransmitterPriority,
//                                          std::make_pair(cmd_id.cmdid_t, ImageTransmitterMsg));
//                topicQueue.emplace(msg);
                    ImageTransmitterPublisher->publish(ImageTransmitterMsg);
                    break;
                }
                case ROBOT_INTERACTIVE_DATA_CMD_ID: {
                    this->InteractiveDataRecvMsg.header.stamp = this->get_clock()->now();
                    InteractiveDataRecvMsg.data_length = this->rmReferee->getCustomeDataLen();
                    std::vector<uint8_t> v(this->rmReferee->get_robotInteractiveDataRecv().data,
                                           this->rmReferee->get_robotInteractiveDataRecv().data +
                                           InteractiveDataRecvMsg.data_length);
                    InteractiveDataRecvMsg.data = v;
                    InteractiveDataRecvMsg.data_cmd_id = this->rmReferee->get_studentInteractiveHeaderData().data_cmd_id;
                    InteractiveDataRecvMsg.receiver_id = this->rmReferee->get_studentInteractiveHeaderData().receiver_ID;
                    InteractiveDataRecvMsg.sender_id = this->rmReferee->get_studentInteractiveHeaderData().sender_ID;
//                auto msg = std::make_pair(InteractiveDataRecvPriority, std::make_pair(cmd_id.cmdid_t, InteractiveDataRecvMsg));
//                topicQueue.emplace(msg);
                    InteractiveDataRecvPublisher->publish(InteractiveDataRecvMsg);
                    break;
                }
                case POWER_HEAT_DATA_CMD_ID: {
                    this->PowerHeatMsg.header.stamp = this->get_clock()->now();
                    PowerHeatMsg.chassis_current =
                            static_cast<float>(this->rmReferee->get_powerHeatData().chassis_current) / 1000;
                    PowerHeatMsg.chassis_power = this->rmReferee->get_powerHeatData().chassis_power;
                    PowerHeatMsg.chassis_power_buffer = this->rmReferee->get_powerHeatData().chassis_power_buffer;
                    PowerHeatMsg.chassis_volt =
                            static_cast<float>(this->rmReferee->get_powerHeatData().chassis_volt) / 1000;
                    PowerHeatMsg.shooter_17mm_id1_heat = this->rmReferee->get_powerHeatData().shooter_id1_17mm_cooling_heat;
                    PowerHeatMsg.shooter_17mm_id2_heat = this->rmReferee->get_powerHeatData().shooter_id2_17mm_cooling_heat;
                    PowerHeatMsg.shooter_42mm_id1_heat = this->rmReferee->get_powerHeatData().shooter_id1_42mm_cooling_heat;
                    PowerHeatPublisher->publish(PowerHeatMsg);
                    break;
                }
                case REFEREE_WARNING_CMD_ID: {
                    this->RefereeWarningMsg.header.stamp = this->get_clock()->now();
                    RefereeWarningMsg.foul_robot_id = this->rmReferee->get_refereeWarning().foul_robot_id;
                    RefereeWarningMsg.level = this->rmReferee->get_refereeWarning().level;
//                auto msg = std::make_pair(RefereeWarningPriority, std::make_pair(cmd_id.cmdid_t, RefereeWarningMsg));
//                topicQueue.emplace(msg);
                    RefereeWarningPublisher->publish(RefereeWarningMsg);
                    break;
                }
                case RFID_STATE_CMD_ID: {
                    this->RFIDStatusMsg.header.stamp = this->get_clock()->now();
                    RFIDStatusMsg.base_buff_point = this->rmReferee->get_rfidStatus().base_state;
                    RFIDStatusMsg.power_rune_buff_point = this->rmReferee->get_rfidStatus().buff_state;
                    RFIDStatusMsg.elevated_ground_buff_point = this->rmReferee->get_rfidStatus().highland_state;
                    RFIDStatusMsg.engineer_robot_revival_card = this->rmReferee->get_rfidStatus().restore_card_state;
                    RFIDStatusMsg.hp_recovery_buff_point = this->rmReferee->get_rfidStatus().healing_state;
                    RFIDStatusMsg.launch_ramp_buff_point = this->rmReferee->get_rfidStatus().slope_state;
                    RFIDStatusMsg.outpost_buff_point = this->rmReferee->get_rfidStatus().tower_state;
                    RFIDStatusPublisher->publish(RFIDStatusMsg);
                    break;
                }
                case BUFF_MUSK_CMD_ID: {
                    this->RobotBuffMsg.header.stamp = this->get_clock()->now();
                    RobotBuffMsg.robot_attack_bonus = this->rmReferee->get_buffMusk().attack_buff;
                    RobotBuffMsg.robot_defense_bonus = this->rmReferee->get_buffMusk().defence_buff;
                    RobotBuffMsg.robot_replenishing_blood = this->rmReferee->get_buffMusk().healing;
                    RobotBuffMsg.shooter_cooling_acceleration = this->rmReferee->get_buffMusk().cooling_buff;
                    RobotBuffPublisher->publish(RobotBuffMsg);
                    break;
                }
                case GAME_ROBOT_HP_CMD_ID: {
                    this->RobotHPMsg.header.stamp = this->get_clock()->now();
                    RobotHPMsg.blue_1_hero_hp = this->rmReferee->get_robotHP().blue_1_robot_HP;
                    RobotHPMsg.blue_2_engineer_hp = this->rmReferee->get_robotHP().blue_2_robot_HP;
                    RobotHPMsg.blue_3_infantry_hp = this->rmReferee->get_robotHP().blue_3_robot_HP;
                    RobotHPMsg.blue_4_infantry_hp = this->rmReferee->get_robotHP().blue_4_robot_HP;
                    RobotHPMsg.blue_5_infantry_hp = this->rmReferee->get_robotHP().blue_5_robot_HP;
                    RobotHPMsg.blue_7_sentry_hp = this->rmReferee->get_robotHP().blue_7_robot_HP;
                    RobotHPMsg.red_1_hero_hp = this->rmReferee->get_robotHP().red_1_robot_HP;
                    RobotHPMsg.red_2_engineer_hp = this->rmReferee->get_robotHP().red_2_robot_HP;
                    RobotHPMsg.red_3_infantry_hp = this->rmReferee->get_robotHP().red_3_robot_HP;
                    RobotHPMsg.red_4_infantry_hp = this->rmReferee->get_robotHP().red_4_robot_HP;
                    RobotHPMsg.red_5_infantry_hp = this->rmReferee->get_robotHP().red_5_robot_HP;
                    RobotHPMsg.red_7_sentry_hp = this->rmReferee->get_robotHP().red_7_robot_HP;
                    RobotHPMsg.blue_base_hp = this->rmReferee->get_robotHP().blue_base_HP;
                    RobotHPMsg.red_base_hp = this->rmReferee->get_robotHP().red_base_HP;
                    RobotHPMsg.blue_outpost_hp = this->rmReferee->get_robotHP().blue_tower_HP;
                    RobotHPMsg.red_outpost_hp = this->rmReferee->get_robotHP().red_tower_HP;
//                auto msg = std::make_pair(RobotHPPriority, std::make_pair(cmd_id.cmdid_t, RobotHPMsg));
//                topicQueue.emplace(msg);
                    RobotHPPublisher->publish(RobotHPMsg);
                    break;
                }
                case ROBOT_HURT_CMD_ID: {
                    this->RobotHurtMsg.header.stamp = this->get_clock()->now();
                    RobotHurtMsg.armor_id = this->rmReferee->get_robotHurt().armor_type;
                    RobotHurtMsg.hurt_type = this->rmReferee->get_robotHurt().hurt_type;
//                auto msg = std::make_pair(RobotHurtPriority, std::make_pair(cmd_id.cmdid_t, RobotHurtMsg));
//                topicQueue.emplace(msg);
                    RobotHurtPublisher->publish(RobotHurtMsg);
                    break;
                }
                case ROBOT_POS_CMD_ID: {
                    this->RobotPositionMsg.header.stamp = this->get_clock()->now();
                    RobotPositionMsg.x = this->rmReferee->get_robotPos().x;
                    RobotPositionMsg.y = this->rmReferee->get_robotPos().y;
                    RobotPositionMsg.z = this->rmReferee->get_robotPos().z;
                    RobotPositionMsg.yaw = this->rmReferee->get_robotPos().yaw;
//                auto msg = std::make_pair(RobotPositionPriority, std::make_pair(cmd_id.cmdid_t, RobotPositionMsg));
//                topicQueue.emplace(msg);
                    RobotPositionPublisher->publish(RobotPositionMsg);
                    break;
                }
                case ROBOT_STATE_CMD_ID: {
                    this->RobotStatusMsg.header.stamp = this->get_clock()->now();
                    RobotStatusMsg.chassis_power_limit = this->rmReferee->get_robotState().chassis_power_limit;
                    RobotStatusMsg.chassis_power_output = this->rmReferee->get_robotState().mains_power_chassis_output;
                    RobotStatusMsg.gimbal_power_output = this->rmReferee->get_robotState().mains_power_gimbal_output;
                    RobotStatusMsg.max_hp = this->rmReferee->get_robotState().max_HP;
                    RobotStatusMsg.remain_hp = this->rmReferee->get_robotState().remain_HP;
                    RobotStatusMsg.robot_id = this->rmReferee->get_robotState().robot_id;
                    RobotStatusMsg.robot_level = this->rmReferee->get_robotState().robot_level;
                    RobotStatusMsg.shooter_17mm_id1_cooling_limit = this->rmReferee->get_robotState().shooter_id1_17mm_heat_limit;
                    RobotStatusMsg.shooter_17mm_id1_cooling_rate = this->rmReferee->get_robotState().shooter_id1_17mm_cooling_rate;
                    RobotStatusMsg.shooter_17mm_id1_speed_limit = this->rmReferee->get_robotState().shooter_id1_17mm_speed_limit;
                    RobotStatusMsg.shooter_17mm_id2_cooling_limit = this->rmReferee->get_robotState().shooter_id2_17mm_heat_limit;
                    RobotStatusMsg.shooter_17mm_id2_cooling_rate = this->rmReferee->get_robotState().shooter_id2_17mm_cooling_rate;
                    RobotStatusMsg.shooter_17mm_id2_speed_limit = this->rmReferee->get_robotState().shooter_id2_17mm_speed_limit;
                    RobotStatusMsg.shooter_42mm_id1_cooling_limit = this->rmReferee->get_robotState().shooter_42mm_heat_limit;
                    RobotStatusMsg.shooter_42mm_id1_cooling_rate = this->rmReferee->get_robotState().shooter_42mm_cooling_rate;
                    RobotStatusMsg.shooter_42mm_id1_speed_limit = this->rmReferee->get_robotState().shooter_42mm_speed_limit;
                    RobotStatusMsg.shooter_power_output = this->rmReferee->get_robotState().mains_power_shooter_output;
                    RobotStatusPublisher->publish(RobotStatusMsg);
                    break;
                }
                case SHOOT_DATA_CMD_ID: {
                    this->ShootDataMsg.header.stamp = this->get_clock()->now();
                    ShootDataMsg.shooter_id = this->rmReferee->get_shootData().shooter_id;
                    ShootDataMsg.bullet_freq = this->rmReferee->get_shootData().bullet_freq;
                    ShootDataMsg.bullet_speed = this->rmReferee->get_shootData().bullet_speed;
                    ShootDataMsg.bullet_type = this->rmReferee->get_shootData().bullet_type;
//                auto msg = std::make_pair(ShootDataPriority, std::make_pair(cmd_id.cmdid_t, ShootDataMsg));
//                topicQueue.emplace(msg);
                    ShootDataPublisher->publish(ShootDataMsg);
                    break;
                }
                case SUPPLY_PROJECTILE_ACTION_CMD_ID: {
                    this->SupplyProjectileActionMsg.header.stamp = this->get_clock()->now();
                    SupplyProjectileActionMsg.supply_bullet_num = this->rmReferee->get_supplyProjectileAction().supply_projectile_num;
                    SupplyProjectileActionMsg.supply_projectile_id = this->rmReferee->get_supplyProjectileAction().supply_projectile_id;
                    SupplyProjectileActionMsg.supply_projectile_step = this->rmReferee->get_supplyProjectileAction().supply_projectile_step;
                    SupplyProjectileActionMsg.supply_robot_id = this->rmReferee->get_supplyProjectileAction().supply_robot_id;
                    SupplyProjectileActionPublisher->publish(SupplyProjectileActionMsg);
                    break;
                }
                default:
                    break; //do nothing
            }
        } catch (UpdateExcept &e) {
            RCLCPP_ERROR(this->get_logger(), "RUNTIME ERROR OCCURRED! : %s", e.what());
            this->refereeSerialStream->Close();
            this->is_opened = false;
            return false;
        } catch (LibSerial::NotOpen::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "NO DATA AVAILABLE: Serial NOT OPEN: %s;", e.what());
            return false;
        } catch (LibSerial::ReadTimeout::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "NO DATA AVAILABLE: Read Time Out: %s;", e.what());
            continue;
        } catch (LibSerial::SerialStream::failure::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "LibSerial ERROR OCCURRED! : %s", e.what());
            this->refereeSerialStream->Close();
            this->is_opened = false;
            return false;
        }
    }
    return true;
}


void RMRefereeSystem::publish_data() {
    auto Obj = topicQueue.top();
    auto msg = Obj.second;
    if(Obj.first == 0) {
        std::vector<uint8_t> v = this->rmReferee->packInteractiveDataSendInteractiveDataRecv(
                msg.data_cmd_id,msg.sender_id,msg.receiver_id,msg.data_length,msg.data);
        if(is_opened){
            auto *buffer = new uint8_t [v.size()*sizeof(uint8_t)];
            if (!v.empty())
            {
                memcpy(buffer, &v[0], v.size()*sizeof(uint8_t));
            }
            refereeSerialStream->write(reinterpret_cast<const char *>(buffer), v.size()*sizeof(uint8_t));
        }else{
            RCLCPP_ERROR(this->get_logger(),"Failed to write Serial: Port not open!");
        }
    }else {
        auto this_time = this->now();
        fp64 this_time_sec = 0;
        this_time_sec += static_cast<double>(this_time.seconds());
        this_time_sec += static_cast<double>(this_time.nanoseconds()) / 1E9;

        auto sender_time = msg.header.stamp;
        fp64 sender_time_sec = 0;
        sender_time_sec += static_cast<double>(sender_time.sec);
        sender_time_sec += static_cast<double>(sender_time.nanosec) / 1E9;

        if(this_time_sec - sender_time_sec > msg.valid_time){
            RCLCPP_INFO(this->get_logger(),"Late package DROPPED!");
            RCLCPP_DEBUG(this->get_logger(),"Send at %f",sender_time_sec);
            RCLCPP_DEBUG(this->get_logger(),"Popped at %f",this_time_sec);
            RCLCPP_DEBUG(this->get_logger(),"Valid:%f",msg.valid_time);
        }else{
            std::vector<uint8_t> v = this->rmReferee->packInteractiveDataSendInteractiveDataRecv(
                    msg.data_cmd_id,msg.sender_id,msg.receiver_id,msg.data_length,msg.data);
            if(is_opened){
                auto *buffer = new uint8_t [v.size()*sizeof(uint8_t)];
                if (!v.empty())
                {
                    memcpy(buffer, &v[0], v.size()*sizeof(uint8_t));
                }
                refereeSerialStream->write(reinterpret_cast<const char *>(buffer), v.size()*sizeof(uint8_t));
            }else{
                RCLCPP_ERROR(this->get_logger(),"Failed to write Serial: Port not open!");
            }
            RCLCPP_DEBUG(this->get_logger(),"Package Sent!");
        }
    }
    topicQueue.pop();
}

void RMRefereeSystem::update() {

    RCLCPP_DEBUG(this->get_logger(), "updating");


    if (!this->read()) {
        RCLCPP_DEBUG(this->get_logger(), "read() failed");
        return;
    }

    //update timestamp
    this->last_update_timestamp = this->get_clock()->now();

}

void RMRefereeSystem::publish_diag() {
    this->diag_msg.header.stamp = this->get_clock()->now();
    if (!this->is_opened) {

        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        this->diag_msg.status[0].message = "serial device offline";

        rclcpp::Clock clock;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "[%s] serial device offline",
                              this->diag_msg.status[0].name.c_str());

    } else if (this->get_clock()->now() - this->last_update_timestamp > 500ms) {

        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        this->diag_msg.status[0].message = "receiver offline";

        rclcpp::Clock clock;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "[%s] receiver offline",
                              this->diag_msg.status[0].name.c_str());

    } else if (this->flag_transmission_jammed) {

        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        this->diag_msg.status[0].message = "transmission jammed";

        rclcpp::Clock clock;
        RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] transmission jammed",
                             this->diag_msg.status[0].name.c_str());

    } else {
        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        this->diag_msg.status[0].message = "ok";
    }
    this->diag_publisher->publish(this->diag_msg);
}

void RMRefereeSystem::detect_jammed() {
    //10 packet decode failed can be considered as transmission jammed
    if (this->decode_fail_cnt > 10) {
        this->flag_transmission_jammed = true;
    } else {
        this->flag_transmission_jammed = false;
    }
    this->decode_fail_cnt = 0;
}

__attribute__((unused)) void RMRefereeSystem::readBuff(uint8_t *buff, uint16_t len) {

}

void RMRefereeSystem::topic_callback(const gary_msgs::msg::InteractiveDataSend::SharedPtr msg) {
        topicQueue.emplace(std::make_pair(msg->priority,*msg));
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<RMRefereeSystem> rmRefereeSystem = std::make_shared<RMRefereeSystem>(rclcpp::NodeOptions());

    exe.add_node(rmRefereeSystem->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_serial::RMRefereeSystem)