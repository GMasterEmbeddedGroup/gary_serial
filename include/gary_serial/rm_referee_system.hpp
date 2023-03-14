#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "utils/msg_include.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "utils/rm_referee.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <queue>
//#include <functional>
#include <map>
#include <boost/any.hpp>
#include <boost/functional.hpp>
//#include <boost/heap/priority_queue.hpp>
//#include <boost/interprocess/ipc/message_queue.hpp>
#include <libserial/SerialStream.h>
#include <libserial/SerialPortConstants.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using topicObject = std::pair<uint16_t,gary_msgs::msg::InteractiveDataSend>;
struct cmp
{
    bool operator() (topicObject& objectL, topicObject& objectR)
    {
        return objectL.first != 0 && ((objectR.first == 0) || (objectL.first < objectR.first));
    }
};

class UpdateExcept : public std::runtime_error {
public:
    explicit UpdateExcept(const std::string &arg = "runtime Read() problem") : runtime_error(arg) {}
};

namespace gary_serial {

    uint16_t AerialRobotEnergyPriority = 0;
    uint16_t BulletRemainingPriority = 0;
    uint16_t ClientCommandPriority = 0;
    uint16_t ClientReceivePriority = 0;
    uint16_t CustomControllerPriority = 0;
    uint16_t DartClientCmdPriority = 0;
    uint16_t DartRemainingTimePriority = 0;
    uint16_t FieldEventsPriority = 0;
    uint16_t GameResultPriority = 0;
    uint16_t GameStatusPriority = 0;
    uint16_t ICRABuffDebuffZoneAndLurkStatusPriority = 0;
    uint16_t ImageTransmitterPriority = 0;
    uint16_t InteractiveDataRecvPriority = 0;
    uint16_t PowerHeatPriority = 0;
    uint16_t RefereeWarningPriority = 0;
    uint16_t RFIDStatusPriority = 0;
    uint16_t RobotBuffPriority = 0;
    uint16_t RobotHPPriority = 0;
    uint16_t RobotHurtPriority = 0;
    uint16_t RobotPositionPriority = 0;
    uint16_t RobotStatusPriority = 0;
    uint16_t ShootDataPriority = 0;
    uint16_t SupplyProjectileActionPriority = 0;
    uint16_t SupplyProjectileRequestPriority = 0;


    class RMRefereeSystem : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit RMRefereeSystem(const rclcpp::NodeOptions & options);


    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        bool open();
        void close();

        bool read();
        __attribute__((deprecated("Use New Serial Lib")))bool get_available_len();
        __attribute__((unused)) void readBuff(uint8_t *buff, uint16_t len);
        void publish_data();

        void update();

        void publish_diag();

        void detect_jammed();

        std::shared_ptr<utils::RMReferee> rmReferee;
        std::shared_ptr<LibSerial::SerialStream> refereeSerialStream;

        //message_queue
        std::priority_queue<topicObject,std::vector<topicObject>,cmp> topicQueue;
//        boost::heap::priority_queue<topicObject,std::vector<topicObject>,cmp> topicQueue;

        //params
        std::string diagnostic_topic;
        double update_freq;
        double publisher_freq;
        double diag_freq;
        std::string serial_port;
        int64_t baudrate;
        std::string override_diag_device_name;

//        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RMRefereeSystem>::SharedPtr msg_publisher;
//        gary_msgs::msg::RMRefereeSystem dr16_msg;
        diagnostic_msgs::msg::DiagnosticArray diag_msg;
        rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher;
        gary_msgs::msg::AerialRobotEnergy AerialRobotEnergyMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::AerialRobotEnergy>::SharedPtr AerialRobotEnergyPublisher;
        gary_msgs::msg::BulletRemaining BulletRemainingMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::BulletRemaining>::SharedPtr BulletRemainingPublisher;
        gary_msgs::msg::ClientCommand ClientCommandMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::ClientCommand>::SharedPtr ClientCommandPublisher;
        gary_msgs::msg::ClientReceive ClientReceiveMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::ClientReceive>::SharedPtr ClientReceivePublisher;
        gary_msgs::msg::CustomController CustomControllerMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::CustomController>::SharedPtr CustomControllerPublisher;
        gary_msgs::msg::DartClientCmd DartClientCmdMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::DartClientCmd>::SharedPtr DartClientCmdPublisher;
        gary_msgs::msg::DartRemainingTime DartRemainingTimeMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::DartRemainingTime>::SharedPtr DartRemainingTimePublisher;
        gary_msgs::msg::FieldEvents FieldEventsMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::FieldEvents>::SharedPtr FieldEventsPublisher;
        gary_msgs::msg::GameResult GameResultMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::GameResult>::SharedPtr GameResultPublisher;
        gary_msgs::msg::GameStatus GameStatusMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::GameStatus>::SharedPtr GameStatusPublisher;
        gary_msgs::msg::ICRABuffDebuffZoneAndLurkStatus ICRABuffDebuffZoneAndLurkStatusMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::ICRABuffDebuffZoneAndLurkStatus>::SharedPtr ICRABuffDebuffZoneAndLurkStatusPublisher;
        gary_msgs::msg::ImageTransmitter ImageTransmitterMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::ImageTransmitter>::SharedPtr ImageTransmitterPublisher;
        gary_msgs::msg::InteractiveDataRecv InteractiveDataRecvMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::InteractiveDataRecv>::SharedPtr InteractiveDataRecvPublisher;
        gary_msgs::msg::PowerHeat PowerHeatMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::PowerHeat>::SharedPtr PowerHeatPublisher;
        gary_msgs::msg::RefereeWarning RefereeWarningMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RefereeWarning>::SharedPtr RefereeWarningPublisher;
        gary_msgs::msg::RFIDStatus RFIDStatusMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RFIDStatus>::SharedPtr RFIDStatusPublisher;
        gary_msgs::msg::RobotBuff RobotBuffMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotBuff>::SharedPtr RobotBuffPublisher;
        gary_msgs::msg::RobotHP RobotHPMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotHP>::SharedPtr RobotHPPublisher;
        gary_msgs::msg::RobotHurt RobotHurtMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotHurt>::SharedPtr RobotHurtPublisher;
        gary_msgs::msg::RobotPosition RobotPositionMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotPosition>::SharedPtr RobotPositionPublisher;
        gary_msgs::msg::RobotStatus RobotStatusMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::RobotStatus>::SharedPtr RobotStatusPublisher;
        gary_msgs::msg::ShootData ShootDataMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::ShootData>::SharedPtr ShootDataPublisher;
        gary_msgs::msg::SupplyProjectileAction SupplyProjectileActionMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::SupplyProjectileAction>::SharedPtr SupplyProjectileActionPublisher;
        gary_msgs::msg::SupplyProjectileRequest SupplyProjectileRequestMsg;
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::SupplyProjectileRequest>::SharedPtr SupplyProjectileRequestPublisher;

        void topic_callback(gary_msgs::msg::InteractiveDataSend::SharedPtr msg);
        rclcpp::Subscription<gary_msgs::msg::InteractiveDataSend>::SharedPtr InteractiveDataSendSubscription;

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;
        rclcpp::TimerBase::SharedPtr timer_publisher;
        rclcpp::TimerBase::SharedPtr timer_diag;
        rclcpp::TimerBase::SharedPtr timer_detect;

        int fd;
        bool is_opened;
        int available_len;
        int decode_fail_cnt;
        bool flag_transmission_jammed;
        uint8_t frame_header[5]{};
        rclcpp::Time last_update_timestamp;
        union {
            uint8_t cmdid_uint8[2];
            utils::_ns_rm_referee::referee_cmd_id_t cmdid_t;
            uint16_t cmd_id_uint16;
        } cmd_id{};
        uint16 data_len;
        uint8_t tail[2]{};
        uint8_t data[];
    };
}