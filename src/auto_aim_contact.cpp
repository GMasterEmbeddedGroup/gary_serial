#include "gary_contact/auto_aim_contact.hpp"


using namespace std::chrono_literals;
using namespace gary_contact;

using std::placeholders::_1;

AutoAIMContact::AutoAIMContact(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "auto_aim_contact", options) {

  this->declare_parameter("serial_port", "/dev/ttyACM0");


  this->serial_port = this->get_parameter("serial_port").as_string();
  this->serial = std::make_shared<LibSerial::SerialPort>(this->serial_port,
                                                           LibSerial::BaudRate::BAUD_115200,
                                                           LibSerial::CharacterSize::CHAR_SIZE_8,
                                                           LibSerial::FlowControl::FLOW_CONTROL_NONE,
                                                           LibSerial::Parity::PARITY_NONE,
                                                           LibSerial::StopBits::STOP_BITS_1
                                                           );

  this->vision_subscription_ = this->create_subscription<gary_msgs::msg::AutoAIM>("vision_msgs",
                                                                                  rclcpp::SystemDefaultsQoS(),
                                                                                  std::bind(&AutoAIMContact::auto_aim_callback, this, _1));
  
  this->aim_timer_ = this->create_wall_timer(10ms, std::bind(&AutoAIMContact::data_send, this));
}

void AutoAIMContact::auto_aim_callback(gary_msgs::msg::AutoAIM msg){
  aim_pitch = msg.pitch;
  aim_yaw = msg.yaw;
  RCLCPP_INFO(this->get_logger(),"pitch = %f;yaw = %f",aim_pitch,aim_yaw);

}

void AutoAIMContact::data_send(){

    for (int i = 0; i < 4; i++) {
        packet[i] = pitch_raw[i];  
        packet[4 + i] = yaw_raw[i];  
    }

    RCLCPP_INFO(this->get_logger(),"Send succeeded!");
    
    for(uint8_t i=0 ; i < 8 ; i++){
        this -> serial -> WriteByte(this->packet[i]);
    }
    
    RCLCPP_INFO(this->get_logger(),"Pushed Already!");
}



int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<AutoAIMContact> auto_aim_contact = std::make_shared<AutoAIMContact>(rclcpp::NodeOptions());

    exe.add_node(auto_aim_contact->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();

}
