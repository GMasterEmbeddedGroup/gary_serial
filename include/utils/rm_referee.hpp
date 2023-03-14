#ifndef BUILD_RM_REFEREE_H
#define BUILD_RM_REFEREE_H

#include <cstddef>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <tiff.h>
#include "utils/struct_typedef.hpp"
#include "utils/rm_referee_protocol.hpp"

namespace utils {
#define packed_ __attribute__((packed))
    using namespace _ns_rm_referee;

    typedef struct //0001
    {
        uint8_t game_type: 4;
        uint8_t game_progress: 4;
        uint16_t stage_remain_time;
        uint64_t UNIX_time_map;
    } packed_ ext_game_state_t;

    typedef struct //0002
    {
        uint8_t winner;
    } packed_ ext_game_result_t;

    typedef struct { //0003
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_tower_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_tower_HP;
        uint16_t blue_base_HP;
    } packed_ ext_game_robot_HP_t;

    typedef struct { //0005
        uint8_t F1_zone_status: 1;
        uint8_t F1_zone_buff_status: 3;
        uint8_t F2_zone_status: 1;
        uint8_t F2_zone_buff_status: 3;
        uint8_t F3_zone_status: 1;
        uint8_t F3_zone_buff_status: 3;
        uint8_t F4_zone_status: 1;
        uint8_t F4_zone_buff_status: 3;
        uint8_t F5_zone_status: 1;
        uint8_t F5_zone_buff_status: 3;
        uint8_t F6_zone_status: 1;
        uint8_t F6_zone_buff_status: 3;
        uint16_t red1_bullet_left;
        uint16_t red2_bullet_left;
        uint16_t blue1_bullet_left;
        uint16_t blue2_bullet_left;
        uint8_t lurk_mode;
        uint8_t res;
    } packed_ ext_ICRA_buff_debuff_zone_and_lurk_status_t;

    typedef struct //0101
    {
        uint32_t heal_point_1: 1;
        uint32_t heal_point_2: 1;
        uint32_t heal_point_3: 1;
        uint32_t hit_point: 1;
        uint32_t small_buff: 1;
        uint32_t big_buff: 1;
        uint32_t highland_2_occupied: 1;
        uint32_t highland_3_occupied: 1;
        uint32_t highland_4_occupied: 1;
        uint32_t base_shield_exist: 1;
        uint32_t tower_alive: 1;
        uint32_t resolved: 21;
    } packed_ ext_event_data_t;

    typedef struct //0x0102
    {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    } packed_ ext_supply_projectile_action_t;

    typedef struct //0x0103
    {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_num;
    } packed_ ext_supply_projectile_booking_t;

    typedef struct {//0x0104
        uint8_t level;
        uint8_t foul_robot_id;
    } packed_ ext_referee_warning_t;

    typedef struct { //0x0105
        uint8_t dart_remaining_time;
    } packed_ ext_dart_remaining_time_t;

    typedef struct //0x0201
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t remain_HP;
        uint16_t max_HP;
        uint16_t shooter_id1_17mm_cooling_rate;
        uint16_t shooter_id1_17mm_heat_limit;
        uint16_t shooter_id1_17mm_speed_limit;
        uint16_t shooter_id2_17mm_cooling_rate;
        uint16_t shooter_id2_17mm_heat_limit;
        uint16_t shooter_id2_17mm_speed_limit;
        uint16_t shooter_42mm_cooling_rate;
        uint16_t shooter_42mm_heat_limit;
        uint16_t shooter_42mm_speed_limit;
        uint16_t chassis_power_limit;
        uint8_t mains_power_gimbal_output: 1;
        uint8_t mains_power_chassis_output: 1;
        uint8_t mains_power_shooter_output: 1;
    } packed_ ext_game_robot_state_t;

    typedef struct { //0x0202
        uint16_t chassis_volt;
        uint16_t chassis_current;
        float chassis_power;
        uint16_t chassis_power_buffer;
        uint16_t shooter_id1_17mm_cooling_heat;
        uint16_t shooter_id2_17mm_cooling_heat;
        uint16_t shooter_id1_42mm_cooling_heat;
    } packed_ ext_power_heat_data_t;

    typedef struct //0x0203
    {
        float x;
        float y;
        float z;
        float yaw;
    } packed_ ext_game_robot_pos_t;

    typedef struct //0x0204
    {
//        uint8_t power_rune_buff;
        uint8_t healing: 1;
        uint8_t cooling_buff: 1;
        uint8_t defence_buff: 1;
        uint8_t attack_buff: 1;
        uint8_t resolved: 4;
    } packed_ ext_buff_musk_t;

    typedef struct //0x0205
    {
        uint8_t attack_time;
    } packed_ aerial_robot_energy_t;

    typedef struct //0x0206
    {
        uint8_t armor_type: 4;
        uint8_t hurt_type: 4;
    } packed_ ext_robot_hurt_t;

    typedef struct //0x0207
    {
        uint8_t bullet_type;
        uint8_t shooter_id;
        uint8_t bullet_freq;
        float bullet_speed;
    } packed_ ext_shoot_data_t;

    typedef struct { //0x0208
        uint16_t bullet_remaining_num_17mm;
        uint16_t bullet_remaining_num_42mm;
        uint16_t coin_remaining_num;
    } packed_ ext_bullet_remaining_t;

    typedef struct { //0x0209
        uint32_t base_state: 1;
        uint32_t highland_state: 1;
        uint32_t buff_state: 1;
        uint32_t slope_state: 1;
        uint32_t tower_state: 1;
        uint32_t resolved: 1; //WTF ?!?!?!?!?!?!?!?
        uint32_t healing_state: 1;
        uint32_t restore_card_state: 1;
        uint32_t res: 24;
    } packed_ ext_rfid_status_t;

    typedef struct { //0x020A
        uint8_t dart_launch_opening_status;
        uint8_t dart_attack_target;
        uint16_t target_change_time;
        uint16_t operate_launch_cmd_time;
    } packed_ ext_dart_client_cmd_t;

    using ext_student_interactive_header_data_t =
            struct {  //0x0301
                uint16_t data_cmd_id;
                uint16_t sender_ID;
                uint16_t receiver_ID;
            } packed_;

    using robot_interactive_data_t =
            struct {  //0x0302
                uint8_t data[114];
            }packed_;

    using ext_robot_command_t =
            struct {  //0x0303
                float target_position_x;
                float target_position_y;
                float target_position_z;
                uint8_t command_keyboard;
                uint16_t target_robot_ID;
            }packed_;

    using ext_client_map_command_t =
            struct { //0x0305
                uint16_t target_robot_ID;
                float target_position_x;
                float target_position_y;
            };

    typedef struct {
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        int8_t left_button_down;
        int8_t right_button_down;
        uint16_t key_w:1;
        uint16_t key_s:1;
        uint16_t key_a:1;
        uint16_t key_d:1;
        uint16_t key_shift:1;
        uint16_t key_ctrl:1;
        uint16_t key_q:1;
        uint16_t key_e:1;
        uint16_t key_r:1;
        uint16_t key_f:1;
        uint16_t key_g:1;
        uint16_t key_z:1;
        uint16_t key_x:1;
        uint16_t key_c:1;
        uint16_t key_v:1;
        uint16_t key_b:1;
        uint16_t reserved;
    } packed_ ext_keyboard_command_t;

#ifndef autoGet
#define autoGet(TYPENAME,VAL) public: \
TYPENAME get_##VAL() const { return VAL; }     \
    private:                          \
    TYPENAME VAL{};
#endif
    
    
    class RMReferee {
    public:
        RMReferee();

        uint16_t solveData(uint8_t *frame);
        uint16_t solveDataLength(uint8_t* header);
        uint8_t getCustomeDataLen() const;
        std::vector<uint8_t> packInteractiveDataSendInteractiveDataRecv(uint16_t data_cmd_id, uint16_t sender_id, uint16_t receiver_id,
                                                         uint8_t data_length, std::vector<uint8_t>& data);
    private:
        uint8_t custom_data_len;
        autoGet(ext_game_state_t,gameState)
        autoGet(ext_game_result_t,gameResult)
        autoGet(ext_game_robot_HP_t,robotHP)
        autoGet(ext_ICRA_buff_debuff_zone_and_lurk_status_t,buffZoneAndLurkStatus)
        autoGet(ext_event_data_t,eventData)
        autoGet(ext_supply_projectile_action_t,supplyProjectileAction)

        autoGet(ext_referee_warning_t,refereeWarning)
        autoGet(ext_dart_remaining_time_t,dartRemainingTime)
        autoGet(ext_game_robot_state_t,robotState)
        autoGet(ext_power_heat_data_t,powerHeatData)
        autoGet(ext_game_robot_pos_t,robotPos)
        autoGet(ext_buff_musk_t,buffMusk)
        autoGet(aerial_robot_energy_t,aerialRobotEnergy)
        autoGet(ext_robot_hurt_t,robotHurt)
        autoGet(ext_shoot_data_t,shootData)
        autoGet(ext_bullet_remaining_t,bulletRemaining)
        autoGet(ext_rfid_status_t,rfidStatus)
        autoGet(ext_dart_client_cmd_t,dartClientCmd)

        autoGet(ext_student_interactive_header_data_t,studentInteractiveHeaderData)
        autoGet(ext_robot_command_t,robotCommand)
        autoGet(ext_keyboard_command_t,keyboardCommand)
        autoGet(ext_client_map_command_t,mapCommand)

        autoGet(robot_interactive_data_t,customControllerData)
        autoGet(robot_interactive_data_t,robotInteractiveDataRecv)
    };

#ifdef autoGet
#undef autoGet
#endif

} // utils

#endif //BUILD_RM_REFEREE_H
