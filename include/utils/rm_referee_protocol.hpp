#ifndef BUILD_RM_REFEREE_PROTOCOL_HPP
#define BUILD_RM_REFEREE_PROTOCOL_HPP

#include "utils/struct_typedef.hpp"
; //to avoid "#pragma pack()" be at EOF.
#pragma pack(push, 1)
namespace utils {
    namespace _ns_rm_referee {
        typedef enum : uint16_t {
            GAME_STATE_CMD_ID = 0x0001,
            GAME_RESULT_CMD_ID = 0x0002,
            GAME_ROBOT_HP_CMD_ID = 0x0003,
            DART_STATE_CMD_ID = 0x0004,
            AI_STATE_CMD_ID = 0x0005,
            FIELD_EVENTS_CMD_ID = 0x0101,
            SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102,
            SUPPLY_PROJECTILE_BOOKING_CMD_ID = 0x0103,
            REFEREE_WARNING_CMD_ID = 0x0104,
            DART_COUNTDOWN_CMD_ID = 0x0105,
            ROBOT_STATE_CMD_ID = 0x0201,
            POWER_HEAT_DATA_CMD_ID = 0x0202,
            ROBOT_POS_CMD_ID = 0x0203,
            BUFF_MUSK_CMD_ID = 0x0204,
            AERIAL_ROBOT_ENERGY_CMD_ID = 0x0205,
            ROBOT_HURT_CMD_ID = 0x0206,
            SHOOT_DATA_CMD_ID = 0x0207,
            BULLET_REMAINING_CMD_ID = 0x0208,
            RFID_STATE_CMD_ID = 0x0209,
            DART_CLIENT_COMMAND_CMD_ID = 0x020A,
            ROBOT_INTERACTIVE_DATA_CMD_ID = 0x0301,
            CONTROLLER_INTERACTIVE_DATA_CMD_ID = 0x0302,
            MAP_INTERACTIVE_DATA_CDM_ID = 0x0303,
            KEYBOARD_AND_MOUSE_INFO_CMD_ID = 0x0304,
            MAP_RECEIVE_INFO = 0x0305,
            IDCustomData,
        } referee_cmd_id_t;

        typedef struct {
            uint8_t SOF;
            uint16_t data_length;
            uint8_t seq;
            uint8_t CRC8;
        } frame_header_struct_t;


        constexpr int HEADER_SOF = 0xA5;
        constexpr int REF_PROTOCOL_FRAME_MAX_SIZE = 128;
        constexpr int REF_PROTOCOL_HEADER_SIZE = sizeof(frame_header_struct_t);
        constexpr int REF_PROTOCOL_CMD_SIZE = sizeof(uint16_t);
        constexpr int REF_PROTOCOL_CRC8_SIZE = 1;
        constexpr int REF_PROTOCOL_CRC16_SIZE = 2;
        constexpr int REF_HEADER_CRC_LEN = (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE);
        constexpr int REF_HEADER_CRC_CMDID_LEN = (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE +
                                                  REF_PROTOCOL_CMD_SIZE);
        constexpr int REF_HEADER_CMDID_LEN = (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CMD_SIZE);
    }
}
#pragma pack(pop)
#endif //BUILD_RM_REFEREE_PROTOCOL_HPP
