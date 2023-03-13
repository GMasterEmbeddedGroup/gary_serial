#include "utils/rm_referee.hpp"
#include "utils/crc8_crc16.hpp"
#include <stdexcept>

namespace utils {
    typedef enum {
        RED_HERO = 1,
        RED_ENGINEER = 2,
        RED_STANDARD_1 = 3,
        RED_STANDARD_2 = 4,
        RED_STANDARD_3 = 5,
        RED_AERIAL = 6,
        RED_SENTRY = 7,
        BLUE_HERO = 11,
        BLUE_ENGINEER = 12,
        BLUE_STANDARD_1 = 13,
        BLUE_STANDARD_2 = 14,
        BLUE_STANDARD_3 = 15,
        BLUE_AERIAL = 16,
        BLUE_SENTRY = 17,
    } robot_id_t;
    typedef enum {
        PROGRESS_UNSTART = 0,
        PROGRESS_PREPARE = 1,
        PROGRESS_SELFCHECK = 2,
        PROGRESS_5sCOUNTDOWN = 3,
        PROGRESS_BATTLE = 4,
        PROGRESS_CALCULATING = 5,
    } game_progress_t;

#ifndef setZero
#define setZero(x) do{std::memset(&(x),0,sizeof((x)));}while(0)
#endif

    RMReferee::RMReferee() {
        setZero(this->keyboardCommand);
        setZero(this->robotCommand);
        setZero(this->robotInteractiveData);
        setZero(this->studentInteractiveHeaderData);
        setZero(this->dartClientCmd);
        setZero(this->rfidStatus);
        setZero(this->bulletRemaining);
        setZero(this->shootData);
        setZero(this->robotHurt);
        setZero(this->aerialRobotEnergy);
        setZero(this->buffMusk);
        setZero(this->robotPos);
        setZero(this->robotState);
        setZero(this->powerHeatData);
        setZero(this->dartRemainingTime);
        setZero(this->refereeWarning);
        setZero(this->gameState);
        setZero(this->eventData);
        setZero(this->supplyProjectileAction);
        setZero(this->gameResult);
        setZero(this->buffZoneAndLurkStatus);
        setZero(this->customControllerData);
        setZero(this->robotHP);
        custom_data_len = 0;
    }

#ifdef setZero
#undef setZero
#endif

/**
  * @brief          solve referee data
  * @param[in]      frame: data frame from referee system
  * @retval         cmd_id
  */
    uint16_t RMReferee::solveData(uint8_t *frame) {
        uint16_t cmd_id = 0;
        uint8_t index = 0;
        union {
            frame_header_struct_t _struct;
            uint8 _arr[5];
        } referee_receive_header{{0, 0, 0, 0}};

        std::memcpy(&referee_receive_header._struct, frame, sizeof(frame_header_struct_t));
        index = REF_PROTOCOL_HEADER_SIZE;

        std::memcpy(&cmd_id, frame + index, REF_PROTOCOL_CMD_SIZE);
        index = REF_HEADER_CMDID_LEN;
        bool CRC16_checked = verify_CRC16_check_sum(frame, referee_receive_header._struct.data_length +
                                                           REF_HEADER_CRC_CMDID_LEN);
//        bool CRC16_checked = true;
        if (!CRC16_checked) {
            std::string s = std::to_string(cmd_id);
            throw (std::runtime_error("ID"+s+" Frame CRC16 Checking Failed."));
        }

#ifndef setData
#define setData(ID, VAL) case (ID):{\
    do{std::memcpy(&(VAL),frame+index,sizeof((VAL)));}while(0);\
    break;}
#endif
        switch (cmd_id) {
            setData(GAME_STATE_CMD_ID, gameState);
            setData(GAME_RESULT_CMD_ID, gameResult);
            setData(GAME_ROBOT_HP_CMD_ID, robotHP);
//            setData(DART_STATE_CMD_ID,); //not given
            setData(AI_STATE_CMD_ID, buffZoneAndLurkStatus);
            setData(FIELD_EVENTS_CMD_ID, eventData);
            setData(SUPPLY_PROJECTILE_ACTION_CMD_ID, supplyProjectileAction);
//            setData(SUPPLY_PROJECTILE_BOOKING_CMD_ID,)
            setData(REFEREE_WARNING_CMD_ID, refereeWarning);
            setData(DART_COUNTDOWN_CMD_ID, dartRemainingTime);
            setData(ROBOT_STATE_CMD_ID, robotState);
            setData(POWER_HEAT_DATA_CMD_ID, powerHeatData);
            setData(ROBOT_POS_CMD_ID, robotPos);
            setData(BUFF_MUSK_CMD_ID, buffMusk);
            setData(AERIAL_ROBOT_ENERGY_CMD_ID, aerialRobotEnergy);
            setData(ROBOT_HURT_CMD_ID, robotHurt);
            setData(SHOOT_DATA_CMD_ID, shootData);
            setData(BULLET_REMAINING_CMD_ID, bulletRemaining);
            setData(RFID_STATE_CMD_ID, rfidStatus);
            setData(DART_CLIENT_COMMAND_CMD_ID, dartClientCmd);
            case ROBOT_INTERACTIVE_DATA_CMD_ID:{
                std::memcpy(&studentInteractiveHeaderData, frame + index,
                            sizeof(studentInteractiveHeaderData));
                std::memcpy(&robotInteractiveData,
                            frame + index + sizeof(studentInteractiveHeaderData),
                            referee_receive_header._struct.data_length - 6);
                custom_data_len = referee_receive_header._struct.data_length - 6;
                break;
            }
            case CONTROLLER_INTERACTIVE_DATA_CMD_ID : {
                std::memcpy(&customControllerData, frame + index,
                            referee_receive_header._struct.data_length);
                custom_data_len = referee_receive_header._struct.data_length;
                break;
            }
            setData(MAP_INTERACTIVE_DATA_CDM_ID, robotCommand);
            setData(KEYBOARD_AND_MOUSE_INFO_CMD_ID, keyboardCommand);
            setData(MAP_RECEIVE_INFO, mapCommand);
            default:
                break;
        }
#ifdef setData
#undef setData
#endif
        return cmd_id;
    }

     uint16_t RMReferee::solveDataLength(uint8_t *header) {
        union {
            uint8 _arr[5];
            frame_header_struct_t _struct;
        } referee_receive_header{{0, 0, 0, 0,0}};
        std::memcpy(&referee_receive_header, header, sizeof(frame_header_struct_t));
        if (referee_receive_header._struct.SOF != HEADER_SOF) {
            throw (std::runtime_error("Incorrect Frame Header SOF."));
        }
        bool CRC8_checked = verify_CRC8_check_sum(referee_receive_header._arr, sizeof(referee_receive_header));
        if (!CRC8_checked) {
            throw (std::runtime_error("Header CRC8 Checking Failed."));
        }
        return referee_receive_header._struct.data_length;
    }

    uint8_t RMReferee::getCustomeDataLen() const {
        return custom_data_len;
    }

     std::vector<uint8_t> RMReferee::packWhateverInteractiveData(uint16_t data_cmd_id,
                                                                uint16_t sender_id, uint16_t receiver_id,
                                                                uint8_t data_length, std::vector<uint8_t>& data){

        std::vector<uint8_t> package;
        uint8_t header[5];
        package.resize(data_length+15);
        std::fill(package.begin(), package.end(),0x00);
        memset(header,0, sizeof(header));

        uint16_t index = 0;
        package[index] = utils::_ns_rm_referee::HEADER_SOF;
        header[index] = utils::_ns_rm_referee::HEADER_SOF;
        index ++;

        const uint16_t total_length = data_length + 6;
        const uint8_t upperLEN = (total_length >> 8)&0xFF;
        package[index] = upperLEN;
        header[index] = upperLEN;
        index ++;
        const uint8_t lowerLEN = (total_length)&0xFF;
        package[index] = lowerLEN;
        header[index] = lowerLEN;
        index ++;

        static uint8_t seq = 0;
        package[index] = seq;
        header[index] = seq;
        if(seq == 0xFF){
            seq = 0;
        }else{
            seq ++;
        }
        index++;

        append_CRC8_check_sum(header,utils::_ns_rm_referee::REF_PROTOCOL_HEADER_SIZE);
        package[index] = header[index];
        index++;

        const uint16_t cmd_id = 0x0301;
        const uint8_t upperID = (cmd_id >> 8)&0xFF;
        package[index] = upperID;
        index ++;
        const uint8_t lowerID = (cmd_id)&0xFF;
        package[index] = lowerID;
        index++;

        const uint8_t upperDataID = (data_cmd_id >> 8)&0xFF;
        package[index] = upperDataID;
        index ++;
        const uint8_t lowerDataID = (data_cmd_id)&0xFF;
        package[index] = lowerDataID;
        index++;

        const uint8_t upperSenderID = (sender_id >> 8)&0xFF;
        package[index] = upperSenderID;
        index ++;
        const uint8_t lowerSenderID = (sender_id)&0xFF;
        package[index] = lowerSenderID;
        index++;

        const uint8_t upperReceiverID = (receiver_id >> 8)&0xFF;
        package[index] = upperReceiverID;
        index ++;
        const uint8_t lowerReceiverID = (receiver_id)&0xFF;
        package[index] = lowerReceiverID;
        index++;

        std::memcpy(&package[index],&data[0], data.size() * sizeof(uint8_t));

        append_CRC16_check_sum(&package[0],package.size());

        return package;
    }

} // utils