#ifndef PROTEZCOMMAND_H
#define PROTEZCOMMAND_H


#include <cstdint>
#include <vector>
#include <map>


/*COMMAND CODE*/
#define PR_PROTOCOL_CODE_SIDEPLATE		0x01
#define PR_PROTOCOL_CODE_WORKMODE		0x02
#define PR_PROTOCOL_CODE_CONFIG			0x03
#define PR_PROTOCOL_CODE_NUMMOTOR		0x04
#define PR_PROTOCOL_CODE_DIRECTION		0x05
#define PR_PROTOCOL_CODE_PWM			0x06
#define PR_PROTOCOL_CODE_TIME			0x07
#define PR_PROTOCOL_CODE_ANGLE			0x08
#define PR_PROTOCOL_CODE_SPEED			0x09
#define PR_PROTOCOL_CODE_DELAY			0x0A
#define PR_PROTOCOL_CODE_ADC			0x0B
#define PR_PROTOCOL_CODE_FEEDBACK		0x0C
#define PR_PROTOCOL_CODE_RUNNING		0x0D
/*WORKMODE*/
#define PR_VAL_WORKMODE_MANUAL			0x01
#define PR_VAL_WORKMODE_ANGLE			0x02
#define PR_VAL_WORKMODE_STATUS			0x03
/*RUNNING*/
#define PR_VAL_RUNNING_STOP				0x00
#define PR_VAL_RUNNING_START			0x01
#define PR_VAL_RUNNING_ALL_START		0x02
#define PR_VAL_RUNNING_ALL_STOP			0x03
/*SIDEPLATE*/
#define PR_VAL_SIDEPLATE_THIS			0x00
#define PR_VAL_SIDEPLATE_OTHER			0x01




/*FOR SEND*/
#define PR_PROTOCOL_START_PACK      std::pair<uint8_t, uint8_t>(0xEA, 0xBC)
#define PR_PROTOCOL_STOP_PACK       std::pair<uint8_t, uint8_t>(0xBC, 0xAE)

#define PR_PROTOCOL_START           std::pair<uint8_t, uint8_t>(0xDE, 0xAD)
#define PR_PROTOCOL_STOP            std::pair<uint8_t, uint8_t>(0xBE, 0xEF)
/*FOR RECV*/
#define PR_PROTOCOL_PACK_DATA_RECV_START	std::pair<uint8_t, uint8_t>(0xAA, 0x55)
#define PR_PROTOCOL_PACK_DATA_RECV_STOP     std::pair<uint8_t, uint8_t>(0x66, 0x11)


#define PR_PROTOCOL_PACK_ADC_START          std::pair<uint8_t, uint8_t>(0xAA, 0xDD)
#define PR_PROTOCOL_PACK_ADC_STOP           std::pair<uint8_t, uint8_t>(0xCC, 0xBB)

#define PR_PROTOCOL_PACK_FEEDBACK_START     std::pair<uint8_t, uint8_t>(0x44, 0xDD)
#define PR_PROTOCOL_PACK_FEEDBACK_STOP      std::pair<uint8_t, uint8_t>(0x77, 0xCC)



class ProtezCommand
{
private:
    std::vector<std::map<uint8_t, std::vector<uint8_t>>> Commands;
public:
    static std::vector<uint8_t> GlobalDataRecv;
    static bool packetStarted;
public:
    ProtezCommand();
    ~ProtezCommand();

    void add(std::map<uint8_t, std::vector<uint8_t>>);
    void clear();

    std::map<uint8_t, std::vector<uint8_t>> getCommand(uint8_t);
    void setCommand(std::vector<std::map<uint8_t, std::vector<uint8_t>>>);

    std::vector<uint8_t> existCollectData();
};

#endif // PROTEZCOMMAND_H
