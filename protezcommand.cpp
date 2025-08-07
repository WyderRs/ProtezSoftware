#include "protezcommand.h"




ProtezCommand::ProtezCommand() {}
ProtezCommand::~ProtezCommand() {}


void ProtezCommand::add(std::map<uint8_t, std::vector<uint8_t>> com)
{
    Commands.push_back(com);
}
void ProtezCommand::clear()
{
    Commands.clear();
}
std::map<uint8_t, std::vector<uint8_t>> ProtezCommand::getCommand(uint8_t i)
{
    return Commands[i];
}
void ProtezCommand::setCommand(std::vector<std::map<uint8_t, std::vector<uint8_t>>> data)
{
    Commands = data;
}
std::vector<std::vector<uint8_t>> ProtezCommand::existCollectData()
{
    std::vector<std::vector<uint8_t>> colDat;
    std::vector<uint8_t> colCom;

    colCom.push_back(PR_PROTOCOL_START_PACK.first);
    colCom.push_back(PR_PROTOCOL_START_PACK.second);

    for (const auto &com : Commands)
    {
        colCom.push_back(PR_PROTOCOL_START.first);
        colCom.push_back(PR_PROTOCOL_START.second);

        for (const auto &mp : com)
        {
            colCom.push_back((mp.first | (mp.second.size() << 5)));
            for (const auto &dat : mp.second)
            {
                colCom.push_back(dat);
            }
        }

        colCom.push_back(PR_PROTOCOL_STOP.first);
        colCom.push_back(PR_PROTOCOL_STOP.second);
    }

    colCom.push_back(PR_PROTOCOL_STOP_PACK.first);
    colCom.push_back(PR_PROTOCOL_STOP_PACK.second);

    return colDat;
}







