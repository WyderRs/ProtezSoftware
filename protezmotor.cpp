#include "protezmotor.h"


Protez_SideType ProtezMotor::SidePlate;
Protez_WorkMode ProtezMotor::WorkMode;


ProtezMotor::ProtezMotor(uint8_t _id) : id(_id)
{
    id = _id;
}

ProtezMotor::~ProtezMotor() {}

void ProtezMotor::setID(uint8_t _id)
{
    id = _id;
}
uint8_t ProtezMotor::getID()
{
    return id;
}
// void setSidePlate(Protez_SideType)
// {

// }
Protez_SideType ProtezMotor::getSidePlate()
{
    if ((MANUAL_CheckBox_SidePlate->isChecked()) || (ANGLE_CheckBox_SidePlate->isChecked()))
        return OtherPlate;
    else
        return ThisPlate;
}
Protez_WorkMode ProtezMotor::getWorkMode()
{
    // if () return PWM_MODE;
    // else if () return ANGLE_MODE;
    // else if () return STATUS_MODE;
    return PWM_MODE;
}


Protez_TypeMove ProtezMotor::getDirection()
{
    if (MANUAL_Button_compress->isChecked()) return Compress;
    else if (MANUAL_Button_decompress->isChecked()) return Decompress;
    else if (MANUAL_Button_hold->isChecked()) return Hold;
    else if (MANUAL_Button_stop->isChecked()) return Stop;
    else return Stop;
}
// void ProtezMotor::setPWM(uint8_t pwm)
// {
//     PWM = pwm;
// }
uint8_t ProtezMotor::getPWM()
{
    return MANUAL_LineEdit_PWM->text().toInt();
}
// void ProtezMotor::setWorkTime(uint16_t time)
// {
//     WorkTime = time;
// }
uint16_t ProtezMotor::getWorkTime()
{
    return (uint16_t)(MANUAL_LineEdit_WorkTime->text().toDouble() * 100);
    // return WorkTime;
}
// void ProtezMotor::setAngle(uint16_t agl)
// {

// }
uint16_t ProtezMotor::getAngle()
{
    return 0;
}

// void ProtezMotor::setSpeed(uint16_t sp)
// {

// }
uint16_t ProtezMotor::getSpeed()
{
    return 0;
}
// void ProtezMotor::setWorkDelay(uint16_t time)
// {
//     WorkDelay = time;
// }
uint16_t ProtezMotor::getWorkDelay()
{
    return (uint16_t)(MANUAL_LineEdit_WorkDelay->text().toDouble() * 100);
    // return WorkDelay;
}
// void ProtezMotor::setADC_State(Protez_ADCState st)
// {
//     ADC_State = st;
// }
Protez_ADCState ProtezMotor::getADC_State()
{
    return (Protez_ADCState)MANUAL_CheckBox_ADC->isChecked();
    // return ADC_State;
}
// void ProtezMotor::setFeedBack(Protez_FeedBack _fb)
// {
//     // FeedBack = _fb;
// }
Protez_FeedBack ProtezMotor::getFeedBack()
{
    return (Protez_FeedBack)MANUAL_CheckBox_FeedBack->isChecked();
    // return FeedBack;
}
// void ProtezMotor::setRunning(Protez_Running run)
// {
//     // Running = run;
// }
Protez_Running ProtezMotor::getRunning()
{
    // return Running;
    return Run_Start;
}

































