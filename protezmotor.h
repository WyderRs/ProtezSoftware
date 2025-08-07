#ifndef PROTEZMOTOR_H
#define PROTEZMOTOR_H

#include "mainwindow.h"
#include <QMainWindow>
#include <cstdint>
#include <vector>
#include <map>


enum Protez_SideType {
    ThisPlate = 0,
    OtherPlate,
};
enum Protez_WorkMode {
    PWM_MODE = 0,
    ANGLE_MODE,
    STATUS_MODE,
};

enum Protez_TypeMove {
    Stop = 0,
    Compress,
    Decompress,
    Hold,
    MoveNone,
};
enum Protez_Config {
    none1 = 0,
};

enum Protez_ADCState {
    ADC_Disable = 0,
    ADC_Enable,
};
enum Protez_FeedBack {
    FB_Disable = 0,
    FB_Enable,
};
enum Protez_Running {
    Run_Stop = 0,
    Run_Start,
    Run_Start_All,
    Run_Stop_All,
};

class ProtezMotor
{
public:
    /*QT_Defines*/
    QCheckBox   *MANUAL_CheckBox_SidePlate;
    QCheckBox   *ANGLE_CheckBox_SidePlate;
    QPushButton *MANUAL_Button_stop;
    QPushButton *MANUAL_Button_compress;
    QPushButton *MANUAL_Button_decompress;
    QPushButton *MANUAL_Button_hold;
    QSlider     *MANUAL_Slider_PWM;
    QLineEdit   *MANUAL_LineEdit_PWM;
    QLineEdit   *MANUAL_LineEdit_WorkTime;
    QLineEdit   *MANUAL_LineEdit_WorkDelay;
    QCheckBox   *MANUAL_CheckBox_ADC;
    QCheckBox   *MANUAL_CheckBox_FeedBack;
    QCustomPlot *MANUAL_Plot_ADC;
    QCustomPlot *MANUAL_Plot_ADCBack;
private:
    /*Static*/
    static Protez_SideType SidePlate;
    static Protez_WorkMode WorkMode;

    /*Variables*/
    uint8_t id;
    // Protez_TypeMove typeMove;
    Protez_Config Config;
    // uint8_t PWM;
    // uint16_t WorkTime;
    // uint16_t WorkDelay;
    // Protez_ADCState ADC_State;
    // Protez_FeedBack FeedBack;
    // Protez_Running Running;










    // QPushButton *TAB1_ComporessButton;
    // QPushButton *TAB1_DecompressButton;
    // QPushButton *TAB1_HoldButton;
    // QPushButton *TAB1_FreeButton;
    // QLineEdit   *TAB1_LineEditPWM;
    // QSlider     *TAB1_SliderPWM;
    // QLineEdit   *TAB1_LineEditWorkTime;
    // QLineEdit   *TAB1_LineEditDelayTime;
    // QCheckBox   *TAB1_CheckBoxADC;
    // QCheckBox   *TAB1_CheckBoxBackSide;
    // QCustomPlot *TAB1_ADCPlot;
    // QCustomPlot *TAB1_ADCPlotBack;
    // QPen        TAB1GraphPen;
    // QCheckBox   *TAB1_CheckBoxAutoCurrectBackPower;

    // QPushButton *TAB2_FingerButton;
    // QLineEdit   *TAB2_LineEditAngle;
    // QLineEdit   *TAB2_LineEditTime;
    // QLineEdit   *TAB2_LineEditSpeed;
    // QLineEdit   *TAB2_LineEditDelay;
    // QCheckBox   *TAB2_CheckBoxCH;
    // QCustomPlot *TAB2_FeedBackPlot;
    // QCheckBox   *TAB2_CheckBoxBackSide;
    // QCheckBox   *TAB2_CheckBoxBackReverse;
    // QCustomPlot *TAB2_FeedBackPlotBack;

    // /*Main com0 bytes*/
    // uint8_t     MD_Config_1;
    // uint8_t     MD_Config_2;
    // WorkModeEnum MD_WorkMode;
    // NumPlate     MD_SidePlate;
    // /*Individual com2 bytes - PWM Mode*/
    // uint8_t     MD1_SelMotor;
    // MoveType    MD1_MoveType;
    // uint16_t    MD1_PWM;
    // uint16_t    MD1_TimeWork;
    // uint16_t    MD1_TimeDelay;
    // uint8_t     MD1_ADC_CH;
    // uint8_t     MD1_StartInstr;
    // /*Individual com3 bytes - Angle Mode*/
    // uint8_t     MD2_SelMotor;
    // MoveType    MD2_MoveType;
    // uint16_t    MD2_Angle;
    // uint16_t    MD2_Time;
    // uint16_t    MD2_Speed;
    // uint16_t    MD2_Delay;
    // uint8_t     MD2_FeedBack;
    // uint8_t     MD2_StartInstr;
    // /*Individual com4 bytes - Setting*/

    // /*Individual com5 bytes - Status Mode*/

public:
    ProtezMotor(uint8_t _id);
    ~ProtezMotor();

    void setID(uint8_t);
    uint8_t getID();

    // void setSidePlate(Protez_SideType);
    Protez_SideType getSidePlate();

    // void setWorkMode(Protez_WorkMode);
    Protez_WorkMode getWorkMode();

    // void setDirection(Protez_TypeMove);
    Protez_TypeMove getDirection();

    // void setPWM(uint8_t);
    uint8_t getPWM();

    // void setWorkTime(uint16_t);
    uint16_t getWorkTime();

    // void setAngle(uint16_t);
    uint16_t getAngle();

    // void setSpeed(uint16_t);
    uint16_t getSpeed();

    // void setWorkDelay(uint16_t);
    uint16_t getWorkDelay();

    // void setADC_State(Protez_ADCState);
    Protez_ADCState getADC_State();

    // void setFeedBack(Protez_FeedBack);
    Protez_FeedBack getFeedBack();

    // void setRunning(Protez_Running);
    Protez_Running getRunning();

    /**********/

};
































#endif // PROTEZMOTOR_H
