#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"
#include "iostream"
#include <string.h>
#include <QTimer>

#include <QDebug>

/*Common com mask*/
#define MASK_COM_SIDEPLATE    0x0001
#define MASK_COM_WORKMODE     0x0002
#define MASK_COM_START_INSTR  0x0100
/*COM1 mask*/
#define MASK_COM1_SELMOTOR    0x0004
#define MASK_COM1_TYPEMOVE    0x0008
#define MASK_COM1_PWM         0x0010
#define MASK_COM1_TIMEWORK    0x0020
#define MASK_COM1_DELAY       0x0040
#define MASK_COM1_ADC         0x0080
/*COM2 mask*/
// #define MASK_COM2_ANGLE       0x0010
// #define MASK_COM2_TIME        0x0020
// #define MASK_COM2_SPEED       0x0040
// #define MASK_COM2_DELAY       0x0080
// #define MASK_COM2_FEEDBACK    0x0100
/*COM3 mask*/
// #define MASK_COM2_ANGLE       0x0010
// #define MASK_COM2_TIME        0x0020
// #define MASK_COM2_SPEED       0x0040
// #define MASK_COM2_DELAY       0x0080
// #define MASK_COM2_FEEDBACK    0x0100

typedef struct CommandStruct
{
    /*Common com bytes*/
    // uint8_t COM0_Config_1;
    // uint8_t COM0_Config_2;
    uint8_t COM0_StartInstruct;

} CommandStruct;



/******************************************************/
    //Windows Definitions
/******************************************************/
typedef struct GLB_WindowsObjects
{
    QLabel *GLB_WindowsLabel[20];
    QPushButton *GLB_WindowsButton[50];
    QSlider *GLB_WindowsSlider[20];
    QLineEdit *GLB_WindowsLineEdit[50];
    QCheckBox *GLB_WindowsCheckBox[20];
    QComboBox *GLB_WindowsComboBox[2];
    QRadioButton *GLB_WindowsRadioButton[10];
    QCustomPlot *GLB_WindowsCustomPlot[5];
    QPlainTextEdit *GLB_WindowsPlainTextEdit[5];

    QTabWidget *GLB_WindowsTab[5];
    QWidget *GLB_WindowsTabWidget[5];
    QFrame *GLB_WindowsFrame[10];

} GLB_WindowsObjects;


/******************************************************/
typedef enum MoveType
{
    FREE = 0,
    LEFT,
    RIGHT,
    HOLD,
    ANGLE_MODE,
} MoveType;
typedef enum NumPlate
{
    INTERN_PLATE = 0x00,
    EXTERN_PLATE = 0x01,
} NumPlate;

typedef struct CMD_IntFlags
{
    bool FL_StatusByte;
    bool FL_SelMotByte;
    bool FL_TypeMoveByte;
    bool FL_PWMByte;
    bool FL_TimeWorkByte;
    bool FL_DelayWorkByte;
    bool FL_ADC_EnByte;

    bool FL2_AngleByte;
    bool FL2_TimeByte;
    bool FL2_SpeedByte;
    bool FL2_DelayByte;
    bool FL2_FeedBack;
} CMD_IntFlags;

typedef struct CMD_Global
{
    uint8_t ModeWorkByte;
    uint8_t StartInstruct;
} CMD_Global;

typedef struct MotorCom
{
    uint16_t configHalfWord;
    uint8_t Status;

    uint8_t SelMotor;
    MoveType TypeMove;
    uint8_t PWM;
    uint16_t TimeWork;
    uint16_t DelayWork;
    uint8_t ADC_En;

    CMD_IntFlags Flags;

    uint16_t CTRL_2_Angle;
    uint16_t CTRL_2_Time;
    uint16_t CTRL_2_Speed;
    uint16_t CTRL_2_Delay;
    uint8_t CTRL_2_FeedBack;

} MotorCom;
typedef enum RatioChannelsNumber
{
    Ratio_CH_1 = 0,
    Ratio_CH_2,
    Ratio_CH_3,
    Ratio_CH_4,
    Ratio_CH_5,
    Ratio_CH_6,
    Ratio_CH_None,
} RatioChannelsNumber;

typedef enum WorkModeEnum
{
    WRM_None = 0,
    WRM_PWM_MODE = 0x01,
    WRM_ANGLE_MODE = 0x02,
} WorkModeEnum;

typedef struct MotorDef
{
    QPushButton *TAB1_ComporessButton;
    QPushButton *TAB1_DecompressButton;
    QPushButton *TAB1_HoldButton;
    QPushButton *TAB1_FreeButton;
    QLineEdit   *TAB1_LineEditPWM;
    QSlider     *TAB1_SliderPWM;
    QLineEdit   *TAB1_LineEditWorkTime;
    QLineEdit   *TAB1_LineEditDelayTime;
    QCheckBox   *TAB1_CheckBoxADC;
    QCheckBox   *TAB1_CheckBoxBackSide;


    /*Main com0 bytes*/
    uint8_t     MD_Config_1;
    uint8_t     MD_Config_2;
    WorkModeEnum MD_WorkMode;
    NumPlate     MD_SidePlate;
    /*Individual com2 bytes - PWM Mode*/
    uint8_t     MD_SelMotor;
    MoveType    MD_MoveType;
    uint16_t    MD_PWM;
    uint16_t    MD_TimeWork;
    uint16_t    MD_TimeDelay;
    uint8_t     MD_ADC_CH;
    /*Individual com1 bytes - Status Mode*/

    /*Individual com3 bytes - Angle Mode*/

} MotorDef;



//
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void ComPortSearch(uint32_t boudrate);
    void ComportDataUpdate_slot();

    void ComPortOpen(QString com, uint32_t boudrate);
    void ComportConnect_slot();

    void ComportClose_slot();

    void ComportRead_slot();
    void ComPortRead();

    void ComPortWrite(uint8_t *datatosend, uint32_t cntdata);
    void ComportWrite_slot(QString back);

    void PaintGraph();
    void PaintGraph2();

private slots:


    void on_pushButton_12_clicked();

    void on_horizontalSlider_valueChanged(int value);

    void on_lineEdit_textEdited(const QString &arg1);

    void on_lineEdit_2_textEdited(const QString &arg1);

    void on_lineEdit_3_textEdited(const QString &arg1);

    void on_lineEdit_4_textEdited(const QString &arg1);

    void on_lineEdit_5_textEdited(const QString &arg1);

    void on_horizontalSlider_2_valueChanged(int value);

    void on_horizontalSlider_3_valueChanged(int value);

    void on_horizontalSlider_4_valueChanged(int value);

    void on_horizontalSlider_5_valueChanged(int value);

    void on_lineEdit_6_textEdited(const QString &arg1);

    void on_lineEdit_7_textEdited(const QString &arg1);

    void on_lineEdit_10_textEdited(const QString &arg1);

    void on_lineEdit_9_textEdited(const QString &arg1);

    void on_lineEdit_8_textEdited(const QString &arg1);

    void on_lineEdit_11_textEdited(const QString &arg1);

    void on_lineEdit_12_textEdited(const QString &arg1);

    void on_lineEdit_15_textEdited(const QString &arg1);

    void on_lineEdit_14_textEdited(const QString &arg1);

    void on_lineEdit_13_textEdited(const QString &arg1);

    void on_comboBox_textActivated(const QString &arg1);

    void on_pushButton_13_clicked();

    void on_lineEdit_23_textEdited(const QString &arg1);

    void on_lineEdit_22_textEdited(const QString &arg1);

    void on_lineEdit_21_textEdited(const QString &arg1);

    void on_lineEdit_24_textEdited(const QString &arg1);

    void on_lineEdit_25_textEdited(const QString &arg1);

    void on_lineEdit_28_textEdited(const QString &arg1);

    void on_lineEdit_27_textEdited(const QString &arg1);

    void on_lineEdit_26_textEdited(const QString &arg1);

    void on_lineEdit_29_textEdited(const QString &arg1);

    void on_lineEdit_30_textEdited(const QString &arg1);

    void on_lineEdit_33_textEdited(const QString &arg1);

    void on_lineEdit_35_textEdited(const QString &arg1);

    void on_lineEdit_34_textEdited(const QString &arg1);

    void on_lineEdit_31_textEdited(const QString &arg1);

    void on_lineEdit_32_textEdited(const QString &arg1);

    void on_checkBox_11_toggled(bool checked);

    void on_pushButton_21_clicked();

    void on_pushButton_23_clicked();

    void on_pushButton_19_clicked();

    void on_pushButton_22_clicked();

    void on_pushButton_20_clicked();

    void on_pushButton_29_clicked();

    void on_pushButton_30_clicked();

    void on_pushButton_32_clicked();

    void on_pushButton_33_clicked();

    void on_pushButton_31_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_27_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_28_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_26_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_24_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_25_clicked();

    void on_horizontalSlider_6_valueChanged(int value);

    void on_pushButton_34_clicked();

    void on_pushButton_36_clicked();

    void on_radioButton_4_clicked();

    void on_radioButton_5_clicked();

    void on_radioButton_6_clicked();

    void on_radioButton_7_clicked();

    void on_radioButton_8_clicked();

    void on_checkBox_6_toggled(bool checked);

    void on_checkBox_7_toggled(bool checked);

    void on_checkBox_8_toggled(bool checked);

    void on_pushButton_14_clicked(bool checked);

    void on_pushButton_15_clicked(bool checked);

    void on_pushButton_16_clicked(bool checked);

    void on_pushButton_17_clicked(bool checked);

    void on_pushButton_18_clicked(bool checked);

    void on_radioButton_clicked();

    void on_pushButton_35_clicked();

    void on_SearchButton_clicked();


    void on_pushButton_37_clicked();

    void on_pushButton_38_clicked();

    void on_pushButton_39_clicked();

    void on_checkBox_9_toggled(bool checked);

    void on_pushButton_11_clicked();

    void on_checkBox_10_toggled(bool checked);

public:
    Ui::MainWindow *ui;
private:
    // QSerialPort *serialDevice1;  // Первый последовательный порт
signals:
    void ComportSearch_signal();
    void ComportConnect_signal();
    void ComportClose_signal();



};

#endif // MAINWINDOW_H
