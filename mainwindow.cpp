#include "mainwindow.h"

#include <QVector>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include "MyThread.h"

/*GUI variables*/
Ui::MainWindow* GLB_ui = nullptr;
QSerialPort *GLB_Ports[2] = {nullptr};
GLB_WindowsObjects GLB_WinObj;
uint16_t GLB_SliderValue[6];
QString NowComBoxItem[2];
QWidget* GLB_mainwindowWidget;
MainWindow *GLB_mainwindow;
MotorCom MotorInstr[6];
CMD_Global GLB_Command;


/*ComPort variables*/
bool isConnectedComPort = false;
uint32_t PackToRecv;
uint32_t CountPoints = 5000;
uint32_t GLB_I;
QList<QString> GLB_Comports;
QString CurrentComPort;
uint32_t CurrentBoundRate;
extern uint32_t ComportCountdataRecv;
uint8_t ComportDataToSend[50];
uint32_t ComportCountDataToSend;

/*Graph variables*/
QVector<uint8_t> GLB_Graph_x, GLB_Graph_y;

/*File variables*/
QString RepositoryURL;
QString RepositoryNameFile[1] = {"config"};
QString FileType[2] = {".txt", ".dat"};
QFile ConfigFile;
QTextStream IOFile;
QString DataFromFile[10];

/*Work variables*/
MotorDef MotorDefStruct[6];
CommandStruct MotorCommand[6];

typedef enum Fingers
{
    Thrunb = 0x00,
    Index = 0x01,
    Middle = 0x02,
    Ring = 0x03,
    Pinkie = 0x04,
} Fingers;
bool GlobalFlagsMotor[10] = {0, }; // [0] FLAGS_ENABLE_ADC_CHANNEL > 0, [1] FLAGS_ENABLE_FEEDBACK > 0

/*Thread variables*/
MyThread_1 *thread_1;
MyThread_2 *thread_2;
bool GLB_Thread_Flag[2] = {false, };
uint32_t dataRecvd2 = 0;
uint8_t TypeThreadInterrupt;    // 0 - Graph, 1 - Search
bool ThreadAutoConnectState;
/*Thread 2*/


/*Angle Control variables*/
RatioChannelsNumber RatioStateNow;
bool CtrlCheckState[3];

/********************************/
void SendToTerminal(QString data, bool Newline, uint8_t tab_widget)
{
    if(tab_widget == 0)
    {
        if(Newline) GLB_WinObj.GLB_WindowsPlainTextEdit[2]->appendPlainText(data);
        else GLB_WinObj.GLB_WindowsPlainTextEdit[2]->insertPlainText(data);
    }
    else if(tab_widget == 1)
    {
        if(Newline) GLB_WinObj.GLB_WindowsPlainTextEdit[1]->appendPlainText(data);
        else GLB_WinObj.GLB_WindowsPlainTextEdit[1]->insertPlainText(data);
    }
    else if(tab_widget == 2)
    {
        if(Newline) GLB_WinObj.GLB_WindowsPlainTextEdit[0]->appendPlainText(data);
        else GLB_WinObj.GLB_WindowsPlainTextEdit[0]->insertPlainText(data);
    }
}
void ClearTerminal(uint8_t tab_widget)
{

    if(tab_widget == 0)
    {
        GLB_WinObj.GLB_WindowsPlainTextEdit[2]->clear();
    }
    else if(tab_widget == 1)
    {
        GLB_WinObj.GLB_WindowsPlainTextEdit[1]->clear();
    }
    else if(tab_widget == 2)
    {
        GLB_WinObj.GLB_WindowsPlainTextEdit[0]->clear();
    }
}
/********************************/
double FormulaADC(uint16_t currentADC)
{
    float Voltage = 3.3;
    float coefficientGaing = 3;
    float Shunt = 0.1;
    return ((Voltage / 4096) * currentADC / coefficientGaing / Shunt);
}
double _2ByteTo_1Byte(uint16_t halfWorld)
{
    return halfWorld;
}
void MainWindow::PaintGraph()
{
    double Step;
    float value_time;
    QVector<double> NewGraph_x, NewGraph_y;
    for(uint16_t i = 0; i < GLB_Graph_y.size() / 2; i = i + 2)
    {
        NewGraph_y.append(FormulaADC(
            (GLB_Graph_y[i]) | (GLB_Graph_y[i + 1] << 8)
            ));
    }
    for(uint16_t i = 5; i < NewGraph_y.size(); i++)
        NewGraph_y[i] = (NewGraph_y[i] + NewGraph_y[i - 1] + NewGraph_y[i - 2] + NewGraph_y[i - 3] + NewGraph_y[i - 4] + NewGraph_y[i - 5]) / 6.0;

    double max_val_x = -100;
    double min_val_x = 1000;

    double max_val_y = -100;
    double min_val_y = 1000;

    /*Settings Graph 1*/
    if(GLB_ui->checkBox->isChecked()) value_time = std::stof(GLB_ui->lineEdit_6->text().toStdString());
    else if(GLB_ui->checkBox_2->isChecked()) value_time = std::stof(GLB_ui->lineEdit_7->text().toStdString());
    else if(GLB_ui->checkBox_3->isChecked()) value_time = std::stof(GLB_ui->lineEdit_10->text().toStdString());
    else if(GLB_ui->checkBox_4->isChecked()) value_time = std::stof(GLB_ui->lineEdit_9->text().toStdString());
    else if(GLB_ui->checkBox_5->isChecked()) value_time = std::stof(GLB_ui->lineEdit_8->text().toStdString());
    else if(GLB_ui->checkBox_13->isChecked()) value_time = std::stof(GLB_ui->lineEdit_37->text().toStdString());

    Step = value_time / NewGraph_y.size();
    for(double i = 0; i < NewGraph_y.size(); i++)
    {
        NewGraph_x.append(Step * i);
        if(max_val_x < NewGraph_x[i]) max_val_x = NewGraph_x[i];
        if(min_val_x > NewGraph_x[i]) min_val_x = NewGraph_x[i];
        if(max_val_y < NewGraph_y[i]) max_val_y = NewGraph_y[i];
        if(min_val_y > NewGraph_y[i]) min_val_y = NewGraph_y[i];
    }


    GLB_ui->widget->xAxis->setRange(min_val_x, max_val_x);
    GLB_ui->widget->yAxis->setRange(min_val_y, max_val_y);
    GLB_ui->widget->addGraph();
    GLB_ui->widget->graph(0)->setPen(QPen(Qt::blue));
    GLB_ui->widget->graph(0)->setData(NewGraph_x, NewGraph_y);
    GLB_ui->widget->replot();
}
void MainWindow::PaintGraph2()
{
    double Step;
    QVector<double> NewGraph2_x, NewGraph2_y;
    for(uint16_t i = 0; i < GLB_Graph_y.size(); i = i + 2)
    {
        NewGraph2_y.append(_2ByteTo_1Byte(
            (GLB_Graph_y[i]) | (GLB_Graph_y[i + 1] << 8)
            ));
    }

    double max_val_x = 0.0;
    double min_val_x = 0.0;

    double max_val_y = 0.0;
    double min_val_y = 0.0;

    Step = 1.0;
    for(double i = 0; i < NewGraph2_y.size(); i++)
    {
        NewGraph2_x.append(i);
        if(max_val_x < NewGraph2_x[i]) max_val_x = NewGraph2_x[i];
        if(min_val_x > NewGraph2_x[i]) min_val_x = NewGraph2_x[i];
        if(max_val_y < NewGraph2_y[i]) max_val_y = NewGraph2_y[i];
        if(min_val_y > NewGraph2_y[i]) min_val_y = NewGraph2_y[i];
    }

    GLB_ui->widget_2->xAxis->setRange(min_val_x, max_val_x);
    GLB_ui->widget_2->yAxis->setRange(min_val_y, max_val_y);
    GLB_ui->widget_2->addGraph();
    GLB_ui->widget_2->graph(0)->setPen(QPen(Qt::red));
    GLB_ui->widget_2->graph(0)->setData(NewGraph2_x, NewGraph2_y);
    GLB_ui->widget_2->replot();
}


void SetStartGUISettings()
{
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsButton[0] = GLB_ui->SearchButton;     // Tab 0 - Search Button
    GLB_WinObj.GLB_WindowsButton[1] = GLB_ui->pushButton_13;    // Tab 0 - File Repository Button


    GLB_WinObj.GLB_WindowsButton[2] = GLB_ui->pushButton;       // Tab 1 - Compres 0
    GLB_WinObj.GLB_WindowsButton[3] = GLB_ui->pushButton_3;     // Tab 1 - Compres 1
    GLB_WinObj.GLB_WindowsButton[4] = GLB_ui->pushButton_5;     // Tab 1 - Compres 2
    GLB_WinObj.GLB_WindowsButton[5] = GLB_ui->pushButton_7;     // Tab 1 - Compres 3
    GLB_WinObj.GLB_WindowsButton[6] = GLB_ui->pushButton_9;     // Tab 1 - Compres 4
    GLB_WinObj.GLB_WindowsButton[7] = GLB_ui->pushButton_30;    // Tab 1 - Left 5

    GLB_WinObj.GLB_WindowsButton[8] = GLB_ui->pushButton_21;    // Tab 1 - Hold 0
    GLB_WinObj.GLB_WindowsButton[9] = GLB_ui->pushButton_23;    // Tab 1 - Hold 1
    GLB_WinObj.GLB_WindowsButton[10] = GLB_ui->pushButton_19;   // Tab 1 - Hold 2
    GLB_WinObj.GLB_WindowsButton[11] = GLB_ui->pushButton_22;   // Tab 1 - Hold 3
    GLB_WinObj.GLB_WindowsButton[12] = GLB_ui->pushButton_20;   // Tab 1 - Hold 4
    GLB_WinObj.GLB_WindowsButton[13] = GLB_ui->pushButton_32;   // Tab 1 - Hold 5

    GLB_WinObj.GLB_WindowsButton[14] = GLB_ui->pushButton_27;   // Tab 1 - Free 0
    GLB_WinObj.GLB_WindowsButton[15] = GLB_ui->pushButton_28;   // Tab 1 - Free 1
    GLB_WinObj.GLB_WindowsButton[16] = GLB_ui->pushButton_26;   // Tab 1 - Free 2
    GLB_WinObj.GLB_WindowsButton[17] = GLB_ui->pushButton_24;   // Tab 1 - Free 3
    GLB_WinObj.GLB_WindowsButton[18] = GLB_ui->pushButton_25;   // Tab 1 - Free 4
    GLB_WinObj.GLB_WindowsButton[19] = GLB_ui->pushButton_33;   // Tab 1 - Free 5

    GLB_WinObj.GLB_WindowsButton[20] = GLB_ui->pushButton_2;    // Tab 1 - Decompress 0
    GLB_WinObj.GLB_WindowsButton[21] = GLB_ui->pushButton_4;    // Tab 1 - Decompress 1
    GLB_WinObj.GLB_WindowsButton[22] = GLB_ui->pushButton_6;    // Tab 1 - Decompress 2
    GLB_WinObj.GLB_WindowsButton[23] = GLB_ui->pushButton_8;    // Tab 1 - Decompress 3
    GLB_WinObj.GLB_WindowsButton[24] = GLB_ui->pushButton_10;   // Tab 1 - Decompress 4
    GLB_WinObj.GLB_WindowsButton[25] = GLB_ui->pushButton_31;   // Tab 1 - Right 5

    GLB_WinObj.GLB_WindowsButton[27] = GLB_ui->pushButton_29;   // Tab 1 - Configurate
    GLB_WinObj.GLB_WindowsButton[28] = GLB_ui->pushButton_34;   // Tab 1 - Start Instruct


    GLB_WinObj.GLB_WindowsButton[29] = GLB_ui->pushButton_14;   // Tab 2 - Thrumb
    GLB_WinObj.GLB_WindowsButton[30] = GLB_ui->pushButton_15;   // Tab 2 - Index
    GLB_WinObj.GLB_WindowsButton[31] = GLB_ui->pushButton_16;   // Tab 2 - Middle
    GLB_WinObj.GLB_WindowsButton[32] = GLB_ui->pushButton_17;   // Tab 2 - Ring
    GLB_WinObj.GLB_WindowsButton[33] = GLB_ui->pushButton_18;   // Tab 2 - Pinkie

    GLB_WinObj.GLB_WindowsButton[34] = GLB_ui->pushButton_36;   // Tab 2 - Configurate
    GLB_WinObj.GLB_WindowsButton[35] = GLB_ui->pushButton_35;   // Tab 2 - Start Instruct

    GLB_WinObj.GLB_WindowsButton[36] = GLB_ui->pushButton_37;   // Tab 2 - Clear Terminal

    GLB_WinObj.GLB_WindowsButton[37] = GLB_ui->pushButton_11;   // Tab 0 - Debug Mode Start motor
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsComboBox[0] = GLB_ui->comboBox;       // Tab 0 - Select Comport
    GLB_WinObj.GLB_WindowsComboBox[1] = GLB_ui->comboBox_2;     // Tab 0 - Debug Mode select motor
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsCheckBox[0] = GLB_ui->checkBox_11;    // Tab 0 - Enable Receive ComPort 1

    GLB_WinObj.GLB_WindowsCheckBox[1] = GLB_ui->checkBox;       // Tab 1 - ADC_CH_0
    GLB_WinObj.GLB_WindowsCheckBox[2] = GLB_ui->checkBox_2;     // Tab 1 - ADC_CH_1
    GLB_WinObj.GLB_WindowsCheckBox[3] = GLB_ui->checkBox_3;     // Tab 1 - ADC_CH_2
    GLB_WinObj.GLB_WindowsCheckBox[4] = GLB_ui->checkBox_4;     // Tab 1 - ADC_CH_3
    GLB_WinObj.GLB_WindowsCheckBox[5] = GLB_ui->checkBox_5;     // Tab 1 - ADC_CH_4
    GLB_WinObj.GLB_WindowsCheckBox[6] = GLB_ui->checkBox_13;    // Tab 1 - ADC_CH_5


    GLB_WinObj.GLB_WindowsCheckBox[7] = GLB_ui->checkBox_6;     // Tab 2 - Enable Angle HalfMode
    GLB_WinObj.GLB_WindowsCheckBox[8] = GLB_ui->checkBox_7;     // Tab 2 - Enable Time HalfMode
    GLB_WinObj.GLB_WindowsCheckBox[9] = GLB_ui->checkBox_8;     // Tab 2 - Enable Speed HalfMode

    GLB_WinObj.GLB_WindowsCheckBox[10] = GLB_ui->checkBox_9;    // Tab 0 - Debug Mode Enable
    GLB_WinObj.GLB_WindowsCheckBox[11] = GLB_ui->checkBox_12;    // Tab 0 - Debug Mode Reverse direction
    GLB_WinObj.GLB_WindowsCheckBox[12] = GLB_ui->checkBox_10;    // Tab 0 - Auto-connect Comport

    GLB_WinObj.GLB_WindowsCheckBox[13] = GLB_ui->checkBox_14;    // Tab 0 - The back of the hand
    GLB_WinObj.GLB_WindowsCheckBox[14] = GLB_ui->checkBox_15;    // Tab 0 - Enable auto currect PWM for the back of the hand
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsLineEdit[0] = GLB_ui->lineEdit_20;    // Tab 0 - File Repository

    GLB_WinObj.GLB_WindowsLineEdit[1] = GLB_ui->lineEdit;       // Tab 1 - PWM 0 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[2] = GLB_ui->lineEdit_2;     // Tab 1 - PWM 1 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[3] = GLB_ui->lineEdit_3;     // Tab 1 - PWM 2 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[4] = GLB_ui->lineEdit_4;     // Tab 1 - PWM 3 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[5] = GLB_ui->lineEdit_5;     // Tab 1 - PWM 4 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[6] = GLB_ui->lineEdit_36;    // Tab 1 - PWM 5 Line Edit

    GLB_WinObj.GLB_WindowsLineEdit[7] = GLB_ui->lineEdit_6;     // Tab 1 - Time 0 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[8] = GLB_ui->lineEdit_7;     // Tab 1 - Time 1 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[9] = GLB_ui->lineEdit_10;    // Tab 1 - Time 2 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[10] = GLB_ui->lineEdit_9;    // Tab 1 - Time 3 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[11] = GLB_ui->lineEdit_8;    // Tab 1 - Time 4 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[12] = GLB_ui->lineEdit_37;   // Tab 1 - Time 5 Line Edit

    GLB_WinObj.GLB_WindowsLineEdit[13] = GLB_ui->lineEdit_11;   // Tab 1 - Delay 0 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[14] = GLB_ui->lineEdit_12;   // Tab 1 - Delay 1 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[15] = GLB_ui->lineEdit_15;   // Tab 1 - Delay 2 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[16] = GLB_ui->lineEdit_14;   // Tab 1 - Delay 3 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[17] = GLB_ui->lineEdit_13;   // Tab 1 - Delay 4 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[18] = GLB_ui->lineEdit_38;   // Tab 1 - Delay 5 Line Edit


    GLB_WinObj.GLB_WindowsLineEdit[19] = GLB_ui->lineEdit_23;   // Tab 2 - Angle 0 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[20] = GLB_ui->lineEdit_22;   // Tab 2 - Angle 1 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[21] = GLB_ui->lineEdit_21;   // Tab 2 - Angle 2 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[22] = GLB_ui->lineEdit_24;   // Tab 2 - Angle 3 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[23] = GLB_ui->lineEdit_25;   // Tab 2 - Angle 4 Line Edit

    GLB_WinObj.GLB_WindowsLineEdit[24] = GLB_ui->lineEdit_28;   // Tab 2 - Time 0 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[25] = GLB_ui->lineEdit_27;   // Tab 2 - Time 1 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[26] = GLB_ui->lineEdit_26;   // Tab 2 - Time 2 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[27] = GLB_ui->lineEdit_29;   // Tab 2 - Time 3 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[28] = GLB_ui->lineEdit_30;   // Tab 2 - Time 4 Line Edit

    GLB_WinObj.GLB_WindowsLineEdit[29] = GLB_ui->lineEdit_33;   // Tab 2 - Speed 0 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[30] = GLB_ui->lineEdit_35;   // Tab 2 - Speed 1 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[31] = GLB_ui->lineEdit_34;   // Tab 2 - Speed 2 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[32] = GLB_ui->lineEdit_31;   // Tab 2 - Speed 3 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[33] = GLB_ui->lineEdit_32;   // Tab 2 - Speed 4 Line Edit

    GLB_WinObj.GLB_WindowsLineEdit[34] = GLB_ui->lineEdit_44;   // Tab 2 - Delay 0 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[35] = GLB_ui->lineEdit_41;   // Tab 2 - Delay 1 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[36] = GLB_ui->lineEdit_42;   // Tab 2 - Delay 2 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[37] = GLB_ui->lineEdit_43;   // Tab 2 - Delay 3 Line Edit
    GLB_WinObj.GLB_WindowsLineEdit[38] = GLB_ui->lineEdit_40;   // Tab 2 - Delay 4 Line Edit

    GLB_WinObj.GLB_WindowsLineEdit[39] = GLB_ui->lineEdit_16;   // Tab 0 - Debug Mode Time work
    GLB_WinObj.GLB_WindowsLineEdit[40] = GLB_ui->lineEdit_17;   // Tab 0 - Baudrate lineEdit
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsRadioButton[0] = GLB_ui->radioButton_4; // Tab 2 - FeedBack CH_0
    GLB_WinObj.GLB_WindowsRadioButton[1] = GLB_ui->radioButton_5; // Tab 2 - FeedBack CH_1
    GLB_WinObj.GLB_WindowsRadioButton[2] = GLB_ui->radioButton_6; // Tab 2 - FeedBack CH_2
    GLB_WinObj.GLB_WindowsRadioButton[3] = GLB_ui->radioButton_7; // Tab 2 - FeedBack CH_3
    GLB_WinObj.GLB_WindowsRadioButton[4] = GLB_ui->radioButton_8; // Tab 2 - FeedBack CH_4
    GLB_WinObj.GLB_WindowsRadioButton[5] = GLB_ui->radioButton;   // Tab 2 - FeedBack NONE

    GLB_WinObj.GLB_WindowsRadioButton[6] = GLB_ui->radioButton_2; // Tab 0 - Debug Mode Upper part
    GLB_WinObj.GLB_WindowsRadioButton[7] = GLB_ui->radioButton_3; // Tab 0 - Debug Mode Lower part
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsCustomPlot[0] = GLB_ui->widget;         // Tab 1 - FeedBack ADC_Graph
    GLB_WinObj.GLB_WindowsCustomPlot[1] = GLB_ui->widget_2;       // Tab 2 - FeedBack FeedBack_Graph
    GLB_WinObj.GLB_WindowsCustomPlot[2] = GLB_ui->widget_3;       // Tab 2 - FeedBack FeedBack_Graph
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsLabel[0] = GLB_ui->label_4;             // Tab 0 - Repository label
    GLB_WinObj.GLB_WindowsLabel[1] = GLB_ui->label_13;            // Tab 1 - PWM label
    GLB_WinObj.GLB_WindowsLabel[2] = GLB_ui->label_14;            // Tab 1 - Time work label
    GLB_WinObj.GLB_WindowsLabel[3] = GLB_ui->label_15;            // Tab 1 - Delay label
    GLB_WinObj.GLB_WindowsLabel[4] = GLB_ui->label_21;            // Tab 1 - ADC label


    GLB_WinObj.GLB_WindowsLabel[5] = GLB_ui->label_5;            // Tab 2 - Angle label
    GLB_WinObj.GLB_WindowsLabel[6] = GLB_ui->label_8;            // Tab 2 - Time work label
    GLB_WinObj.GLB_WindowsLabel[7] = GLB_ui->label_9;            // Tab 2 - Speed label
    GLB_WinObj.GLB_WindowsLabel[8] = GLB_ui->label_10;           // Tab 2 - Delay label

    GLB_WinObj.GLB_WindowsLabel[9] = GLB_ui->label;              // Tab 0 - Debug Mode Title
    GLB_WinObj.GLB_WindowsLabel[10] = GLB_ui->label_2;           // Tab 0 - Baudrate label
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsSlider[0] = GLB_ui->horizontalSlider;            // Tab 1 - PWM 0 value Slider
    GLB_WinObj.GLB_WindowsSlider[1] = GLB_ui->horizontalSlider_2;          // Tab 1 - PWM 1 value Slider
    GLB_WinObj.GLB_WindowsSlider[2] = GLB_ui->horizontalSlider_3;          // Tab 1 - PWM 2 value Slider
    GLB_WinObj.GLB_WindowsSlider[3] = GLB_ui->horizontalSlider_4;          // Tab 1 - PWM 3 value Slider
    GLB_WinObj.GLB_WindowsSlider[4] = GLB_ui->horizontalSlider_5;          // Tab 1 - PWM 4 value Slider
    GLB_WinObj.GLB_WindowsSlider[5] = GLB_ui->horizontalSlider_6;          // Tab 1 - PWM 5 value Slider
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsPlainTextEdit[0] = GLB_ui->plainTextEdit;        // Tab 0 - Terminal Line Edit

    GLB_WinObj.GLB_WindowsPlainTextEdit[1] = GLB_ui->plainTextEdit_2;      // Tab 1 - Terminal Line Edit

    GLB_WinObj.GLB_WindowsPlainTextEdit[2] = GLB_ui->plainTextEdit_3;      // Tab 2 - Terminal Line Edit
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsFrame[0] = GLB_ui->frame;                        // Tab 0 - Debug Mode frame
    GLB_WinObj.GLB_WindowsFrame[1] = GLB_ui->frame_2;                      // Tab 0 - Debug Mode frame Motor check
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsTab[0] = GLB_ui->tabWidget;                      // Tab self
    /****************************************************************************************/
    GLB_WinObj.GLB_WindowsTabWidget[0] = GLB_ui->tab_3;                    // Tab widget 0
    GLB_WinObj.GLB_WindowsTabWidget[1] = GLB_ui->tab;                      // Tab widget 1
    GLB_WinObj.GLB_WindowsTabWidget[2] = GLB_ui->tab_2;                    // Tab widget 2
    /****************************************************************************************/


    /*****************************************/
    MotorDefStruct[0].TAB1_ComporessButton = GLB_WinObj.GLB_WindowsButton[2];
    MotorDefStruct[0].TAB1_DecompressButton = GLB_WinObj.GLB_WindowsButton[20];
    MotorDefStruct[0].TAB1_HoldButton = GLB_WinObj.GLB_WindowsButton[8];
    MotorDefStruct[0].TAB1_FreeButton = GLB_WinObj.GLB_WindowsButton[14];

    MotorDefStruct[0].TAB1_LineEditPWM = GLB_WinObj.GLB_WindowsLineEdit[1];
    MotorDefStruct[0].TAB1_LineEditWorkTime = GLB_WinObj.GLB_WindowsLineEdit[7];
    MotorDefStruct[0].TAB1_LineEditDelayTime = GLB_WinObj.GLB_WindowsLineEdit[13];
    MotorDefStruct[0].TAB1_SliderPWM = GLB_WinObj.GLB_WindowsSlider[0];
    MotorDefStruct[0].TAB1_CheckBoxADC = GLB_WinObj.GLB_WindowsCheckBox[1];
    MotorDefStruct[0].TAB1_CheckBoxBackSide = GLB_WinObj.GLB_WindowsCheckBox[13];
    /*****************************************/
    MotorDefStruct[1].TAB1_ComporessButton = GLB_WinObj.GLB_WindowsButton[3];
    MotorDefStruct[1].TAB1_DecompressButton = GLB_WinObj.GLB_WindowsButton[21];
    MotorDefStruct[1].TAB1_HoldButton = GLB_WinObj.GLB_WindowsButton[9];
    MotorDefStruct[1].TAB1_FreeButton = GLB_WinObj.GLB_WindowsButton[15];

    MotorDefStruct[1].TAB1_LineEditPWM = GLB_WinObj.GLB_WindowsLineEdit[2];
    MotorDefStruct[1].TAB1_LineEditWorkTime = GLB_WinObj.GLB_WindowsLineEdit[8];
    MotorDefStruct[1].TAB1_LineEditDelayTime = GLB_WinObj.GLB_WindowsLineEdit[14];
    MotorDefStruct[1].TAB1_SliderPWM = GLB_WinObj.GLB_WindowsSlider[1];
    MotorDefStruct[1].TAB1_CheckBoxADC = GLB_WinObj.GLB_WindowsCheckBox[2];
    MotorDefStruct[1].TAB1_CheckBoxBackSide = GLB_WinObj.GLB_WindowsCheckBox[13];
    /*****************************************/
    MotorDefStruct[2].TAB1_ComporessButton = GLB_WinObj.GLB_WindowsButton[4];
    MotorDefStruct[2].TAB1_DecompressButton = GLB_WinObj.GLB_WindowsButton[22];
    MotorDefStruct[2].TAB1_HoldButton = GLB_WinObj.GLB_WindowsButton[10];
    MotorDefStruct[2].TAB1_FreeButton = GLB_WinObj.GLB_WindowsButton[16];


    MotorDefStruct[2].TAB1_LineEditPWM = GLB_WinObj.GLB_WindowsLineEdit[3];
    MotorDefStruct[2].TAB1_LineEditWorkTime = GLB_WinObj.GLB_WindowsLineEdit[9];
    MotorDefStruct[2].TAB1_LineEditDelayTime = GLB_WinObj.GLB_WindowsLineEdit[15];
    MotorDefStruct[2].TAB1_SliderPWM = GLB_WinObj.GLB_WindowsSlider[2];
    MotorDefStruct[2].TAB1_CheckBoxADC = GLB_WinObj.GLB_WindowsCheckBox[3];
    MotorDefStruct[2].TAB1_CheckBoxBackSide = GLB_WinObj.GLB_WindowsCheckBox[13];
    /*****************************************/
    MotorDefStruct[3].TAB1_ComporessButton = GLB_WinObj.GLB_WindowsButton[5];
    MotorDefStruct[3].TAB1_DecompressButton = GLB_WinObj.GLB_WindowsButton[23];
    MotorDefStruct[3].TAB1_HoldButton = GLB_WinObj.GLB_WindowsButton[11];
    MotorDefStruct[3].TAB1_FreeButton = GLB_WinObj.GLB_WindowsButton[17];


    MotorDefStruct[3].TAB1_LineEditPWM = GLB_WinObj.GLB_WindowsLineEdit[4];
    MotorDefStruct[3].TAB1_LineEditWorkTime = GLB_WinObj.GLB_WindowsLineEdit[10];
    MotorDefStruct[3].TAB1_LineEditDelayTime = GLB_WinObj.GLB_WindowsLineEdit[16];
    MotorDefStruct[3].TAB1_SliderPWM = GLB_WinObj.GLB_WindowsSlider[3];
    MotorDefStruct[3].TAB1_CheckBoxADC = GLB_WinObj.GLB_WindowsCheckBox[4];
    MotorDefStruct[3].TAB1_CheckBoxBackSide = GLB_WinObj.GLB_WindowsCheckBox[13];
    /*****************************************/
    MotorDefStruct[4].TAB1_ComporessButton = GLB_WinObj.GLB_WindowsButton[6];
    MotorDefStruct[4].TAB1_DecompressButton = GLB_WinObj.GLB_WindowsButton[24];
    MotorDefStruct[4].TAB1_HoldButton = GLB_WinObj.GLB_WindowsButton[12];
    MotorDefStruct[4].TAB1_FreeButton = GLB_WinObj.GLB_WindowsButton[18];

    MotorDefStruct[4].TAB1_LineEditPWM = GLB_WinObj.GLB_WindowsLineEdit[5];
    MotorDefStruct[4].TAB1_LineEditWorkTime = GLB_WinObj.GLB_WindowsLineEdit[11];
    MotorDefStruct[4].TAB1_LineEditDelayTime = GLB_WinObj.GLB_WindowsLineEdit[17];
    MotorDefStruct[4].TAB1_SliderPWM = GLB_WinObj.GLB_WindowsSlider[4];
    MotorDefStruct[4].TAB1_CheckBoxADC = GLB_WinObj.GLB_WindowsCheckBox[5];
    MotorDefStruct[4].TAB1_CheckBoxBackSide = GLB_WinObj.GLB_WindowsCheckBox[13];
    /*****************************************/
    MotorDefStruct[5].TAB1_ComporessButton = GLB_WinObj.GLB_WindowsButton[7];
    MotorDefStruct[5].TAB1_DecompressButton = GLB_WinObj.GLB_WindowsButton[25];
    MotorDefStruct[5].TAB1_HoldButton = GLB_WinObj.GLB_WindowsButton[13];
    MotorDefStruct[5].TAB1_FreeButton = GLB_WinObj.GLB_WindowsButton[19];

    MotorDefStruct[5].TAB1_LineEditPWM = GLB_WinObj.GLB_WindowsLineEdit[6];
    MotorDefStruct[5].TAB1_LineEditWorkTime = GLB_WinObj.GLB_WindowsLineEdit[12];
    MotorDefStruct[5].TAB1_LineEditDelayTime = GLB_WinObj.GLB_WindowsLineEdit[18];
    MotorDefStruct[5].TAB1_SliderPWM = GLB_WinObj.GLB_WindowsSlider[5];
    MotorDefStruct[5].TAB1_CheckBoxADC = GLB_WinObj.GLB_WindowsCheckBox[6];
    MotorDefStruct[5].TAB1_CheckBoxBackSide = GLB_WinObj.GLB_WindowsCheckBox[13];
    /*****************************************/

    /****************************************************************************************/
    /*****************************************/
    /*Set validator for Line Edit*/
    QDoubleValidator *validator = new QDoubleValidator(0, 100, 2, GLB_mainwindowWidget);
    validator->setNotation(QDoubleValidator::StandardNotation);
    validator->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));

    QList<QLineEdit*> lineEdits;
    lineEdits = GLB_WinObj.GLB_WindowsTabWidget[1]->findChildren<QLineEdit*>();
    for (QLineEdit *lineEdit : lineEdits) lineEdit->setValidator(validator);
    lineEdits = GLB_WinObj.GLB_WindowsTabWidget[2]->findChildren<QLineEdit*>();
    for (QLineEdit *lineEdit : lineEdits) lineEdit->setValidator(validator);
    /*****************************************/
    /*Set background for buttons*/
    const QList<QPushButton*> PushButtons = GLB_mainwindowWidget->findChildren<QPushButton*>();
    for(QPushButton *pushbutton : PushButtons) pushbutton->setStyleSheet("background-color: rgb(150, 200, 250);");
    /*****************************************/
    // Set motor def
    GLB_WinObj.GLB_WindowsComboBox[1]->addItem("Thumb");
    GLB_WinObj.GLB_WindowsComboBox[1]->addItem("Index");
    GLB_WinObj.GLB_WindowsComboBox[1]->addItem("Middle");
    GLB_WinObj.GLB_WindowsComboBox[1]->addItem("Ring");
    GLB_WinObj.GLB_WindowsComboBox[1]->addItem("Pinkie");
    /*****************************************/
    /*Enable Debug panel*/
    GLB_WinObj.GLB_WindowsCheckBox[10]->setChecked(false);
    GLB_WinObj.GLB_WindowsFrame[1]->setEnabled(false);
}


void SetStartVariables()
{
    /*File variables*/
    RepositoryURL = "";
    ConfigFile.setFileName(RepositoryURL + RepositoryNameFile[0] + FileType[0]);
    IOFile.setDevice(&ConfigFile);

    if(ConfigFile.open(QFile::ReadOnly))
    {
        for(uint8_t i = 0; i < 10; i++) DataFromFile[i] = IOFile.readLine(i);
        if(DataFromFile[0] != "") GLB_ui->lineEdit_20->setText(DataFromFile[0]);
        ConfigFile.close();
    }
    /*Other*/
    for(uint8_t i = 0; i < 6; i++)
    {
        memset(&MotorInstr[i].Flags, 0, sizeof(MotorInstr[i].Flags));
        // MotorInstr[i].Flags = 0;
    }

}
void MainWindow::ComPortSearch(uint32_t boudrate)
{
    GLB_WinObj.GLB_WindowsComboBox[0]->setEnabled(true);
    NowComBoxItem[0] = "Select comport...";
    TypeThreadInterrupt = 1;
    if(GLB_WinObj.GLB_WindowsCheckBox[12]->isChecked()) ThreadAutoConnectState = true;
    else ThreadAutoConnectState = false;
    CurrentBoundRate = boudrate;

    thread_1->start();
    while(!thread_1->isRunning()) {}
    if(!ThreadAutoConnectState) SendToTerminal("Thread#1: search.", true, 0);
    else if(ThreadAutoConnectState) SendToTerminal("Thread#1: search with auto-connect.", true, 0);

    GLB_WinObj.GLB_WindowsComboBox[0]->setEnabled(false);
    GLB_WinObj.GLB_WindowsCheckBox[12]->setEnabled(false);
    GLB_WinObj.GLB_WindowsButton[0]->setEnabled(false);
}

void MainWindow::ComPortOpen(QString com, uint32_t boudrate)
{
    CurrentComPort = com;
    CurrentBoundRate = boudrate;

    TypeThreadInterrupt = 2;
    thread_1->start();
    while(!thread_1->isRunning()) {}
}
void MainWindow::ComPortWrite(uint8_t *datatosend, uint32_t cntdata)
{
    TypeThreadInterrupt = 4;

    for (uint32_t i = 0; i < cntdata; i++)
    {
        ComportDataToSend[i] = datatosend[i];
    }
    ComportCountDataToSend = cntdata;

    thread_1->start();
    while(!thread_1->isRunning()) {}
    SendToTerminal("Thread#1: sending...", true, 1);
}
void MainWindow::ComPortRead()
{
    TypeThreadInterrupt = 0;

    thread_1->start();
    while(!thread_1->isRunning()) {}
    GLB_Thread_Flag[0] = true;
    SendToTerminal("Thread#1: reading...", true, 0);
}


void MainWindow::ComportDataUpdate_slot()
{
    GLB_WinObj.GLB_WindowsComboBox[0]->clear();
    GLB_WinObj.GLB_WindowsComboBox[0]->addItems(GLB_Comports);

    if(GLB_WinObj.GLB_WindowsCheckBox[12]->isChecked())
    {
        uint8_t index = GLB_WinObj.GLB_WindowsComboBox[0]->findText(CurrentComPort);
        QStandardItemModel* model = (QStandardItemModel*) GLB_WinObj.GLB_WindowsComboBox[0]->model();
        model->item(index)->setEnabled(false);
        NowComBoxItem[0] = CurrentComPort;
    }
    GLB_WinObj.GLB_WindowsComboBox[0]->setCurrentText(CurrentComPort);

    SendToTerminal("Thread#1: data update.", true, 0);
    GLB_WinObj.GLB_WindowsComboBox[0]->setEnabled(true);
    GLB_WinObj.GLB_WindowsCheckBox[12]->setEnabled(true);
    GLB_WinObj.GLB_WindowsButton[0]->setEnabled(true);

}
void MainWindow::ComportConnect_slot()
{
    uint8_t index = GLB_WinObj.GLB_WindowsComboBox[0]->findText(CurrentComPort);
    QStandardItemModel* model = (QStandardItemModel*) GLB_WinObj.GLB_WindowsComboBox[0]->model();
    model->item(index)->setEnabled(false);
    NowComBoxItem[0] = CurrentComPort;

    if(CurrentComPort != "Select comport...")
        SendToTerminal("Thread#1: open port " + CurrentComPort + " with speed: " + QString::number(CurrentBoundRate, 10) + ".", true, 0);
}
void MainWindow::ComportClose_slot()
{
    uint8_t index = GLB_WinObj.GLB_WindowsComboBox[0]->findText(NowComBoxItem[0]);
    QStandardItemModel* model = (QStandardItemModel*) GLB_WinObj.GLB_WindowsComboBox[0]->model();
    model->item(index)->setEnabled(true);

    if(NowComBoxItem[0] != "Select comport...") SendToTerminal("Thread#1: close port " + NowComBoxItem[0] + ".", true, 0);
}
void MainWindow::ComportRead_slot()
{
    SendToTerminal("Number of received data: " + QString::number(ComportCountdataRecv, 10) + "bytes.", true, 0);
}
void MainWindow::ComportWrite_slot(QString back)
{
    SendToTerminal("Feedback: " + back, true, 1);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    GLB_ui = ui;
    GLB_mainwindowWidget = this;


    thread_1 = new MyThread_1(GLB_mainwindow);

    connect(thread_1, &MyThread_1::PaintGraph2_signal, this, &MainWindow::PaintGraph2);
    connect(thread_1, &MyThread_1::PaintGraph_signal, this, &MainWindow::PaintGraph);

    connect(thread_1, &MyThread_1::ComportDataUpdate_signal, this, &MainWindow::ComportDataUpdate_slot);
    connect(thread_1, &MyThread_1::ComportConnect_signal, this, &MainWindow::ComportConnect_slot);
    connect(thread_1, &MyThread_1::ComportClose_signal, this, &MainWindow::ComportClose_slot);
    connect(thread_1, &MyThread_1::ComportRead_signal, this, &MainWindow::ComportRead_slot);
    connect(thread_1, &MyThread_1::ComPortWrite_signal, this, &MainWindow::ComportWrite_slot);

    SetStartGUISettings();
    SetStartVariables();

}

MainWindow::~MainWindow()
{
    while(!thread_1->isFinished()) {}
    delete thread_1;

    ConfigFile.open(QFile::WriteOnly);
    IOFile.setDevice(&ConfigFile);

    IOFile << GLB_ui->lineEdit_20->text();

    ConfigFile.close();

    delete ui;
}

void MainWindow::on_pushButton_12_clicked()
{
    uint8_t data[10];
    for(uint8_t i = 0; i < 10; i++)
    {
        data[i] = i + 1;
    }

    ComPortWrite((unsigned char *)data, 10);

}

/* PWM LineEdit */
void MainWindow::on_lineEdit_textEdited(const QString &arg1)
{
    // if(arg1 > 0)
    // {
    //     std::string str = arg1.toStdString();
    //     uint8_t i = 0;
    //     for(; i < str.length(); i++)
    //     {
    //         if(!isdigit(str[i])) break;
    //     }
    //     if(i == str.length())
    //     {
    //         uint16_t value = std::stoi(arg1.toStdString());
    //         std::cout << arg1.toStdString() << std::endl;
    //         GLB_ui->horizontalSlider->setValue(value);
    //     }
    // }
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        GLB_ui->horizontalSlider->setValue(value);
    }

}
void MainWindow::on_lineEdit_2_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        GLB_ui->horizontalSlider_2->setValue(value);

    }
}
void MainWindow::on_lineEdit_3_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        GLB_ui->horizontalSlider_3->setValue(value);
    }
}
void MainWindow::on_lineEdit_4_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        GLB_ui->horizontalSlider_4->setValue(value);
    }
}
void MainWindow::on_lineEdit_5_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        GLB_ui->horizontalSlider_5->setValue(value);
    }
}


/* TIME LineEdit */

void MainWindow::on_lineEdit_7_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}
void MainWindow::on_lineEdit_10_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}
void MainWindow::on_lineEdit_9_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}
void MainWindow::on_lineEdit_8_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


/* DELAY LineEdit */
void MainWindow::on_lineEdit_11_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}
void MainWindow::on_lineEdit_12_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}
void MainWindow::on_lineEdit_15_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}
void MainWindow::on_lineEdit_14_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}
void MainWindow::on_lineEdit_13_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}

/* Operation */
void MainWindow::on_SearchButton_clicked()
{
    SendToTerminal("Seacrhing Comports...", true, 0);
    ComPortSearch(std::stof(GLB_WinObj.GLB_WindowsLineEdit[40]->text().toStdString()));
}

void MainWindow::on_comboBox_textActivated(const QString &arg1)
{
    ComPortOpen(arg1, std::stof(GLB_WinObj.GLB_WindowsLineEdit[40]->text().toStdString()));
}
void MainWindow::on_pushButton_13_clicked()
{
    QString directory = QFileDialog::getExistingDirectory(nullptr, "Выберите папку", "",
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks); // Опции диалога
    GLB_ui->lineEdit_20->setText(directory);
}


//////* Angle Mode *//////
void MainWindow::on_lineEdit_23_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_22_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_21_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_24_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_25_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_28_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_27_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_26_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_29_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_30_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_33_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_35_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_34_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_31_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_lineEdit_32_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}


void MainWindow::on_checkBox_11_toggled(bool checked)
{
    if(checked)
    {
        GLB_Thread_Flag[0] = true;
        // thread_1 = new MyThread_1(GLB_mainwindow);
        // thread_1->start();
        // while(!thread_1->isRunning()) {}
        SendToTerminal("Thread #1 enable.", true, 0);
    }
    else if(!checked)
    {
        GLB_Thread_Flag[0] = false;
        SendToTerminal("Thread #1 disable.", true, 0);
    }
}

    // СОЗДАТЬ ЧЕКБОКС ЕСЛИ ТРЕБУЕТСЯ ЗАПУСК ВТОРОГО ПОТОКА (НАСТРОЙКА)
//     if(checked)
//     {
//         GLB_Thread_Flag[1] = true;
//         thread_2 = new MyThread_2(GLB_mainwindow);
//         thread_2->start();
//         while(!thread_2->isRunning()) {}
//         std::cout << "Thread #2 is started!" << std::endl;
//     }
//     else if(!checked)
//     {
//         GLB_Thread_Flag[1] = false;
//         while(!thread_2->isFinished()) {}
//         delete thread_2;
//         std::cout << "Thread #2 disable!" << std::endl;
//     }








// MOTOR 0
void MainWindow::on_pushButton_clicked()
{

    GLB_ui->pushButton_2->setChecked(false);
    GLB_ui->pushButton_21->setChecked(false);
    GLB_ui->pushButton_27->setChecked(false);

    MotorDefStruct[0].MD_MoveType = LEFT;

}
void MainWindow::on_pushButton_2_clicked()
{

    GLB_ui->pushButton->setChecked(false);
    GLB_ui->pushButton_21->setChecked(false);
    GLB_ui->pushButton_27->setChecked(false);

    MotorDefStruct[0].MD_MoveType = RIGHT;
}
void MainWindow::on_pushButton_21_clicked()
{

    GLB_ui->pushButton->setChecked(false);
    GLB_ui->pushButton_2->setChecked(false);
    GLB_ui->pushButton_27->setChecked(false);

    MotorDefStruct[0].MD_MoveType = HOLD;
}
void MainWindow::on_pushButton_27_clicked()
{

    GLB_ui->pushButton->setChecked(false);
    GLB_ui->pushButton_2->setChecked(false);
    GLB_ui->pushButton_21->setChecked(false);

    MotorDefStruct[0].MD_MoveType = FREE;
}
void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    uint8_t numMotor = 0;

    GLB_SliderValue[0] = value;
    GLB_ui->lineEdit->setText(QString::number(GLB_SliderValue[0]));

    MotorInstr[numMotor].PWM = value;
}
void MainWindow::on_lineEdit_6_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        // float value = std::stof(arg1.toStdString());
    }
}
// MOTOR 1
void MainWindow::on_pushButton_3_clicked()
{
    GLB_ui->pushButton_23->setChecked(false);
    GLB_ui->pushButton_4->setChecked(false);
    GLB_ui->pushButton_28->setChecked(false);

    MotorDefStruct[1].MD_MoveType = LEFT;
}
void MainWindow::on_pushButton_4_clicked()
{
    GLB_ui->pushButton_23->setChecked(false);
    GLB_ui->pushButton_3->setChecked(false);
    GLB_ui->pushButton_28->setChecked(false);

    MotorDefStruct[1].MD_MoveType = RIGHT;
}
void MainWindow::on_pushButton_23_clicked()
{

    GLB_ui->pushButton_3->setChecked(false);
    GLB_ui->pushButton_4->setChecked(false);
    GLB_ui->pushButton_28->setChecked(false);

    MotorDefStruct[1].MD_MoveType = HOLD;
}
void MainWindow::on_pushButton_28_clicked()
{
    GLB_ui->pushButton_23->setChecked(false);
    GLB_ui->pushButton_4->setChecked(false);
    GLB_ui->pushButton_3->setChecked(false);

    MotorDefStruct[1].MD_MoveType = FREE;
}
void MainWindow::on_horizontalSlider_2_valueChanged(int value)
{
    uint8_t numMotor = 1;

    GLB_SliderValue[1] = value;
    GLB_ui->lineEdit_2->setText(QString::number(GLB_SliderValue[1]));

    // MotorInstr[numMotor].Flags.FL_PWMByte = true;
    // if(value != 0) MotorInstr[numMotor].configHalfWord |= COM1_PWM;
    MotorInstr[numMotor].PWM = value;
}
// MOTOR 2
void MainWindow::on_pushButton_5_clicked()
{
    GLB_ui->pushButton_6->setChecked(false);
    GLB_ui->pushButton_19->setChecked(false);
    GLB_ui->pushButton_26->setChecked(false);

    MotorDefStruct[2].MD_MoveType = LEFT;
}
void MainWindow::on_pushButton_6_clicked()
{
    GLB_ui->pushButton_5->setChecked(false);
    GLB_ui->pushButton_19->setChecked(false);
    GLB_ui->pushButton_26->setChecked(false);

    MotorDefStruct[2].MD_MoveType = RIGHT;
}
void MainWindow::on_pushButton_19_clicked()
{

    GLB_ui->pushButton_5->setChecked(false);
    GLB_ui->pushButton_6->setChecked(false);
    GLB_ui->pushButton_26->setChecked(false);

    MotorDefStruct[2].MD_MoveType = HOLD;
}
void MainWindow::on_pushButton_26_clicked()
{
    GLB_ui->pushButton_5->setChecked(false);
    GLB_ui->pushButton_6->setChecked(false);
    GLB_ui->pushButton_19->setChecked(false);

    MotorDefStruct[2].MD_MoveType = FREE;
}
void MainWindow::on_horizontalSlider_3_valueChanged(int value)
{
    uint8_t numMotor = 2;

    GLB_SliderValue[2] = value;
    GLB_ui->lineEdit_3->setText(QString::number(GLB_SliderValue[2]));

    // MotorInstr[numMotor].Flags.FL_PWMByte = true;
    // if(value != 0) MotorInstr[numMotor].configHalfWord |= COM1_PWM;
    MotorInstr[numMotor].PWM = value;
}
// MOTOR 3
void MainWindow::on_pushButton_7_clicked()
{


    GLB_ui->pushButton_22->setChecked(false);
    GLB_ui->pushButton_24->setChecked(false);
    GLB_ui->pushButton_8->setChecked(false);

    MotorDefStruct[3].MD_MoveType = LEFT;
}
void MainWindow::on_pushButton_8_clicked()
{

    GLB_ui->pushButton_22->setChecked(false);
    GLB_ui->pushButton_24->setChecked(false);
    GLB_ui->pushButton_7->setChecked(false);

    MotorDefStruct[3].MD_MoveType = RIGHT;
}
void MainWindow::on_pushButton_22_clicked()
{

    GLB_ui->pushButton_8->setChecked(false);
    GLB_ui->pushButton_24->setChecked(false);
    GLB_ui->pushButton_7->setChecked(false);

    MotorDefStruct[3].MD_MoveType = HOLD;
}
void MainWindow::on_pushButton_24_clicked()
{

    GLB_ui->pushButton_8->setChecked(false);
    GLB_ui->pushButton_22->setChecked(false);
    GLB_ui->pushButton_7->setChecked(false);

    MotorDefStruct[3].MD_MoveType = FREE;
}
void MainWindow::on_horizontalSlider_4_valueChanged(int value)
{
    uint8_t numMotor = 3;

    GLB_SliderValue[3] = value;
    GLB_ui->lineEdit_4->setText(QString::number(GLB_SliderValue[3]));

    // MotorInstr[numMotor].Flags.FL_PWMByte = true;
    // if(value != 0) MotorInstr[numMotor].configHalfWord |= COM1_PWM;
    MotorInstr[numMotor].PWM = value;
}
// MOTOR 4
void MainWindow::on_pushButton_9_clicked()
{

    GLB_ui->pushButton_10->setChecked(false);
    GLB_ui->pushButton_20->setChecked(false);
    GLB_ui->pushButton_25->setChecked(false);

    MotorDefStruct[4].MD_MoveType = LEFT;
}
void MainWindow::on_pushButton_10_clicked()
{
    GLB_ui->pushButton_9->setChecked(false);
    GLB_ui->pushButton_20->setChecked(false);
    GLB_ui->pushButton_25->setChecked(false);

    MotorDefStruct[4].MD_MoveType = RIGHT;
}
void MainWindow::on_pushButton_20_clicked()
{

    GLB_ui->pushButton_9->setChecked(false);
    GLB_ui->pushButton_10->setChecked(false);
    GLB_ui->pushButton_25->setChecked(false);

    MotorDefStruct[4].MD_MoveType = HOLD;
}
void MainWindow::on_pushButton_25_clicked()
{

    GLB_ui->pushButton_9->setChecked(false);
    GLB_ui->pushButton_10->setChecked(false);
    GLB_ui->pushButton_20->setChecked(false);

    MotorDefStruct[4].MD_MoveType = FREE;
}
void MainWindow::on_horizontalSlider_5_valueChanged(int value)
{
    uint8_t numMotor = 4;

    GLB_SliderValue[4] = value;
    GLB_ui->lineEdit_5->setText(QString::number(GLB_SliderValue[4]));

    // MotorInstr[numMotor].Flags.FL_PWMByte = true;
    // if(value != 0) MotorInstr[numMotor].configHalfWord |= COM1_PWM;
    MotorInstr[numMotor].PWM = value;
}
// MOTOR 5
void MainWindow::on_pushButton_30_clicked()
{

    GLB_ui->pushButton_31->setChecked(false);
    GLB_ui->pushButton_32->setChecked(false);
    GLB_ui->pushButton_33->setChecked(false);

    MotorDefStruct[5].MD_MoveType = LEFT;
}
void MainWindow::on_pushButton_31_clicked()
{

    GLB_ui->pushButton_30->setChecked(false);
    GLB_ui->pushButton_32->setChecked(false);
    GLB_ui->pushButton_33->setChecked(false);

    MotorDefStruct[5].MD_MoveType = RIGHT;
}
void MainWindow::on_pushButton_32_clicked()
{

    GLB_ui->pushButton_30->setChecked(false);
    GLB_ui->pushButton_31->setChecked(false);
    GLB_ui->pushButton_33->setChecked(false);

    MotorDefStruct[5].MD_MoveType = HOLD;
}
void MainWindow::on_pushButton_33_clicked()
{

    GLB_ui->pushButton_30->setChecked(false);
    GLB_ui->pushButton_31->setChecked(false);
    GLB_ui->pushButton_32->setChecked(false);

    MotorDefStruct[5].MD_MoveType = FREE;
}
void MainWindow::on_horizontalSlider_6_valueChanged(int value)
{
    uint8_t numMotor = 5;

    GLB_SliderValue[5] = value;
    GLB_ui->lineEdit_36->setText(QString::number(GLB_SliderValue[5]));

    // MotorInstr[numMotor].Flags.FL_PWMByte = true;
    // if(value != 0) MotorInstr[numMotor].configHalfWord |= COM1_PWM;
    MotorInstr[numMotor].PWM = value;
}
// Configurate
void MainWindow::on_pushButton_29_clicked()
{
    SendToTerminal("Configurate", true, 1);

    bool flags_Enable[6] = {false, };       // Is it enabled?
    uint8_t DataToSend[6][100] = {{0, }, };
    uint8_t DataToSendALL[100] = {0, };
    uint32_t CountData = 0;

    for(uint8_t i = 0; i < 6; i++)
    {
        MotorDefStruct[i].MD_SidePlate = (NumPlate)MotorDefStruct[i].TAB1_CheckBoxBackSide->isChecked();
        // if(MotorDefStruct[i].MD_SidePlate > 0)
        // {
        MotorDefStruct[i].MD_Config_1 |= MASK_COM_SIDEPLATE;

        // }

        MotorDefStruct[i].MD_WorkMode = WRM_PWM_MODE;
        MotorDefStruct[i].MD_Config_1 |= MASK_COM_WORKMODE;

        MotorDefStruct[i].MD_SelMotor = i;
        // if(MotorDefStruct[i].MD_SelMotor > 0)
        // {
            MotorDefStruct[i].MD_Config_1 |= MASK_COM1_SELMOTOR;

        // }

        if((!MotorDefStruct[i].TAB1_ComporessButton->isChecked()) &&
            (!MotorDefStruct[i].TAB1_DecompressButton->isChecked()) &&
            (!MotorDefStruct[i].TAB1_FreeButton->isChecked()) &&
                (!MotorDefStruct[i].TAB1_HoldButton->isChecked())) flags_Enable[i] = false;
        else
        {
            MotorDefStruct[i].MD_MoveType;
            flags_Enable[i] = true;
        }
        // if(MotorDefStruct[i].MD_MoveType > 0)
        // {
            MotorDefStruct[i].MD_Config_1 |= MASK_COM1_TYPEMOVE;

        // }

        MotorDefStruct[i].MD_PWM = std::stof(MotorDefStruct[i].TAB1_LineEditPWM->text().toStdString());
        // if(MotorDefStruct[i].MD_PWM > 0)
        // {
            MotorDefStruct[i].MD_Config_1 |= MASK_COM1_PWM;

        // }

        MotorDefStruct[i].MD_TimeWork = std::stof(MotorDefStruct[i].TAB1_LineEditWorkTime->text().toStdString()) * 100;
        // if(MotorDefStruct[i].MD_TimeWork > 0)
        // {
            MotorDefStruct[i].MD_Config_1 |= MASK_COM1_TIMEWORK;

        // }

        MotorDefStruct[i].MD_TimeDelay = std::stof(MotorDefStruct[i].TAB1_LineEditDelayTime->text().toStdString()) * 100;
        // if(MotorDefStruct[i].MD_TimeDelay > 0)
        // {
            MotorDefStruct[i].MD_Config_1 |= MASK_COM1_DELAY;
        // }

        MotorDefStruct[i].MD_ADC_CH = MotorDefStruct[i].TAB1_CheckBoxADC->isChecked();
        // if(MotorDefStruct[i].MD_ADC_CH > 0)
        // {
            MotorDefStruct[i].MD_Config_1 |= MASK_COM1_ADC;
        // }
    }

    uint8_t nowCnt = 0;
    for(uint8_t i = 0; i < 6; i++)
    {
        if(!flags_Enable[i]) continue;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_Config_1;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_Config_2;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_SidePlate;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_WorkMode;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_SelMotor;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_MoveType;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_PWM;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_TimeWork & 0x00FF;
        nowCnt++;
        DataToSend[i][nowCnt] = (MotorDefStruct[i].MD_TimeWork & 0xFF00) >> 8;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_TimeDelay & 0x00FF;
        nowCnt++;
        DataToSend[i][nowCnt] = (MotorDefStruct[i].MD_TimeDelay & 0xFF00) >> 8;
        nowCnt++;
        DataToSend[i][nowCnt] = MotorDefStruct[i].MD_ADC_CH;
        nowCnt++;

        DataToSend[i][nowCnt] = 0xFF;
        nowCnt++;
        DataToSend[i][nowCnt] = 0xDD;
        nowCnt++;

        for(uint8_t t = 0; t < nowCnt; t++)
        {
            DataToSendALL[(i * nowCnt) + t] = DataToSend[i][t];
        }

        CountData += nowCnt;
        nowCnt = 0;
    }

    ComPortWrite(DataToSendALL, CountData);
    CountData = 0;

    memset(&DataToSend, 0, sizeof(DataToSend));
    memset(&DataToSendALL, 0, sizeof(DataToSendALL));

}
// Start insturction
void MainWindow::on_pushButton_34_clicked()
{
    uint8_t DataToSendALL[100] = {0, };
    uint8_t nowCnt = 0;

    SendToTerminal("Start instruction", true, 1);

    DataToSendALL[nowCnt] |= MASK_COM_SIDEPLATE | MASK_COM_WORKMODE;    // config_1
    nowCnt++;
    DataToSendALL[nowCnt] |= MASK_COM_START_INSTR >> 8;                 // config_2
    nowCnt++;
    DataToSendALL[nowCnt] |= (NumPlate)MotorDefStruct[0].TAB1_CheckBoxBackSide->isChecked();   // select plate
    nowCnt++;
    DataToSendALL[nowCnt] |= WRM_PWM_MODE;                              // select workmode
    nowCnt++;
    DataToSendALL[nowCnt] = 0x01;                                       // start instruct byte
    nowCnt++;

    DataToSendALL[nowCnt] = 0xFF;
    nowCnt++;
    DataToSendALL[nowCnt] = 0xDD;
    nowCnt++;

    ComPortWrite(DataToSendALL, nowCnt);





    // /*Start Received ADC channel*/
    // if(GLB_ui->checkBox->isChecked()) PackToRecv = std::stof(GLB_ui->lineEdit_6->text().toStdString()) * CountPoints;
    // else if(GLB_ui->checkBox_2->isChecked()) PackToRecv = std::stof(GLB_ui->lineEdit_7->text().toStdString()) * CountPoints;
    // else if(GLB_ui->checkBox_3->isChecked()) PackToRecv = std::stof(GLB_ui->lineEdit_10->text().toStdString()) * CountPoints;
    // else if(GLB_ui->checkBox_4->isChecked()) PackToRecv = std::stof(GLB_ui->lineEdit_9->text().toStdString()) * CountPoints;
    // else if(GLB_ui->checkBox_5->isChecked()) PackToRecv = std::stof(GLB_ui->lineEdit_8->text().toStdString()) * CountPoints;
    // else if(GLB_ui->checkBox_13->isChecked()) PackToRecv = std::stof(GLB_ui->lineEdit_37->text().toStdString()) * CountPoints;

    // if(GLB_ui->checkBox_11->isChecked() && (GlobalFlagsMotor[0])/* && GLB_Thread_Flag[0]*/)
    // {
    //     // thread_1 = new MyThread_1(GLB_mainwindow);
    //     connect(thread_1, &MyThread_1::PaintGraph_signal, this, &MainWindow::PaintGraph);
    //     thread_1->start();
    //     while(!thread_1->isRunning()) {}
    //     GLB_Thread_Flag[0] = true;
    //     SendToTerminal("Thread #1 is started!", true, 0);
    // }
    // // Flags clearing after checking
    // for(uint8_t i = 0; i < 6; i++) memset(&MotorInstr[i].Flags, 0, sizeof(MotorInstr[i]));
    // /**/

    // uint8_t DataToSend[10] = {0, };
    // uint8_t cnt = 0;
    // uint16_t newConf = COM1_START_INSTR | COM_WORKMODE;
    // DataToSend[cnt] = newConf & 0x00FF;
    // cnt++;
    // DataToSend[cnt] = (newConf & 0xFF00) >> 8;
    // cnt++;
    // DataToSend[cnt] = GLB_Command.ModeWorkByte;
    // cnt++;
    // DataToSend[cnt] = GLB_Command.StartInstruct;
    // cnt++;
    // DataToSend[cnt] = 0xFF;           ///////////// БАЙТ РАЗДЕЛИТЕЛЬ / УКАЗАТЕЛЬ
    // cnt++;
    // ComPortWrite(0, (unsigned char*)DataToSend, cnt);
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
/*************************************ANGLE CONTROL*************************************/
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
/*Configuration*/
void MainWindow::on_pushButton_36_clicked()
{
    // SendToTerminal("Configurate", true, 2);
    // uint8_t DataToSend[5][30] = {0, };
    // uint8_t numMotor;
    // uint16_t value = 0;
    // uint8_t cnt[5] = {0, };

    // GLB_Command.ModeWorkByte = GLB_ui->tabWidget->currentIndex();
    // GLB_Command.StartInstruct = 0x01;   // Instruction start byte

    // for(uint8_t i = 0; i < 6; i++) MotorInstr[i].configHalfWord = 0x00;

    // for(uint8_t i = 0; i < 6; i++) MotorInstr[i].configHalfWord |= COM_WORKMODE;


    // /*Check ChecBox*/
    // // Motor 1
    // numMotor = 0;
    // for(uint8_t ii = 0; ii < 3; ii++)
    // {
    //     if(CtrlCheckState[ii])
    //     {
    //         if(ii == 0)      //angle
    //         {
    //             value = std::stof(GLB_ui->lineEdit_23->text().toStdString());
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Angle = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_ANGLE;
    //                 MotorInstr[numMotor].Flags.FL2_AngleByte = true;
    //             }
    //         }
    //         else if(ii == 1) // time
    //         {
    //             value = std::stof(GLB_ui->lineEdit_28->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Time = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_TIME;
    //                 MotorInstr[numMotor].Flags.FL2_TimeByte = true;
    //             }
    //         }
    //         else             // speed
    //         {
    //             value = std::stof(GLB_ui->lineEdit_33->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Speed = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_SPEED;
    //                 MotorInstr[numMotor].Flags.FL2_SpeedByte = true;
    //             }
    //         }
    //     }
    // }
    // value = std::stof(GLB_ui->lineEdit_44->text().toStdString()) * 100; // Delay
    // if(value >= 0)
    // {
    //     MotorInstr[numMotor].Flags.FL2_DelayByte = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_DELAY;
    //     MotorInstr[numMotor].CTRL_2_Delay = value;
    // }
    // if(GLB_ui->radioButton_4->isChecked())
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }
    // else
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x00;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }
    // MotorInstr[numMotor].SelMotor = numMotor;
    // MotorInstr[numMotor].configHalfWord |= COM1_SETMOTOR;
    // MotorInstr[numMotor].configHalfWord |= COM1_TYPEMOVE;
    // MotorInstr[numMotor].Flags.FL_TypeMoveByte = true;
    // MotorInstr[numMotor].Flags.FL_SelMotByte = true;
    // if(GLB_ui->radioButton_4->isChecked())                                   // FEEDBACK
    // {
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    // }

    // if(GLB_ui->pushButton_14->isChecked()) MotorInstr[numMotor].TypeMove = ANGLE_MODE;
    // else if(GLB_ui->pushButton_14->isChecked()) MotorInstr[numMotor].TypeMove = HOLD;

    // // Motor 2
    // numMotor = 1;
    // for(uint8_t ii = 0; ii < 3; ii++)
    // {
    //     if(CtrlCheckState[ii])
    //     {
    //         if(ii == 0)      //angle
    //         {
    //             value = std::stof(GLB_ui->lineEdit_22->text().toStdString());
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Angle = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_ANGLE;
    //                 MotorInstr[numMotor].Flags.FL2_AngleByte = true;
    //             }
    //         }
    //         else if(ii == 1) // time
    //         {
    //             value = std::stof(GLB_ui->lineEdit_27->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Time = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_TIME;
    //                 MotorInstr[numMotor].Flags.FL2_TimeByte = true;
    //             }
    //         }
    //         else             // speed
    //         {
    //             if(value > 0)
    //             {
    //                 value = std::stof(GLB_ui->lineEdit_35->text().toStdString()) * 100;
    //                 MotorInstr[numMotor].CTRL_2_Speed = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_SPEED;
    //                 MotorInstr[numMotor].Flags.FL2_SpeedByte = true;
    //             }
    //         }
    //     }
    // }
    // value = std::stof(GLB_ui->lineEdit_41->text().toStdString()) * 100; // Delay
    // if(value >= 0)
    // {
    //     MotorInstr[numMotor].Flags.FL2_DelayByte = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_DELAY;
    //     MotorInstr[numMotor].CTRL_2_Delay = value;
    // }
    // if(GLB_ui->radioButton_5->isChecked())
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }
    // else
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x00;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }
    // MotorInstr[numMotor].SelMotor = numMotor;
    // MotorInstr[numMotor].configHalfWord |= COM1_SETMOTOR;
    // MotorInstr[numMotor].configHalfWord |= COM1_TYPEMOVE;
    // MotorInstr[numMotor].Flags.FL_TypeMoveByte = true;
    // MotorInstr[numMotor].Flags.FL_SelMotByte = true;
    // if(GLB_ui->radioButton_5->isChecked())                                   // FEEDBACK
    // {
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    // }
    // if(GLB_ui->pushButton_15->isChecked()) MotorInstr[numMotor].TypeMove = ANGLE_MODE;
    // else if(GLB_ui->pushButton_15->isChecked()) MotorInstr[numMotor].TypeMove = HOLD;
    // // Motor 3
    // numMotor = 2;
    // for(uint8_t ii = 0; ii < 3; ii++)
    // {
    //     if(CtrlCheckState[ii])
    //     {
    //         if(ii == 0)      //angle
    //         {
    //             value = std::stof(GLB_ui->lineEdit_21->text().toStdString());
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Angle = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_ANGLE;
    //                 MotorInstr[numMotor].Flags.FL2_AngleByte = true;
    //             }
    //         }
    //         else if(ii == 1) // time
    //         {
    //             value = std::stof(GLB_ui->lineEdit_26->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Time = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_TIME;
    //                 MotorInstr[numMotor].Flags.FL2_TimeByte = true;
    //             }
    //         }
    //         else             // speed
    //         {
    //             value = std::stof(GLB_ui->lineEdit_34->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Speed = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_SPEED;
    //                 MotorInstr[numMotor].Flags.FL2_SpeedByte = true;
    //             }
    //         }
    //     }
    // }
    // value = std::stof(GLB_ui->lineEdit_42->text().toStdString()) * 100; // Delay
    // if(value >= 0)
    // {
    //     MotorInstr[numMotor].Flags.FL2_DelayByte = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_DELAY;
    //     MotorInstr[numMotor].CTRL_2_Delay = value;
    // }
    // if(GLB_ui->radioButton_6->isChecked())
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }
    // else
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x00;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }
    // MotorInstr[numMotor].SelMotor = numMotor;
    // MotorInstr[numMotor].configHalfWord |= COM1_SETMOTOR;
    // MotorInstr[numMotor].configHalfWord |= COM1_TYPEMOVE;
    // MotorInstr[numMotor].Flags.FL_TypeMoveByte = true;
    // MotorInstr[numMotor].Flags.FL_SelMotByte = true;
    // if(GLB_ui->radioButton_6->isChecked())                                   // FEEDBACK
    // {
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    // }
    // if(GLB_ui->pushButton_16->isChecked()) MotorInstr[numMotor].TypeMove = ANGLE_MODE;
    // else if(GLB_ui->pushButton_16->isChecked()) MotorInstr[numMotor].TypeMove = HOLD;
    // // Motor 4
    // numMotor = 3;
    // for(uint8_t ii = 0; ii < 3; ii++)
    // {
    //     if(CtrlCheckState[ii])
    //     {
    //         if(ii == 0)      //angle
    //         {
    //             value = std::stof(GLB_ui->lineEdit_24->text().toStdString());
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Angle = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_ANGLE;
    //                 MotorInstr[numMotor].Flags.FL2_AngleByte = true;
    //             }
    //         }
    //         else if(ii == 1) // time
    //         {
    //             value = std::stof(GLB_ui->lineEdit_29->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Time = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_TIME;
    //                 MotorInstr[numMotor].Flags.FL2_TimeByte = true;
    //             }
    //         }
    //         else             // speed
    //         {
    //             value = std::stof(GLB_ui->lineEdit_31->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Speed = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_SPEED;
    //                 MotorInstr[numMotor].Flags.FL2_SpeedByte = true;
    //             }
    //         }
    //     }
    // }
    // value = std::stof(GLB_ui->lineEdit_43->text().toStdString()) * 100; // Delay
    // if(value >= 0)
    // {
    //     MotorInstr[numMotor].Flags.FL2_DelayByte = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_DELAY;
    //     MotorInstr[numMotor].CTRL_2_Delay = value;
    // }
    // if(GLB_ui->radioButton_7->isChecked())
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }
    // else
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x00;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }

    // MotorInstr[numMotor].SelMotor = numMotor;
    // MotorInstr[numMotor].configHalfWord |= COM1_SETMOTOR;
    // MotorInstr[numMotor].configHalfWord |= COM1_TYPEMOVE;
    // MotorInstr[numMotor].Flags.FL_TypeMoveByte = true;
    // MotorInstr[numMotor].Flags.FL_SelMotByte = true;
    // if(GLB_ui->radioButton_7->isChecked())                                   // FEEDBACK
    // {
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    // }
    // if(GLB_ui->pushButton_17->isChecked()) MotorInstr[numMotor].TypeMove = ANGLE_MODE;
    // else if(GLB_ui->pushButton_17->isChecked()) MotorInstr[numMotor].TypeMove = HOLD;
    // // Motor 5
    // numMotor = 4;
    // for(uint8_t ii = 0; ii < 3; ii++)
    // {
    //     if(CtrlCheckState[ii])
    //     {
    //         if(ii == 0)      //angle
    //         {
    //             value = std::stof(GLB_ui->lineEdit_25->text().toStdString());
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Angle = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_ANGLE;
    //                 MotorInstr[numMotor].Flags.FL2_AngleByte = true;
    //             }
    //         }
    //         else if(ii == 1) // time
    //         {
    //             value = std::stof(GLB_ui->lineEdit_30->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Time = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_TIME;
    //                 MotorInstr[numMotor].Flags.FL2_TimeByte = true;
    //             }
    //         }
    //         else             // speed
    //         {
    //             value = std::stof(GLB_ui->lineEdit_32->text().toStdString()) * 100;
    //             if(value > 0)
    //             {
    //                 MotorInstr[numMotor].CTRL_2_Speed = value;
    //                 MotorInstr[numMotor].configHalfWord |= COM2_SPEED;
    //                 MotorInstr[numMotor].Flags.FL2_SpeedByte = true;
    //             }
    //         }
    //     }
    // }
    // value = std::stof(GLB_ui->lineEdit_40->text().toStdString()) * 100; // Delay
    // if(value >= 0)
    // {
    //     MotorInstr[numMotor].Flags.FL2_DelayByte = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_DELAY;
    //     MotorInstr[numMotor].CTRL_2_Delay = value;
    // }
    // if(GLB_ui->radioButton_8->isChecked())
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }
    // else
    // {
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x00;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    // }

    // MotorInstr[numMotor].SelMotor = numMotor;
    // MotorInstr[numMotor].configHalfWord |= COM1_SETMOTOR;
    // MotorInstr[numMotor].configHalfWord |= COM1_TYPEMOVE;
    // MotorInstr[numMotor].Flags.FL_TypeMoveByte = true;
    // MotorInstr[numMotor].Flags.FL_SelMotByte = true;
    // if(GLB_ui->radioButton_8->isChecked())                                   // FEEDBACK
    // {
    //     MotorInstr[numMotor].Flags.FL2_FeedBack = true;
    //     MotorInstr[numMotor].configHalfWord |= COM2_FEEDBACK;
    //     MotorInstr[numMotor].CTRL_2_FeedBack = 0x01;
    // }
    // if(GLB_ui->pushButton_18->isChecked()) MotorInstr[numMotor].TypeMove = ANGLE_MODE;
    // else if(GLB_ui->pushButton_18->isChecked()) MotorInstr[numMotor].TypeMove = HOLD;

    // for(uint8_t i = 0; i < 5; i++)
    // {
    //     if(MotorInstr[i].Flags.FL2_AngleByte || MotorInstr[i].Flags.FL2_TimeByte
    //         || MotorInstr[i].Flags.FL2_SpeedByte)
    //     {
    //         DataToSend[i][cnt[i]] = MotorInstr[i].configHalfWord & 0x00FF;
    //         cnt[i]++;
    //         DataToSend[i][cnt[i]] = (MotorInstr[i].configHalfWord & 0xFF00) >> 8;
    //         cnt[i]++;
    //         DataToSend[i][cnt[i]] = GLB_Command.ModeWorkByte;
    //         cnt[i]++;

    //         if(MotorInstr[i].Flags.FL_StatusByte)
    //         {
    //             DataToSend[i][cnt[i]] = MotorInstr[i].Status;
    //             cnt[i]++;
    //         }
    //         if(MotorInstr[i].Flags.FL_SelMotByte)
    //         {
    //             DataToSend[i][cnt[i]] = MotorInstr[i].SelMotor;
    //             cnt[i]++;
    //         }
    //         if(MotorInstr[i].Flags.FL_TypeMoveByte)
    //         {
    //             DataToSend[i][cnt[i]] = MotorInstr[i].TypeMove;
    //             cnt[i]++;
    //         }
    //         if(MotorInstr[i].Flags.FL2_AngleByte)
    //         {
    //             DataToSend[i][cnt[i]] = MotorInstr[i].CTRL_2_Angle & 0x00FF;
    //             cnt[i]++;
    //             DataToSend[i][cnt[i]] = (MotorInstr[i].CTRL_2_Angle & 0xFF00) >> 8;
    //             cnt[i]++;
    //         }
    //         if(MotorInstr[i].Flags.FL2_TimeByte)
    //         {
    //             DataToSend[i][cnt[i]] = MotorInstr[i].CTRL_2_Time & 0x00FF;
    //             cnt[i]++;
    //             DataToSend[i][cnt[i]] = (MotorInstr[i].CTRL_2_Time & 0xFF00) >> 8;
    //             cnt[i]++;
    //         }
    //         if(MotorInstr[i].Flags.FL2_SpeedByte)
    //         {
    //             DataToSend[i][cnt[i]] = MotorInstr[i].CTRL_2_Speed & 0x00FF;
    //             cnt[i]++;
    //             DataToSend[i][cnt[i]] = (MotorInstr[i].CTRL_2_Speed & 0xFF00) >> 8;
    //             cnt[i]++;
    //         }
    //         if(MotorInstr[i].Flags.FL2_DelayByte)
    //         {
    //             DataToSend[i][cnt[i]] = MotorInstr[i].CTRL_2_Delay & 0x00FF;
    //             cnt[i]++;
    //             DataToSend[i][cnt[i]] = (MotorInstr[i].CTRL_2_Delay & 0xFF00) >> 8;
    //             cnt[i]++;
    //         }
    //         if(MotorInstr[i].Flags.FL2_FeedBack)
    //         {
    //             DataToSend[i][cnt[i]] = MotorInstr[i].CTRL_2_FeedBack;
    //             cnt[i]++;
    //         }
    //         DataToSend[i][cnt[i]] = 0xFF;           ///////////// БАЙТ - РАЗДЕЛИТЕЛЬ
    //         cnt[i]++;
    //         // ComPortWrite(0, (unsigned char*)DataToSend[i], cnt[i]);
    //     }
    // }
    // memset(&DataToSend, 0, sizeof(DataToSend));
    // memset(&cnt, 0, sizeof(cnt));
}
/*Start Instuction 2*/
void MainWindow::on_pushButton_35_clicked()
{
    // SendToTerminal("Start instruction", true, 2);

    // CheckGlobalStateMotorVariables();

    // /*Start Received FeedBack channel*/

    // if(GLB_ui->checkBox_11->isChecked() && (GlobalFlagsMotor[1])/* && GLB_Thread_Flag[0]*/)
    // {
    //     // thread_1 = new MyThread_1(GLB_mainwindow);
    //     // connect(thread_1, &MyThread_1::PaintGraph2_signal, this, &MainWindow::PaintGraph2);
    //     thread_1->start();
    //     while(!thread_1->isRunning()) {}
    //     GLB_Thread_Flag[0] = true;
    //     SendToTerminal("Thread #1 is started!", true, 2);
    // }
    // for(uint8_t i = 0; i < 6; i++) memset(&MotorInstr[i].Flags, 0, sizeof(MotorInstr[i]));
    // /**/

    // uint8_t DataToSend[10] = {0, };
    // uint8_t cnt = 0;
    // uint16_t newConf = COM2_START_INSTR | COM_WORKMODE;
    // DataToSend[cnt] = newConf & 0x00FF;
    // cnt++;
    // DataToSend[cnt] = (newConf & 0xFF00) >> 8;
    // cnt++;
    // DataToSend[cnt] = GLB_Command.ModeWorkByte;
    // cnt++;
    // DataToSend[cnt] = GLB_Command.StartInstruct;
    // cnt++;
    // DataToSend[cnt] = 0xFF;           ///////////// БАЙТ РАЗДЕЛИТЕЛЬ / УКАЗАТЕЛЬ
    // cnt++;
    // ComPortWrite(0, (unsigned char*)DataToSend, cnt);
}


/*Channels ADC*/
// Channel_1
void MainWindow::on_radioButton_4_clicked()
{
    RatioStateNow = Ratio_CH_1;
}
// Channel_2
void MainWindow::on_radioButton_5_clicked()
{
    RatioStateNow = Ratio_CH_2;
}
// Channel_3
void MainWindow::on_radioButton_6_clicked()
{
    RatioStateNow = Ratio_CH_3;
}
// Channel_4
void MainWindow::on_radioButton_7_clicked()
{
    RatioStateNow = Ratio_CH_4;
}
// Channel_5
void MainWindow::on_radioButton_8_clicked()
{
    RatioStateNow = Ratio_CH_5;
}
// Channel_None
void MainWindow::on_radioButton_clicked()
{
    RatioStateNow = Ratio_CH_None;
}

/*TypeControl*/
//Angle
void MainWindow::on_checkBox_6_toggled(bool checked)
{
    if(GLB_ui->checkBox_6->isChecked())
    {
        GLB_ui->lineEdit_21->setEnabled(true);
        GLB_ui->lineEdit_22->setEnabled(true);
        GLB_ui->lineEdit_23->setEnabled(true);
        GLB_ui->lineEdit_24->setEnabled(true);
        GLB_ui->lineEdit_25->setEnabled(true);
        CtrlCheckState[0] = true;

        if(GLB_ui->checkBox_7->isChecked())
        {
            GLB_ui->checkBox_8->setChecked(false);
            GLB_ui->lineEdit_31->setEnabled(false);
            GLB_ui->lineEdit_32->setEnabled(false);
            GLB_ui->lineEdit_33->setEnabled(false);
            GLB_ui->lineEdit_34->setEnabled(false);
            GLB_ui->lineEdit_35->setEnabled(false);
            CtrlCheckState[2] = false;
        }
        else if(GLB_ui->checkBox_8->isChecked())
        {
            GLB_ui->checkBox_7->setChecked(false);
            GLB_ui->lineEdit_26->setEnabled(false);
            GLB_ui->lineEdit_27->setEnabled(false);
            GLB_ui->lineEdit_28->setEnabled(false);
            GLB_ui->lineEdit_29->setEnabled(false);
            GLB_ui->lineEdit_30->setEnabled(false);
            CtrlCheckState[1] = false;
        }
        else
        {
            GLB_ui->lineEdit_31->setEnabled(false);
            GLB_ui->lineEdit_32->setEnabled(false);
            GLB_ui->lineEdit_33->setEnabled(false);
            GLB_ui->lineEdit_34->setEnabled(false);
            GLB_ui->lineEdit_35->setEnabled(false);

            GLB_ui->lineEdit_26->setEnabled(false);
            GLB_ui->lineEdit_27->setEnabled(false);
            GLB_ui->lineEdit_28->setEnabled(false);
            GLB_ui->lineEdit_29->setEnabled(false);
            GLB_ui->lineEdit_30->setEnabled(false);

            CtrlCheckState[1] = false;
            CtrlCheckState[2] = false;
        }
    }
    else
    {
        GLB_ui->lineEdit_21->setEnabled(false);
        GLB_ui->lineEdit_22->setEnabled(false);
        GLB_ui->lineEdit_23->setEnabled(false);
        GLB_ui->lineEdit_24->setEnabled(false);
        GLB_ui->lineEdit_25->setEnabled(false);

        CtrlCheckState[1] = false;
    }
}
//Time
void MainWindow::on_checkBox_7_toggled(bool checked)
{
    if(GLB_ui->checkBox_7->isChecked())
    {
        GLB_ui->lineEdit_26->setEnabled(true);
        GLB_ui->lineEdit_27->setEnabled(true);
        GLB_ui->lineEdit_28->setEnabled(true);
        GLB_ui->lineEdit_29->setEnabled(true);
        GLB_ui->lineEdit_30->setEnabled(true);
        CtrlCheckState[1] = true;

        if(GLB_ui->checkBox_6->isChecked())
        {
            GLB_ui->checkBox_8->setChecked(false);
            GLB_ui->lineEdit_31->setEnabled(false);
            GLB_ui->lineEdit_32->setEnabled(false);
            GLB_ui->lineEdit_33->setEnabled(false);
            GLB_ui->lineEdit_34->setEnabled(false);
            GLB_ui->lineEdit_35->setEnabled(false);
            CtrlCheckState[2] = false;

        }
        else if(GLB_ui->checkBox_8->isChecked())
        {
            GLB_ui->checkBox_6->setChecked(false);
            GLB_ui->lineEdit_21->setEnabled(false);
            GLB_ui->lineEdit_22->setEnabled(false);
            GLB_ui->lineEdit_23->setEnabled(false);
            GLB_ui->lineEdit_24->setEnabled(false);
            GLB_ui->lineEdit_25->setEnabled(false);
            CtrlCheckState[1] = false;
        }
        else
        {
            GLB_ui->lineEdit_31->setEnabled(false);
            GLB_ui->lineEdit_32->setEnabled(false);
            GLB_ui->lineEdit_33->setEnabled(false);
            GLB_ui->lineEdit_34->setEnabled(false);
            GLB_ui->lineEdit_35->setEnabled(false);

            GLB_ui->lineEdit_21->setEnabled(false);
            GLB_ui->lineEdit_22->setEnabled(false);
            GLB_ui->lineEdit_23->setEnabled(false);
            GLB_ui->lineEdit_24->setEnabled(false);
            GLB_ui->lineEdit_25->setEnabled(false);

            CtrlCheckState[0] = false;
            CtrlCheckState[2] = false;
        }
    }
    else
    {
        GLB_ui->lineEdit_26->setEnabled(false);
        GLB_ui->lineEdit_27->setEnabled(false);
        GLB_ui->lineEdit_28->setEnabled(false);
        GLB_ui->lineEdit_29->setEnabled(false);
        GLB_ui->lineEdit_30->setEnabled(false);
        CtrlCheckState[2] = false;
    }
}
//Speed
void MainWindow::on_checkBox_8_toggled(bool checked)
{
    if(GLB_ui->checkBox_8->isChecked())
    {
        GLB_ui->lineEdit_31->setEnabled(true);
        GLB_ui->lineEdit_32->setEnabled(true);
        GLB_ui->lineEdit_33->setEnabled(true);
        GLB_ui->lineEdit_34->setEnabled(true);
        GLB_ui->lineEdit_35->setEnabled(true);
        CtrlCheckState[2] = true;

        if(GLB_ui->checkBox_7->isChecked())
        {
            GLB_ui->checkBox_6->setChecked(false);
            GLB_ui->lineEdit_21->setEnabled(false);
            GLB_ui->lineEdit_22->setEnabled(false);
            GLB_ui->lineEdit_23->setEnabled(false);
            GLB_ui->lineEdit_24->setEnabled(false);
            GLB_ui->lineEdit_25->setEnabled(false);
            CtrlCheckState[0] = false;
        }
        else if(GLB_ui->checkBox_6->isChecked())
        {
            GLB_ui->checkBox_7->setChecked(false);
            GLB_ui->lineEdit_26->setEnabled(false);
            GLB_ui->lineEdit_27->setEnabled(false);
            GLB_ui->lineEdit_28->setEnabled(false);
            GLB_ui->lineEdit_29->setEnabled(false);
            GLB_ui->lineEdit_30->setEnabled(false);
            CtrlCheckState[1] = false;
        }
        else
        {
            GLB_ui->lineEdit_21->setEnabled(false);
            GLB_ui->lineEdit_22->setEnabled(false);
            GLB_ui->lineEdit_23->setEnabled(false);
            GLB_ui->lineEdit_24->setEnabled(false);
            GLB_ui->lineEdit_25->setEnabled(false);

            GLB_ui->lineEdit_26->setEnabled(false);
            GLB_ui->lineEdit_27->setEnabled(false);
            GLB_ui->lineEdit_28->setEnabled(false);
            GLB_ui->lineEdit_29->setEnabled(false);
            GLB_ui->lineEdit_30->setEnabled(false);

            CtrlCheckState[0] = false;
            CtrlCheckState[1] = false;
        }
    }
    else
    {
        GLB_ui->lineEdit_31->setEnabled(false);
        GLB_ui->lineEdit_32->setEnabled(false);
        GLB_ui->lineEdit_33->setEnabled(false);
        GLB_ui->lineEdit_34->setEnabled(false);
        GLB_ui->lineEdit_35->setEnabled(false);
        CtrlCheckState[2] = false;
    }
}
/*Buttons Fingers*/
void MainWindow::on_pushButton_14_clicked(bool checked)
{
    SendToTerminal("Thumb: " + QString(checked ? "true" : "false"), true, 2);
}
void MainWindow::on_pushButton_15_clicked(bool checked)
{
    SendToTerminal("Index: " + QString(checked ? "true" : "false"), true, 2);
}
void MainWindow::on_pushButton_16_clicked(bool checked)
{
    SendToTerminal("Middle: " + QString(checked ? "true" : "false"), true, 2);
}
void MainWindow::on_pushButton_17_clicked(bool checked)
{
    SendToTerminal("Ring: " + QString(checked ? "true" : "false"), true, 2);
}
void MainWindow::on_pushButton_18_clicked(bool checked)
{
    SendToTerminal("Pinkie: " + QString(checked ? "true" : "false"), true, 2);
}
/*Clear Terminal*/
void MainWindow::on_pushButton_39_clicked()
{
    ClearTerminal(0);
}
void MainWindow::on_pushButton_38_clicked()
{
    ClearTerminal(1);
}
void MainWindow::on_pushButton_37_clicked()
{
    ClearTerminal(2);
}
/*Debug Mode*/
void MainWindow::on_checkBox_9_toggled(bool checked)
{
    if (checked)
    {
        GLB_WinObj.GLB_WindowsFrame[1]->setEnabled(true);
    }
    else
    {
        GLB_WinObj.GLB_WindowsFrame[1]->setEnabled(false);
    }
}
void MainWindow::on_pushButton_11_clicked()
{
    // CheckGlobalStateMotorVariables();

    // for(uint8_t i = 0; i < 6; i++) memset(&MotorInstr[i].Flags, 0, sizeof(MotorInstr[i]));
    // /**/
    // uint8_t DataToSend[10] = {0, };
    // uint8_t cnt = 0;
    // uint16_t newConf = COM2_START_INSTR | COM_WORKMODE;
    // DataToSend[cnt] = newConf & 0x00FF;
    // cnt++;
    // DataToSend[cnt] = (newConf & 0xFF00) >> 8;
    // cnt++;
    // DataToSend[cnt] = GLB_Command.ModeWorkByte;
    // cnt++;

    // if(MotorInstr[i].Flags.FL_SelMotByte)
    // {
    //     DataToSend[i][cnt[i]] = MotorInstr[i].SelMotor;
    //     cnt[i]++;
    // }
    // if(MotorInstr[i].Flags.FL_TypeMoveByte)
    // {
    //     DataToSend[i][cnt[i]] = MotorInstr[i].TypeMove;
    //     cnt[i]++;
    // }
    // if(MotorInstr[i].Flags.FL_PWMByte)
    // {
    //     DataToSend[i][cnt[i]] = MotorInstr[i].PWM;
    //     cnt[i]++;
    // }
    // if(MotorInstr[i].Flags.FL_TimeWorkByte)
    // {
    //     DataToSend[i][cnt[i]] = MotorInstr[i].TimeWork & 0x00FF;
    //     cnt[i]++;
    //     DataToSend[i][cnt[i]] = (MotorInstr[i].TimeWork & 0xFF00) >> 8;
    //     cnt[i]++;
    // }
    // if(MotorInstr[i].Flags.FL_DelayWorkByte)
    // {
    //     DataToSend[i][cnt[i]] = MotorInstr[i].DelayWork & 0x00FF;
    //     cnt[i]++;
    //     DataToSend[i][cnt[i]] = (MotorInstr[i].DelayWork & 0xFF00) >> 8;
    //     cnt[i]++;
    // }
    // if(MotorInstr[i].Flags.FL_ADC_EnByte)
    // {
    //     DataToSend[i][cnt[i]] = MotorInstr[i].ADC_En;
    //     cnt[i]++;
    // }
    // DataToSend[i][cnt[i]] = 0xFF;           ///////////// БАЙТ РАЗДЕЛИТЕЛЬ


    // DataToSend[cnt] = GLB_Command.StartInstruct;
    // cnt++;
    // DataToSend[cnt] = 0xFF;           ///////////// БАЙТ РАЗДЕЛИТЕЛЬ / УКАЗАТЕЛЬ
    // cnt++;
    // ComPortWrite(0, (unsigned char*)DataToSend, cnt);


    // !!! CHECK CHECBOX   GLB_WinObj.GLB_WindowsCheckBox[11]
}

// Auto-Detect CheckBox
void MainWindow::on_checkBox_10_toggled(bool checked)
{

}
