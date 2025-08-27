#include "ui_mainwindow.h"
#include "mainwindow.h"
#include <QVector>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include "MyThread.h"
#include <string>


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
uint32_t GLB_I;
QList<QString> GLB_Comports;
uint32_t CurrentBoundRate;
uint8_t ComportDataToSend[50];
uint32_t ComportCountDataToSend;


uint16_t ComportCountDataIndexCnt;

/*Graph variables*/
QVector<uint8_t> GLB_Graph_x;
QVector<uint8_t> GLB_RecvRowData;

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
LastTypeCommand LTC;
uint8_t LNCM;

bool Instruct_FLAG = false;

typedef enum Fingers
{
    Thrunb = 0x00,
    Index = 0x01,
    Middle = 0x02,
    Ring = 0x03,
    Pinkie = 0x04,
} Fingers;

/*Thread variables*/
MyThread_1 *thread_1;
uint32_t dataRecvd2 = 0;
uint8_t TypeThreadInterrupt;    // 0 - Graph, 1 - Search
bool ThreadAutoConnectState;

/*Thread 2*/


/*Angle Control variables*/
RatioChannelsNumber RatioStateNow;
bool CtrlCheckState[3];








ProtezCommand Command;
ProtezMotor Motor[6] = {
    ProtezMotor(0),
    ProtezMotor(1),
    ProtezMotor(2),
    ProtezMotor(3),
    ProtezMotor(4),
    ProtezMotor(5),
};




/********************************/
void SendToTerminal(QString data, bool Newline, uint8_t tab_widget)
{
    if(tab_widget == 0)
    {
        if(Newline) GLB_ui->plainTextEdit_3->appendPlainText(data);
        else GLB_ui->plainTextEdit_3->insertPlainText(data);
    }
    else if(tab_widget == 1)
    {
        if(Newline) GLB_ui->plainTextEdit_2->appendPlainText(data);
        else GLB_ui->plainTextEdit_2->insertPlainText(data);
    }
    else if(tab_widget == 2)
    {
        if(Newline) GLB_ui->plainTextEdit->appendPlainText(data);
        else GLB_ui->plainTextEdit->insertPlainText(data);
    }
}
void ClearTerminal(uint8_t tab_widget)
{

    if(tab_widget == 0)
    {
        GLB_ui->plainTextEdit_3->clear();
    }
    else if(tab_widget == 1)
    {
        GLB_ui->plainTextEdit_2->clear();
    }
    else if(tab_widget == 2)
    {
        GLB_ui->plainTextEdit->clear();
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


void MainWindow::on_PaintGraphADC()
{
    QVector<double> DataMotor_x[12], DataMotor_y[12];
    QVector<double> AllDataToGraph;
    if ((GLB_RecvRowData.size() % 2) == 1)
    {
        qInfo() << "======ERROR: data more on 1======";
        return;
    }
    for(uint32_t i = 0; i < (uint32_t)(GLB_RecvRowData.size()); i = i + 2) {
        AllDataToGraph.append(FormulaADC((GLB_RecvRowData[i]) | (GLB_RecvRowData[i + 1] << 8)));
    }
    QByteArray TempIndexes = 0;
    uint8_t countEnabled = 0;
    bool IndexEnabled[12] = {0, };

    uint8_t index = 0;
    for (auto &mot : Motor) {
        if ((mot.getADC_State() == ADC_Enable) && (mot.getDirection() != MoveNone)) {
            IndexEnabled[index] = true;
            countEnabled++;
        }
        index++;
    }
    uint8_t i_2 = 0;
    for(uint8_t i = 0; i < 12; i++) {
        if(IndexEnabled[i]) {
            for(uint32_t j = i_2; j < (uint32_t)(AllDataToGraph.size() - countEnabled); j = j + countEnabled) {
                DataMotor_y[i].append(AllDataToGraph[j]);
            }
            TempIndexes.append(i);
            i_2++;
        }
    }

    // for(uint8_t i = 0; i < LNCMAN_len; i++)
    // {
    //     for(uint32_t j = i; j < (uint32_t)(AllDataToGraph.size() - LNCMAN_len); j = j + LNCMAN_len)
    //     {
    //         DataMotor_y[(uint8_t)LNCMAN[i]].append(AllDataToGraph[j]);
    //     }
    // }


    double cf_mid = 0.2;
    for(uint8_t i = 0; i < countEnabled; i++) {
        for(uint16_t j = 1; j < DataMotor_y[(uint16_t)TempIndexes[i]].size(); j++) {
            DataMotor_y[(uint16_t)TempIndexes[i]][j] =
                cf_mid * DataMotor_y[(uint16_t)TempIndexes[i]][j] + (1 - cf_mid) * DataMotor_y[(uint16_t)TempIndexes[i]][j - 1];
        }
    }

    for(uint8_t i = 0; i < countEnabled; i++)
    {
        double MaxVal_x = 0;
        double MaxVal_y = 0;
        double MinVal_x = 0;
        double MinVal_y = 0;
        float value_time = 0;

        if((uint16_t)TempIndexes[i] < 6)
            value_time = MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1_LineEditWorkTime->text().toFloat();
        else if((uint16_t)TempIndexes[i] >= 6)
            value_time = MotorDefStruct[(uint16_t)TempIndexes[i - 6]].TAB1_LineEditWorkTime->text().toFloat();

        double Step = value_time / DataMotor_y[(uint16_t)TempIndexes[i]].size();


        for(uint16_t j = 0; j < DataMotor_y[(uint16_t)TempIndexes[i]].size(); j++) {
            DataMotor_x[(uint16_t)TempIndexes[i]].append(Step * j);
            if(MaxVal_x < DataMotor_x[(uint16_t)TempIndexes[i]][j]) MaxVal_x = DataMotor_x[(uint16_t)TempIndexes[i]][j];
            if(MinVal_x > DataMotor_x[(uint16_t)TempIndexes[i]][j]) MinVal_x = DataMotor_x[(uint16_t)TempIndexes[i]][j];
            if(MaxVal_y < DataMotor_y[(uint16_t)TempIndexes[i]][j]) MaxVal_y = DataMotor_y[(uint16_t)TempIndexes[i]][j];
            if(MinVal_y > DataMotor_y[(uint16_t)TempIndexes[i]][j]) MinVal_y = DataMotor_y[(uint16_t)TempIndexes[i]][j];
        }

        qDebug().nospace() << "CH #" << i << " NumPack: " <<
            DataMotor_y[(uint8_t)IndexEnabled[i]].size() << " ==> "
                           << "MaxX: " << MaxVal_x << ", "
                           << "MaxY: " << MaxVal_y << ", "
                           << "MinX: " << MinVal_x << ", "
                           << "MinY: " << MinVal_y;

        // qDebug() << QString("%1%2%3%4").arg("CH #", i).arg("MaxX:", MaxVal_x).arg("MaxY:", MaxVal_y).arg("MinX:", MinVal_x).arg("MinY:", MinVal_y);

        if((uint16_t)TempIndexes[i] < 6) {
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1_ADCPlot->xAxis->setRange(MinVal_x, MaxVal_x);
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1_ADCPlot->yAxis->setRange(MinVal_y, MaxVal_y);
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1_ADCPlot->addGraph();
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1_ADCPlot->graph(0)->setPen(MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1GraphPen);
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1_ADCPlot->graph(0)->setData(DataMotor_x[(uint16_t)TempIndexes[i]], DataMotor_y[(uint16_t)TempIndexes[i]]);
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1_ADCPlot->replot();
        }
        if((uint16_t)TempIndexes[i] >= 6) {
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1_ADCPlotBack->xAxis->setRange(MinVal_x, MaxVal_x);
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1_ADCPlotBack->yAxis->setRange(MinVal_y, MaxVal_y);
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1_ADCPlotBack->addGraph();
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1_ADCPlotBack->graph(0)->setPen(MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1GraphPen);
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1_ADCPlotBack->graph(0)->setData(DataMotor_x[(uint16_t)TempIndexes[i]], DataMotor_y[(uint16_t)TempIndexes[i]]);
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1_ADCPlotBack->replot();
        }

    }

    if (GLB_ui->checkBox_29->isChecked()) {
        int fileIndex = 1;
        QString fileName;
        QFile file;

        // Ищем первый свободный номер файла
        do {
            fileName = QString("C:/Users/Roman/Desktop/ProtezHolder/Current/" + GLB_ui->lineEdit_18->text()
                               + "_N" + "1" + "P" + GLB_ui->lineEdit->text() + "_%1" + ".txt").arg(fileIndex);
            // fileName = QString("C:/Users/Roman/Desktop/ProtezHolder/Current/file.txt");

            file.setFileName(fileName);
            fileIndex++;
        } while (file.exists());

        // Открываем файл для записи
        if (file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QTextStream out(&file);

            int size = DataMotor_x[0].size();
            for (int i = 0; i < size; ++i)
            {
                out << DataMotor_x[0][i] << '\t' << DataMotor_y[0][i] << "\n";
            }

            file.close();
            qDebug() << "Data is saved to file:" << fileName;
        }
        else
        {
            qWarning() << "Data don`t save to file" << fileName;
        }
        file.close();
    }

    LNCM = 0;
    memset(IndexEnabled, '\0', 12);
    GLB_RecvRowData.clear();
}

void MainWindow::on_PaintGraphFeedBack()
{
    QVector<double> DataMotor_x[12], DataMotor_y[12];
    // Собираем все данные по парам байт
    QVector<double> AllDataToGraph;
    for(uint32_t i = 0; i < (uint32_t)(GLB_RecvRowData.size()); i = i + 2) {
        AllDataToGraph.append((GLB_RecvRowData[i]) | (GLB_RecvRowData[i + 1] << 8));
    }

    QByteArray TempIndexes;
    uint8_t countEnabled = 0;
    bool IndexEnabled[12];

    uint8_t index = 0;
    for (auto &mot : Motor) {
        if ((mot.getADC_State() == ADC_Enable) && (mot.getDirection() != MoveNone)) {
            IndexEnabled[index] = true;
            countEnabled++;
        }
        index++;
    }
    for(uint8_t i = 0; i < 12; i++) {
        if(IndexEnabled[i]) {
            for(uint32_t j = i; j < (uint32_t)(AllDataToGraph.size() - countEnabled); j = j + countEnabled) {
                DataMotor_y[i].append(AllDataToGraph[j]);
            }
            TempIndexes.append(i);
        }
    }


    // for(uint8_t i = 0; i < LNCMAN_len; i++)
    // {
    //     for(uint32_t j = i; j < (uint32_t)(AllDataToGraph.size() - LNCMAN_len); j = j + LNCMAN_len)
    //     {
    //         DataMotor_y[(uint8_t)LNCMAN[i]].append(AllDataToGraph[j]);
    //     }
    // }
    // Скользящяя средняя 2 варианта
    // for(uint16_t i = 5; i < NewGraph_y.size(); i++)
    //     NewGraph_y[i] = (NewGraph_y[i] + NewGraph_y[i - 1] + NewGraph_y[i - 2] + NewGraph_y[i - 3] + NewGraph_y[i - 4] + NewGraph_y[i - 5]) / 6.0;

    double cf_mid = 0.2;
    for(uint8_t i = 0; i < countEnabled; i++)
    {
        for(uint16_t j = 1; j < DataMotor_y[(uint16_t)TempIndexes[i]].size(); j++)
        {
            DataMotor_y[(uint16_t)TempIndexes[i]][j] =
                cf_mid * DataMotor_y[(uint16_t)TempIndexes[i]][j] + (1 - cf_mid) * DataMotor_y[(uint16_t)TempIndexes[i]][j - 1];
        }
    }

    for(uint8_t i = 0; i < countEnabled; i++)
    {
        double MaxVal_x = 0;
        double MaxVal_y = 0;
        double MinVal_x = 0;
        double MinVal_y = 0;

        float value_time = 0;

        // if(TempIndexes[i] < 6) value_time = std::stof(MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1_LineEditWorkTime->text().toStdString());
        // else if(TempIndexes[i] >= 6) value_time = std::stof(MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1_LineEditWorkTime->text().toStdString());


        double Step = 1.0 / DataMotor_y[(uint16_t)TempIndexes[i]].size();


        for(double j = 0; j < DataMotor_y[(uint16_t)TempIndexes[i]].size(); j++)
        {
            DataMotor_x[(uint16_t)TempIndexes[i]].append(Step * j);
            if(MaxVal_x < DataMotor_x[(uint16_t)TempIndexes[i]][j]) MaxVal_x = DataMotor_x[(uint16_t)TempIndexes[i]][j];
            if(MinVal_x > DataMotor_x[(uint16_t)TempIndexes[i]][j]) MinVal_x = DataMotor_x[(uint16_t)TempIndexes[i]][j];
            if(MaxVal_y < DataMotor_y[(uint16_t)TempIndexes[i]][j]) MaxVal_y = DataMotor_y[(uint16_t)TempIndexes[i]][j];
            if(MinVal_y > DataMotor_y[(uint16_t)TempIndexes[i]][j]) MinVal_y = DataMotor_y[(uint16_t)TempIndexes[i]][j];
        }

        qDebug().nospace() << "CH #" << i << " NumPack: " << DataMotor_y[(uint8_t)IndexEnabled[i]].size() << " ==> "<< "MaxX: " << MaxVal_x << ", " << "MaxY: " << MaxVal_y << ", " << "MinX: " << MinVal_x << ", " << "MinY: " << MinVal_y;

        // qDebug() << QString("%1%2%3%4").arg("CH #", i).arg("MaxX:", MaxVal_x).arg("MaxY:", MaxVal_y).arg("MinX:", MinVal_x).arg("MinY:", MinVal_y);

        if((uint16_t)TempIndexes[i] < 6)
        {
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB2_FeedBackPlot->xAxis->setRange(MinVal_x, MaxVal_x);
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB2_FeedBackPlot->yAxis->setRange(MinVal_y, MaxVal_y);
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB2_FeedBackPlot->addGraph();
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB2_FeedBackPlot->graph(0)->setPen(MotorDefStruct[(uint16_t)TempIndexes[i]].TAB1GraphPen);
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB2_FeedBackPlot->graph(0)->setData(DataMotor_x[(uint16_t)TempIndexes[i]], DataMotor_y[(uint16_t)TempIndexes[i]]);
            MotorDefStruct[(uint16_t)TempIndexes[i]].TAB2_FeedBackPlot->replot();
        }
        if((uint16_t)TempIndexes[i] >= 6)
        {
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB2_FeedBackPlotBack->xAxis->setRange(MinVal_x, MaxVal_x);
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB2_FeedBackPlotBack->yAxis->setRange(MinVal_y, MaxVal_y);
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB2_FeedBackPlotBack->addGraph();
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB2_FeedBackPlotBack->graph(0)->setPen(MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB1GraphPen);
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB2_FeedBackPlotBack->graph(0)->setData(DataMotor_x[(uint16_t)TempIndexes[i]], DataMotor_y[(uint16_t)TempIndexes[i]]);
            MotorDefStruct[(uint16_t)TempIndexes[i] - 6].TAB2_FeedBackPlotBack->replot();
        }
    }
    LNCM = 0;
    memset(IndexEnabled, '\0', 12);
    GLB_RecvRowData.clear();
}






void SetStartGUISettings()
{
    /*****************************************/
    MotorDefStruct[0].TAB1_ComporessButton = GLB_ui->pushButton;
    MotorDefStruct[0].TAB1_DecompressButton = GLB_ui->pushButton_2;
    MotorDefStruct[0].TAB1_HoldButton = GLB_ui->pushButton_21;
    MotorDefStruct[0].TAB1_FreeButton = GLB_ui->pushButton_27;

    MotorDefStruct[0].TAB1_LineEditPWM = GLB_ui->lineEdit;
    MotorDefStruct[0].TAB1_LineEditWorkTime = GLB_ui->lineEdit_6;
    MotorDefStruct[0].TAB1_LineEditDelayTime = GLB_ui->lineEdit_11;
    MotorDefStruct[0].TAB1_SliderPWM = GLB_ui->horizontalSlider;
    MotorDefStruct[0].TAB1_CheckBoxADC = GLB_ui->checkBox;
    MotorDefStruct[0].TAB1_CheckBoxBackSide = GLB_ui->checkBox_14;
    MotorDefStruct[0].TAB2_CheckBoxBackSide = GLB_ui->checkBox_22;
    MotorDefStruct[0].TAB1_ADCPlot = GLB_ui->widget;
    MotorDefStruct[0].TAB1GraphPen = QPen(Qt::red);
    MotorDefStruct[0].TAB1_CheckBoxAutoCurrectBackPower = GLB_ui->checkBox_15;


    MotorDefStruct[0].TAB2_FingerButton = GLB_ui->pushButton_14;
    MotorDefStruct[0].TAB2_LineEditAngle = GLB_ui->lineEdit_23;
    MotorDefStruct[0].TAB2_LineEditTime = GLB_ui->lineEdit_28;
    MotorDefStruct[0].TAB2_LineEditSpeed = GLB_ui->lineEdit_33;
    MotorDefStruct[0].TAB2_LineEditDelay = GLB_ui->lineEdit_44;
    MotorDefStruct[0].TAB2_CheckBoxCH = GLB_ui->checkBox_16;
    MotorDefStruct[0].TAB2_FeedBackPlot = GLB_ui->widget_2;
    MotorDefStruct[0].TAB2_CheckBoxBackReverse = GLB_ui->checkBox_23;

    MotorDefStruct[0].TAB2_FeedBackPlot = GLB_ui->widget_2;


    /*****************************************/
    MotorDefStruct[1].TAB1_ComporessButton = GLB_ui->pushButton_3;
    MotorDefStruct[1].TAB1_DecompressButton = GLB_ui->pushButton_4;
    MotorDefStruct[1].TAB1_HoldButton = GLB_ui->pushButton_23;
    MotorDefStruct[1].TAB1_FreeButton = GLB_ui->pushButton_28;

    MotorDefStruct[1].TAB1_LineEditPWM = GLB_ui->lineEdit_2;
    MotorDefStruct[1].TAB1_LineEditWorkTime = GLB_ui->lineEdit_7;
    MotorDefStruct[1].TAB1_LineEditDelayTime = GLB_ui->lineEdit_12;
    MotorDefStruct[1].TAB1_SliderPWM = GLB_ui->horizontalSlider_2;
    MotorDefStruct[1].TAB1_CheckBoxADC = GLB_ui->checkBox_2;
    MotorDefStruct[1].TAB1_CheckBoxBackSide = GLB_ui->checkBox_14;
    MotorDefStruct[1].TAB2_CheckBoxBackSide = GLB_ui->checkBox_22;
    MotorDefStruct[1].TAB1_ADCPlot = GLB_ui->widget_5;
    MotorDefStruct[1].TAB1GraphPen = QPen(Qt::red);
    MotorDefStruct[1].TAB1_CheckBoxAutoCurrectBackPower = GLB_ui->checkBox_15;

    MotorDefStruct[1].TAB2_FingerButton = GLB_ui->pushButton_15;
    MotorDefStruct[1].TAB2_LineEditAngle = GLB_ui->lineEdit_22;
    MotorDefStruct[1].TAB2_LineEditTime = GLB_ui->lineEdit_27;
    MotorDefStruct[1].TAB2_LineEditSpeed = GLB_ui->lineEdit_35;
    MotorDefStruct[1].TAB2_LineEditDelay = GLB_ui->lineEdit_41;
    MotorDefStruct[1].TAB2_CheckBoxCH = GLB_ui->checkBox_17;
    MotorDefStruct[1].TAB2_FeedBackPlot = GLB_ui->widget_9;
    MotorDefStruct[1].TAB2_CheckBoxBackReverse = GLB_ui->checkBox_24;

    MotorDefStruct[1].TAB2_FeedBackPlot = GLB_ui->widget_9;



    /*****************************************/
    MotorDefStruct[2].TAB1_ComporessButton = GLB_ui->pushButton_5;
    MotorDefStruct[2].TAB1_DecompressButton = GLB_ui->pushButton_6;
    MotorDefStruct[2].TAB1_HoldButton = GLB_ui->pushButton_19;
    MotorDefStruct[2].TAB1_FreeButton = GLB_ui->pushButton_26;


    MotorDefStruct[2].TAB1_LineEditPWM = GLB_ui->lineEdit_3;
    MotorDefStruct[2].TAB1_LineEditWorkTime = GLB_ui->lineEdit_10;
    MotorDefStruct[2].TAB1_LineEditDelayTime = GLB_ui->lineEdit_15;
    MotorDefStruct[2].TAB1_SliderPWM = GLB_ui->horizontalSlider_3;
    MotorDefStruct[2].TAB1_CheckBoxADC = GLB_ui->checkBox_3;
    MotorDefStruct[2].TAB1_CheckBoxBackSide = GLB_ui->checkBox_14;
    MotorDefStruct[2].TAB2_CheckBoxBackSide = GLB_ui->checkBox_22;
    MotorDefStruct[2].TAB1_ADCPlot = GLB_ui->widget_4;

    MotorDefStruct[2].TAB1GraphPen = QPen(Qt::red);
    MotorDefStruct[2].TAB1_CheckBoxAutoCurrectBackPower = GLB_ui->checkBox_15;

    MotorDefStruct[2].TAB2_FingerButton = GLB_ui->pushButton_16;
    MotorDefStruct[2].TAB2_LineEditAngle = GLB_ui->lineEdit_21;
    MotorDefStruct[2].TAB2_LineEditTime = GLB_ui->lineEdit_26;
    MotorDefStruct[2].TAB2_LineEditSpeed = GLB_ui->lineEdit_34;
    MotorDefStruct[2].TAB2_LineEditDelay = GLB_ui->lineEdit_42;
    MotorDefStruct[2].TAB2_CheckBoxCH = GLB_ui->checkBox_18;
    MotorDefStruct[2].TAB2_FeedBackPlot = GLB_ui->widget_3;
    MotorDefStruct[2].TAB2_CheckBoxBackReverse = GLB_ui->checkBox_25;

    MotorDefStruct[2].TAB2_FeedBackPlot = GLB_ui->widget_3;






    /*****************************************/
    MotorDefStruct[3].TAB1_ComporessButton = GLB_ui->pushButton_7;
    MotorDefStruct[3].TAB1_DecompressButton = GLB_ui->pushButton_8;
    MotorDefStruct[3].TAB1_HoldButton = GLB_ui->pushButton_22;
    MotorDefStruct[3].TAB1_FreeButton = GLB_ui->pushButton_24;


    MotorDefStruct[3].TAB1_LineEditPWM =  GLB_ui->lineEdit_4;
    MotorDefStruct[3].TAB1_LineEditWorkTime = GLB_ui->lineEdit_9;
    MotorDefStruct[3].TAB1_LineEditDelayTime = GLB_ui->lineEdit_14;
    MotorDefStruct[3].TAB1_SliderPWM = GLB_ui->horizontalSlider_4;
    MotorDefStruct[3].TAB1_CheckBoxADC = GLB_ui->checkBox_4;
    MotorDefStruct[3].TAB1_CheckBoxBackSide = GLB_ui->checkBox_14;
    MotorDefStruct[3].TAB2_CheckBoxBackSide = GLB_ui->checkBox_22;
    MotorDefStruct[3].TAB1_ADCPlot = GLB_ui->widget_6;
    MotorDefStruct[3].TAB1GraphPen = QPen(Qt::red);
    MotorDefStruct[3].TAB1_CheckBoxAutoCurrectBackPower = GLB_ui->checkBox_15;

    MotorDefStruct[3].TAB2_FingerButton = GLB_ui->pushButton_17;
    MotorDefStruct[3].TAB2_LineEditAngle = GLB_ui->lineEdit_24;
    MotorDefStruct[3].TAB2_LineEditTime = GLB_ui->lineEdit_29;
    MotorDefStruct[3].TAB2_LineEditSpeed = GLB_ui->lineEdit_31;
    MotorDefStruct[3].TAB2_LineEditDelay = GLB_ui->lineEdit_43;
    MotorDefStruct[3].TAB2_CheckBoxCH = GLB_ui->checkBox_19;
    MotorDefStruct[3].TAB2_FeedBackPlot = GLB_ui->widget_10;
    MotorDefStruct[3].TAB2_CheckBoxBackReverse = GLB_ui->checkBox_26;

    MotorDefStruct[3].TAB2_FeedBackPlot = GLB_ui->widget_10;



    /*****************************************/
    MotorDefStruct[4].TAB1_ComporessButton = GLB_ui->pushButton_9;
    MotorDefStruct[4].TAB1_DecompressButton = GLB_ui->pushButton_10;
    MotorDefStruct[4].TAB1_HoldButton = GLB_ui->pushButton_20;
    MotorDefStruct[4].TAB1_FreeButton = GLB_ui->pushButton_25;

    MotorDefStruct[4].TAB1_LineEditPWM = GLB_ui->lineEdit_5;
    MotorDefStruct[4].TAB1_LineEditWorkTime = GLB_ui->lineEdit_8;
    MotorDefStruct[4].TAB1_LineEditDelayTime = GLB_ui->lineEdit_13;
    MotorDefStruct[4].TAB1_SliderPWM = GLB_ui->horizontalSlider_5;
    MotorDefStruct[4].TAB1_CheckBoxADC = GLB_ui->checkBox_5;
    MotorDefStruct[4].TAB1_CheckBoxBackSide = GLB_ui->checkBox_14;
    MotorDefStruct[4].TAB2_CheckBoxBackSide = GLB_ui->checkBox_22;
    MotorDefStruct[4].TAB1_ADCPlot = GLB_ui->widget_7;
    MotorDefStruct[4].TAB1GraphPen = QPen(Qt::red);
    MotorDefStruct[4].TAB1_CheckBoxAutoCurrectBackPower = GLB_ui->checkBox_15;

    MotorDefStruct[4].TAB2_FingerButton = GLB_ui->pushButton_18;
    MotorDefStruct[4].TAB2_LineEditAngle = GLB_ui->lineEdit_25;
    MotorDefStruct[4].TAB2_LineEditTime = GLB_ui->lineEdit_30;
    MotorDefStruct[4].TAB2_LineEditSpeed = GLB_ui->lineEdit_32;
    MotorDefStruct[4].TAB2_LineEditDelay = GLB_ui->lineEdit_40;
    MotorDefStruct[4].TAB2_CheckBoxCH = GLB_ui->checkBox_20;
    MotorDefStruct[4].TAB2_FeedBackPlot = GLB_ui->widget_11;
    MotorDefStruct[4].TAB2_CheckBoxBackReverse = GLB_ui->checkBox_27;

    MotorDefStruct[4].TAB2_FeedBackPlot = GLB_ui->widget_11;

    /*****************************************/
    MotorDefStruct[5].TAB1_ComporessButton = GLB_ui->pushButton_30;
    MotorDefStruct[5].TAB1_DecompressButton = GLB_ui->pushButton_31;
    MotorDefStruct[5].TAB1_HoldButton = GLB_ui->pushButton_32;
    MotorDefStruct[5].TAB1_FreeButton = GLB_ui->pushButton_33;

    MotorDefStruct[5].TAB1_LineEditPWM = GLB_ui->lineEdit_36;
    MotorDefStruct[5].TAB1_LineEditWorkTime = GLB_ui->lineEdit_37;
    MotorDefStruct[5].TAB1_LineEditDelayTime = GLB_ui->lineEdit_38;
    MotorDefStruct[5].TAB1_SliderPWM = GLB_ui->horizontalSlider_6;
    MotorDefStruct[5].TAB1_CheckBoxADC = GLB_ui->checkBox_13;
    MotorDefStruct[5].TAB1_CheckBoxBackSide = GLB_ui->checkBox_14;
    MotorDefStruct[5].TAB2_CheckBoxBackSide = GLB_ui->checkBox_22;
    MotorDefStruct[5].TAB1_ADCPlot = GLB_ui->widget_8;
    MotorDefStruct[5].TAB1GraphPen = QPen(Qt::red);
    MotorDefStruct[5].TAB1_CheckBoxAutoCurrectBackPower = GLB_ui->checkBox_15;

    MotorDefStruct[5].TAB2_FingerButton = GLB_ui->pushButton_40;
    MotorDefStruct[5].TAB2_LineEditAngle = GLB_ui->lineEdit_47;
    MotorDefStruct[5].TAB2_LineEditTime = GLB_ui->lineEdit_39;
    MotorDefStruct[5].TAB2_LineEditSpeed = GLB_ui->lineEdit_45;
    MotorDefStruct[5].TAB2_LineEditDelay = GLB_ui->lineEdit_46;
    MotorDefStruct[5].TAB2_CheckBoxCH = GLB_ui->checkBox_21;
    MotorDefStruct[5].TAB2_FeedBackPlot = GLB_ui->widget_12;
    MotorDefStruct[5].TAB2_CheckBoxBackReverse = GLB_ui->checkBox_28;

    MotorDefStruct[5].TAB2_FeedBackPlot = GLB_ui->widget_12;
    /*****************************************/


    /****************************************************************************************/
    /*****************************************/
    /*Set validator for Line Edit*/
    // QDoubleValidator *validator = new QDoubleValidator(0, 100, 2, GLB_mainwindowWidget);
    // validator->setNotation(QDoubleValidator::StandardNotation);
    // validator->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));

    // QList<QLineEdit*> lineEdits;
    // lineEdits = GLB_ui->tab_1->findChildren<QLineEdit*>();

    // for (QLineEdit *lineEdit : lineEdits)
    // {
    //     if (lineEdit != GLB_ui->lineEdit_18)
    //     {
    //         lineEdit->setValidator(validator);
    //     }
    // }
    // lineEdits = GLB_ui->tab_2->findChildren<QLineEdit*>();
    // for (QLineEdit *lineEdit : lineEdits)
    // {
    //     if (lineEdit != GLB_WinObj.GLB_WindowsLineEdit[47])
    //     {
    //         lineEdit->setValidator(validator);
    //     }
    // }
    /*****************************************/
    /*Set background for buttons*/
    const QList<QPushButton*> PushButtons = GLB_mainwindowWidget->findChildren<QPushButton*>();
    for(QPushButton *pushbutton : PushButtons) pushbutton->setStyleSheet("background-color: rgb(150, 200, 250);");
    /*****************************************/
    // Set motor def
    GLB_ui->comboBox_2->addItem("Thumb");
    GLB_ui->comboBox_2->addItem("Index");
    GLB_ui->comboBox_2->addItem("Middle");
    GLB_ui->comboBox_2->addItem("Ring");
    GLB_ui->comboBox_2->addItem("Pinkie");
    /*****************************************/
    /*Enable Debug panel*/
    // GLB_ui->checkBox_9->setChecked(false);
    // GLB_WinObj.GLB_WindowsFrame[1]->setEnabled(false);
    /*Enable SaveFile lineEdits*/
    // GLB_ui->lineEdit_18->setEnabled(false);
    // GLB_ui->lineEdit_50->setEnabled(false);
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


void MainWindow::on_ComportSearchBack(QList<QString> list)
{
    GLB_ui->comboBox->clear();
    qInfo() << list;
    GLB_ui->comboBox->addItems(list);
}
void MainWindow::on_ComportConnectBack(QString portName)
{
    if (portName != "Select comport...")
    {
        qInfo() << "Connect to " + portName;
        SendToTerminal("Connect to " + portName, true, 0);
        SendToTerminal("Connect to " + portName, true, 1);
        SendToTerminal("Connect to " + portName, true, 2);
    }
    QStandardItemModel* model = (QStandardItemModel*) GLB_ui->comboBox->model();
    if (!model) return;

    int count = model->rowCount();
    for (int i = 0; i < count; ++i) {
        QStandardItem* item = model->item(i);
        if (!item) continue;

        if (model->item(i)->text() == portName) {
            item->setEnabled(false);
        } else {
            item->setEnabled(true);
        }
    }

}
void MainWindow::on_ComportCloseBack(QString portName)
{
    if (portName != "Select comport...")
    {
        SendToTerminal("Comport " + portName + " close.", true, 0);
        SendToTerminal("Comport " + portName + " close.", true, 1);
        SendToTerminal("Comport " + portName + " close.", true, 2);
    }
}
void MainWindow::on_ComportWriteBack()
{
    Command.clear();
}
void MainWindow::on_ComportReadBack()
{

}

// void MainWindow::ComPortWrite(uint8_t *datatosend, uint32_t cntdata)
// {
//     for (uint32_t i = 0; i < cntdata; i++)
//     {
//         ComportDataToSend[i] = datatosend[i];
//     }
//     ComportCountDataToSend = cntdata;

//     TypeThreadInterrupt = 4;

//     // thread_1->start();
//     // while(!thread_1->isRunning()) {}
//     SendToTerminal("Thread#1: sending...", true, 1);
//     SendToTerminal("Thread#1: sending...", true, 2);
// }
// void MainWindow::ComPortRead()
// {
//     TypeThreadInterrupt = 5;

//     // thread_1->start();
//     // while(!thread_1->isRunning()) {}
//     SendToTerminal("Thread#1: reading...", true, 0);
// }


// void MainWindow::ComportDataUpdate_slot()
// {
//     GLB_ui->comboBox->clear();
//     GLB_ui->comboBox->addItems(GLB_Comports);

//     if(GLB_ui->checkBox_10->isChecked())
//     {
//         uint8_t index = GLB_ui->comboBox->findText(CurrentComPort);
//         QStandardItemModel* model = (QStandardItemModel*) GLB_ui->comboBox->model();
//         model->item(index)->setEnabled(false);
//         NowComBoxItem[0] = CurrentComPort;
//     }
//     GLB_ui->comboBox->setCurrentText(CurrentComPort);

//     SendToTerminal("Thread#1: data update.", true, 0);
//     GLB_ui->comboBox->setEnabled(true);
//     GLB_ui->checkBox_10->setEnabled(true);
//     GLB_ui->SearchButton->setEnabled(true);

// }
// void MainWindow::ComportConnect_slot()
// {
//     uint8_t index = GLB_ui->comboBox->findText(CurrentComPort);
//     QStandardItemModel* model = (QStandardItemModel*) GLB_ui->comboBox->model();
//     model->item(index)->setEnabled(false);
//     NowComBoxItem[0] = CurrentComPort;

//     if(CurrentComPort != "Select comport...")
//         SendToTerminal("Thread#1: open port " + CurrentComPort + " with speed: " + QString::number(CurrentBoundRate, 10) + ".", true, 0);
// }
// void MainWindow::ComportClose_slot()
// {
//     uint8_t index = GLB_ui->comboBox->findText(NowComBoxItem[0]);
//     QStandardItemModel* model = (QStandardItemModel*) GLB_ui->comboBox->model();
//     model->item(index)->setEnabled(true);

//     if(NowComBoxItem[0] != "Select comport...") SendToTerminal("Thread#1: close port " + NowComBoxItem[0] + ".", true, 0);
// }
// void MainWindow::ComportRead_slot()
// {
//     // SendToTerminal("Number of received data: " + QString::number(ComportCountdataRecv, 10) + "bytes.", true, 0);
// }
// void MainWindow::ComportWrite_slot(QString back)
// {
//     SendToTerminal("Feedback: " + back, true, 1);

//     if(Instruct_FLAG)
//     {
//         if(LTC == Last_PWM_MODE)
//         {
//             for(uint8_t i = 0; i < 6; i++)
//             {
//                 if(MotorDefStruct[i].MD1_ADC_CH == 0x01)
//                 {
//                     ComPortRead();
//                     break;
//                 }
//             }
//         }
//         else if(LTC == Last_ANGLE_MODE)
//         {
//             for(uint8_t i = 0; i < 6; i++)
//             {
//                 if(MotorDefStruct[i].MD2_FeedBack == 0x01)
//                 {
//                     ComPortRead();
//                     break;
//                 }
//             }
//         }
//         Instruct_FLAG = false;
//     }
// }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    std::string ver;
    switch (__cplusplus){
    case 1:
        ver = "pre-standard C++";
        break;
    case 199711L:
        ver = "C++98";
        break;
    case 201103L:
        ver = "C++11";
        break;
    case 201402L:
        ver = "C++14";
        break;
    case 201703L:
        ver = "C++17";
        break;
    case 202002L:
        ver = "C++20";
        break;
    case 202100L:
        ver = "C++20";
        break;
    default:
        ver = "Unknown";
    }
    std::cout << "Your standard is " << ver << '\n' << "__cplusplus = " << __cplusplus << std::endl;
    GLB_ui = ui;
    GLB_mainwindowWidget = this;
    showMaximized();


    thread_1 = new MyThread_1(this);

    QObject::connect(this, &MainWindow::signal_ComportSearch, thread_1, &MyThread_1::on_ComportSearch);
    QObject::connect(this, &MainWindow::signal_ComportConnect, thread_1, &MyThread_1::on_ComportConnect);
    QObject::connect(this, &MainWindow::signal_ComportClose, thread_1, &MyThread_1::on_ComportClose);
    QObject::connect(this, &MainWindow::signal_ComportWrite, thread_1, &MyThread_1::on_ComportWrite);
    QObject::connect(this, &MainWindow::signal_ComportStartRead, thread_1, &MyThread_1::on_ComportStartRead);

    QObject::connect(thread_1, &MyThread_1::signal_ComportSearchBack, this, &MainWindow::on_ComportSearchBack);
    QObject::connect(thread_1, &MyThread_1::signal_ComportConnectBack, this, &MainWindow::on_ComportConnectBack);
    QObject::connect(thread_1, &MyThread_1::signal_ComportCloseBack, this, &MainWindow::on_ComportCloseBack);
    QObject::connect(thread_1, &MyThread_1::signal_ComportWriteBack, this, &MainWindow::on_ComportWriteBack);
    QObject::connect(thread_1, &MyThread_1::signal_ComportReadBack, this, &MainWindow::on_ComportReadBack);

    QObject::connect(thread_1, &::MyThread_1::signal_PaintADC, this, &MainWindow::on_PaintGraphADC);
    QObject::connect(thread_1, &::MyThread_1::signal_PaintFeedBack, this, &MainWindow::on_PaintGraphFeedBack);


    // void signal_ComportCloseBack(QString);
    // void on_ComportCloseBack(QString);


    thread_1->start();
    while(!thread_1->isRunning()) {}


    SetStartGUISettings();
    SetStartVariables();




    Motor[0].MANUAL_Button_compress = GLB_ui->pushButton;
    Motor[0].MANUAL_Button_decompress = GLB_ui->pushButton_2;
    Motor[0].MANUAL_Button_hold = GLB_ui->pushButton_21;
    Motor[0].MANUAL_Button_stop = GLB_ui->pushButton_27;
    Motor[0].MANUAL_LineEdit_PWM = GLB_ui->lineEdit;
    Motor[0].MANUAL_LineEdit_WorkTime = GLB_ui->lineEdit_6;
    Motor[0].MANUAL_LineEdit_WorkDelay = GLB_ui->lineEdit_11;
    Motor[0].MANUAL_CheckBox_ADC = GLB_ui->checkBox;
    Motor[0].MANUAL_CheckBox_FeedBack = GLB_ui->checkBox_16;
    Motor[0].MANUAL_CheckBox_SidePlate = GLB_ui->checkBox_14;
    Motor[0].ANGLE_CheckBox_SidePlate = GLB_ui->checkBox_22;

    Motor[1].MANUAL_Button_compress = GLB_ui->pushButton_3;
    Motor[1].MANUAL_Button_decompress = GLB_ui->pushButton_23;
    Motor[1].MANUAL_Button_hold = GLB_ui->pushButton_4;
    Motor[1].MANUAL_Button_stop = GLB_ui->pushButton_28;
    Motor[1].MANUAL_LineEdit_PWM = GLB_ui->lineEdit_2;
    Motor[1].MANUAL_LineEdit_WorkTime = GLB_ui->lineEdit_7;
    Motor[1].MANUAL_LineEdit_WorkDelay = GLB_ui->lineEdit_12;
    Motor[1].MANUAL_CheckBox_ADC = GLB_ui->checkBox_2;
    Motor[1].MANUAL_CheckBox_FeedBack = GLB_ui->checkBox_17;
    Motor[1].MANUAL_CheckBox_SidePlate = GLB_ui->checkBox_14;
    Motor[1].ANGLE_CheckBox_SidePlate = GLB_ui->checkBox_22;

    Motor[2].MANUAL_Button_compress = GLB_ui->pushButton_5;
    Motor[2].MANUAL_Button_decompress = GLB_ui->pushButton_6;
    Motor[2].MANUAL_Button_hold = GLB_ui->pushButton_19;
    Motor[2].MANUAL_Button_stop = GLB_ui->pushButton_26;
    Motor[2].MANUAL_LineEdit_PWM = GLB_ui->lineEdit_3;
    Motor[2].MANUAL_LineEdit_WorkTime = GLB_ui->lineEdit_10;
    Motor[2].MANUAL_LineEdit_WorkDelay = GLB_ui->lineEdit_15;
    Motor[2].MANUAL_CheckBox_ADC = GLB_ui->checkBox_3;
    Motor[2].MANUAL_CheckBox_FeedBack = GLB_ui->checkBox_18;
    Motor[2].MANUAL_CheckBox_SidePlate = GLB_ui->checkBox_14;
    Motor[2].ANGLE_CheckBox_SidePlate = GLB_ui->checkBox_22;

    Motor[3].MANUAL_Button_compress = GLB_ui->pushButton_7;
    Motor[3].MANUAL_Button_decompress = GLB_ui->pushButton_22;
    Motor[3].MANUAL_Button_hold = GLB_ui->pushButton_24;
    Motor[3].MANUAL_Button_stop = GLB_ui->pushButton_8;
    Motor[3].MANUAL_LineEdit_PWM = GLB_ui->lineEdit_4;
    Motor[3].MANUAL_LineEdit_WorkTime = GLB_ui->lineEdit_9;
    Motor[3].MANUAL_LineEdit_WorkDelay = GLB_ui->lineEdit_14;
    Motor[3].MANUAL_CheckBox_ADC = GLB_ui->checkBox_4;
    Motor[3].MANUAL_CheckBox_FeedBack = GLB_ui->checkBox_19;
    Motor[3].MANUAL_CheckBox_SidePlate = GLB_ui->checkBox_14;
    Motor[3].ANGLE_CheckBox_SidePlate = GLB_ui->checkBox_22;

    Motor[4].MANUAL_Button_compress = GLB_ui->pushButton_9;
    Motor[4].MANUAL_Button_decompress = GLB_ui->pushButton_10;
    Motor[4].MANUAL_Button_hold = GLB_ui->pushButton_20;
    Motor[4].MANUAL_Button_stop = GLB_ui->pushButton_25;
    Motor[4].MANUAL_LineEdit_PWM = GLB_ui->lineEdit_5;
    Motor[4].MANUAL_LineEdit_WorkTime = GLB_ui->lineEdit_8;
    Motor[4].MANUAL_LineEdit_WorkDelay = GLB_ui->lineEdit_13;
    Motor[4].MANUAL_CheckBox_ADC = GLB_ui->checkBox_5;
    Motor[4].MANUAL_CheckBox_FeedBack = GLB_ui->checkBox_20;
    Motor[4].MANUAL_CheckBox_SidePlate = GLB_ui->checkBox_14;
    Motor[4].ANGLE_CheckBox_SidePlate = GLB_ui->checkBox_22;

    Motor[5].MANUAL_Button_compress = GLB_ui->pushButton_30;
    Motor[5].MANUAL_Button_decompress = GLB_ui->pushButton_31;
    Motor[5].MANUAL_Button_hold = GLB_ui->pushButton_32;
    Motor[5].MANUAL_Button_stop = GLB_ui->pushButton_33;
    Motor[5].MANUAL_LineEdit_PWM = GLB_ui->lineEdit_36;
    Motor[5].MANUAL_LineEdit_WorkTime = GLB_ui->lineEdit_37;
    Motor[5].MANUAL_LineEdit_WorkDelay = GLB_ui->lineEdit_38;
    Motor[5].MANUAL_CheckBox_ADC = GLB_ui->checkBox_13;
    Motor[5].MANUAL_CheckBox_FeedBack = GLB_ui->checkBox_21;
    Motor[5].MANUAL_CheckBox_SidePlate = GLB_ui->checkBox_14;
    Motor[5].ANGLE_CheckBox_SidePlate = GLB_ui->checkBox_22;

}

MainWindow::~MainWindow()
{
    while(!thread_1->isFinished()) {}
    delete thread_1;

    ConfigFile.open(QFile::WriteOnly);
    IOFile.setDevice(&ConfigFile);
    ConfigFile.close();

    delete ui;
}

void MainWindow::on_pushButton_12_clicked()
{
    uint8_t data[100] =
        {
            0xEA, 0xBC,
            0xDE, 0xAD,

            0x21, 0x00,
            0x22, 0x01,
            0x24, 0x00,
            0x25, 0x01,
            0x26, 0x64,
            0x27, 0x64,
            0x2A, 0x64,

            0xBE, 0xEF,
            0xBC, 0xAE,
        };


    // emit signal_ComportWrite(data);
}
void MainWindow::on_pushButton_41_clicked()
{
    uint8_t data[100] =
        {
            0xEA, 0xBC,
            0xDE, 0xAD,

            0x21, 0x00,
            0x22, 0x01,
            0x24, 0x00,
            0x2D, 0x01,

            0xBE, 0xEF,
            0xBC, 0xAE,
        };
    // ComPortWrite((unsigned char *)data, 16);
}





/* PWM LineEdit */
void MainWindow::on_lineEdit_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        MotorDefStruct[0].TAB1_SliderPWM->setValue(value);
    }
}
void MainWindow::on_lineEdit_2_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        MotorDefStruct[1].TAB1_SliderPWM->setValue(value);
    }
}
void MainWindow::on_lineEdit_3_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        MotorDefStruct[2].TAB1_SliderPWM->setValue(value);
    }
}
void MainWindow::on_lineEdit_4_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        MotorDefStruct[3].TAB1_SliderPWM->setValue(value);
    }
}
void MainWindow::on_lineEdit_5_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        MotorDefStruct[4].TAB1_SliderPWM->setValue(value);
    }
}

void MainWindow::on_lineEdit_36_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
        uint16_t value = std::stoi(arg1.toStdString());
        MotorDefStruct[5].TAB1_SliderPWM->setValue(value);
    }
}
/* TIME LineEdit */
void MainWindow::on_lineEdit_7_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_10_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_9_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_8_textEdited(const QString &arg1){}
/* DELAY LineEdit */
void MainWindow::on_lineEdit_11_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_12_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_15_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_14_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_13_textEdited(const QString &arg1){}
/* Operation */
void MainWindow::on_SearchButton_clicked()
{
    SendToTerminal("Seacrhing Comports...", true, 0);

    emit signal_ComportSearch();


}

void MainWindow::on_comboBox_textActivated(const QString &arg1)
{
    // ComPortOpen(arg1, std::stof(GLB_ui->lineEdit_17->text().toStdString()));
}
void MainWindow::on_pushButton_13_clicked()
{
    QString directory =
        QFileDialog::getExistingDirectory(nullptr, "Выберите папку", "",
                                          QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks); // Опции диалога
    GLB_ui->lineEdit_20->setText(directory);
}
//////* Angle Mode *//////
void MainWindow::on_lineEdit_23_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_22_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_21_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_24_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_25_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_28_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_27_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_26_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_29_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_30_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_33_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_35_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_34_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_31_textEdited(const QString &arg1){}
void MainWindow::on_lineEdit_32_textEdited(const QString &arg1){}
void MainWindow::on_checkBox_11_toggled(bool checked)
{
    if(checked) SendToTerminal("Thread #1 enable.", true, 0);
    else if(!checked) SendToTerminal("Thread #1 disable.", true, 0);
}
// MOTOR 0
void MainWindow::on_pushButton_clicked()
{
    GLB_ui->pushButton_2->setChecked(false);
    GLB_ui->pushButton_21->setChecked(false);
    GLB_ui->pushButton_27->setChecked(false);
}
void MainWindow::on_pushButton_2_clicked()
{
    GLB_ui->pushButton->setChecked(false);
    GLB_ui->pushButton_21->setChecked(false);
    GLB_ui->pushButton_27->setChecked(false);
}
void MainWindow::on_pushButton_21_clicked()
{
    GLB_ui->pushButton->setChecked(false);
    GLB_ui->pushButton_2->setChecked(false);
    GLB_ui->pushButton_27->setChecked(false);
}
void MainWindow::on_pushButton_27_clicked()
{
    GLB_ui->pushButton->setChecked(false);
    GLB_ui->pushButton_2->setChecked(false);
    GLB_ui->pushButton_21->setChecked(false);
}
void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    GLB_ui->lineEdit->setText(QString::number(value));
}
void MainWindow::on_lineEdit_6_textEdited(const QString &arg1)
{
    if(arg1 > 0)
    {
        std::string str = arg1.toStdString();
    }
}
// MOTOR 1
void MainWindow::on_pushButton_3_clicked()
{
    GLB_ui->pushButton_23->setChecked(false);
    GLB_ui->pushButton_4->setChecked(false);
    GLB_ui->pushButton_28->setChecked(false);
}
void MainWindow::on_pushButton_4_clicked()
{
    GLB_ui->pushButton_23->setChecked(false);
    GLB_ui->pushButton_3->setChecked(false);
    GLB_ui->pushButton_28->setChecked(false);
}
void MainWindow::on_pushButton_23_clicked()
{

    GLB_ui->pushButton_3->setChecked(false);
    GLB_ui->pushButton_4->setChecked(false);
    GLB_ui->pushButton_28->setChecked(false);
}
void MainWindow::on_pushButton_28_clicked()
{
    GLB_ui->pushButton_23->setChecked(false);
    GLB_ui->pushButton_4->setChecked(false);
    GLB_ui->pushButton_3->setChecked(false);
}
void MainWindow::on_horizontalSlider_2_valueChanged(int value)
{
    GLB_ui->lineEdit_2->setText(QString::number(value));
}
// MOTOR 2
void MainWindow::on_pushButton_5_clicked()
{
    GLB_ui->pushButton_6->setChecked(false);
    GLB_ui->pushButton_19->setChecked(false);
    GLB_ui->pushButton_26->setChecked(false);
}
void MainWindow::on_pushButton_6_clicked()
{
    GLB_ui->pushButton_5->setChecked(false);
    GLB_ui->pushButton_19->setChecked(false);
    GLB_ui->pushButton_26->setChecked(false);
}
void MainWindow::on_pushButton_19_clicked()
{
    GLB_ui->pushButton_5->setChecked(false);
    GLB_ui->pushButton_6->setChecked(false);
    GLB_ui->pushButton_26->setChecked(false);
}
void MainWindow::on_pushButton_26_clicked()
{
    GLB_ui->pushButton_5->setChecked(false);
    GLB_ui->pushButton_6->setChecked(false);
    GLB_ui->pushButton_19->setChecked(false);
}
void MainWindow::on_horizontalSlider_3_valueChanged(int value)
{
    GLB_ui->lineEdit_3->setText(QString::number(value));
}
// MOTOR 3
void MainWindow::on_pushButton_7_clicked()
{
    GLB_ui->pushButton_22->setChecked(false);
    GLB_ui->pushButton_24->setChecked(false);
    GLB_ui->pushButton_8->setChecked(false);
}
void MainWindow::on_pushButton_8_clicked()
{
    GLB_ui->pushButton_22->setChecked(false);
    GLB_ui->pushButton_24->setChecked(false);
    GLB_ui->pushButton_7->setChecked(false);
}
void MainWindow::on_pushButton_22_clicked()
{

    GLB_ui->pushButton_8->setChecked(false);
    GLB_ui->pushButton_24->setChecked(false);
    GLB_ui->pushButton_7->setChecked(false);
}
void MainWindow::on_pushButton_24_clicked()
{
    GLB_ui->pushButton_8->setChecked(false);
    GLB_ui->pushButton_22->setChecked(false);
    GLB_ui->pushButton_7->setChecked(false);
}
void MainWindow::on_horizontalSlider_4_valueChanged(int value)
{
    GLB_ui->lineEdit_4->setText(QString::number(value));
}
// MOTOR 4
void MainWindow::on_pushButton_9_clicked()
{
    GLB_ui->pushButton_10->setChecked(false);
    GLB_ui->pushButton_20->setChecked(false);
    GLB_ui->pushButton_25->setChecked(false);
}
void MainWindow::on_pushButton_10_clicked()
{
    GLB_ui->pushButton_9->setChecked(false);
    GLB_ui->pushButton_20->setChecked(false);
    GLB_ui->pushButton_25->setChecked(false);
}
void MainWindow::on_pushButton_20_clicked()
{

    GLB_ui->pushButton_9->setChecked(false);
    GLB_ui->pushButton_10->setChecked(false);
    GLB_ui->pushButton_25->setChecked(false);
}
void MainWindow::on_pushButton_25_clicked()
{

    GLB_ui->pushButton_9->setChecked(false);
    GLB_ui->pushButton_10->setChecked(false);
    GLB_ui->pushButton_20->setChecked(false);
}
void MainWindow::on_horizontalSlider_5_valueChanged(int value)
{
    GLB_ui->lineEdit_5->setText(QString::number(value));
}
// MOTOR 5
void MainWindow::on_pushButton_30_clicked()
{
    GLB_ui->pushButton_31->setChecked(false);
    GLB_ui->pushButton_32->setChecked(false);
    GLB_ui->pushButton_33->setChecked(false);
}
void MainWindow::on_pushButton_31_clicked()
{
    GLB_ui->pushButton_30->setChecked(false);
    GLB_ui->pushButton_32->setChecked(false);
    GLB_ui->pushButton_33->setChecked(false);
}
void MainWindow::on_pushButton_32_clicked()
{
    GLB_ui->pushButton_30->setChecked(false);
    GLB_ui->pushButton_31->setChecked(false);
    GLB_ui->pushButton_33->setChecked(false);
}
void MainWindow::on_pushButton_33_clicked()
{
    GLB_ui->pushButton_30->setChecked(false);
    GLB_ui->pushButton_31->setChecked(false);
    GLB_ui->pushButton_32->setChecked(false);
}
void MainWindow::on_horizontalSlider_6_valueChanged(int value)
{
    GLB_ui->lineEdit_36->setText(QString::number(value));
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
/*************************************PWM CONTROL*************************************/
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// Configurate
void MainWindow::on_pushButton_29_clicked()
{
    /*Получение значений*/


    /*Установка значений в команду*/
    std::vector<std::map<uint8_t, std::vector<uint8_t>>> tempMap;
    // for (auto &mot : Motor) mot.c
    for (auto &mot : Motor)
    {
        std::map<uint8_t, std::vector<uint8_t>> t_mp;

        /*Проверка стороны платы на которую отправляем.*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getSidePlate());
            t_mp[PR_PROTOCOL_CODE_SIDEPLATE] = temp;
        }
        /*Проверка на режим работы ШИМ, УГЛЫ, СТАТУС...*/
        // mot.getWorkMode()
        {
            std::vector<uint8_t> temp;
            if (GLB_ui->tab_1->isActiveWindow())                /*Режим ШИМ*/
                temp.push_back(PR_VAL_WORKMODE_MANUAL);
            else if (GLB_ui->tab_2->isActiveWindow())           /*Режим углов*/
                temp.push_back(PR_VAL_WORKMODE_ANGLE);
            t_mp[PR_PROTOCOL_CODE_WORKMODE] = temp;
        }
        /******************/
        /*Проверка на команду конфиуграции*/
        /*
         *
         *
         *
         *
         */
        /*Устанавливаем номер двигателя*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getID());
            t_mp[PR_PROTOCOL_CODE_NUMMOTOR] = temp;
        }
        /*Устанавливаем направление*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getDirection());
            t_mp[PR_PROTOCOL_CODE_DIRECTION] = temp;
        }
        /*Устанавливаем PWM*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getPWM());
            t_mp[PR_PROTOCOL_CODE_PWM] = temp;
        }
        /*Устанавливаем WorkTime*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getWorkTime() & 0x00FF);
            temp.push_back((mot.getWorkTime() & 0xFF00) >> 8);
            t_mp[PR_PROTOCOL_CODE_TIME] = temp;
        }
        /*Устанавливаем WorkDelay*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getWorkDelay() & 0x00FF);
            temp.push_back((mot.getWorkDelay()& 0xFF00) >> 8);
            t_mp[PR_PROTOCOL_CODE_DELAY] = temp;
        }
        /*Устанавливаем ADC*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getADC_State());
            t_mp[PR_PROTOCOL_CODE_ADC] = temp;
        }
        /*Устанавливаем FeedBack*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getFeedBack());
            t_mp[PR_PROTOCOL_CODE_FEEDBACK] = temp;
        }
        /*Устанавливаем Running*/
        // {
        //     std::vector<uint8_t> temp;
        //     temp.push_back(mot.getRunning());
        //     t_mp[PR_PROTOCOL_CODE_RUNNING] = temp;
        // }


        if (((mot.getPWM() > 0) && (mot.getWorkTime() > 0) && (mot.getDirection() != MoveNone)) || (mot.getAngle() > 0) || (mot.getSpeed() > 0))
        {
            tempMap.push_back(t_mp);
        }
        else
        {
            // qInfo() << "Please select parameters to move";
        }
    }
    Command.setCommand(tempMap);

    std::vector<uint8_t> temp_data = Command.existCollectData();



    // Выводим данные в формате hex

    qInfo() << "============ SEND DATA ============";
    QByteArray byteArray(reinterpret_cast<const char*>(temp_data.data()), temp_data.size());
    QString hexString;
    for (uint8_t byte : temp_data) {
        hexString += QString("%1 ").arg(byte, 2, 16, QChar('0')).toUpper();
    }
    hexString = hexString.trimmed();
    qInfo() << hexString;
    // qInfo() << temp_data;
    qInfo() << "============ ============ ============";
    emit signal_ComportWrite(temp_data);
    Command.clear();
}
// Start insturction
void MainWindow::on_pushButton_34_clicked()
{
    bool flagFeedBack = false;
    std::vector<std::map<uint8_t, std::vector<uint8_t>>> tempMap;
    for (auto &mot : Motor)
    {
        std::map<uint8_t, std::vector<uint8_t>> t_mp;

        /*Проверка стороны платы на которую отправляем.*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getSidePlate());
            t_mp[PR_PROTOCOL_CODE_SIDEPLATE] = temp;
        }
        /*Проверка на режим работы ШИМ, УГЛЫ, СТАТУС...*/
        // mot.getWorkMode()
        {
            std::vector<uint8_t> temp;
            if (GLB_ui->tab_1->isActiveWindow())                /*Режим ШИМ*/
                temp.push_back(PR_VAL_WORKMODE_MANUAL);
            else if (GLB_ui->tab_2->isActiveWindow())           /*Режим углов*/
                temp.push_back(PR_VAL_WORKMODE_ANGLE);
            t_mp[PR_PROTOCOL_CODE_WORKMODE] = temp;
        }
        /*Устанавливаем номер двигателя*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getID());
            t_mp[PR_PROTOCOL_CODE_NUMMOTOR] = temp;
        }
        /*Проверяем установку ADC*/
        {
            if (mot.getADC_State() == ADC_Enable) flagFeedBack = true;
        }
        /*Проверяем установку FeedBack*/
        {
            // if (mot.getFeedBack() == ADC_Enable) flagFeedBack = true;
        }
        /*Устанавливаем Running*/
        {
            std::vector<uint8_t> temp;
            temp.push_back(mot.getRunning());
            t_mp[PR_PROTOCOL_CODE_RUNNING] = temp;
        }
        if (((mot.getPWM() > 0) && (mot.getWorkTime() > 0)) || (mot.getAngle() > 0) || (mot.getSpeed() > 0))
            tempMap.push_back(t_mp);
    }
    Command.setCommand(tempMap);

    std::vector<uint8_t> temp_data = Command.existCollectData();

    qInfo() << "============ SEND DATA ============";
    qInfo() << temp_data;
    qInfo() << "============ ============ ============";
    emit signal_ComportWrite(temp_data);
    if (flagFeedBack) emit signal_ComportStartRead();
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
/*************************************ANGLE CONTROL*************************************/
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
/*Configuration*/
void MainWindow::on_pushButton_36_clicked()
{

}
/*Start Instuction 2*/
void MainWindow::on_pushButton_35_clicked()
{

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

}

// Auto-Detect CheckBox
void MainWindow::on_checkBox_10_toggled(bool checked)
{

}


void MainWindow::on_checkBox_29_clicked(bool checked)
{
    if (checked)
    {
        GLB_ui->lineEdit_18->setEnabled(true);
    }
    else
    {
        GLB_ui->lineEdit_18->setEnabled(false);
    }
}

void MainWindow::on_checkBox_31_clicked(bool checked)
{
    if (checked)
    {
        GLB_ui->lineEdit_50->setEnabled(true);
    }
    else
    {
        GLB_ui->lineEdit_50->setEnabled(false);
    }
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
   emit signal_ComportConnect(GLB_ui->comboBox->itemText(index), GLB_ui->lineEdit_17->text().toInt());
}
