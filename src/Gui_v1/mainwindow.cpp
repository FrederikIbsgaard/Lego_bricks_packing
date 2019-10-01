#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Setup GUI appearance:
    ui->centralWidget->setLayout(&mainVertical);

    //Setup and add state diagram:
    //Setup texts:
    stateExecuting.setText("Executing");
    stateCompleting.setText("Completing");
    stateComplete.setText("Complete");
    stateSuspended.setText("Suspended");
    stateSuspending.setText("Suspending");
    stateUnsuspending.setText("Unsuspending");
    stateHolding.setText("Holding");
    stateHeld.setText("Held");
    stateUnheld.setText("Unheld");
    stateStarting.setText("Starting");
    stateIdle.setText("Idle");
    stateReset.setText("Reset");
    stateStopped.setText("Stopped");
    stateStopping.setText("Stopping");
    stateClearing.setText("Clearing");
    stateAborted.setText("Arborted");
    stateAborting.setText("Arborting");

    //Add to scene:
    stateDiagramScene.addWidget(&stateExecuting);
    stateDiagramScene.addWidget(&stateCompleting);
    stateDiagramScene.addWidget(&stateComplete);
    stateDiagramScene.addWidget(&stateSuspending);
    stateDiagramScene.addWidget(&stateSuspended);
    stateDiagramScene.addWidget(&stateUnsuspending);
    stateDiagramScene.addWidget(&stateHolding);
    stateDiagramScene.addWidget(&stateHeld);
    stateDiagramScene.addWidget(&stateUnheld);
    stateDiagramScene.addWidget(&stateStarting);
    stateDiagramScene.addWidget(&stateIdle);
    stateDiagramScene.addWidget(&stateReset);
    stateDiagramScene.addWidget(&stateStopped);
    stateDiagramScene.addWidget(&stateStopping);
    stateDiagramScene.addWidget(&stateClearing);
    stateDiagramScene.addWidget(&stateAborted);
    stateDiagramScene.addWidget(&stateAborting);

    //Position things correctly:
    QList<QGraphicsItem*> allStates = stateDiagramScene.items();

    for(int i = 0; i < allStates.size(); i++)
    {
        //Move things to correct places...
    }

    mainVertical.addWidget(&stateDiagramView);

    //Add "Controls:
    ctrlsLabel.setText("Controls");
    mainVertical.addWidget(&ctrlsLabel);
    mainVertical.addLayout(&controlsMainLayout);
    controlsMainLayout.addLayout(&ctrlBtnsLayout);

    //Log:
    controlsMainLayout.addLayout(&logMainLayout);
    logMainLayout.addLayout(&logHeaderLayout);
    logTitle.setText("Log");
    logHeaderLayout.addWidget(&logTitle);
    clearLogBtn.setText("Clear log");
    logHeaderLayout.addWidget(&clearLogBtn);
    logMainLayout.addWidget(&logDisplay);

    //Current order view:
    controlsMainLayout.addLayout(&currentOrderLayout);
    currentOrderTitle.setText("Current order");
    currentOrderLayout.addWidget(&currentOrderTitle);
    currentOrderLayout.addWidget(&currentOrder);


    //Add control buttons:
    ctrlBtnsLayout.addWidget(&onBtn, 0, 0);
    ctrlBtnsLayout.addWidget(&offBtn, 0, 1);
    ctrlBtnsLayout.addWidget(&startBtn, 1, 0);
    ctrlBtnsLayout.addWidget(&stopBtn, 1, 1);
    ctrlBtnsLayout.addWidget(&resetBtn, 2, 0);
    ctrlBtnsLayout.addWidget(&clearBtn, 2, 1);

    //Add text to buttons:
    onBtn.setText("ON");
    offBtn.setText("OFF");
    startBtn.setText("Start");
    stopBtn.setText("Stop");
    resetBtn.setText("Reset");
    clearBtn.setText("Clear");


    //View state diagram:
    stateDiagramView.setScene(&stateDiagramScene);
    stateDiagramView.show();
}

MainWindow::~MainWindow()
{
    delete ui;
}
/*
void MainWindow::on_clearBtn_clicked()
{
    ui->logDisplay->clear();
}

void MainWindow::on_startBtn_clicked()
{
    addLogItem("Started");
    ui->stateExecuting->setStyleSheet("QLabel { background-color : #5d9ed4; color : #ffffff;}");
}

void MainWindow::on_stopBtn_clicked()
{
    addLogItem("Stopped");
    ui->stateExecuting->setStyleSheet("QLabel { background-color : #b3b3b3; color : #ffffff; }");
}

void MainWindow::on_pauseBtn_clicked()
{
    addLogItem("Paused");
}

void MainWindow::on_resetBtn_clicked()
{
    addLogItem("Reset");
}

void MainWindow::addLogItem(std::string msg)
{
    QTime currentTime;
    currentTime.start(); //Get system time

    QString toLog = currentTime.toString("hh:mm:ss.zzz") + ": " + QString(msg.c_str());
    ui->logDisplay->addItem(toLog);
    ui->logDisplay->scrollToBottom();
}

void MainWindow::on_onBtn_clicked()
{
    bool canStart = true; //false; //NOTE: must be set to false when not debugging!!!!!

    //TODO: Call ros service to ask main task/node if we can start....

    if(canStart)
    {
        //Change apperance of start/stop buttons:
        ui->onBtn->setStyleSheet("background-color: rgb(93, 158, 212); color: white; border-style: solid; border-width:0px; border-top-left-radius: 20px; border-bottom-left-radius: 20px; font-size: 35px;");
        ui->offBtn->setStyleSheet("background-color: rgb(255, 255, 255); color: rgb(179,179,179); border-style: solid; border-width:0px; border-top-right-radius: 20px; border-bottom-right-radius: 20px; font-size: 35px;");
    }
}

void MainWindow::on_offBtn_clicked()
{
    //TODO: Notify the rest of the system...

    //Change apperance of start/stop buttons:
    ui->onBtn->setStyleSheet("background-color: rgb(255, 255, 255); color: rgb(179,179,179); border-style: solid; border-width:0px; border-top-left-radius: 20px; border-bottom-left-radius: 20px; font-size: 35px;");
    ui->offBtn->setStyleSheet("background-color: rgb(93, 158, 212); color: white; border-style: solid; border-width:0px; border-top-right-radius: 20px; border-bottom-right-radius: 20px; font-size: 35px;");
}*/
