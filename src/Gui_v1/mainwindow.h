#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTime>
#include<string>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QWidget>
#include <QListWidget>
#include <QGraphicsScene>
#include <QGraphicsView>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

private:
    Ui::MainWindow *ui;


    //Layouts:
    QVBoxLayout mainVertical;
    QHBoxLayout controlsMainLayout;
    QGridLayout ctrlBtnsLayout;
    QVBoxLayout logMainLayout;
    QHBoxLayout logHeaderLayout;
    QVBoxLayout currentOrderLayout;

    //Buttons:
    QPushButton onBtn;
    QPushButton offBtn;
    QPushButton startBtn;
    QPushButton stopBtn;
    QPushButton resetBtn;
    QPushButton clearBtn;

    QPushButton clearLogBtn;

    //Labels:
    QLabel ctrlsLabel;
    QLabel logTitle;
    QLabel currentOrderTitle;

    //State diagram:
    QGraphicsScene stateDiagramScene;
    QGraphicsView stateDiagramView;

    //List views:
    QListWidget logDisplay;
    QListWidget currentOrder;

    //States:
    QLabel stateExecuting;
    QLabel stateCompleting;
    QLabel stateComplete;
    QLabel stateSuspended;
    QLabel stateSuspending;
    QLabel stateUnsuspending;
    QLabel stateHolding;
    QLabel stateHeld;
    QLabel stateUnheld;
    QLabel stateStarting;
    QLabel stateIdle;
    QLabel stateReset;
    QLabel stateStopped;
    QLabel stateStopping;
    QLabel stateClearing;
    QLabel stateAborted;
    QLabel stateAborting;
};

#endif // MAINWINDOW_H
