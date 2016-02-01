#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include <QMessageBox>
//#include "our_basic/Joint.h"
#include <math.h>
#include <config.h>

std_msgs::Float32MultiArray joints;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_jointContorlHandler(new JointControlCanBus()),
    m_step(0.1),
    m_speed(50.0)
{
    ui->setupUi(this);

    joints.data.resize(ARM_DOF);

    if(m_jointContorlHandler->JointControlInit(DEFAULT_NODE))
    {
        joints.data[5] = m_jointContorlHandler->readPosJ(5, MODEL_TYPE_J60);
        ui->lb_joint6->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));

//        for(int i=0; i < ARM_DOF/2; ++i)
//        {
//            joints.data[i] = m_jointContorlHandler->readPosJ(i, MODEL_TYPE_J60);
//            ui->lb_joint5->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));
//        }

//        for(int i=ARM_DOF/2; i < ARM_DOF; ++i)
//        {
//            joints.data[5] = m_jointContorlHandler->readPosJ(0x05, MODEL_TYPE_J60);
//            ui->lb_joint5->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));
//        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
    delete m_jointContorlHandler;
}

void MainWindow::on_pbn_joint1Left_pressed()
{
    joints.data[0] = joints.data[0] - m_step * m_speed/100 < -M_PI ?
                     joints.data[0] :
                     joints.data[0] - m_step  * m_speed/100;

    ui->lb_joint1->setText(QString::number(joints.data[0]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint1Right_pressed()
{
    joints.data[0] = joints.data[0] + m_step * m_speed/100 > M_PI ?
                     joints.data[0] :
                     joints.data[0] + m_step * m_speed/100;

    ui->lb_joint1->setText(QString::number(joints.data[0]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint2Left_pressed()
{
    joints.data[1] = joints.data[1] - m_step * m_speed/100 < -M_PI ?
                     joints.data[1] :
                     joints.data[1] - m_step * m_speed/100;

    ui->lb_joint2->setText(QString::number(joints.data[1]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint2Right_pressed()
{
    joints.data[1] = joints.data[1] + m_step * m_speed/100 > M_PI ?
                     joints.data[1] :
                     joints.data[1] + m_step * m_speed/100;

    ui->lb_joint2->setText(QString::number(joints.data[1]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint3Left_pressed()
{
    joints.data[2] = joints.data[2] - m_step * m_speed/100 < -M_PI ?
                     joints.data[2] :
                     joints.data[2] - m_step * m_speed/100;
    ui->lb_joint3->setText(QString::number(joints.data[2]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint3Right_pressed()
{
    joints.data[2] = joints.data[2] + m_step * m_speed/100 > M_PI ?
                     joints.data[2] :
                     joints.data[2] + m_step * m_speed/100;

    ui->lb_joint3->setText(QString::number(joints.data[2]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint4Left_pressed()
{
    joints.data[3] = joints.data[3] - m_step * m_speed/100 < -M_PI ?
                     joints.data[3] :
                     joints.data[3] - m_step * m_speed/100;

    ui->lb_joint4->setText(QString::number(joints.data[3]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint4Right_pressed()
{
    joints.data[3] = joints.data[3] + m_step * m_speed/100 > M_PI ?
                     joints.data[3] :
                     joints.data[3] + m_step * m_speed/100;

    ui->lb_joint4->setText(QString::number(joints.data[3]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint5Left_pressed()
{
    joints.data[4] = joints.data[4] - m_step * m_speed/100 < -M_PI ?
                     joints.data[4] :
                     joints.data[4] - m_step * m_speed/100;

    ui->lb_joint5->setText(QString::number(joints.data[4]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint5Right_pressed()
{
    joints.data[4] = joints.data[4] + m_step * m_speed/100 > M_PI ?
                     joints.data[4] :
                     joints.data[4] + m_step * m_speed/100;

    ui->lb_joint5->setText(QString::number(joints.data[4]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint6Left_pressed()
{
    joints.data[5] = joints.data[5] - m_step * m_speed/100 < -M_PI ?
                     joints.data[5] :
                     joints.data[5] - m_step * m_speed/100;

    ui->lb_joint6->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint6Right_pressed()
{
    joints.data[5] = joints.data[5] + m_step * m_speed/100 > M_PI ?
                     joints.data[5] :
                     joints.data[5] + m_step * m_speed/100;

    ui->lb_joint6->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));
}
