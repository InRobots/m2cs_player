#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui_(new Ui::MainWindow)
{
    ros_thread_ = new ROSThread(this, &mutex);
    ui_->setupUi(this);
    ros_thread_->start();

    slider_checker_ = false;
    play_flag_ = false;
    pause_flag_ = false;
    loop_flag_ = false;
    stop_skip_flag_ = true;

    connect(ros_thread_, SIGNAL(StampShow(quint64)), this, SLOT(SetStamp(quint64)));
    connect(ros_thread_, SIGNAL(StartSignal()), this, SLOT(Play()));

    connect(ui_->quitButton, SIGNAL(pressed()), this, SLOT(TryClose()));
    connect(ui_->pushButton, SIGNAL(pressed()), this, SLOT(FilePathSet()));
    connect(ui_->pushButton_2, SIGNAL(pressed()), this, SLOT(Play()));
    connect(ui_->pushButton_3, SIGNAL(pressed()), this, SLOT(Pause()));

    connect(ui_->doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(PlaySpeedChange(double)));
    ui_->doubleSpinBox->setRange(0.01, 20.0);
    ui_->doubleSpinBox->setValue(1.0);
    ui_->doubleSpinBox->setSingleStep(0.01);
    connect(ui_->checkBox, SIGNAL(stateChanged(int)), this, SLOT(LoopFlagChange(int)));
    if (loop_flag_ == true)
    {
        ui_->checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox->setCheckState(Qt::Unchecked);
    }
    connect(ui_->checkBox_2, SIGNAL(stateChanged(int)), this, SLOT(StopSkipFlagChange(int)));
    if (stop_skip_flag_ == true)
    {
        ui_->checkBox_2->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox_2->setCheckState(Qt::Unchecked);
    }
    connect(ui_->checkBox_3, SIGNAL(stateChanged(int)), this, SLOT(AutoStartFlagChange(int)));
    if (ros_thread_->auto_start_flag_ == true)
    {
        ui_->checkBox_3->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox_3->setCheckState(Qt::Unchecked);
    }

    connect(ui_->checkBox_4, SIGNAL(stateChanged(int)), this, SLOT(TFFlagChange(int)));
    if (ros_thread_->tf_flag_ == true)
    {
        ui_->checkBox_4->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox_4->setCheckState(Qt::Unchecked);
    }

    connect(ui_->checkBox_5, SIGNAL(stateChanged(int)), this, SLOT(NavFlagChange(int)));
    if (ros_thread_->nav_flag_ == true)
    {
        ui_->checkBox_5->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox_5->setCheckState(Qt::Unchecked);
    }

    connect(ui_->checkBox_6, SIGNAL(stateChanged(int)), this, SLOT(OusterFlagChange(int)));
    if (ros_thread_->ouster_flag_ == true)
    {
        ui_->checkBox_6->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox_6->setCheckState(Qt::Unchecked);
    }

    connect(ui_->checkBox_7, SIGNAL(stateChanged(int)), this, SLOT(LeftLivoxFlagChange(int)));
    if (ros_thread_->livox_left_flag_ == true)
    {
        ui_->checkBox_7->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox_7->setCheckState(Qt::Unchecked);
    }

    connect(ui_->checkBox_8, SIGNAL(stateChanged(int)), this, SLOT(RightLivoxFlagChange(int)));
    if (ros_thread_->livox_right_flag_ == true)
    {
        ui_->checkBox_8->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox_8->setCheckState(Qt::Unchecked);
    }

    connect(ui_->checkBox_9, SIGNAL(stateChanged(int)), this, SLOT(CameraFlagChange(int)));
    if (ros_thread_->camera_flag_ == true)
    {
        ui_->checkBox_9->setCheckState(Qt::Checked);
    }
    else
    {
        ui_->checkBox_9->setCheckState(Qt::Unchecked);
    }

    connect(ui_->horizontalSlider, SIGNAL(sliderPressed()), this, SLOT(SliderPressed()));
    connect(ui_->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChange(int)));
    connect(ui_->horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(SliderValueApply()));

    ui_->horizontalSlider->setRange(0, 10000);
    ui_->horizontalSlider->setValue(0);
    slider_value_ = 0;
}

MainWindow::~MainWindow()
{
    emit setThreadFinished(true); // Tell the thread to finish
    delete ui_;
    ros_thread_->quit();
    if (!ros_thread_->wait(500)) // Wait until it actually has terminated (max. 3 sec)
    {
        ros_thread_->terminate(); // Thread didn't exit in time, probably deadlocked, terminate it!
        ros_thread_->wait();      // We have to wait again here!
    }
}

void MainWindow::RosInit(ros::NodeHandle &nh)
{
    ros_thread_->ros_initialize(nh);
}

void MainWindow::TryClose()
{
    close();
}

void MainWindow::FilePathSet()
{
    play_flag_ = false;
    ros_thread_->play_flag_ = false;
    this->ui_->pushButton_2->setText(QString::fromStdString("Play"));

    pause_flag_ = false;
    ros_thread_->pause_flag_ = false;
    this->ui_->pushButton_3->setText(QString::fromStdString("Pause"));

    QFileDialog dialog;
    this->ui_->label->setText("Data is beging loaded.....");
    data_folder_path_ = dialog.getExistingDirectory();
    ros_thread_->data_folder_path_ = data_folder_path_.toUtf8().constData();

    ros_thread_->Ready();

    this->ui_->label->setText(data_folder_path_);
}

void MainWindow::SetStamp(quint64 stamp)
{
    this->ui_->label_2->setText(QString::number(stamp));

    // set slide bar
    if (slider_checker_ == false)
    {
        ui_->horizontalSlider->setValue(static_cast<int>(static_cast<float>(stamp - ros_thread_->initial_data_stamp_) / static_cast<float>(ros_thread_->last_data_stamp_ - ros_thread_->initial_data_stamp_) * 10000));
    }
}

void MainWindow::Play()
{
    if (ros_thread_->play_flag_ == false)
    {
        play_flag_ = true;
        ros_thread_->play_flag_ = true;
        this->ui_->pushButton_2->setText(QString::fromStdString("End"));

        pause_flag_ = false;
        ros_thread_->pause_flag_ = false;
        this->ui_->pushButton_3->setText(QString::fromStdString("Pause"));
    }
    else
    {
        play_flag_ = false;
        ros_thread_->play_flag_ = false;
        this->ui_->pushButton_2->setText(QString::fromStdString("Play"));
    }
}

void MainWindow::Pause()
{
    if (pause_flag_ == false)
    {
        pause_flag_ = true;
        ros_thread_->pause_flag_ = true;
        this->ui_->pushButton_3->setText(QString::fromStdString("Resume"));
    }
    else
    {
        pause_flag_ = false;
        ros_thread_->pause_flag_ = false;
        this->ui_->pushButton_3->setText(QString::fromStdString("Pause"));
    }
}

void MainWindow::PlaySpeedChange(double value)
{
    ros_thread_->play_rate_ = value;
}

void MainWindow::LoopFlagChange(int value)
{
    if (value == 2)
    {
        loop_flag_ = true;
        ros_thread_->loop_flag_ = true;
    }
    else if (value == 0)
    {
        loop_flag_ = false;
        ros_thread_->loop_flag_ = false;
    }
}

void MainWindow::StopSkipFlagChange(int value)
{
    if (value == 2)
    {
        stop_skip_flag_ = true;
        ros_thread_->stop_skip_flag_ = true;
    }
    else if (value == 0)
    {
        stop_skip_flag_ = false;
        ros_thread_->stop_skip_flag_ = false;
    }
}

void MainWindow::AutoStartFlagChange(int value)
{
    if (value == 2)
    {
        ros_thread_->auto_start_flag_ = true;
    }
    else if (value == 0)
    {
        ros_thread_->auto_start_flag_ = false;
    }
}

void MainWindow::TFFlagChange(int value)
{
    if (value == 2)
    {
        ros_thread_->tf_flag_ = true;
    }
    else if (value == 0)
    {
        ros_thread_->tf_flag_ = false;
    }
    // std::cout << ros_thread_->tf_flag_ << std::endl;
}

void MainWindow::NavFlagChange(int value)
{
    if (value == 2)
    {
        ros_thread_->nav_flag_ = true;
    }
    else if (value == 0)
    {
        ros_thread_->nav_flag_ = false;
    }
    // std::cout << ros_thread_->nav_flag_ << std::endl;
}

void MainWindow::OusterFlagChange(int value)
{
    if (value == 2)
    {
        ros_thread_->ouster_flag_ = true;
    }
    else if (value == 0)
    {
        ros_thread_->ouster_flag_ = false;
    }
    // std::cout << ros_thread_->ouster_flag_ << std::endl;
}

void MainWindow::LeftLivoxFlagChange(int value)
{
    if (value == 2)
    {
        ros_thread_->livox_left_flag_ = true;
    }
    else if (value == 0)
    {
        ros_thread_->livox_left_flag_ = false;
    }
    // std::cout << ros_thread_->livox_left_flag_ << std::endl;
}

void MainWindow::RightLivoxFlagChange(int value)
{
    if (value == 2)
    {
        ros_thread_->livox_right_flag_ = true;
    }
    else if (value == 0)
    {
        ros_thread_->livox_right_flag_ = false;
    }
    // std::cout << ros_thread_->livox_right_flag_ << std::endl;
}

void MainWindow::CameraFlagChange(int value)
{
    if (value == 2)
    {
        ros_thread_->camera_flag_ = true;
    }
    else if (value == 0)
    {
        ros_thread_->camera_flag_ = false;
    }
    // std::cout << ros_thread_->camera_flag_ << std::endl;
}

void MainWindow::SliderValueChange(int value)
{
    slider_value_ = value;
}

void MainWindow::SliderPressed()
{
    slider_checker_ = true;
}

void MainWindow::SliderValueApply()
{
    ros_thread_->ResetProcessStamp(slider_value_);
    slider_checker_ = false;
}
