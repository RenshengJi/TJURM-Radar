/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/displayer_qt5/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace displayer_qt5 {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&qnode,SIGNAL(loggingCamera()),this,SLOT(updateLogcamera()));
    QObject::connect(&qnode,SIGNAL(loggingCameraSecondWindow()),this,SLOT(updateLogcameraSecondWindow()));
    QObject::connect(&qnode,SIGNAL(loggingCameraCalibrateMainWindow()),this,SLOT(updateLogcameraCalibrateMainWindow()));
    QObject::connect(&qnode,SIGNAL(loggingCameraCalibrateSecondWindow()),this,SLOT(updateLogcameraCalibrateSecondWindow()));
    // setWindowIcon(QIcon(":/image/images/Icon.ico"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(ui.labelCalibrateCameraMainWindow, SIGNAL(mouseMovePoint(QPoint)), this, SLOT(on_labelCalibrateCameraMainWindow_mouseLocationChanged()));
    QObject::connect(&qnode, SIGNAL(loggingGameStateUpdate()), this, SLOT(updateGameState()));
    QObject::connect(&qnode, SIGNAL(loggingSmallMapUpdate()), this, SLOT(updateSmallMap()));
    if ( !qnode.init() ) {
        showNoMasterMessage();
    }
    initUI();

}

MainWindow::~MainWindow() {
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::initUI()
{
    ui.comboBoxCalibrateCamera->clear();
    QStringList stringList;
    stringList << qnode.sensorFarImgRaw << qnode.sensorCloseImgRaw;
    ui.comboBoxCalibrateCamera->addItems(stringList);

    ui.tabWidget->setCurrentIndex(1);

    ui.labelCalibrateCameraMainWindow->sensor_far_points = qnode.sensor_far_points;
    ui.labelCalibrateCameraMainWindow->sensor_close_points = qnode.sensor_close_points;

    ui.labelCalibrateCameraMainWindow->sensorFarImgRaw = qnode.sensorFarImgRaw;
    ui.labelCalibrateCameraMainWindow->sensorCloseImgRaw = qnode.sensorCloseImgRaw;

    qimage_mutex_.lock();
    ui.label_camera2->setPixmap(QPixmap::fromImage(qnode.imageShowSecondWindow));
    ui.label_camera2->resize(ui.label_camera2->pixmap()->size());
    qimage_mutex_.unlock();

    qimage_mutex_.lock();
    ui.label_camera->setPixmap(QPixmap::fromImage(qnode.image));
    ui.label_camera->resize(ui.label_camera->pixmap()->size());
    qimage_mutex_.unlock();

    qimage_mutex_.lock();
    ui.labelSmallMap->setPixmap(QPixmap::fromImage(qnode.imageSmallMap));
    ui.labelSmallMap->resize(ui.labelSmallMap->pixmap()->size());
    qimage_mutex_.unlock();

    qimage_mutex_.lock();
    ui.labelLogo->setPixmap(QPixmap::fromImage(qnode.imageLogo));
    ui.labelLogo->resize(ui.labelLogo->pixmap()->size());
    qimage_mutex_.unlock();

    

    QFile file_red(":/qss/qss/progressBarRed.qss");
    QFile file_blue(":/qss/qss/progressBarBlue.qss");
    file_red.open(QFile::ReadOnly);
    file_blue.open(QFile::ReadOnly);
    QString style_red = QString::fromLatin1(file_red.readAll());
    QString style_blue = QString::fromLatin1(file_blue.readAll());
    if(qnode.battle_color == std::string("red"))
    {
        ui.progressBarRobotHealth1_1->setStyleSheet(style_red);
        ui.progressBarRobotHealth2_1->setStyleSheet(style_red);
        ui.progressBarRobotHealth3_1->setStyleSheet(style_red);
        ui.progressBarRobotHealth4_1->setStyleSheet(style_red);
        ui.progressBarRobotHealth5_1->setStyleSheet(style_red);
        ui.progressBarRobotHealthBase_1->setStyleSheet(style_red);
        ui.progressBarRobotHealthGuard_1->setStyleSheet(style_red);
        ui.progressBarRobotHealthOutpose_1->setStyleSheet(style_red);

        ui.progressBarRobotHealth1_2->setStyleSheet(style_blue);
        ui.progressBarRobotHealth2_2->setStyleSheet(style_blue);
        ui.progressBarRobotHealth3_2->setStyleSheet(style_blue);
        ui.progressBarRobotHealth4_2->setStyleSheet(style_blue);
        ui.progressBarRobotHealth5_2->setStyleSheet(style_blue);
        ui.progressBarRobotHealthBase_2->setStyleSheet(style_blue);
        ui.progressBarRobotHealthGuard_2->setStyleSheet(style_blue);
        ui.progressBarRobotHealthOutpose_2->setStyleSheet(style_blue);

        ui.labelRobotName1_1->setText("R1");
        ui.labelRobotName2_1->setText("R2");
        ui.labelRobotName3_1->setText("R3");
        ui.labelRobotName4_1->setText("R4");
        ui.labelRobotName5_1->setText("R5");
        ui.labelRobotNameGuard_1->setText("R Guard");
        ui.labelRobotNameOutpose_1->setText("R Outpose");
        ui.labelRobotNameBase_1->setText("R Base");

        ui.labelRobotName1_2->setText("B1");
        ui.labelRobotName2_2->setText("B2");
        ui.labelRobotName3_2->setText("B3");
        ui.labelRobotName4_2->setText("B4");
        ui.labelRobotName5_2->setText("B5");
        ui.labelRobotNameGuard_2->setText("B Guard");
        ui.labelRobotNameOutpose_2->setText("B Outpose");
        ui.labelRobotNameBase_2->setText("B Base");

    }
    else if(qnode.battle_color == std::string("blue"))
    {
        ui.progressBarRobotHealth1_1->setStyleSheet(style_blue);
        ui.progressBarRobotHealth2_1->setStyleSheet(style_blue);
        ui.progressBarRobotHealth3_1->setStyleSheet(style_blue);
        ui.progressBarRobotHealth4_1->setStyleSheet(style_blue);
        ui.progressBarRobotHealth5_1->setStyleSheet(style_blue);
        ui.progressBarRobotHealthBase_1->setStyleSheet(style_blue);
        ui.progressBarRobotHealthGuard_1->setStyleSheet(style_blue);
        ui.progressBarRobotHealthOutpose_1->setStyleSheet(style_blue);

        ui.progressBarRobotHealth1_2->setStyleSheet(style_red);
        ui.progressBarRobotHealth2_2->setStyleSheet(style_red);
        ui.progressBarRobotHealth3_2->setStyleSheet(style_red);
        ui.progressBarRobotHealth4_2->setStyleSheet(style_red);
        ui.progressBarRobotHealth5_2->setStyleSheet(style_red);
        ui.progressBarRobotHealthBase_2->setStyleSheet(style_red);
        ui.progressBarRobotHealthGuard_2->setStyleSheet(style_red);
        ui.progressBarRobotHealthOutpose_2->setStyleSheet(style_red);

        ui.labelRobotName1_1->setText("B1");
        ui.labelRobotName2_1->setText("B2");
        ui.labelRobotName3_1->setText("B3");
        ui.labelRobotName4_1->setText("B4");
        ui.labelRobotName5_1->setText("B5");
        ui.labelRobotNameGuard_1->setText("B Guard");
        ui.labelRobotNameOutpose_1->setText("B Outpose");
        ui.labelRobotNameBase_1->setText("B Base");

        ui.labelRobotName1_2->setText("R1");
        ui.labelRobotName2_2->setText("R2");
        ui.labelRobotName3_2->setText("R3");
        ui.labelRobotName4_2->setText("R4");
        ui.labelRobotName5_2->setText("R5");
        ui.labelRobotNameGuard_2->setText("R Guard");
        ui.labelRobotNameOutpose_2->setText("R Outpose");
        ui.labelRobotNameBase_2->setText("R Base");
    }
    ui.progressBarRobotHealth1_1->setMinimum(0);
    ui.progressBarRobotHealth1_1->setMaximum(0);
    ui.progressBarRobotHealth1_1->setFormat("%v/100");

    ui.progressBarRobotHealth2_1->setMinimum(0);
    ui.progressBarRobotHealth2_1->setMaximum(0);
    ui.progressBarRobotHealth2_1->setFormat("%v/100");

    ui.progressBarRobotHealth3_1->setMinimum(0);
    ui.progressBarRobotHealth3_1->setMaximum(0);
    ui.progressBarRobotHealth3_1->setFormat("%v/100");

    ui.progressBarRobotHealth4_1->setMinimum(0);
    ui.progressBarRobotHealth4_1->setMaximum(0);
    ui.progressBarRobotHealth4_1->setFormat("%v/100");

    ui.progressBarRobotHealth5_1->setMinimum(0);
    ui.progressBarRobotHealth5_1->setMaximum(0);
    ui.progressBarRobotHealth5_1->setFormat("%v/100");

    ui.progressBarRobotHealthOutpose_1->setMinimum(0);
    ui.progressBarRobotHealthOutpose_1->setMaximum(0);
    ui.progressBarRobotHealthOutpose_1->setFormat("%v/100");

    ui.progressBarRobotHealthGuard_1->setMinimum(0);
    ui.progressBarRobotHealthGuard_1->setMaximum(0);
    ui.progressBarRobotHealthGuard_1->setFormat("%v/100");

    ui.progressBarRobotHealthBase_1->setMinimum(0);
    ui.progressBarRobotHealthBase_1->setMaximum(0);
    ui.progressBarRobotHealthBase_1->setFormat("%v/100");

    ui.progressBarRobotHealth1_2->setMinimum(0);
    ui.progressBarRobotHealth1_2->setMaximum(0);
    ui.progressBarRobotHealth1_2->setFormat("%v/100");

    ui.progressBarRobotHealth2_2->setMinimum(0);
    ui.progressBarRobotHealth2_2->setMaximum(0);
    ui.progressBarRobotHealth2_2->setFormat("%v/100");

    ui.progressBarRobotHealth3_2->setMinimum(0);
    ui.progressBarRobotHealth3_2->setMaximum(0);
    ui.progressBarRobotHealth3_2->setFormat("%v/100");

    ui.progressBarRobotHealth4_2->setMinimum(0);
    ui.progressBarRobotHealth4_2->setMaximum(0);
    ui.progressBarRobotHealth4_2->setFormat("%v/100");

    ui.progressBarRobotHealth5_2->setMinimum(0);
    ui.progressBarRobotHealth5_2->setMaximum(0);
    ui.progressBarRobotHealth5_2->setFormat("%v/100");

    ui.progressBarRobotHealthOutpose_2->setMinimum(0);
    ui.progressBarRobotHealthOutpose_2->setMaximum(0);
    ui.progressBarRobotHealthOutpose_2->setFormat("%v/100");

    ui.progressBarRobotHealthGuard_2->setMinimum(0);
    ui.progressBarRobotHealthGuard_2->setMaximum(0);
    ui.progressBarRobotHealthGuard_2->setFormat("%v/100");

    ui.progressBarRobotHealthBase_2->setMinimum(0);
    ui.progressBarRobotHealthBase_2->setMaximum(0);
    ui.progressBarRobotHealthBase_2->setFormat("%v/100");

    fTimer = new QTimer(this);
    fTimer->stop();
    fTimer->setInterval(333);
    connect(fTimer, SIGNAL(timeout()), this, SLOT(on_timer_timeout()));
    fTimer->start();

    ui.labelCalibrateCameraMainWindow->calibrateMainWindowHeight = qnode.calibrateMainWindowHeight;
    ui.labelCalibrateCameraMainWindow->calibrateMainWindowWidth = qnode.calibrateMainWindowWidth;

    ui.label_camera->setMaximumSize(QSize(qnode.showMainWindowWidth, qnode.showMainWindowHeight));
    ui.label_camera->setMinimumSize(QSize(qnode.showMainWindowWidth, qnode.showMainWindowHeight));
    ui.label_camera2->setMaximumSize(QSize(qnode.showSecondWindowWidth, qnode.showSecondWindowHeight));
    ui.label_camera2->setMinimumSize(QSize(qnode.showSecondWindowWidth, qnode.showSecondWindowHeight));
    ui.labelSmallMap->setMaximumSize(QSize(qnode.smallMapWidth, qnode.smallMapHeight));
    ui.labelSmallMap->setMinimumSize(QSize(qnode.smallMapWidth, qnode.smallMapHeight));
    ui.labelLogo->setMaximumSize(QSize(qnode.logoWidth, qnode.logoHeight));
    ui.labelLogo->setMinimumSize(QSize(qnode.logoWidth, qnode.logoHeight));
    ui.labelCalibrateCameraMainWindow->setMaximumSize(QSize(qnode.calibrateMainWindowWidth, qnode.calibrateMainWindowHeight));
    ui.labelCalibrateCameraMainWindow->setMinimumSize(QSize(qnode.calibrateMainWindowWidth, qnode.calibrateMainWindowHeight));
    ui.labelCalibrateCameraSecondWindow->setMaximumSize(QSize(qnode.calibrateSecondWindowWidth, qnode.calibrateSecondWindowHeight));
    ui.labelCalibrateCameraSecondWindow->setMinimumSize(QSize(qnode.calibrateSecondWindowWidth, qnode.calibrateSecondWindowHeight));

    for(size_t i = 0; i < LOG_LIMIT_TIMER_COUNT; i++)
    {
        log_limit_timer[i] = 0;
    }
}

void MainWindow::updateLogcamera()
{
    // displayCamera(qnode.image); // !!!
    displayCamera(qnode.image);
}

void MainWindow::updateLogcameraSecondWindow()
{
    displayCameraSecondWindow(qnode.imageShowSecondWindow);
}

void MainWindow::updateLogcameraCalibrateMainWindow()
{
    displayCameraCalibrateMainWindow(qnode.imageCalibrateMainWindow);
}

void MainWindow::updateLogcameraCalibrateSecondWindow()
{
    cv::Rect r;
    r.width = qnode.calibrateSecondWindowWidth / qnode.calibrateRate;
    r.height = qnode.calibrateSecondWindowHeight / qnode.calibrateRate;
    int halfWidth = (qnode.calibrateSecondWindowWidth * 0.5) / qnode.calibrateRate;
    int halfHeight = (qnode.calibrateSecondWindowHeight * 0.5) / qnode.calibrateRate;
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.x() > qnode.calibrateMainWindowWidth)
    {
        ui.labelCalibrateCameraMainWindow->selectedPoint.setX(qnode.calibrateMainWindowWidth);
    }
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.y() > qnode.calibrateMainWindowHeight)
    {
        ui.labelCalibrateCameraMainWindow->selectedPoint.setY(qnode.calibrateMainWindowHeight);
    }
    cv::Mat m;
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.x() - halfWidth < 0)
    {
        r.x = 0;
    }
    else if((ui.labelCalibrateCameraMainWindow->selectedPoint.x() + halfWidth) > (qnode.calibrateMainWindowWidth))
    {
        r.x = qnode.calibrateMainWindowWidth - qnode.calibrateSecondWindowWidth / qnode.calibrateRate;
    }
    else
    {
        r.x = ui.labelCalibrateCameraMainWindow->selectedPoint.x() - halfWidth;
    }

    if(ui.labelCalibrateCameraMainWindow->selectedPoint.y() - halfHeight < 0)
    {
        r.y = 0;
    }
    else if((ui.labelCalibrateCameraMainWindow->selectedPoint.y() + halfHeight) > (qnode.calibrateMainWindowHeight))
    {
        r.y = qnode.calibrateMainWindowHeight - qnode.calibrateSecondWindowHeight / qnode.calibrateRate;
    }
    else
    {
        r.y = ui.labelCalibrateCameraMainWindow->selectedPoint.y() - halfHeight;
    }
    if (r.x < 0)
    {
        r.x = 0;
    }
    if (r.y < 0)
    {
        r.y = 0;
    }
    if ((r.x + r.width) > qnode.calibrateMainWindowWidth)
    {
        r.width = qnode.calibrateMainWindowWidth - r.x;
    }
    if ((r.y + r.height) > qnode.calibrateMainWindowWidth)
    {
        r.height = qnode.calibrateMainWindowWidth - r.y;
    }
    if(qnode.cameraCelibrating == qnode.sensorFarImgRaw && !qnode.imgSensorFar.empty())
    {
        qnode.imgSensorFar(r).copyTo(m);
    }
    else if(qnode.cameraCelibrating == qnode.sensorCloseImgRaw && !qnode.imgSensorClose.empty())
    {
        qnode.imgSensorClose(r).copyTo(m);
    }
    if(!m.empty())
    {
        cv::resize(m, m, cv::Size(qnode.calibrateSecondWindowWidth, qnode.calibrateSecondWindowHeight));
        cv::line(m, cv::Point(0, (ui.labelCalibrateCameraMainWindow->selectedPoint.y() - r.y) * qnode.calibrateRate), cv::Point(qnode.calibrateSecondWindowWidth, (ui.labelCalibrateCameraMainWindow->selectedPoint.y() - r.y) * qnode.calibrateRate), cv::Scalar(255, 255, 255));
        cv::line(m, cv::Point((ui.labelCalibrateCameraMainWindow->selectedPoint.x() - r.x) * qnode.calibrateRate, 0), cv::Point((ui.labelCalibrateCameraMainWindow->selectedPoint.x() - r.x) * qnode.calibrateRate, qnode.calibrateSecondWindowHeight), cv::Scalar(255, 255, 255));
        if(qnode.cameraCelibrating == qnode.sensorFarImgRaw)
        {
            for(int i = 0; i < 4; i++)
            {
                cv::Point center((ui.labelCalibrateCameraMainWindow->sensor_far_points[i].x() - r.x) * qnode.calibrateRate, (ui.labelCalibrateCameraMainWindow->sensor_far_points[i].y() - r.y) * qnode.calibrateRate);
                cv::circle(m, center, 5 * qnode.calibrateRate, cv::Scalar(255, 255, 255), 2);
                cv::circle(m, center, 2, cv::Scalar(255, 0, 0), 2);
            }

        }
        else if(qnode.cameraCelibrating == qnode.sensorCloseImgRaw)
        {
            for(int i = 0; i < 4; i++)
            {
                cv::Point center((ui.labelCalibrateCameraMainWindow->sensor_close_points[i].x() - r.x) * qnode.calibrateRate, (ui.labelCalibrateCameraMainWindow->sensor_close_points[i].y() - r.y) * qnode.calibrateRate);
                cv::circle(m, center, 5 * qnode.calibrateRate, cv::Scalar(255, 255, 255), 2);
                cv::circle(m, center, 2, cv::Scalar(255, 0, 0), 2);
            }
        }
        qnode.imageCalibrateSecondWindow = QImage(m.data,m.cols,m.rows,m.step[0],QImage::Format_RGB888);
        displayCameraCalibrateSecondWindow(qnode.imageCalibrateSecondWindow);
        QString qstr((std::string("(") + std::to_string(ui.labelCalibrateCameraMainWindow->selectedPoint.x() * qnode.rawImageWidth / qnode.calibrateMainWindowWidth) + std::string(", ") + std::to_string(ui.labelCalibrateCameraMainWindow->selectedPoint.y() * qnode.rawImageHeight / qnode.calibrateMainWindowHeight) + std::string(")")).c_str());
        QPalette pal = ui.labelPointLocation->palette();
        if(ui.labelCalibrateCameraMainWindow->if_is_dragging)
        {
            pal.setColor(QPalette::WindowText, Qt::red);

        }
        else
        {
            pal.setColor(QPalette::WindowText, Qt::black);
        }
        ui.labelPointLocation->setPalette(pal);

        ui.labelPointLocation->setText(qstr);
    }
}

void MainWindow::displayCamera(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_ = image.copy();
    ui.label_camera->setPixmap(QPixmap::fromImage(qimage_));
    ui.label_camera->resize(ui.label_camera->pixmap()->size());
    qimage_mutex_.unlock();

}

void MainWindow::displayCameraCalibrateMainWindow(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_calibrate_main_window_ = image.copy();
    ui.labelCalibrateCameraMainWindow->setPixmap(QPixmap::fromImage(qimage_calibrate_main_window_));
    ui.labelCalibrateCameraMainWindow->resize(ui.labelCalibrateCameraMainWindow->pixmap()->size());
    qimage_mutex_.unlock();
}

void MainWindow::displayCameraCalibrateSecondWindow(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_calibrate_second_window_ = image.copy();
    ui.labelCalibrateCameraSecondWindow->setPixmap(QPixmap::fromImage(qimage_calibrate_second_window_));
    ui.labelCalibrateCameraSecondWindow->resize(ui.labelCalibrateCameraSecondWindow->pixmap()->size());
    qimage_mutex_.unlock();
}

void MainWindow::displayCameraSecondWindow(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_second_window_ = image.copy();
    ui.label_camera2->setPixmap(QPixmap::fromImage(qimage_second_window_));
    ui.label_camera2->resize(ui.label_camera2->pixmap()->size());
    qimage_mutex_.unlock();
}

void MainWindow::updateLoggingView() {
    QListWidgetItem *item = new QListWidgetItem(ui.view_logging);
    item->setText(qnode.logInformation->qstring);
    QFont font = item->font();
    switch (qnode.logInformation->level) {
        case(Debug) : {
                font.setPointSize(15);
                font.setBold(false);
                item->setFont(font);
                item->setTextColor(Qt::gray);
                break;
        }
        case(Info) : {
                font.setPointSize(18);
                font.setBold(false);
                item->setFont(font);
                item->setTextColor(Qt::black);
                break;
        }
        case(Warn) : {
                font.setPointSize(20);
                font.setBold(false);
                item->setFont(font);
                item->setTextColor(Qt::darkYellow);
                break;
        }
        case(Error) : {
                font.setPointSize(23);
                font.setBold(true);
                item->setFont(font);
                item->setTextColor(Qt::red);
                break;
        }
        case(Fatal) : {
                font.setPointSize(28);
                font.setBold(true);
                font.setItalic(true);
                item->setFont(font);
                item->setTextColor(Qt::darkRed);
                break;
        }
    }
    item->setFont(font);
    ui.view_logging->addItem(item);
    ui.view_logging->scrollToBottom();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}

void displayer_qt5::MainWindow::on_comboBoxCalibrateCamera_currentIndexChanged(const QString &arg1)
{
    qnode.cameraCelibrating = arg1;
    ui.labelCalibrateCameraMainWindow->cameraCelibrating = arg1;
}

void displayer_qt5::MainWindow::on_labelCalibrateCameraMainWindow_mouseLocationChanged()
{
    qnode.mouseLoaction = ui.labelCalibrateCameraMainWindow->selectedPoint;
    emit qnode.loggingCameraCalibrateSecondWindow();
}

void displayer_qt5::MainWindow::on_tabWidget_currentChanged(int index)
{
    if(index == 1)
    {
        qnode.if_is_celibrating = true;
    }
    else
    {
        qnode.if_is_celibrating = false;
    }
}

void displayer_qt5::MainWindow::on_pushButtonCalibrate_clicked()
{
    qnode.pubCelibrateResult();
    // if(ui.comboBoxCalibrateCamera->currentIndex() == 1)
    // {
    //     ui.comboBoxCalibrateCamera->setCurrentIndex(0);
    // }
    // else
    // {
    //     ui.comboBoxCalibrateCamera->setCurrentIndex(1);
    // }
}

void displayer_qt5::MainWindow::updateSmallMap()
{
    ui.labelSmallMap->drawSmallMap(qnode.worldPoints);
    unsigned short i = qnode.roiWarnState;
    ui.labelSmallMap->drawROI(&i);
    if(i)
    {
        if((i & 0x01) && log_limit_timer[0] >= LOG_TIME)
        {
            qnode.log(Fatal, std::string("敌方飞坡！敌方飞坡！敌方飞坡！"));
            log_limit_timer[0] = 0;
        }
        if((i & 0x02) && log_limit_timer[1] >= LOG_TIME)
        {
            qnode.log(Warn, std::string("敌方登上能量机关激活点。"));
            log_limit_timer[1] = 0;
        }
        if((i & 0x04) && log_limit_timer[2] >= LOG_TIME)
        {
            //qnode.log(Info, std::string("敌方登上能量机关激活点。"));
            log_limit_timer[2] = 0;
        }
        if((i & 0x08) && log_limit_timer[3] >= LOG_TIME)
        {
            qnode.log(Warn, std::string("敌方位于英雄快乐点。"));
            log_limit_timer[3] = 0;
        }
        if((i & 16) && log_limit_timer[4] >= LOG_TIME)
        {
            qnode.log(Warn, std::string("敌方位于狙击点。"));
            log_limit_timer[4] = 0;
        }
        if((i & 32) && log_limit_timer[5] >= LOG_TIME)
        {
            qnode.log(Error, std::string("敌方位于我方公路区。"));
            log_limit_timer[5] = 0;
        }
        if((i & 64) && log_limit_timer[6] >= LOG_TIME)
        {
            qnode.log(Fatal, std::string("敌方位于我方环高，注意保护哨兵。"));
            log_limit_timer[6] = 0;
        }
        if((i & 128) && log_limit_timer[7] >= LOG_TIME)
        {
            qnode.log(Error, std::string("敌方位于我方狙击点。"));
            log_limit_timer[7] = 0;
        }
        if((i & 256) && log_limit_timer[8] >= LOG_TIME)
        {
            qnode.log(Error, std::string("敌方位于我方能量机关激活点。"));
            log_limit_timer[8] = 0;
        }
        if((i & 512) && log_limit_timer[9] >= LOG_TIME)
        {
            qnode.log(Error, std::string("敌方位于我方飞坡点。"));
            log_limit_timer[9] = 0;
        }
        if((i & 1024) && log_limit_timer[10] >= LOG_TIME)
        {
            qnode.log(Error, std::string("敌方位于我方前哨站处。"));
            log_limit_timer[10] = 0;
        }
    }
}

void displayer_qt5::MainWindow::on_timer_timeout()
{
    ui.labelSmallMap->tim = !ui.labelSmallMap->tim;
    ui.labelSmallMap->update();
    for(size_t i = 0; i < LOG_LIMIT_TIMER_COUNT; i++)
    {
        log_limit_timer[i] ++;
    }
}

void displayer_qt5::MainWindow::updateGameState()
{
    std::string str;
    if(qnode.battle_color == std::string("red"))
    {
        ui.progressBarRobotHealth1_1->setMaximum(qnode.robot_red1.hpMax);
        ui.progressBarRobotHealth1_1->setValue(qnode.robot_red1.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red1.hpMax);
        ui.progressBarRobotHealth1_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth3_1->setMaximum(qnode.robot_red3.hpMax);
        ui.progressBarRobotHealth3_1->setValue(qnode.robot_red3.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red3.hpMax);
        ui.progressBarRobotHealth3_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth4_1->setMaximum(qnode.robot_red4.hpMax);
        ui.progressBarRobotHealth4_1->setValue(qnode.robot_red4.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red4.hpMax);
        ui.progressBarRobotHealth4_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth5_1->setMaximum(qnode.robot_red5.hpMax);
        ui.progressBarRobotHealth5_1->setValue(qnode.robot_red5.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red5.hpMax);
        ui.progressBarRobotHealth5_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth2_1->setValue(qnode.robot_red2.hpCurrent);
        ui.progressBarRobotHealth2_1->setMaximum(qnode.robot_red2.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red2.hpMax);
        ui.progressBarRobotHealth2_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthBase_1->setValue(qnode.robot_redBase.hpCurrent);
        ui.progressBarRobotHealthBase_1->setMaximum(qnode.robot_redBase.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_redBase.hpMax);
        ui.progressBarRobotHealthBase_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthOutpose_1->setValue(qnode.robot_redOutpose.hpCurrent);
        ui.progressBarRobotHealthOutpose_1->setMaximum(qnode.robot_redOutpose.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_redOutpose.hpMax);
        ui.progressBarRobotHealthOutpose_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthGuard_1->setValue(qnode.robot_redGuard.hpCurrent);
        ui.progressBarRobotHealthGuard_1->setMaximum(qnode.robot_redGuard.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_redGuard.hpMax);
        ui.progressBarRobotHealthGuard_1->setFormat(QString(str.c_str()));


        ui.progressBarRobotHealth1_2->setMaximum(qnode.robot_blue1.hpMax);
        ui.progressBarRobotHealth1_2->setValue(qnode.robot_blue1.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue1.hpMax);
        ui.progressBarRobotHealth1_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth3_2->setMaximum(qnode.robot_blue3.hpMax);
        ui.progressBarRobotHealth3_2->setValue(qnode.robot_blue3.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue3.hpMax);
        ui.progressBarRobotHealth3_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth4_2->setMaximum(qnode.robot_blue4.hpMax);
        ui.progressBarRobotHealth4_2->setValue(qnode.robot_blue4.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue4.hpMax);
        ui.progressBarRobotHealth4_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth5_2->setMaximum(qnode.robot_blue5.hpMax);
        ui.progressBarRobotHealth5_2->setValue(qnode.robot_blue5.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue5.hpMax);
        ui.progressBarRobotHealth5_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth2_2->setValue(qnode.robot_blue2.hpCurrent);
        ui.progressBarRobotHealth2_2->setMaximum(qnode.robot_blue2.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue2.hpMax);
        ui.progressBarRobotHealth2_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthBase_2->setValue(qnode.robot_blueBase.hpCurrent);
        ui.progressBarRobotHealthBase_2->setMaximum(qnode.robot_blueBase.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blueBase.hpMax);
        ui.progressBarRobotHealthBase_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthOutpose_2->setValue(qnode.robot_blueOutpose.hpCurrent);
        ui.progressBarRobotHealthOutpose_2->setMaximum(qnode.robot_blueOutpose.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blueOutpose.hpMax);
        ui.progressBarRobotHealthOutpose_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthGuard_2->setValue(qnode.robot_blueGuard.hpCurrent);
        ui.progressBarRobotHealthGuard_2->setMaximum(qnode.robot_blueGuard.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blueGuard.hpMax);
        ui.progressBarRobotHealthGuard_2->setFormat(QString(str.c_str()));
    }
    else if(qnode.battle_color == std::string("blue"))
    {
        ui.progressBarRobotHealth1_2->setMaximum(qnode.robot_red1.hpMax);
        ui.progressBarRobotHealth1_2->setValue(qnode.robot_red1.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red1.hpMax);
        ui.progressBarRobotHealth1_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth3_2->setMaximum(qnode.robot_red3.hpMax);
        ui.progressBarRobotHealth3_2->setValue(qnode.robot_red3.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red3.hpMax);
        ui.progressBarRobotHealth3_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth4_2->setMaximum(qnode.robot_red4.hpMax);
        ui.progressBarRobotHealth4_2->setValue(qnode.robot_red4.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red4.hpMax);
        ui.progressBarRobotHealth4_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth5_2->setMaximum(qnode.robot_red5.hpMax);
        ui.progressBarRobotHealth5_2->setValue(qnode.robot_red5.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red5.hpMax);
        ui.progressBarRobotHealth5_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth2_2->setValue(qnode.robot_red2.hpCurrent);
        ui.progressBarRobotHealth2_2->setMaximum(qnode.robot_red2.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_red2.hpMax);
        ui.progressBarRobotHealth2_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthBase_2->setValue(qnode.robot_redBase.hpCurrent);
        ui.progressBarRobotHealthBase_2->setMaximum(qnode.robot_redBase.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_redBase.hpMax);
        ui.progressBarRobotHealthBase_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthOutpose_2->setValue(qnode.robot_redOutpose.hpCurrent);
        ui.progressBarRobotHealthOutpose_2->setMaximum(qnode.robot_redOutpose.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_redOutpose.hpMax);
        ui.progressBarRobotHealthOutpose_2->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthGuard_2->setValue(qnode.robot_redGuard.hpCurrent);
        ui.progressBarRobotHealthGuard_2->setMaximum(qnode.robot_redGuard.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_redGuard.hpMax);
        ui.progressBarRobotHealthGuard_2->setFormat(QString(str.c_str()));


        ui.progressBarRobotHealth1_1->setMaximum(qnode.robot_blue1.hpMax);
        ui.progressBarRobotHealth1_1->setValue(qnode.robot_blue1.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue1.hpMax);
        ui.progressBarRobotHealth1_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth3_1->setMaximum(qnode.robot_blue3.hpMax);
        ui.progressBarRobotHealth3_1->setValue(qnode.robot_blue3.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue3.hpMax);
        ui.progressBarRobotHealth3_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth4_1->setMaximum(qnode.robot_blue4.hpMax);
        ui.progressBarRobotHealth4_1->setValue(qnode.robot_blue4.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue4.hpMax);
        ui.progressBarRobotHealth4_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth5_1->setMaximum(qnode.robot_blue5.hpMax);
        ui.progressBarRobotHealth5_1->setValue(qnode.robot_blue5.hpCurrent);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue5.hpMax);
        ui.progressBarRobotHealth5_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealth2_1->setValue(qnode.robot_blue2.hpCurrent);
        ui.progressBarRobotHealth2_1->setMaximum(qnode.robot_blue2.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blue2.hpMax);
        ui.progressBarRobotHealth2_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthBase_1->setValue(qnode.robot_blueBase.hpCurrent);
        ui.progressBarRobotHealthBase_1->setMaximum(qnode.robot_blueBase.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blueBase.hpMax);
        ui.progressBarRobotHealthBase_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthOutpose_1->setValue(qnode.robot_blueOutpose.hpCurrent);
        ui.progressBarRobotHealthOutpose_1->setMaximum(qnode.robot_blueOutpose.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blueOutpose.hpMax);
        ui.progressBarRobotHealthOutpose_1->setFormat(QString(str.c_str()));

        ui.progressBarRobotHealthGuard_1->setValue(qnode.robot_blueGuard.hpCurrent);
        ui.progressBarRobotHealthGuard_1->setMaximum(qnode.robot_blueGuard.hpMax);
        str = std::string("%v/");
        str += std::to_string(qnode.robot_blueGuard.hpMax);
        ui.progressBarRobotHealthGuard_1->setFormat(QString(str.c_str()));
    }

    ui.labelGameStage->setText(qnode.gameProgress);
    int min = qnode.stageRemainTime / 60;
    int sec = qnode.stageRemainTime % 60;
    QString remain_time = QString((std::to_string(min) + std::string(":") + std::to_string(sec)).c_str());
    QPalette pal = ui.labelRemainTime->palette();
    if(qnode.stageRemainTime <= 5)
    {
        pal.setColor(QPalette::WindowText, Qt::red);

    }
    else
    {
        pal.setColor(QPalette::WindowText, Qt::black);
    }
    ui.labelRemainTime->setPalette(pal);
    ui.labelRemainTime->setText(remain_time);
}

