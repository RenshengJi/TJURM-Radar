/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/displayer_qt5/qnode.hpp"
#include "sensor_msgs/image_encodings.h"
#include "displayer_qt5/qlabel_with_mouse_event.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace displayer_qt5 {


radar_msgs::points far_points_msg;
radar_msgs::points close_points_msg;

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::imgShowCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if(!if_is_celibrating)
    {
        try
        {
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
          if(!cv_ptr->image.empty())
          {
              img = cv_ptr->image;
              cv::resize(img, img, cv::Size(showMainWindowWidth, showMainWindowHeight));
              image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);//change  to QImage format
          }
          Q_EMIT loggingCamera();
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}

void QNode::imgShowSecondWindowCallback(const sensor_msgs::ImageConstPtr &msg)
{
    static int i = 0;
    static std::string path(PROJECT_PATH);
    if(!if_is_celibrating)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            if(!cv_ptr->image.empty())
            {
//              if(recorder_fps <= 1e-5)
//              {
//                  static ros::Time begin;
//                  static ros::Time end;
//                  if(i == 0)
//                  {
//                      begin = ros::Time::now();
//                  }
//                  else if(i == 10)
//                  {
//                      end = ros::Time::now();
//                      recorder_fps = 10 / (end - begin).toSec();
//                  }
//                  i++;
//              }
//              else if(i == 11)
//              {
//                  int codec = cv::VideoWriter::fourcc('D', 'I', 'V', '3');
//                  path += "/recorded.avi";
//                  recorder.open(path, codec, recorder_fps, cv_ptr->image.size(), true);
//                  i++;
//              }
              imgShowSecondWindow = cv_ptr->image;
//              if(ifBeginToRecord)
//              {
//                  ifRecordDone = false;
//                  recorder << imgShowSecondWindow;
//              }
//              if(ifRecordDone)
//              {
//                  recorder.release();
//                  replayer.open(path);
//                  log(Info, std::string("我方飞镖闸门关闭，开始回放！"));
//              }
//              static int i = 0;
//              if(ifBeginToReplay)
//              {
//                  ifRecordDone = false;
//                  cv::Mat m;
//                  replayer >> m;
//                  if(!m.empty())
//                  {
//                      cv::Rect re(0, 0, m.cols / 2, m.rows / 2);
//                      m(re).copyTo(imgShowSecondWindow);
//
//                  }
//                  else
//                  {
//                      if(i == 2)
//                      {
//                          ifReplayDone = true;
//                          ifBeginToReplay = false;
//                          i = 0;
//                          log(Info, std::string("第3次回放结束！"));
//                      }
//                      else
//                      {
//                          i++;
//                          replayer.open(path);
//                          log(Info, std::string("第") + std::to_string(i) + std::string("次回放结束！"));
//                      }
//                  }
//              }
//              if(ifReplayDone)
//              {
//                  replayer.release();
//              }

              cv::resize(imgShowSecondWindow, imgShowSecondWindow, cv::Size(showSecondWindowWidth, showSecondWindowHeight));
              imageShowSecondWindow = QImage(imgShowSecondWindow.data,imgShowSecondWindow.cols,imgShowSecondWindow.rows,imgShowSecondWindow.step[0],QImage::Format_RGB888);//change  to QImage format
            
              img = cv_ptr->image; // !!!
              cv::resize(img, img, cv::Size(showMainWindowWidth, showMainWindowHeight)); // !!!
              image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888); // !!!
            }
          Q_EMIT loggingCamera();
          Q_EMIT loggingCameraSecondWindow();
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}

void QNode::imgSensorFarCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if(cameraCelibrating == sensorFarImgRaw && if_is_celibrating)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            if(!cv_ptr->image.empty())
            {
                imgSensorFar = cv_ptr->image;
                cv::resize(imgSensorFar, imgSensorFar, cv::Size(calibrateMainWindowWidth, calibrateMainWindowHeight));
                imageCalibrateMainWindow = QImage(imgSensorFar.data,imgSensorFar.cols,imgSensorFar.rows,imgSensorFar.step[0],QImage::Format_RGB888);//change  to QImage format
                Q_EMIT loggingCameraCalibrateMainWindow();
                Q_EMIT loggingCameraCalibrateSecondWindow();
            }
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    qDebug("close publich!\n");
    for (int i = 0; i < 4; i++) {
       qDebug("%f\t%f\n", far_points_msg.data[i].x, far_points_msg.data[i].y); 
    }
    // calibration_pub_sensor_far.publish(far_points_msg);
}

void QNode::imgSensorCloseCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if(cameraCelibrating == sensorCloseImgRaw && if_is_celibrating)
    {
        try
        {
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
          if(!cv_ptr->image.empty())
          {
              imgSensorClose = cv_ptr->image;
              cv::resize(imgSensorClose, imgSensorClose, cv::Size(calibrateMainWindowWidth, calibrateMainWindowHeight));
              imageCalibrateMainWindow = QImage(imgSensorClose.data,imgSensorClose.cols,imgSensorClose.rows,imgSensorClose.step[0],QImage::Format_RGB888);//change  to QImage format
              Q_EMIT loggingCameraCalibrateMainWindow();
              Q_EMIT loggingCameraCalibrateSecondWindow();
          }

        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
    qDebug("far publich!\n");
    for (int i = 0; i < 4; i++) {
       qDebug("%f\t%f\n", close_points_msg.data[i].x, close_points_msg.data[i].y); 
    }
    // calibration_pub_sensor_close.publish(close_points_msg);
}

void QNode::gameStateCallback(const radar_msgs::game_stateConstPtr &msg)
{
    robot_red1.hpCurrent = msg->red_1_robot_HP;
    robot_red2.hpCurrent = msg->red_2_robot_HP;
    robot_red3.hpCurrent = msg->red_3_robot_HP;
    robot_red4.hpCurrent = msg->red_4_robot_HP;
    robot_red5.hpCurrent = msg->red_5_robot_HP;
    robot_redBase.hpCurrent = msg->red_base_HP;
    robot_redGuard.hpCurrent = msg->red_7_robot_HP;
    robot_redOutpose.hpCurrent = msg->red_outpose_HP;

    if(msg->red_1_robot_HP > robot_red1.hpMax)
    {
        if(msg->red_1_robot_HP <= 150)
        {
            robot_red1.hpMax = 150;
        }
        else if(msg->red_1_robot_HP <= 200)
        {
            robot_red1.hpMax = 200;
        }
        else if(msg->red_1_robot_HP <= 250)
        {
            robot_red1.hpMax = 250;
        }
        else if(msg->red_1_robot_HP <= 300)
        {
            robot_red1.hpMax = 300;
        }
        else if(msg->red_1_robot_HP <= 350)
        {
            robot_red1.hpMax = 350;
        }
        else if(msg->red_1_robot_HP <= 450)
        {
            robot_red1.hpMax = 450;
        }
    }

    if(msg->red_3_robot_HP > robot_red3.hpMax)
    {
        if(msg->red_3_robot_HP <= 100)
        {
            robot_red3.hpMax = 100;
        }
        else if(msg->red_3_robot_HP <= 150)
        {
            robot_red3.hpMax = 150;
        }
        else if(msg->red_3_robot_HP <= 200)
        {
            robot_red3.hpMax = 200;
        }
        else if(msg->red_3_robot_HP <= 250)
        {
            robot_red3.hpMax = 250;
        }
        else if(msg->red_3_robot_HP <= 300)
        {
            robot_red3.hpMax = 300;
        }
        else if(msg->red_3_robot_HP <= 400)
        {
            robot_red3.hpMax = 400;
        }
        else if(msg->red_3_robot_HP <= 500)
        {
            robot_red3.hpMax = 500;
        }
    }

    if(msg->red_4_robot_HP > robot_red4.hpMax)
    {
        if(msg->red_4_robot_HP <= 100)
        {
            robot_red4.hpMax = 100;
        }
        else if(msg->red_4_robot_HP <= 150)
        {
            robot_red4.hpMax = 150;
        }
        else if(msg->red_4_robot_HP <= 200)
        {
            robot_red4.hpMax = 200;
        }
        else if(msg->red_4_robot_HP <= 250)
        {
            robot_red4.hpMax = 250;
        }
        else if(msg->red_4_robot_HP <= 300)
        {
            robot_red4.hpMax = 300;
        }
        else if(msg->red_4_robot_HP <= 400)
        {
            robot_red4.hpMax = 400;
        }
        else if(msg->red_4_robot_HP <= 500)
        {
            robot_red4.hpMax = 500;
        }
    }

    if(msg->red_5_robot_HP > robot_red5.hpMax)
    {
        if(msg->red_5_robot_HP <= 100)
        {
            robot_red5.hpMax = 100;
        }
        else if(msg->red_5_robot_HP <= 150)
        {
            robot_red5.hpMax = 150;
        }
        else if(msg->red_5_robot_HP <= 200)
        {
            robot_red5.hpMax = 200;
        }
        else if(msg->red_5_robot_HP <= 250)
        {
            robot_red5.hpMax = 250;
        }
        else if(msg->red_5_robot_HP <= 300)
        {
            robot_red5.hpMax = 300;
        }
        else if(msg->red_5_robot_HP <= 400)
        {
            robot_red5.hpMax = 400;
        }
        else if(msg->red_5_robot_HP <= 500)
        {
            robot_red5.hpMax = 500;
        }
    }


    robot_blue1.hpCurrent = msg->blue_1_robot_HP;
    robot_blue2.hpCurrent = msg->blue_2_robot_HP;
    robot_blue3.hpCurrent = msg->blue_3_robot_HP;
    robot_blue4.hpCurrent = msg->blue_4_robot_HP;
    robot_blue5.hpCurrent = msg->blue_5_robot_HP;
    robot_blueBase.hpCurrent = msg->blue_base_HP;
    robot_blueGuard.hpCurrent = msg->blue_7_robot_HP;
    robot_blueOutpose.hpCurrent = msg->blue_outpose_HP;

    if(msg->blue_1_robot_HP > robot_blue1.hpMax)
    {
        if(msg->blue_1_robot_HP <= 150)
        {
            robot_blue1.hpMax = 150;
        }
        else if(msg->blue_1_robot_HP <= 200)
        {
            robot_blue1.hpMax = 200;
        }
        else if(msg->blue_1_robot_HP <= 250)
        {
            robot_blue1.hpMax = 250;
        }
        else if(msg->blue_1_robot_HP <= 300)
        {
            robot_blue1.hpMax = 300;
        }
        else if(msg->blue_1_robot_HP <= 350)
        {
            robot_blue1.hpMax = 350;
        }
        else if(msg->blue_1_robot_HP <= 450)
        {
            robot_blue1.hpMax = 450;
        }
    }

    if(msg->blue_3_robot_HP > robot_blue3.hpMax)
    {
        if(msg->blue_3_robot_HP <= 100)
        {
            robot_blue3.hpMax = 100;
        }
        else if(msg->blue_3_robot_HP <= 150)
        {
            robot_blue3.hpMax = 150;
        }
        else if(msg->blue_3_robot_HP <= 200)
        {
            robot_blue3.hpMax = 200;
        }
        else if(msg->blue_3_robot_HP <= 250)
        {
            robot_blue3.hpMax = 250;
        }
        else if(msg->blue_3_robot_HP <= 300)
        {
            robot_blue3.hpMax = 300;
        }
        else if(msg->blue_3_robot_HP <= 400)
        {
            robot_blue3.hpMax = 400;
        }
        else if(msg->blue_3_robot_HP <= 500)
        {
            robot_blue3.hpMax = 500;
        }
    }

    if(msg->blue_4_robot_HP > robot_blue4.hpMax)
    {
        if(msg->blue_4_robot_HP <= 100)
        {
            robot_blue4.hpMax = 100;
        }
        else if(msg->blue_4_robot_HP <= 150)
        {
            robot_blue4.hpMax = 150;
        }
        else if(msg->blue_4_robot_HP <= 200)
        {
            robot_blue4.hpMax = 200;
        }
        else if(msg->blue_4_robot_HP <= 250)
        {
            robot_blue4.hpMax = 250;
        }
        else if(msg->blue_4_robot_HP <= 300)
        {
            robot_blue4.hpMax = 300;
        }
        else if(msg->blue_4_robot_HP <= 400)
        {
            robot_blue4.hpMax = 400;
        }
        else if(msg->blue_4_robot_HP <= 500)
        {
            robot_blue4.hpMax = 500;
        }
    }

    if(msg->blue_5_robot_HP > robot_blue5.hpMax)
    {
        if(msg->blue_5_robot_HP <= 100)
        {
            robot_blue5.hpMax = 100;
        }
        else if(msg->blue_5_robot_HP <= 150)
        {
            robot_blue5.hpMax = 150;
        }
        else if(msg->blue_5_robot_HP <= 200)
        {
            robot_blue5.hpMax = 200;
        }
        else if(msg->blue_5_robot_HP <= 250)
        {
            robot_blue5.hpMax = 250;
        }
        else if(msg->blue_5_robot_HP <= 300)
        {
            robot_blue5.hpMax = 300;
        }
        else if(msg->blue_5_robot_HP <= 400)
        {
            robot_blue5.hpMax = 400;
        }
        else if(msg->blue_5_robot_HP <= 500)
        {
            robot_blue5.hpMax = 500;
        }
    }

    if(msg->game_progress == 0)
    {
        gameProgress = gameProgress.fromLocal8Bit("比赛未开始");
    }
    else if(msg->game_progress == 1)
    {
        gameProgress = gameProgress.fromLocal8Bit("准备阶段");
    }
    else if(msg->game_progress == 2)
    {
        gameProgress = gameProgress.fromLocal8Bit("自检阶段");
    }
    else if(msg->game_progress == 3)
    {
        gameProgress = gameProgress.fromLocal8Bit("5s倒计时");
    }
    else if(msg->game_progress == 4)
    {
        gameProgress = gameProgress.fromLocal8Bit("比赛中");
    }
    else if(msg->game_progress == 1)
    {
        gameProgress = "比赛结算";
    }
    if(msg->dart_remaining_time <= 15 && msg->dart_remaining_time > 1)
    {
        ifBeginToRecord = true;
        ifReplayDone = false;
        if(msg->dart_remaining_time == 15)
        {
            log(Info, std::string("我方飞镖闸门成功开启！"));
        }
        log(Info, std::string("飞镖闸门关闭倒计时：！") + std::to_string(msg->dart_remaining_time));
    }
    else if(msg->dart_remaining_time <= 1)
    {
        ifRecordDone = true;
        ifBeginToRecord = false;
        ifBeginToReplay = true;
        ifReplayDone = false;
        dart_first_close_time = msg->stage_remain_time;
    }
    stageRemainTime = msg->stage_remain_time;
    if(stageRemainTime >= 385 && stageRemainTime <= 390)
    {
        log(Error, std::string("请及时击发飞镖！"));
    }
    if((dart_first_close_time - stageRemainTime) >= 21 && (dart_first_close_time - stageRemainTime) <= 26)
    {
        log(Error, std::string("飞镖冷却结束，请及时击发飞镖！"));
    }
    Q_EMIT loggingGameStateUpdate();
}

void QNode::supplyProjectileActionCallback(const radar_msgs::supply_projectile_actionConstPtr &msg)
{

}

void QNode::refereeWarningCallback(const radar_msgs::referee_warningConstPtr &msg)
{
    if(msg->foul_robot_id == 0)
    {
        log(Fatal, std::string("我方被判负！比赛结束。"));
    }
    else if(msg->foul_robot_id >= 100)
    {
        if(msg->level == 1)
        {
            log(Fatal, std::string("我方") + std::to_string(msg->foul_robot_id - 100) + std::string("号被裁判给黄牌！"));
        }
        else if(msg->level == 2)
        {
            log(Fatal, std::string("我方") + std::to_string(msg->foul_robot_id - 100) + std::string("号被裁判罚下！"));
        }

    }
    else
    {
        if(msg->level == 1)
        {
            log(Fatal, std::string("我方") + std::to_string(msg->foul_robot_id) + std::string("号被裁判给黄牌！"));
        }
        else if(msg->level == 2)
        {
            log(Fatal, std::string("我方") + std::to_string(msg->foul_robot_id) + std::string("号被裁判罚下！"));
        }
    }
}

void QNode::pubCelibrateResult()
{
    radar_msgs::point one_point_msg;
    // radar_msgs::points points_msg;
    if(cameraCelibrating == sensorFarImgRaw)
    {
        far_points_msg.data.clear();
        for(int i = 0; i < 4; i++)
        {
            one_point_msg.id=i;
            one_point_msg.x = sensor_far_points[i].x() * 1.0 / calibrateMainWindowWidth;
            one_point_msg.y = sensor_far_points[i].y() * 1.0 / calibrateMainWindowHeight;
            far_points_msg.data.push_back(one_point_msg);
        }
        calibration_pub_sensor_far.publish(far_points_msg);
    }
    else if (cameraCelibrating == sensorCloseImgRaw)
    {
        close_points_msg.data.clear();
        for(int i = 0; i < 4; i++)
        {
            one_point_msg.id=i;
            one_point_msg.x = sensor_close_points[i].x() * 1.0 / calibrateMainWindowWidth;
            one_point_msg.y = sensor_close_points[i].y() * 1.0 / calibrateMainWindowHeight;
            close_points_msg.data.push_back(one_point_msg);
        }
        calibration_pub_sensor_close.publish(close_points_msg);
    }
}

void QNode::worldPointCallback(const radar_msgs::points &msg)
{
    world_point wp;
    std::vector<world_point>().swap(worldPoints);
    for(size_t i = 0; i < msg.data.size(); i++)
    {
        wp.point = QPoint(msg.data[i].x * smallMapWidth, (1 - msg.data[i].y) * smallMapHeight);
        wp.id = msg.data[i].id;
        worldPoints.push_back(wp);
    }
    memcpy(&roiWarnState, msg.text.c_str(), 2);
    Q_EMIT loggingSmallMapUpdate();
}

void close_cali_callback(const radar_msgs::points & points) {
    qDebug("close_cali received\n");
}

bool QNode::init()
{   
    qDebug("QNode init\n");
	ros::init(init_argc,init_argv,"displayer_qt5");
    if ( ! ros::master::check() )
    {
		return false;
	}
    loadParams();
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    image_transport::ImageTransport it(n);
    // image_sub = it.subscribe(realsenseImgRaw.toStdString(),1,&QNode::imgShowCallback,this);
    // image_sub_second_window = it.subscribe(secondWindowTopic,1,&QNode::imgShowSecondWindowCallback,this);
    image_sub_sensor_far = it.subscribe(sensorFarImgRaw.toStdString(),1,&QNode::imgSensorFarCallback,this);
    image_sub_sensor_close = it.subscribe(sensorCloseImgRaw.toStdString(),1,&QNode::imgSensorCloseCallback,this);

    calibration_pub_sensor_far = n.advertise <radar_msgs::points>(calibrationTopicSensorFar.toStdString(), 1);
    calibration_pub_sensor_close = n.advertise <radar_msgs::points>(calibrationTopicSensorClose.toStdString(), 1);

    // ros::Subscriber close_cali_sub = n.subscribe("/sensor_close/calibration", 1, &close_cali_callback);

    gameStateSub = n.subscribe(gameStateTopic, 1, &QNode::gameStateCallback, this);
    supplyProjectileActionSub = n.subscribe(supplyProjectileActionTopic, 1, &QNode::supplyProjectileActionCallback, this);
    refereeWarningSub = n.subscribe(refereeWarningTopic, 1, &QNode::refereeWarningCallback, this);
    worldPointSub = n.subscribe(worldPointTopic, 1, &QNode::worldPointCallback, this);

    radar_msgs::point one_point_msg;
    for(int i = 0; i < 4; i++)
        {
            one_point_msg.id=i;
            one_point_msg.x = 0;
            one_point_msg.y = 0;
            far_points_msg.data.push_back(one_point_msg);
            close_points_msg.data.push_back(one_point_msg);
        }

    qDebug("start\n");
    start();
	return true;
}


void QNode::run()
{

    log(Debug,std::string("程序开始！"));
    qDebug("QNode run\n");
    ros::spin();
    qDebug("Ros shutdown\n");
    // std::cout << "Ros shutdown" << std::endl;
    Q_EMIT rosShutdown();
}


void QNode::log( const LogLevel &level, const std::string &msg) {
    logInformation = new log_information;
    logInformation->level = level;
    std::stringstream logging_model_msg;
    switch ( level ) {
        case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
        case(Info) : {
                ROS_INFO_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
        case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
        case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
        case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
    }
    logInformation->qstring = (QString(logging_model_msg.str().c_str()));
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::loadParams()
{
    std::string str;
    // sensorFarImgRaw = "/sensor_far/image_raw";
    sensorFarImgRaw = "/rgb/image_raw";
    ros::param::get("/camera/list/farCam/topic", str);
    sensorFarImgRaw = QString(str.c_str());
    // sensorCloseImgRaw = "/sensor_close/image_raw";
    sensorCloseImgRaw = "/hikrobot_camera/rgb";
    ros::param::get("/camera/list/closeCam/topic", str);
    sensorCloseImgRaw = QString(str.c_str());

    calibrateRate = 3;
    ros::param::get("/calibrate/rate", calibrateRate);

    realsenseImgRaw = "/camera/color/image_raw";
    ros::param::get("/camera/list/realsense/topic", str);
    realsenseImgRaw = QString(str.c_str());

    calibrationTopicSensorFar = "/sensor_far/calibration";
    ros::param::get("/camera/list/farCam/calibrationTopic", str);
    calibrationTopicSensorFar = QString(str.c_str());
    calibrationTopicSensorClose = "/sensor_close/calibration";
    ros::param::get("/camera/list/closeCam/calibrationTopic", str);
    calibrationTopicSensorClose = QString(str.c_str());

    gameStateTopic = "/game_state";
    ros::param::get("/judgeSystem/gameStateTopic", gameStateTopic);

    supplyProjectileActionTopic = "/supply_projectile_action";
    ros::param::get("/judgeSystem/supplyProjectileActionTopic", supplyProjectileActionTopic);

    // secondWindowTopic = "/sensor_far/image_raw";
    secondWindowTopic = "/rgb/image_raw";
    ros::param::get("/game/secondWindowTopic", secondWindowTopic);

    worldPointTopic = "/world_point";
    ros::param::get("/minimap/subscribeTopic", worldPointTopic);

    ifRecord = false;
    ros::param::get("/game/record/ifRecord", ifRecord);

    recordPath = "/home/dovejh/project/radar_station/recorder.avi";
    ros::param::get("/game/record/recordPath", recordPath);

    refereeWarningTopic = "/referee_warning";
    ros::param::get("/judgeSystem/refereeWarningTopic", refereeWarningTopic);

    battle_color = "blue";
    ros::param::get("/battle_state/battle_color", battle_color);

    rawImageWidth = 1280;
    ros::param::get("/calibrate/rawImageWidth", rawImageWidth);

    rawImageHeight = 1024;
    ros::param::get("/calibrate/rawImageHeight", rawImageHeight);

    // smallMapWidth = 360;
    // smallMapHeight = 672;

    smallMapWidth = 240;
    smallMapHeight = 420;

    std::string ad(PROJECT_PATH);
    if(battle_color == std::string("red"))
    {
        ad += "/resources/images/red_minimap.png";
    }
    else if(battle_color == std::string("blue"))
    {
        ad += "/resources/images/blue_minimap.png";
    }
    imgSmallMap = cv::imread(ad);
    cv::cvtColor(imgSmallMap, imgSmallMap, CV_BGR2RGB);
    cv::resize(imgSmallMap, imgSmallMap, cv::Size(smallMapWidth, smallMapHeight));
    imageSmallMap = QImage(imgSmallMap.data,imgSmallMap.cols,imgSmallMap.rows,imgSmallMap.step[0],QImage::Format_RGB888);

    // logoHeight = 448;
    // logoWidth = 222;
    // ad = std::string(PROJECT_PATH);
    // ad += "/resources/images/radar_logo.png";
    // imgLogo = cv::imread(ad);
    // cv::cvtColor(imgLogo, imgLogo, CV_BGR2RGB);
    // imageLogo.load(ad.c_str());
    // imageLogo = imageLogo.scaled(logoWidth, logoHeight, Qt::KeepAspectRatio);
    //cv::resize(imgLogo, imgLogo, cv::Size(logoWidth, logoHeight));
    //imageLogo = QImage(imgLogo.data,imgLogo.cols,imgLogo.rows,imgLogo.step[0],QImage::Format_RGB888);

    // calibrateMainWindowWidth = 1256;
    // calibrateMainWindowHeight = 1005;
    calibrateMainWindowWidth = 1280;
    calibrateMainWindowHeight = 720;
    // calibrateSecondWindowWidth = 618;
    // calibrateSecondWindowHeight = 618;
    calibrateSecondWindowWidth = 400;
    calibrateSecondWindowHeight = 400;

    // showMainWindowWidth = 1280;
    // showMainWindowHeight = 720;
    showMainWindowWidth = 960;
    showMainWindowHeight = 540;
    showSecondWindowWidth = 346;
    showSecondWindowHeight = 277;

    img = cv::Mat(showMainWindowHeight, showMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    imgSensorFar = cv::Mat(calibrateMainWindowHeight, calibrateMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    imgSensorClose = cv::Mat(calibrateMainWindowHeight, calibrateMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    imgShowSecondWindow = cv::Mat(showSecondWindowWidth, showSecondWindowHeight, CV_8UC3, cv::Scalar(255, 255, 255));

    if_is_celibrating = false;

    float x = 0, y = 0;
    QPoint point;
    ros::param::get("/camera/list/farCam/calibrationDefault/point1/x", x);
    ros::param::get("/camera/list/farCam/calibrationDefault/point1/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_far_points[0] = point;
    ros::param::get("/camera/list/farCam/calibrationDefault/point2/x", x);
    ros::param::get("/camera/list/farCam/calibrationDefault/point2/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_far_points[1] = point;
    ros::param::get("/camera/list/farCam/calibrationDefault/point3/x", x);
    ros::param::get("/camera/list/farCam/calibrationDefault/point3/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_far_points[2] = point;
    ros::param::get("/camera/list/farCam/calibrationDefault/point4/x", x);
    ros::param::get("/camera/list/farCam/calibrationDefault/point4/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_far_points[3] = point;
    ros::param::get("/camera/list/closeCam/calibrationDefault/point1/x", x);
    ros::param::get("/camera/list/closeCam/calibrationDefault/point1/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_close_points[0] = point;
    ros::param::get("/camera/list/closeCam/calibrationDefault/point2/x", x);
    ros::param::get("/camera/list/closeCam/calibrationDefault/point2/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_close_points[1] = point;
    ros::param::get("/camera/list/closeCam/calibrationDefault/point3/x", x);
    ros::param::get("/camera/list/closeCam/calibrationDefault/point3/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_close_points[2] = point;
    ros::param::get("/camera/list/closeCam/calibrationDefault/point4/x", x);
    ros::param::get("/camera/list/closeCam/calibrationDefault/point4/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_close_points[3] = point;

    robot_red2.hpCurrent = 500;
    robot_red2.hpMax = 500;
    robot_redGuard.hpCurrent = 600;
    robot_redGuard.hpMax = 600;
    robot_redBase.hpCurrent = 5000;
    robot_redBase.hpMax = 5000;
    robot_red1.hpCurrent = 150;
    robot_red1.hpMax = 150;
    robot_red3.hpCurrent = 100;
    robot_red3.hpMax = 100;
    robot_red4.hpCurrent = 100;
    robot_red4.hpMax = 100;
    robot_red5.hpCurrent = 100;
    robot_red5.hpMax = 100;
    robot_redOutpose.hpMax = 1500;
    robot_redOutpose.hpCurrent = 1500;

    robot_blue2.hpCurrent = 500;
    robot_blue2.hpMax = 500;
    robot_blueGuard.hpCurrent = 600;
    robot_blueGuard.hpMax = 600;
    robot_blueBase.hpCurrent = 5000;
    robot_blueBase.hpMax = 5000;
    robot_blue1.hpCurrent = 150;
    robot_blue1.hpMax = 150;
    robot_blue3.hpCurrent = 100;
    robot_blue3.hpMax = 100;
    robot_blue4.hpCurrent = 100;
    robot_blue4.hpMax = 100;
    robot_blue5.hpCurrent = 100;
    robot_blue5.hpMax = 100;
    robot_blueOutpose.hpMax = 1500;
    robot_blueOutpose.hpCurrent = 1500;
    recorder_fps = 0;

    ifBeginToRecord = false;
    ifBeginToReplay = false;
    ifReplayDone = false;
    ifRecordDone = false;
}

}  // namespace displayer_qt5
