#include "displayer_qt5/qlabel_with_mouse_event.h"
#include <iostream>
void QLabel_with_mouse_event::mouseMoveEvent(QMouseEvent *event)
{
    QPoint point = event->pos();
    QLabel::mouseMoveEvent(event);
    selectedPoint = point;
    double min_distance = 100000000000;
    static int min_i = 4;
    if(if_is_dragging)
    {
        if(cameraCelibrating == sensorFarImgRaw)
        {   if(if_point_being_selected)
            {
                sensor_far_points[min_i] = point;
                std::string str("/camera/list/farCam/calibrationDefault/point"), str2;
                str += std::to_string(min_i + 1);
                str2 = str + "/x";
                ros::param::set(str2, point.x() * 1.0 / calibrateMainWindowWidth);
                str2 = str + "/y";
                ros::param::set(str2, point.y() * 1.0 / calibrateMainWindowHeight);
            }
            else
            {
                for(int i = 0; i < 4; i++)
                {
                    double distance = (point.x() - sensor_far_points[i].x()) * (point.x() - sensor_far_points[i].x()) + (point.y() - sensor_far_points[i].y()) * (point.y() - sensor_far_points[i].y());
                    if(min_distance > distance)
                    {
                        min_distance = distance;
                        min_i = i;
                    }
                }
                sensor_far_points[min_i] = point;
                std::string str("/camera/list/farCam/calibrationDefault/point"), str2;
                str += std::to_string(min_i + 1);
                str2 = str + "/x";
                ros::param::set(str2, point.x() * 1.0 / calibrateMainWindowWidth);
                str2 = str + "/y";
                ros::param::set(str2, point.y() * 1.0 / calibrateMainWindowHeight);
                if_point_being_selected = true;
            }
        }
        else if(cameraCelibrating == sensorCloseImgRaw)
        {
            if(if_point_being_selected)
            {
                sensor_close_points[min_i] = point;
                std::string str("/camera/list/closeCam/calibrationDefault/point"), str2;
                str += std::to_string(min_i + 1);
                str2 = str + "/x";
                ros::param::set(str2, point.x() * 1.0 / calibrateMainWindowWidth);
                str2 = str + "/y";
                ros::param::set(str2, point.y() * 1.0 / calibrateMainWindowHeight);
            }
            else
            {
                for(int i = 0; i < 4; i++)
                {
                    double distance = (point.x() - sensor_close_points[i].x()) * (point.x() - sensor_close_points[i].x()) + (point.y() - sensor_close_points[i].y()) * (point.y() - sensor_close_points[i].y());
                    if(min_distance > distance)
                    {
                        min_distance = distance;
                        min_i = i;
                    }
                }
                sensor_close_points[min_i] = point;
                std::string str("/camera/list/closeCam/calibrationDefault/point"), str2;
                str += std::to_string(min_i + 1);
                str2 = str + "/x";
                ros::param::set(str2, point.x() * 1.0 / calibrateMainWindowWidth);
                str2 = str + "/y";
                ros::param::set(str2, point.y() * 1.0 / calibrateMainWindowHeight);
                if_point_being_selected = true;
            }
        }
    }
    emit mouseMovePoint(point);
    update();
}

void QLabel_with_mouse_event::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPoint point = event -> pos();
        if_is_dragging = true;
        emit mouseClicked(point);
    }
    QLabel::mousePressEvent(event);
}

void QLabel_with_mouse_event::mouseReleaseEvent(QMouseEvent *event)
{
     QPoint point = event -> pos();
     if_is_dragging = false;
     if_point_being_selected = false;
     emit mouseReleased(point);
}

void QLabel_with_mouse_event::paintEvent(QPaintEvent *event)
{
    QLabel::paintEvent(event);
    QPainter painter(this);
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidth(1);
    painter.setPen(pen);
    painter.drawEllipse(selectedPoint, 3, 3);

    pen.setColor(Qt::white);
    pen.setWidth(1);
    painter.setPen(pen);

    QBrush brush;
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::white);
    painter.setBrush(brush);



    if(cameraCelibrating == sensorFarImgRaw)
    {
        for(size_t i = 0; i < 4; i++)
        {
            painter.drawEllipse(sensor_far_points[i], 3, 3);
        }
        brush.setStyle(Qt::Dense7Pattern);
        painter.setBrush(brush);
        painter.drawConvexPolygon(sensor_far_points, 4);

    }
    else if(cameraCelibrating == sensorCloseImgRaw)
    {
        for(size_t i = 0; i < 4; i++)
        {
            painter.drawEllipse(sensor_close_points[i], 3, 3);
        }
        brush.setStyle(Qt::Dense7Pattern);
        painter.setBrush(brush);
        painter.drawConvexPolygon(sensor_close_points, 4);
    }

}


QLabel_with_mouse_event::QLabel_with_mouse_event(QWidget *parent) : QLabel{parent}
{
    setMouseTracking(true);
    selectedPoint = QPoint(0, 0);
    if_is_dragging = false;
    if_point_being_selected = false;
}

