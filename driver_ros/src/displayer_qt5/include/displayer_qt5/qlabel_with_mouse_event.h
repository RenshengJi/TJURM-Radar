/**
 * @file /include/displayer_qt5/main_window.hpp
 *
 * @brief Qt based gui for displayer_qt5.
 *
 * @date November 2010
 **/
#ifndef displayer_qt5_QLABEL_WITH_MOUSE_EVENT_H
#define displayer_qt5_QLABEL_WITH_MOUSE_EVENT_H

#include <QMainWindow>
#include <QObject>
#include <QLabel>
#include <QPoint>
#include <QMouseEvent>
#include <QPainter>
#include "qnode.hpp"

class QLabel_with_mouse_event : public QLabel
{
    Q_OBJECT
protected:
    void  mouseMoveEvent(QMouseEvent *event);
    void  mousePressEvent(QMouseEvent *event);
    void  mouseReleaseEvent(QMouseEvent *event);
    void  paintEvent(QPaintEvent* );
private:
    bool if_point_being_selected;//是否有点被选中
public:
    explicit QLabel_with_mouse_event(QWidget *parent = nullptr);
    QPoint selectedPoint;//选中的点
    QString cameraCelibrating;//正在标定的相机
    QString sensorFarImgRaw;//左相机话题名
    QString sensorCloseImgRaw;//右相机话题名
    QPoint* sensor_far_points;//左相机中的四点位置
    QPoint* sensor_close_points;//右相机中的四点位置
    bool if_is_dragging;//是否正在拖拽
    int calibrateMainWindowHeight;//主窗口高度
    int calibrateMainWindowWidth;//主窗口宽度
signals:
    void mouseMovePoint(QPoint point);
    void mouseClicked(QPoint point);
    void mouseReleased(QPoint point);
};

#endif // displayer_qt5_QLABEL_WITH_MOUSE_EVENT_H
