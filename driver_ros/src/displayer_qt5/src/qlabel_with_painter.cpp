#include "displayer_qt5/qlabel_with_painter.h"

void QLabel_with_painter::paintEvent(QPaintEvent * event)
{
    //auto end = std::chrono::system_clock::now();
    //std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()<< "ms" << std::endl;
    QLabel::paintEvent(event);
    QPainter painter(this);
    QPen pen;
    QFont font;
    QBrush brush;
    pen.setColor(Qt::red);
    pen.setWidth(3);
    painter.setPen(pen); 
    font.setPointSize(15);
    painter.setFont(font);
    if(tim)
    {
        brush.setStyle(Qt::Dense7Pattern);
        brush.setColor(Qt::red);
        painter.setBrush(brush);
        if(roiWarnState & 0x01)
        {
            painter.drawPolygon(placeLeap_en, 4);
        }
        if(roiWarnState & 0x02)
        {
            painter.drawPolygon(placeHitWindMill_en, 4);
        }
        if(roiWarnState & 0x04)
        {
            painter.drawPolygon(placeOutpose_en, 4);
        }
        if(roiWarnState & 0x08)
        {
            painter.drawPolygon(placeRB1_en, 4);
        }
        if(roiWarnState & 16)
        {
            painter.drawPolygon(placeRB3_en, 5);
        }
        if(roiWarnState & 32)
        {
            painter.drawPolygon(placeRB1_te, 4);
        }
        if(roiWarnState & 64)
        {
            painter.drawPolygon(placeRB2_te, 7);
        }
        if(roiWarnState & 128)
        {
            painter.drawPolygon(placeRB3_te, 5);
        }
        if(roiWarnState & 256)
        {
            painter.drawPolygon(placeHitWindMill_te, 4);
        }
        if(roiWarnState & 512)
        {
            painter.drawPolygon(placeLeap_te, 4);
        }
        if(roiWarnState & 1024)
        {
            painter.drawPolygon(placeOutpose_te, 4);
        }
        //painter.drawPolygon(placeRB2_en, 7);
    }
    brush.setStyle(Qt::SolidPattern);
    for(size_t i = 0; i < worldPoints.size(); i++)
    {
        int id = worldPoints[i].id;
        if(id <= 5 || id == 12)
        {
            pen.setColor(Qt::red);
            brush.setColor(Qt::red);
        }
        else
        {
            pen.setColor(Qt::blue);
            brush.setColor(Qt::blue);
        }

        painter.setPen(pen);
        painter.setBrush(brush);
        painter.drawEllipse(worldPoints[i].point, 10, 10);
        if(id < 12)
        {
            if(id <= 4)
            {
                id += 1;
            }
            else if(id == 5)
            {
                id = 7;
            }
            else if(id <= 10)
            {
                id -= 5;
            }
            else
            {
                id = 7;
            }
            pen.setColor(Qt::white);
            painter.setPen(pen);
            painter.drawText(worldPoints[i].point.x() - 6, worldPoints[i].point.y() + 8, std::to_string(id).c_str());
        }
    }
}

QLabel_with_painter::QLabel_with_painter(QWidget *parent) : QLabel{parent}
{
    placeRB1_te[0] = QPoint(0, 316);
    placeRB1_te[1] = QPoint(0, 450);
    placeRB1_te[2] = QPoint(26, 450);
    placeRB1_te[3] = QPoint(26, 316);
    placeRB2_te[0] = QPoint(80, 382); //280 290
    placeRB2_te[1] = QPoint(61, 409); //299 263
    placeRB2_te[2] = QPoint(128, 455); //232 217
    placeRB2_te[3] = QPoint(198, 455); //162 217
    placeRB2_te[4] = QPoint(207, 450); //153 222
    placeRB2_te[5] = QPoint(188, 424); //172 248
    placeRB2_te[6] = QPoint(138, 424); //222 248
    placeRB3_te[0] = QPoint(0, 458); //360 214
    placeRB3_te[1] = QPoint(0, 564); //360 108
    placeRB3_te[2] = QPoint(126, 564); //234 108
    placeRB3_te[3] = QPoint(126, 523); //234 149
    placeRB3_te[4] = QPoint(25, 458); //335 214
    placeOutpose_te[0] = QPoint(254, 356);
    placeOutpose_te[1] = QPoint(254, 446);
    placeOutpose_te[2] = QPoint(331, 446);
    placeOutpose_te[3] = QPoint(331, 356);
    placeLeap_te[0] = QPoint(334, 371);
    placeLeap_te[1] = QPoint(334, 515);
    placeLeap_te[2] = QPoint(360, 515);
    placeLeap_te[3] = QPoint(360, 371);
    placeHitWindMill_te[0] = QPoint(296, 446);
    placeHitWindMill_te[1] = QPoint(296, 487);
    placeHitWindMill_te[2] = QPoint(333, 487);
    placeHitWindMill_te[3] = QPoint(333, 446);

    placeRB1_en[0] = QPoint(334, 266);
    placeRB1_en[1] = QPoint(334, 356);
    placeRB1_en[2] = QPoint(360, 356);
    placeRB1_en[3] = QPoint(360, 266);
    placeRB2_en[0] = QPoint(280, 290);
    placeRB2_en[1] = QPoint(299, 263);
    placeRB2_en[2] = QPoint(232, 217);
    placeRB2_en[3] = QPoint(162, 217);
    placeRB2_en[4] = QPoint(153, 222);
    placeRB2_en[5] = QPoint(172, 248);
    placeRB2_en[6] = QPoint(222, 248);
    placeRB3_en[0] = QPoint(360, 214);
    placeRB3_en[1] = QPoint(360, 108);
    placeRB3_en[2] = QPoint(234, 108);
    placeRB3_en[3] = QPoint(234, 149);
    placeRB3_en[4] = QPoint(335, 214);
    placeOutpose_en[0] = QPoint(29, 226);
    placeOutpose_en[1] = QPoint(29, 316);
    placeOutpose_en[2] = QPoint(106, 316);
    placeOutpose_en[3] = QPoint(106, 226);
    placeLeap_en[0] = QPoint(0, 157);
    placeLeap_en[1] = QPoint(0, 301);
    placeLeap_en[2] = QPoint(28, 301);
    placeLeap_en[3] = QPoint(28, 157);
    placeHitWindMill_en[0] = QPoint(27, 185);
    placeHitWindMill_en[1] = QPoint(27, 227);
    placeHitWindMill_en[2] = QPoint(64, 227);
    placeHitWindMill_en[3] = QPoint(64, 185);

    tim = false;

    roiWarnState = 0x00;
}

void QLabel_with_painter::drawSmallMap(std::vector<world_point>& wp)
{
    std::vector<world_point>().swap(worldPoints);
    for(size_t i = 0; i < wp.size(); i++)
    {
        worldPoints.push_back(wp[i]);
    }
    update();
}

void QLabel_with_painter::drawROI(unsigned short* input)
{
    roiWarnState = *input;

}
