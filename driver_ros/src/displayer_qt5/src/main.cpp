/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/displayer_qt5/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    // return 0;
    qDebug("main start\n");

    QApplication app(argc, argv);
    displayer_qt5::MainWindow w(argc,argv);
    w.show();
    int result = app.exec();
    std::cout << w.size().width() << '\t' << w.size().height() << std::endl; //输出窗口大小，用于测试
	return result;
}
