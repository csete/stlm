#include <iostream>

#include <Ice/Ice.h>
#include <QtGui/QApplication>

#include "mainwindow.h"

int main(int argc, char *argv[])
{
    Ice::CommunicatorPtr ice_com;
    Ice::ObjectPrx       ice_prx;

    try
    {
        // Get proxy object
        ice_com = Ice::initialize(argc, argv);
        ice_prx = ice_com->stringToProxy("gnuradio:tcp -h localhost -p 43243");
    }
    catch (const Ice::Exception& ex)
    {
        std::cerr << ex << std::endl;
        return 1;
    }

    QApplication a(argc, argv);
    MainWindow w(ice_prx);
    w.show();
    
    return a.exec();
}
