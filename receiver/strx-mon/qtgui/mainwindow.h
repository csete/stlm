#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <Ice/Ice.h>
#include <QMainWindow>
#include <QTimer>

#include "../common/gnuradio.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(Ice::ObjectPrx ice_prx, QWidget *parent = 0);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui;

    GNURadio::ControlPortPrx ctrlport;
    GNURadio::KnobIDList     id_list;  // vector<string>
    QTimer *dataTimer;  /*!< Timer used to fetch data from remote receiver. */

    void makeParamList(void);

private slots:
    void refresh(void);
};

#endif // MAINWINDOW_H
