#include <Ice/Ice.h>
#include <QDebug>
#include <QMainWindow>
#include <QTimer>

#include "../common/gnuradio.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(Ice::ObjectPrx ice_prx, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->mainToolBar->setVisible(false);

    // create control-port instance
    ctrlport = GNURadio::ControlPortPrx::checkedCast(ice_prx);
    makeParamList();

    // setup data refreshimer
    dataTimer = new QTimer(this);
    connect(dataTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    dataTimer->start(1000);
}

MainWindow::~MainWindow()
{
    dataTimer->stop();
    delete dataTimer;
    delete ui;
}

void MainWindow::makeParamList(void)
{
    id_list.push_back("strx::lo");
    id_list.push_back("strx::fft");
}

void MainWindow::refresh(void)
{
    GNURadio::KnobMap knob_map; // map<string, GNURadio::KnobPtr>
    GNURadio::KnobPtr  knob;
    GNURadio::KnobDPtr knob_lo;
    GNURadio::KnobVecFPtr knob_fft;

    knob_map = ctrlport->get(id_list);

    knob = knob_map["strx::lo"];
    knob_lo = (GNURadio::KnobDPtr)(knob);
    qDebug() << "strx::lo" << knob_lo->value;

    knob = knob_map["strx::fft"];
    knob_fft = (GNURadio::KnobVecFPtr)(knob);
    qDebug() << "strx::fft" << knob_fft->value.size();
}
