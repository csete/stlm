/*
 * Copyright (C) 2013 Alexandru Csete, OZ9AEC
 *
 * Strx is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Strx is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
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
    // setup UI
    ui->setupUi(this);
    ui->mainToolBar->setVisible(false);
    ui->plotter->setPercent2DScreen(100);
    ui->plotter->setCenterFreq(2330e6);
    ui->plotter->setFftCenterFreq(2330e6);
    ui->plotter->setSampleRate(4.e6);
    ui->plotter->setSpanFreq(4e6);
    ui->plotter->setFilterBoxEnabled(true);
    ui->plotter->setClickResolution(1e3);
    ui->plotter->setFilterClickResolution(1e3);
    ui->plotter->setDemodRanges(-1e6, -50e3, 50e3, 1e6, true);
    ui->plotter->setHiLowCutFrequencies(-400e3, 400e3);
    ui->plotter->setFftPlotColor(QColor(0x7F,0xFA,0xFA,0xFF));
    ui->plotter->setFftFill(true);
    ui->plotter->resetHorizontalZoom(); // weird that we need to call this...

    // create control-port instance
    ctrlport = GNURadio::ControlPortPrx::checkedCast(ice_prx);
    makeParamList();

    // setup data refreshimer
    cb_counter = 0;
    dataTimer = new QTimer(this);
    connect(dataTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    dataTimer->start(100);
}

MainWindow::~MainWindow()
{
    dataTimer->stop();
    delete dataTimer;
    delete ui;
}

void MainWindow::makeParamList(void)
{
    id_list_all.push_back("strx::frequency");
    id_list_all.push_back("strx::offset");
    id_list_all.push_back("strx::fft");
    id_list_all.push_back("strx::channel");

    id_list_fft.push_back("strx::fft");

    id_list_filt.push_back("strx::frequency");
    id_list_filt.push_back("strx::offset");
    id_list_filt.push_back("strx::cutoff");

    id_list_chan.push_back("strx::channel");
}

void MainWindow::refresh(void)
{
    GNURadio::KnobMap knob_map; // map<string, GNURadio::KnobPtr>
    GNURadio::KnobPtr  knob;
    GNURadio::KnobDPtr knob_d;
    GNURadio::KnobVecFPtr knob_fft;

    cb_counter++;

    // FFT is refreshed in each cycle
    knob_map = ctrlport->get(id_list_fft);

    knob = knob_map["strx::fft"];
    knob_fft = (GNURadio::KnobVecFPtr)(knob);
    if (knob_fft->value.size())
    {
        ui->plotter->setNewFttData(&knob_fft->value[0], knob_fft->value.size());
    }

    // update frequencies and filters parameters at 1Hz
    if (!(cb_counter % 10))
    {
        cb_counter = 0;
        knob_map = ctrlport->get(id_list_filt);

        knob = knob_map["strx::offset"];
        knob_d = (GNURadio::KnobDPtr)(knob);
        ui->plotter->setFilterOffset((qint64)knob_d->value);

        knob = knob_map["strx::cutoff"];
        knob_d = (GNURadio::KnobDPtr)(knob);
        int cutoff = (int)knob_d->value;
        ui->plotter->setHiLowCutFrequencies(-cutoff, cutoff);

        knob = knob_map["strx::frequency"];
        knob_d = (GNURadio::KnobDPtr)(knob);
        ui->plotter->setCenterFreq((qint64)knob_d->value);
    }

}

/*! Cwap channel button has been clicked */
void MainWindow::on_chanButton_clicked(void)
{
    GNURadio::KnobMap  knob_map; // map<string, GNURadio::KnobPtr>
    GNURadio::KnobPtr  knob;
    GNURadio::KnobIPtr knob_i;

    knob_map = ctrlport->get(id_list_chan);
    knob = knob_map["strx::channel"];
    knob_i = (GNURadio::KnobIPtr)(knob);

    qDebug() << "CHANNEL: " << knob_i->value;

    if (knob_i->value)
        knob_i->value = 0;
    else
        knob_i->value = 1;

    // set new value
    ctrlport->set(knob_map);

}
