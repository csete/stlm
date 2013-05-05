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
