/*
 * Copyright (C) 2013 Alexandru Csete, OZ9AEC
 *
 * strx-mon is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * strx-mon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#include <QDebug>
#include <QIODevice>

#include "statistics_client.h"

CStatisticsClient::CStatisticsClient(QString _host, quint16 _port, QObject *parent) :
    QObject(parent)
{
    host = _host;
    port = _port;
    connected = false;
    running = false;

    // create socket and establish connection
    socket = new QTcpSocket(parent);
    connect(socket, SIGNAL(connected()), this, SLOT(scConnected()));
    socket->connectToHost(host, port, QIODevice::ReadWrite);

    stat_timer = new QTimer(this);
    connect(stat_timer, SIGNAL(timeout()), this, SLOT(scRunStats()));
}


CStatisticsClient::~CStatisticsClient()
{
    if (running)
        stat_timer->stop();
    delete stat_timer;
    running = false;

    if (connected)
    {
        socket->disconnectFromHost();
        socket->close();
    }
    delete socket;
}

/*! Start cyclic processing of statistics. */
void CStatisticsClient::scStart(void)
{
    stat_timer->start(1000);
    running = true;
}

/*! Stop cyclic processing of statistics. */
void CStatisticsClient::scStop(void)
{
    stat_timer->stop();
    running = false;
}

/*! Connection notification.
 *
 * This slot is used to receive notification when the TCP connection
 * has been established.
 */
void CStatisticsClient::scConnected(void)
{
    qDebug() << __func__;
    connected = true;
}

/*! This slot is called when the connection to the correlator get closed.
 *
 * If this is because of a network error, we try to reconnect.
 */
void CStatisticsClient::scDisconnected(void)
{
    qDebug() << __func__;
    connected = false;

    // if timer is still running, try to reconnect
    if (running)
    {
        socket->connectToHost(host, port, QIODevice::ReadWrite);
    }
}

void CStatisticsClient::scRunStats(void)
{
    if (!connected)
        return;

    qDebug() << __func__;
}
