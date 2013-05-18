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
#include <QByteArray>
#include <QDebug>
#include <QIODevice>
#include <QRegExp>
#include <QString>
#include <QStringList>

#include "statistics_client.h"

CStatisticsClient::CStatisticsClient(QString _host, quint16 _port, QObject *parent) :
    QObject(parent)
{
    host = _host;
    port = _port;
    connected = false;
    running = false;

    // intiailise stats variables
    last_uptime = 0.f;
    last_volt   = 0.f;
    last_tx     = 0.f;
    last_gnc    = 0.f;
    last_aau    = 0.f;

    // create socket and establish connection
    socket = new QTcpSocket(parent);
    connect(socket, SIGNAL(connected()), this, SLOT(scConnected()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(scDataAvailable()));
    socket->connectToHost(host, port, QIODevice::ReadWrite);

    stat_timer = new QTimer(this);
    connect(stat_timer, SIGNAL(timeout()), this, SLOT(scSendByte()));
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

/*! Send one byte to correlator.
 *
 * Sending a random byte to the correlator will trigger a respons
 * with the latest statistics. Reading the data happens in scDataAvilable().
 *
 * \sa scDataAvailable
 */
void CStatisticsClient::scSendByte(void)
{
    if (!connected)
        return;

    qDebug() << __func__ << ":" << socket->write("x");;
}

/*! Data is available from correlator. */
void CStatisticsClient::scDataAvailable(void)
{
    QByteArray data = socket->readAll();
    QString data_str = QString(data);

    qDebug() << __func__ << ":" << data_str;

    scParseData(data_str);

    emit scTlmReceived(last_volt, last_tx, last_gnc, last_aau);
}


/*! Parse string received from correlator.
 *
 * The string received from the correlator has the form:
 *
 * TX: 0  Uptime: 98.4  Vbat: 8.43  Flags: 8000  0:<P>,<B>  1:<P>,<B>  ...
 *
 *   <P> : Decoded packets
 *   <B> : Decoded bytes
 * The umber in front of <P>,<B> indicates the source address. For the
 * Sapphire mission in 2013 we use:
 *      0x00: NULL packet
 *      0x01: TX1 TLM
 *      0x02: TX2 TLM
 *      0x11: TX1 / AAU
 *      0x12: TX1 / GNC
 *      0x13: TX2 / AAU
 *      0x14: TX2 / GNC
 *
 * The fields are separated with tab character and the TX status (from TX to flags)
 * is one field. The TX statrus field is always returned even if we have never received
 * anything.
 */
void CStatisticsClient::scParseData(const QString data)
{
    QString tx_status_str;
    QRegExp regexp;
    QStringList data_list = data.split('\t',QString::SkipEmptyParts, Qt::CaseSensitive);

    int n = data_list.size();

    // First field is TX status and should always be returned by correlator
    if (n > 0)
    {
        QStringList list;
        int pos = 0;
        tx_status_str = data_list[0];

        // extract the numbers
        regexp.setPattern("([-+]?[0-9]*\\.?[0-9]+)");  // must enclose expr in () otherwise rx.cap won't work
        while ((pos = regexp.indexIn(tx_status_str, pos)) != -1)
        {
            list << regexp.cap(1);  // return match by first (and only) expression
            pos += regexp.matchedLength();
        }
        if (list.size() == 4)
        {
            // TX ID is in list[0]
            last_uptime = list[1].toFloat();
            last_volt   = list[2].toFloat();
            // TX flags is in list[3]
        }
    }
}
