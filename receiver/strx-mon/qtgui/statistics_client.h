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
#ifndef STATISTICS_CLIENT_H
#define STATISTICS_CLIENT_H

#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <QTimer>

/*! Statistics client.
 *
 * This class is used to connect to the strx correlator and fetch statistics
 * about received telemetry, which includes:
 *  - TX uptime
 *  - TX supply voltage
 *  - TX data input status (from payload)
 *  - Decoded packets and bytes
 *
 * The connection is done using TCP to port 5000. Sending any byte to this
 * port will trigger a reply from the correlator.
 *
 * Once a statistics object is created, the processing is started using scStart()
 * and stopped using scStop(). The statistics object will emit scTlmReceived() whenever
 * new telemetry statistics is available.
 */
class CStatisticsClient : public QObject
{
    Q_OBJECT

public:
    CStatisticsClient(QString _host, quint16 _port, QObject *parent);
    ~CStatisticsClient();

    void scStart(void);
    void scStop(void);

signals:
    void scTlmReceived(float volt, float tx, float gnc, float aau);

private slots:
    void scConnected(void);
    void scDisconnected(void);
    void scSendByte(void);
    void scDataAvailable(void);

private:
    QString     host;
    quint16     port;
    QTcpSocket *socket;
    QTimer     *stat_timer;

    bool        connected;
    bool        running;
};

#endif // STATISTICS_CLIENT_H
