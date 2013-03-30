/* -*- c++ -*- */
/*
 * Copyright 2013 Alexandru Csete, OZ9AEC
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
 * along with Gqrx; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#ifndef RECEIVER_H
#define RECEIVER_H

// standard includes
#include <string>

// GNU Radio includes
#include <analog/quadrature_demod_cf.h>
#include <blocks/file_sink.h>
#include <blocks/sub_ff.h>
#include <digital/clock_recovery_mm_ff.h>
#include <filter/fft_filter_ccc.h>
#include <filter/firdes.h>
#include <filter/single_pole_iir_filter_ff.h>
#include <gr_complex.h>
#include <gr_top_block.h>

// strx includes
#include "strx_source_c.h"


/*! \defgroup RX High level receiver blocks. */

/*! \brief Top-level receiver class.
 *  \ingroup RX
 *
 * This class encapsulates the GNU Radio flow graph for the receiver.
 * Front-ends should only control the receiver through the interface provided
 * by this class.
 *
 */
class receiver
{

public:

    receiver(const std::string input="", const std::string output="", double quad_rate=2.e6);
    ~receiver();

    void start();
    void stop();

    void set_input_device(const std::string device);
    void set_output_device(const std::string device);

    void set_antenna(std::string antenna);

    void rf_freq_range(double *start, double *stop, double *step);
    void set_rf_freq(double freq);
    double rf_freq(void);

    void rf_gain_range(double *start, double *stop, double *step);
    void set_rf_gain(double gain);
    double rf_gain(void);

    void set_filter(double low, double high, double trans_width);

private:
    void connect_all(void);

private:

    /*! Input type. */
    enum input_type_e
    {
        INPUT_TYPE_FILE = 0, /*!< Input is complex 2 Msps I/Q file. */
        INPUT_TYPE_UHD  = 1, /*!< Input is a USRP device. */
    };
    
    input_type_e input_type;

    gr_top_block_sptr tb;  /*!< Receiver top block. */
    
    strx_source_c_sptr                         src;    /*!> Input source. */
    std::vector<gr_complex>                     taps;
    gr::filter::fft_filter_ccc::sptr            filter; /*!< Channel filter. */
    gr::analog::quadrature_demod_cf::sptr       demod;  /*!< Demodulator. */
    gr::filter::single_pole_iir_filter_ff::sptr iir;    /*!< IIR filter for carrier offset estimation. */
    gr::blocks::sub_ff::sptr                    sub;    /*!< Carrier offset correction. */
    
    gr::digital::clock_recovery_mm_ff::sptr clock_recov;
    gr::blocks::file_sink::sptr fifo;

    bool d_running;
    double d_quad_rate;
};

#endif // RECEIVER_H
