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
#include <vector>

// Boost includes
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>

// GNU Radio includes
#include <analog/quadrature_demod_cf.h>
#include <blocks/file_sink.h>
#include <blocks/sub_ff.h>
#include <config.h>
#include <digital/clock_recovery_mm_ff.h>
#include <filter/freq_xlating_fir_filter_ccf.h>
#include <filter/firdes.h>
#include <filter/single_pole_iir_filter_ff.h>
#include <gr_complex.h>
#include <gr_top_block.h>
#ifdef GR_CTRLPORT
#include <rpcregisterhelpers.h>
#endif

// strx includes
#include "strx_fft.h"
#include "strx_source_c.h"

using namespace gr;

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

    receiver(const std::string name="", const std::string input="", const std::string output="", double quad_rate=4.e6);
    ~receiver();

    void start();
    void stop();

    void set_antenna(std::string antenna);

    void rf_freq_range(double *start, double *stop, double *step);
    void set_rf_freq(double freq);
    double rf_freq(void);

    void rf_gain_range(double *start, double *stop, double *step);
    void set_rf_gain(double gain);
    double rf_gain(void);

    void set_lo(double lo);
    double lo(void);

    void set_filter(double low, double high, double trans_width);

    void set_fft_rate(long rate);

    void process_fft(void);
    void process_snr(void);

    std::vector<float> get_fft_data(void);

private:
    void connect_all(void);

#ifdef GR_CTRLPORT
protected:
    std::vector<boost::any> d_rpc_vars;
public:
    void add_rpc_variable(rpcbasic_sptr s)
    {
        d_rpc_vars.push_back(s);
    }
#endif

private:

    /*! Input type. */
    enum input_type_e
    {
        INPUT_TYPE_FILE = 0, /*!< Input is complex 4 Msps I/Q file. */
        INPUT_TYPE_UHD  = 1, /*!< Input is a USRP device. */
    };
    
    input_type_e input_type;

    gr_top_block_sptr tb;  /*!< Receiver top block. */
    
    strx::source_c::sptr                       src;    /*!< Input source. */
    strx::fft_c::sptr                          fft;    /*!< Receiver FFT block. */
    std::vector<float>                        taps;   /*!< Channel filter taps. */
    filter::freq_xlating_fir_filter_ccf::sptr  filter; /*!< Channel filter. */
    analog::quadrature_demod_cf::sptr          demod;  /*!< Demodulator. */
    filter::single_pole_iir_filter_ff::sptr    iir;    /*!< IIR filter for carrier offset estimation. */
    blocks::sub_ff::sptr                       sub;    /*!< Carrier offset correction. */
    
    digital::clock_recovery_mm_ff::sptr        clock_recov; /*!< M&M clock recovery block .*/
    blocks::file_sink::sptr fifo;

    std::string d_name; /*!< Receiver name. */
    bool d_running;
    double d_quad_rate;

    boost::thread        fft_thread;  /*!< FFT thread. */
    boost::shared_mutex  fft_lock;    /*!< Mutex for locking FFT data while processing and reading. */
    std::complex<float>*  d_fftData;
    int    d_fftLen;  /*!< Number of points returned by FFT block. */
    float *d_realFftData; /** FIXME: use vector */
    float *d_iirFftData;  /** FIXME: use vector */
    float  d_fftAvg;      /*!< FFT averaging parameter set by user (not the true gain). */

};

#endif // RECEIVER_H
