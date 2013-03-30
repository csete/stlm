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

#include <rpcregisterhelpers.h>
#include "receiver.h"


/*! \brief Public contructor.
 *  \param input Input device specifier (see below).
 *  \param output Output file name. Using stdout if empty.
 *  \param quad_rate Quadrature rate in samples per second.
 * 
 * The input can be a complex I/Q file or a USRP device. I/Q file is selected if the device string
 * is of the form "file:/some/path", otherwise UHD is assumed with subdev in the string.
 * 
 * \todo Use gr-osmosdr as soon as it support gnuradio 3.7
 */
receiver::receiver(const std::string input, const std::string output, double quad_rate)
    : d_running(false),
      d_quad_rate(quad_rate)
{

    tb = gr_make_top_block("strx");
    
    src = strx_make_source_c(input, d_quad_rate);

    taps = gr::filter::firdes::complex_band_pass(1.0, d_quad_rate, -400e3, 400.3e3, 900.e3);
    filter = gr::filter::fft_filter_ccc::make(1, taps);
    demod = gr::analog::quadrature_demod_cf::make(1.f);
    iir = gr::filter::single_pole_iir_filter_ff::make(1.e-3);
    sub = gr::blocks::sub_ff::make();
    clock_recov = gr::digital::clock_recovery_mm_ff::make(8.f, 10.e-3f, 10.e-3f, 1.e-3f, 10.e-3f);

    if (output.empty())
    {
        fifo = gr::blocks::file_sink::make(sizeof(float), "/dev/fd/1");
    }
    else
    {
        fifo = gr::blocks::file_sink::make(sizeof(float), output.c_str());
    }

    connect_all();
}

/*! \brief Public destructor. */
receiver::~receiver()
{
    tb->stop();
}


/*! \brief Start the receiver. */
void receiver::start()
{
    if (!d_running)
    {
        tb->run();
        //tb->start();
        d_running = true;
    }
}

/*! \brief Stop the receiver. */
void receiver::stop()
{
    if (d_running)
    {
        tb->stop();
        tb->wait(); // If the graph is needed to run again, wait() must be called after stop
        d_running = false;
    }
}

void receiver::set_antenna(std::string antenna)
{
    src->set_antenna(antenna);
}

/*! Set new RF frequency.
 * \param freq_hz The new frequency in Hz.
 */
void receiver::set_rf_freq(double freq_hz)
{
    src->set_freq(freq_hz);
}

/*! Get current RF frequency.
 * \returns The current RF frequency or 0 if using a file input.
 */
double receiver::rf_freq(void)
{
    return src->get_freq();
}

/*! Get RF frequency range for the current device
 * \param[out] start The lower end of the frequency range.
 * \param[out] stop The upper end of the frequency range.
 * \param[out] step Not used
 */
void receiver::rf_freq_range(double *start, double *stop, double *step)
{
    src->get_freq_range(start, stop, step);
}

/*! Set new RF gain.
 * \param gain The new gain in dB.
 */
void receiver::set_rf_gain(double gain)
{
    src->set_gain(gain);
}

/*! Get current RF gain.
 * \returns The current RF gain.
 */
double receiver::rf_gain(void)
{
    return src->get_gain();
}

/*! Get gain range for current USRP device.
 * \param[out] start The lower limit of the gain range.
 * \param[out] stop The upper limit of the gain range.
 * \param[out] step Not used
 */
void receiver::rf_gain_range(double *start, double *stop, double *step)
{
    src->get_gain_range(start, stop, step);
}

void receiver::set_filter(double low, double high, double trans_width)
{
}

/*! Connect all blocks in the receiver chain. */
void receiver::connect_all()
{
    tb->connect(src, 0, filter, 0);
        
    tb->connect(filter, 0, demod, 0);
    tb->connect(demod, 0, iir, 0);
    tb->connect(demod, 0, sub, 0);
    tb->connect(iir, 0, sub, 1);
    tb->connect(sub, 0, clock_recov, 0);
    tb->connect(clock_recov, 0, fifo, 0);
}
