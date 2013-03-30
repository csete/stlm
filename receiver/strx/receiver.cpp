/* -*- c++ -*- */
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
      d_quad_rate(quad_rate),
      d_rf_freq(0.0)
{
    // Check input type
    if (input.find("file:") != std::string::npos)
    {
        input_type = INPUT_TYPE_FILE;
        std::string filename = input.substr(5);
        file_src = gr::blocks::file_source::make(sizeof(gr_complex), filename.c_str(), true);
        throttle = gr::blocks::throttle::make(sizeof(gr_complex), d_quad_rate);
    }
    else
    {
        input_type = INPUT_TYPE_UHD;
        usrp_src = gr::uhd::usrp_source::make(uhd::device_addr_t(""), uhd::io_type_t::COMPLEX_FLOAT32, 1);
        usrp_src->set_samp_rate(d_quad_rate);
        if (!input.empty())
            usrp_src->set_subdev_spec(input);
    }

    tb = gr_make_top_block("strx");

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

/*! \brief Select new input device.
 *
 * \bug When using ALSA, program will crash if the new device
 *      is the same as the previously used device:
 *      audio_alsa_source[hw:1]: Device or resource busy
 */
void receiver::set_input_device(const std::string device)
{
    /*
    if (device.empty())
        return;

    if (input_devstr.compare(device) == 0)
    {
        return;
    }

    tb->lock();

    tb->unlock();
    */
}


/*! \brief Select new audio output device. */
void receiver::set_output_device(const std::string device)
{
    /*
    if (output_devstr.compare(device) == 0)
    {
        return;
    }

    tb->lock();

    tb->unlock();
    */
}

void receiver::set_antenna(std::string antenna)
{
    if (input_type == INPUT_TYPE_UHD)
        usrp_src->set_antenna(antenna);
}

/*! Set new RF frequency.
 * \param freq_hz The new frequency in Hz.
 *
 * This function has no effect when using an I/Q file as input.
 */
void receiver::set_rf_freq(double freq_hz)
{
    if (input_type == INPUT_TYPE_UHD)
    {
        usrp_src->set_center_freq(freq_hz);
        d_rf_freq = usrp_src->get_center_freq();
    }
}

/*! Get current RF frequency.
 * \returns The current RF frequency or 0 if using a file input.
 */
double receiver::rf_freq(void)
{
    if (input_type == INPUT_TYPE_UHD)
        return usrp_src->get_center_freq();
    else
        return 0.0;
}

/*! Get RF frequency range for the current device
 * \param[out] start The lower end of the frequency range.
 * \param[out] stop The upper end of the frequency range.
 * \param[out] step ???
 */
void receiver::rf_freq_range(double *start, double *stop, double *step)
{
    if (input_type == INPUT_TYPE_UHD)
    {
        uhd::freq_range_t range = usrp_src->get_freq_range();
        *start = range.start();
        *stop = range.stop();
        *step = range.step();
    }
}

/*! Set new RF gain.
 * \param gain The new gain in dB.
 *
 * This function has no effect when using an I/Q file input.
 */
void receiver::set_rf_gain(double gain)
{
    if (input_type == INPUT_TYPE_UHD)
        usrp_src->set_gain(gain);
}

/*! Get current RF gain.
 * \returns The current RF gain or 0 if using an I/Q file as input.
 */
double receiver::rf_gain(void)
{
    if (input_type == INPUT_TYPE_UHD)
        return usrp_src->get_gain();
    else
        return 0.0;
}

/*! Get gain range for current USRP device.
 * \param[out] start The lower limit of the gain range.
 * \param[out] stop The upper limit of the gain range.
 * \param[out] step ???
 */
void receiver::rf_gain_range(double *start, double *stop, double *step)
{
    if (input_type == INPUT_TYPE_UHD)
    {
        uhd::gain_range_t range = usrp_src->get_gain_range();
        *start = range.start();
        *stop = range.stop();
        *step = range.step();
    }
}

void receiver::set_filter(double low, double high, double trans_width)
{
}

/*! Connect all blocks in the receiver chain. */
void receiver::connect_all()
{
    if (input_type == INPUT_TYPE_FILE)
    {
        tb->connect(file_src, 0, throttle, 0);
        tb->connect(throttle, 0, filter, 0);
    }
    else
    {
        tb->connect(usrp_src, 0, filter, 0);
    }
        
    tb->connect(filter, 0, demod, 0);
    tb->connect(demod, 0, iir, 0);
    tb->connect(demod, 0, sub, 0);
    tb->connect(iir, 0, sub, 1);
    tb->connect(sub, 0, clock_recov, 0);
    tb->connect(clock_recov, 0, fifo, 0);
}
