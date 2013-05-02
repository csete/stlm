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
#include <boost/thread.hpp>

#ifdef GR_CTRLPORT
#include <rpcregisterhelpers.h>
#endif
#include "receiver.h"


static long fft_delay_msec = 40;

/*! \brief FFT thread function.
 *  \param rx The active instance of the receiver object.
 *
 * The FFT thread function rund periodically and performs the following
 * tasks:
 *   - Get new FFT data and scale the FFT properly
 *   - Calculate SNR for both receiver channels
 *   - Send SNR for the active channel to the audio indicator.
 */
static void fft_thread_func(receiver *rx)
{
    for(;;)
    {
        std::cout << "FFT thread running: " << rx->lo() << std::endl;

        try
        {
            // Sleep and check for interrupt.
            // To check for interrupt without sleep,
            // use boost::this_thread::interruption_point()
            // which also throws boost::thread_interrupted
            boost::this_thread::sleep(boost::posix_time::milliseconds(fft_delay_msec));
        }
        catch(boost::thread_interrupted&)
        {
            std::cout << "FFT thread is stopped" << std::endl;
            return;
        }
    }
}

/*! \brief Public contructor.
 *  \param name The receiver name. Used for ctrlport names.
 *  \param input Input device specifier (see below).
 *  \param output Output file name. Using stdout if empty.
 *  \param quad_rate Quadrature rate in samples per second.
 * 
 * The input can be a complex I/Q file or a USRP device. I/Q file is selected if the device string
 * is of the form "file:/some/path", otherwise UHD is assumed with subdev in the string.
 * 
 * \todo Use gr-osmosdr as soon as it support gnuradio 3.7
 */
receiver::receiver(const std::string name, const std::string input, const std::string output, double quad_rate)
    : d_running(false),
      d_quad_rate(quad_rate)
{

    if (name.empty())
        d_name = "strx";
    else
        d_name = name;

    tb = gr_make_top_block(d_name);

    src = strx::source_c::make(input, d_quad_rate);
    fft = strx::fft_c::make();

    taps = filter::firdes::low_pass(1.0, d_quad_rate, 400e3, 900.e3);
    filter = filter::freq_xlating_fir_filter_ccf::make(1, taps, 0.0, d_quad_rate);
    demod = analog::quadrature_demod_cf::make(1.f);
    iir = filter::single_pole_iir_filter_ff::make(1.e-3);
    sub = blocks::sub_ff::make();
    clock_recov = digital::clock_recovery_mm_ff::make(8.f, 10.e-3f, 10.e-3f, 1.e-3f, 10.e-3f);

    if (output.empty())
    {
        fifo = blocks::file_sink::make(sizeof(float), "/dev/fd/1");
    }
    else
    {
        fifo = blocks::file_sink::make(sizeof(float), output.c_str());
    }

    fft_thread = boost::thread(&fft_thread_func, this);

#ifdef GR_CTRLPORT
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, double>
            (
                d_name,   // const std::string& name,
                "frequency",  // const char* functionbase,
                this,      // T* obj,
                &receiver::rf_freq, // Tfrom (T::*function)(),
                pmt::mp(50.0e6), pmt::mp(2.0e9), pmt::mp(100.0e6),
                "Hz", // const char* units_ = "",
                "RF frequency", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));

    // Filter offset (aka. receiver LO)
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, double>
            (
                d_name,   // const std::string& name,
                "lo",  // const char* functionbase,
                this,      // T* obj,
                &receiver::lo, // Tfrom (T::*function)(),
                pmt::mp(-d_quad_rate/2.0), pmt::mp(d_quad_rate/2.0), pmt::mp(0.0),
                "Hz", // const char* units_ = "",
                "Receiver LO", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_set<receiver, double>
            (
                d_name,   // const std::string& name,
                "lo",  // const char* functionbase,
                this,      // T* obj,
                &receiver::set_lo, // Tfrom (T::*function)(),
                pmt::mp(-d_quad_rate/2.0), pmt::mp(d_quad_rate/2.0), pmt::mp(0.0),
                "Hz", // const char* units_ = "",
                "Receiver LO", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));

#endif

    connect_all();
}

/*! \brief Public destructor. */
receiver::~receiver()
{
    fft_thread.interrupt();
    fft_thread.join();
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

/*! Set receiver LO.
 * \param lo The new LO frequency in Hz.
 */
void receiver::set_lo(double lo)
{
    filter->set_center_freq(lo);
}

/*! Get receiver LO.
 * \returns The current receiver LO frequency in Hz.
 */
double receiver::lo(void)
{
    return filter->center_freq();
}

void receiver::set_filter(double low, double high, double trans_width)
{
}

/*! Set FFT rate.
 *  \param rate The new FFT rate in Hz (updates per second).
 *
 * The FFT rate is the frequency by which the FFT thread is running.
 *
 * \sa fft_thread_func
 */
void receiver::set_fft_rate(long rate)
{
    fft_delay_msec = 1000 / rate;
}

/*! Connect all blocks in the receiver chain. */
void receiver::connect_all()
{
    tb->connect(src, 0, filter, 0);
    tb->connect(src, 0, fft, 0);
    tb->connect(filter, 0, demod, 0);
    tb->connect(demod, 0, iir, 0);
    tb->connect(demod, 0, sub, 0);
    tb->connect(iir, 0, sub, 1);
    tb->connect(sub, 0, clock_recov, 0);
    tb->connect(clock_recov, 0, fifo, 0);
}
