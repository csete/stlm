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

#include "strx_source_c_impl.h"

#include <config.h>
#include <gr_io_signature.h>
#include <gruel/attributes.h>
#include <rpcregisterhelpers.h>


/* Create a new instance of fcd_source_c_impl and return
 * an upcasted boost shared_ptr. This is effectively the public constructor.
 */
strx_source_c_sptr strx_make_source_c(const std::string input, double quad_rate)
{
    return gnuradio::get_initial_sptr(new strx_source_c_impl(input, quad_rate));
}

strx_source_c_impl::strx_source_c_impl(const std::string input, double quad_rate)
  : gr_hier_block2("strx_source_c",
                   gr_make_io_signature(0, 0, sizeof (gr_complex)),
                   gr_make_io_signature(1, 1, sizeof (gr_complex))),
    d_quad_rate(quad_rate),
    d_freq(0.0)
{

    // Check input type
    if (input.find("file:") != std::string::npos)
    {
        input_type = INPUT_TYPE_FILE;
        std::string filename = input.substr(5);
        file_src = gr::blocks::file_source::make(sizeof(gr_complex), filename.c_str(), true);
        throttle = gr::blocks::throttle::make(sizeof(gr_complex), d_quad_rate);
        
        connect(file_src, 0, throttle, 0);
        connect(throttle, 0, self(), 0);
    }
    else
    {
        input_type = INPUT_TYPE_UHD;
        usrp_src = gr::uhd::usrp_source::make(uhd::device_addr_t(""), uhd::io_type_t::COMPLEX_FLOAT32, 1);
        usrp_src->set_samp_rate(d_quad_rate);
        if (!input.empty())
            usrp_src->set_subdev_spec(input);
            
        connect(usrp_src, 0, self(), 0);
    }
}

void strx_source_c_impl::set_freq(double freq)
{
    if (input_type == INPUT_TYPE_UHD)
    {
        usrp_src->set_center_freq(freq);
        d_freq = usrp_src->get_center_freq();
    }
}

double strx_source_c_impl::get_freq(void)
{
    if (input_type == INPUT_TYPE_UHD)
        return usrp_src->get_center_freq();
    else
        return 0.0;
}

void strx_source_c_impl::get_freq_range(double *start, double *stop, double *step)
{
    if (input_type == INPUT_TYPE_UHD)
    {
        uhd::freq_range_t range = usrp_src->get_freq_range();
        *start = range.start();
        *stop = range.stop();
        *step = range.step();
    }
}

void strx_source_c_impl::set_gain(double gain)
{
    if (input_type == INPUT_TYPE_UHD)
        usrp_src->set_gain(gain);
}

double strx_source_c_impl::get_gain(void)
{
    if (input_type == INPUT_TYPE_UHD)
        return usrp_src->get_gain();
    else
        return 0.0;
}

void strx_source_c_impl::get_gain_range(double *start, double *stop, double *step)
{
    if (input_type == INPUT_TYPE_UHD)
    {
        uhd::gain_range_t range = usrp_src->get_gain_range();
        *start = range.start();
        *stop = range.stop();
        *step = range.step();
    }
}

void strx_source_c_impl::set_antenna(std::string antenna)
{
    if (input_type == INPUT_TYPE_UHD)
        usrp_src->set_antenna(antenna);
}

void strx_source_c_impl::setup_rpc(void)
{
#ifdef GR_CTRLPORT
    double start, stop, step;

    // Frequency
    get_freq_range(&start, &stop, &step);
    add_rpc_variable(
        rpcbasic_sptr(new rpcbasic_register_get<strx_source_c, double>(
            alias(), "Frequency",
            &strx_source_c::get_freq,
            pmt::mp(start), pmt::mp(stop), pmt::mp((stop-start)/2.0),
            "Hz", "Get frequency",
            RPC_PRIVLVL_MIN, DISPNULL)
        )
    );
    add_rpc_variable(
        rpcbasic_sptr(new rpcbasic_register_set<strx_source_c, double>(
            alias(), "Frequency",
            &strx_source_c::set_freq,
            pmt::mp(start), pmt::mp(stop), pmt::mp((stop-start)/2.0),
            "Hz", "Set frequency",
            RPC_PRIVLVL_MIN, DISPNULL)
        )
    );

    // Gain
    get_gain_range(&start, &stop, &step);
    /*add_rpc_variable(
        rpcbasic_sptr(new rpcbasic_register_get<strx_source_c, double>(
            alias(), "Gain",
            &strx_source_c::get_gain,
            pmt::mp(start), pmt::mp(stop), pmt::mp((stop-start)/2.0),
            "dB", "Get gain",
            RPC_PRIVLVL_MIN, DISPNULL)
        )
    );*/
    add_rpc_variable(
        rpcbasic_sptr(new rpcbasic_register_set<strx_source_c, double>(
            alias(), "Gain",
            &strx_source_c::set_gain,
            pmt::mp(start), pmt::mp(stop), pmt::mp((stop-start)/2.0),
            "dB", "Set gain",
            RPC_PRIVLVL_MIN, DISPNULL)
        )
    );

#endif
}
