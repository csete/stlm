/*
 * Copyright 2013 Alexandru Csete
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

// Include header files for each block used in flowgraph
#include <gr_complex.h>
#include <gr_top_block.h>
#include <analog/quadrature_demod_cf.h>
#include <blocks/file_sink.h>
#include <blocks/file_source.h>
#include <blocks/null_sink.h>
#include <blocks/sub_ff.h>
#include <blocks/throttle.h>
#include <digital/clock_recovery_mm_ff.h>
#include <filter/fft_filter_ccc.h>
#include <filter/firdes.h>
#include <filter/single_pole_iir_filter_ff.h>

// other includes
#include <iostream>
#include <boost/program_options.hpp>


namespace po = boost::program_options;

int main(int argc, char **argv)
{
    double quad_rate = 2.e6;

    // command line options
    std::string device;
    int freq;
    float gain;

    po::options_description desc("Command line options");
    desc.add_options()
        ("help", "This help message")
        ("device", po::value<std::string>(&device)->default_value("hw:1"), "Audio input device")
        ("freq", po::value<int>(&freq)->default_value(145500), "RF frequency in kHz")
        ("gain", po::value<float>(&gain)->default_value(20.0), "LNA gain in dB")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")){
        std::cout << "Narrow band FM receiver example" << std::endl << desc << std::endl;
        return ~0;
    }

    // Construct a top block that will contain flowgraph blocks.
    gr_top_block_sptr tb = gr_make_top_block("strx");

    gr::blocks::file_source::sptr src = gr::blocks::file_source::make(sizeof(gr_complex), "rf@2M-samples.raw", true);
    gr::blocks::throttle::sptr throttle = gr::blocks::throttle::make(sizeof(gr_complex), quad_rate);
    gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(float));

    // Complex band pass filter
    std::vector<gr_complex> taps = gr::filter::firdes::complex_band_pass(1.0, quad_rate, -400e3, 400.3e3, 900.e3);
    gr::filter::fft_filter_ccc::sptr filter = gr::filter::fft_filter_ccc::make(1, taps);

    // FM demodulator
    // gain = sample_rate / (2*pi*max_dev)
    //gr_quadrature_demod_cf_sptr demod = gr_make_quadrature_demod_cf (rate/(2.0*pi*5000.0));
    gr::analog::quadrature_demod_cf::sptr demod = gr::analog::quadrature_demod_cf::make(1.f);

    // carrier offset compensation
    gr::filter::single_pole_iir_filter_ff::sptr iir = gr::filter::single_pole_iir_filter_ff::make(1.e-3);
    gr::blocks::sub_ff::sptr sub = gr::blocks::sub_ff::make();
    
    // clock recovery
    gr::digital::clock_recovery_mm_ff::sptr clock_recov = gr::digital::clock_recovery_mm_ff::make(8.f, 10.e-3f, 10.e-3f, 1.e-3f, 10.e-3f);

    // fifo sink
    gr::blocks::file_sink::sptr fifo = gr::blocks::file_sink::make(sizeof(float), "fifo");
    
    // Connect blocks
    tb->connect(src, 0, throttle, 0);
    tb->connect(throttle, 0, filter, 0);
    tb->connect(filter, 0, demod, 0);
    tb->connect(demod, 0, iir, 0);
    tb->connect(demod, 0, sub, 0);
    tb->connect(iir, 0, sub, 1);
    tb->connect(sub, 0, clock_recov, 0);
    tb->connect(clock_recov, 0, fifo, 0);

    // Tell GNU Radio runtime to start flowgraph threads; the foreground thread
    // will block until either flowgraph exits (this example doesn't) or the
    // application receives SIGINT (e.g., user hits CTRL-C).
    //
    // Real applications may use tb->start() which returns, allowing the foreground
    // thread to proceed, then later use tb->stop(), followed by tb->wait(), to cleanup
    // GNU Radio before exiting.
    tb->run();

    // Exit normally.
    return 0;
}
