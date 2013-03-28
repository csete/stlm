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
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "receiver.h"

// other includes
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

int main(int argc, char **argv)
{
    receiver *rx;

    // command line options
    int freq;
    float gain;
    bool clierr=false;

    po::options_description desc("Command line options");
    desc.add_options()
        ("help,h", "This help message")
        ("freq,f", po::value<int>(&freq)->default_value(145500), "RF frequency in kHz")
        ("gain,g", po::value<float>(&gain)->default_value(20.0), "RF gain in dB")
    ;
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    }
    catch(const boost::program_options::invalid_command_line_syntax& ex)
    {
        /* happens if e.g. -c without file name */
        clierr = true;
    }
    po::notify(vm);

    if (vm.count("help") || clierr)
    {
        std::cout << "Sapphire telemetry receiver " << VERSION << std::endl << desc << std::endl;
        return 1;
    }


    rx = new receiver();
    rx->start();

    delete rx;

    return 0;
}
