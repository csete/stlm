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
#include <string>

namespace po = boost::program_options;

int main(int argc, char **argv)
{
    receiver *rx;

    // command line options
    double freq;
    double gain;
    bool clierr=false;
    std::string output;

    po::options_description desc("Command line options");
    desc.add_options()
        ("help,h", "This help message")
        ("freq,f", po::value<double>(&freq)->default_value(2335.0e6), "RF frequency in Hz")
        ("gain,g", po::value<double>(&gain)->default_value(20.0), "RF gain in dB")
        ("output,o", po::value<std::string>(&output)->default_value(""), "Output file (use stdout if omitted)")
    ;
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    }
    catch(const boost::program_options::invalid_command_line_syntax& ex)
    {
        clierr = true;
    }
    po::notify(vm);

    if (vm.count("help") || clierr)
    {
        std::cout << "Sapphire telemetry receiver " << VERSION << std::endl << desc << std::endl;
        return 1;
    }


    rx = new receiver("", output);
    rx->start();

    delete rx;

    return 0;
}
