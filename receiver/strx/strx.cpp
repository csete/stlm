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
//#include <iostream>
//#include <boost/program_options.hpp>

//namespace po = boost::program_options;

int main(int argc, char **argv)
{
    receiver *rx;

/*
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
*/

    rx = new receiver();
    rx->start();


    delete rx;

    return 0;
}
